#!/usr/bin/env bash
# Copyright 2026 Gautham Anil
#
# Licensed under the Apache License, Version 2.0 (the "License").
#
# verify_all.sh — end-to-end verification of the rebuilt climbable ramp.
#
# Runs the whole plan on a machine with ROS 2 Jazzy + Gazebo Harmonic.
# Source nothing first; the script sources setup_env.sh itself.
#
#   ./verify_all.sh                 # build + unit tests + sim + climb + RL smoke
#   ./verify_all.sh --with-nav      # also run one Nav2 goal end to end
#   ./verify_all.sh --no-build      # skip colcon build (use existing overlay)
#   ./verify_all.sh --no-rl         # skip the PPO smoke run
#
# Each stage prints PASS/FAIL; the script exits non-zero if any stage failed.
#
# NOTE: deliberately no `set -u`. ROS 2's /opt/ros/jazzy/setup.bash (and the
# workspace overlay it chains into) reference unbound variables such as
# AMENT_TRACE_SETUP_FILES, so nounset kills the script on the very first
# source before any stage runs. `set -e` is also omitted on purpose: every
# stage records PASS/FAIL and the run continues, so one failure does not hide
# the state of everything after it.
set -o pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# Colcon builds from the workspace root (<ws>/src/coco-robot-ros2 -> <ws>).
WS="$(cd "$REPO/../.." 2>/dev/null && pwd || echo "$REPO")"
[ -d "$WS/src" ] || WS="$REPO"

DO_BUILD=1; DO_RL=1; DO_NAV=0; RAMP_ANGLE=18
for a in "$@"; do
  case "$a" in
    --no-build) DO_BUILD=0 ;;
    --no-rl)    DO_RL=0 ;;
    --with-nav) DO_NAV=1 ;;
    --angle=*)  RAMP_ANGLE="${a#*=}" ;;
    *) echo "unknown arg: $a"; exit 2 ;;
  esac
done

PKGS="coco_config coco_moveit_config coco_rl custom_teleop gazebo_models coco_web"
SIM_PID=""; NAV_PID=""
FAILURES=()

# Tearing down a `ros2 launch` tree needs more than killing `gz sim`: the
# bridge, robot_state_publisher and controller spawners survive and keep
# publishing TF stamped with the OLD sim clock. Start the next sim and its
# clock restarts at 0, so tf2 sees "jump back in time", the buffer is cleared,
# and Nav2's container dies with tf2::NoDataForExtrapolationException. So:
# launches run via setsid (their own process group) and we kill the group,
# then sweep any stragglers by name and wait for the clock to go quiet.
stop_all() {
  for pid in "$NAV_PID" "$SIM_PID"; do
    [ -n "$pid" ] && kill -TERM -- "-$pid" 2>/dev/null
  done
  NAV_PID=""; SIM_PID=""
  pkill -f 'full_world_robo.launch.py' 2>/dev/null
  pkill -f 'nav.launch.py' 2>/dev/null
  pkill -f 'component_container_isolated' 2>/dev/null
  pkill -f 'parameter_bridge' 2>/dev/null
  pkill -f 'robot_state_publisher' 2>/dev/null
  pkill -f 'controller_manager' 2>/dev/null
  pkill -f 'gz sim' 2>/dev/null
  sleep 3
  pkill -9 -f 'gz sim' 2>/dev/null
  sleep 2
}
trap stop_all EXIT

# Launch in its own process group so the whole tree can be killed later.
launch_bg() {  # $1 logfile, rest: ros2 launch args
  local log="$1"; shift
  setsid ros2 launch "$@" >"$log" 2>&1 &
  echo $!
}

step()  { echo; echo "======== $* ========"; }
pass()  { echo "PASS: $*"; }
fail()  { echo "FAIL: $*"; FAILURES+=("$*"); }

wait_for_topic() {  # $1 topic, $2 timeout_s
  local t=$(( $(date +%s) + ${2:-90} ))
  while [ "$(date +%s)" -lt "$t" ]; do
    ros2 topic list 2>/dev/null | grep -qx "$1" && return 0
    sleep 2
  done
  return 1
}

# Actions must be probed with `ros2 action list`, NOT `ros2 topic list`: an
# action's topics live under <action>/_action/... and the leading underscore
# makes them *hidden*, so `ros2 topic list` omits them unless
# --include-hidden-topics is passed. Waiting on the topic name reported Nav2 as
# "never came up" while its lifecycle log clearly showed all nodes active.
wait_for_action() {  # $1 action, $2 timeout_s
  local t=$(( $(date +%s) + ${2:-90} ))
  while [ "$(date +%s)" -lt "$t" ]; do
    ros2 action list 2>/dev/null | grep -qx "$1" && return 0
    sleep 2
  done
  return 1
}

# ── env ──────────────────────────────────────────────────────────────────────
step "0  source setup_env.sh + disk check"
# shellcheck disable=SC1091
source "$REPO/setup_env.sh" || { echo "could not source setup_env.sh"; exit 1; }
# Prove the environment actually came up: a silently half-sourced env would
# otherwise sail past every stage and report a meaningless pass.
for tool in ros2 colcon gz; do
  command -v "$tool" >/dev/null \
    || { echo "FATAL: '$tool' not on PATH after sourcing setup_env.sh"; exit 1; }
done
echo "ros2/colcon/gz on PATH; ROS_DISTRO=${ROS_DISTRO:-unset}"
df -h "$WS" | tail -1

# ── build ────────────────────────────────────────────────────────────────────
if [ "$DO_BUILD" -eq 1 ]; then
  step "1  colcon build (installs the new wedge mesh, scripts and launch)"
  ( cd "$WS" && colcon build --packages-select $PKGS ) \
    && pass "colcon build" || fail "colcon build"
  # shellcheck disable=SC1091
  source "$WS/install/setup.bash"
else
  step "1  colcon build SKIPPED (--no-build); sourcing existing overlay"
  [ -f "$WS/install/setup.bash" ] && source "$WS/install/setup.bash"
fi

# ── unit tests ───────────────────────────────────────────────────────────────
step "2  colcon test (97 unit tests incl. gen_ramp + summit goal)"
( cd "$WS" && colcon test --packages-select $PKGS \
      && colcon test-result --verbose ) \
  && pass "colcon test" || fail "colcon test"

# ── bring up the sim ─────────────────────────────────────────────────────────
step "3  launch full_world_robo.launch.py (headless, ${RAMP_ANGLE} deg wedge)"
stop_all   # never start on top of a previous run's nodes
SIM_PID=$(launch_bg /tmp/coco_sim.log gazebo_models full_world_robo.launch.py \
                    gui:=false "ramp_angle:=${RAMP_ANGLE}")
# Gate on BOTH odometry sources: /model/coco/odometry (gz plugin) appears well
# before the ros2_control stack finishes activating, and verify_sim also checks
# /diff_drive_controller/odom — waiting only for the former raced the controller
# and produced a spurious "no messages" failure.
if wait_for_topic /model/coco/odometry 120 \
   && wait_for_topic /diff_drive_controller/odom 120; then
  sleep 5   # let the controllers settle before measuring rates
  pass "sim topics up"
else
  fail "sim never published both odometry topics (see /tmp/coco_sim.log)"
fi

# ── six-topic health ─────────────────────────────────────────────────────────
step "4  verify_sim.py (all six topics at nominal rate)"
ros2 run gazebo_models verify_sim.py && pass "verify_sim" || fail "verify_sim"

# ── the core proof: can it climb? ────────────────────────────────────────────
step "5  climb_check.py (drive to the summit — the teleop proof)"
ros2 run gazebo_models climb_check.py --duration 60 \
  && pass "robot climbed the ${RAMP_ANGLE} deg wedge" \
  || fail "robot did not reach the summit"

# ── RL smoke run ─────────────────────────────────────────────────────────────
if [ "$DO_RL" -eq 1 ]; then
  step "6  PPO smoke run (2048 steps, seeded) — episodes should not die early"
  timeout 900 python3 -m coco_rl.train_ppo --fast --steps 2048 --seed 0 \
      --ramp-angle "${RAMP_ANGLE}" --out /tmp/ppo_smoke
  if [ -s /tmp/ppo_smoke.monitor.csv ]; then
    ep=$(python3 - <<'PY'
import csv
# A Monitor CSV opens with a '#{"t_start": ...}' JSON comment line; feeding
# that straight to DictReader makes it the header and yields zero usable rows
# (which silently printed nothing here before).
with open('/tmp/ppo_smoke.monitor.csv') as f:
    f.readline()
    rows = list(csv.DictReader(f))
if rows:
    ls = [int(r['l']) for r in rows]
    rs = [float(r['r']) for r in rows]
    print(f'{len(ls)} episodes, mean length {sum(ls)/len(ls):.1f} steps '
          f'(max {max(ls)}), mean return {sum(rs)/len(rs):.2f} '
          f'(max {max(rs):.2f})')
else:
    print('NO EPISODES PARSED')
PY
)
    echo "  $ep"
    pass "PPO smoke produced episodes"
  else
    fail "PPO smoke produced no episode log"
  fi
else
  step "6  PPO smoke run SKIPPED (--no-rl)"
fi

# ── Nav2 unaffected (opt-in) ─────────────────────────────────────────────────
if [ "$DO_NAV" -eq 1 ]; then
  step "7  Nav2 goal end-to-end (arena unchanged bar the ramp + one cylinder)"
  # nav.launch.py is NOT self-contained: RUNNING.md runs the sim in one
  # terminal and Nav2 in another. Killing the sim here left the local costmap
  # with no odom frame ("Invalid frame ID \"odom\"") and no action server.
  # Restart the sim *fresh* instead: climb_check leaves the robot parked on
  # the ramp, while AMCL's initial pose is hardcoded to the spawn pose, so
  # Nav2 must start from a robot that is actually at spawn.
  stop_all   # full teardown, or stale TF publishers crash Nav2 (see stop_all)
  SIM_PID=$(launch_bg /tmp/coco_sim_nav.log gazebo_models \
                      full_world_robo.launch.py gui:=false \
                      "ramp_angle:=${RAMP_ANGLE}")
  if ! wait_for_topic /model/coco/odometry 120 \
     || ! wait_for_topic /diff_drive_controller/odom 120; then
    fail "sim did not come back up for the Nav2 stage"
  fi
  sleep 8   # let controllers settle and TF stabilise before Nav2 subscribes
  NAV_PID=$(launch_bg /tmp/coco_nav.log gazebo_models nav.launch.py)
  if wait_for_action /navigate_to_pose 180; then
    sleep 5   # let AMCL settle on its initial pose before sending a goal
    out=$(ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
      "{pose: {header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.0}, orientation: {w: 1.0}}}}" 2>&1)
    echo "$out" | grep -q "Goal finished with status: SUCCEEDED" \
      && pass "Nav2 goal SUCCEEDED" || { echo "$out" | tail -5; fail "Nav2 goal"; }
  else
    fail "Nav2 action server never came up (see /tmp/coco_nav.log)"
  fi
fi

# ── summary ──────────────────────────────────────────────────────────────────
step "summary"
if [ "${#FAILURES[@]}" -eq 0 ]; then
  echo "ALL STAGES PASSED"
  exit 0
fi
echo "${#FAILURES[@]} stage(s) failed:"
printf '  - %s\n' "${FAILURES[@]}"
exit 1
