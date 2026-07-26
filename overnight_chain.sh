#!/usr/bin/env bash
# Copyright 2026 Gautham Anil
#
# Licensed under the Apache License, Version 2.0 (the "License").
#
# overnight_chain.sh — unattended follow-up programme.
#
# REDIRECTED: the --randomize curriculum this originally queued was dropped in
# favour of two experiments that are worth far more per minute of simulator time,
# and that the fetch-mission plan is blocked on:
#
#   A. MIN_LIN A/B. The deterministic policy stalls at 4.34 m. ramp_env.py:110
#      records that 0.10 m/s "times out at x=4.38" while "every speed from
#      0.17 m/s upward reaches the goal 2/2". MIN_LIN is 0.15 — between the
#      measured failing speed and the measured succeeding floor — and ACTION_TAU
#      plus the 2.0 m/s^2 accel limit means a commanded 0.15 delivers less on the
#      grade. One constant may turn the headline number from 0/10 into a result.
#
#   B. The M0 grasp gate. Whether the whole fetch mission is viable at all. The
#      joint tolerance (0.02 rad -> up to 6.47 mm of pinch error) was larger than
#      the entire descent clearance (5.88-7.76 mm); it is now 0.003. Re-run the
#      four points that measured 0/5, plus a +/-10 mm box, which is the number
#      the mission actually depends on.
#
# Only ONE Gazebo instance can run on this machine, so every step is serial.
# Each records PASS/FAIL and the chain continues; one failure must not cost the
# rest of the night. Progress: tail -f ~/coco_rl_runs/overnight_chain.log
set -o pipefail

REPO="/home/gautham/ros2_ws/src/coco-robot-ros2"
WS="/home/gautham/ros2_ws"
RUNS="$HOME/coco_rl_runs"
LOG="$RUNS/overnight_chain.log"
REPORT="$RUNS/REPORT.md"
BASE_RUN="${1:?usage: overnight_chain.sh <in_flight_run_dir>}"

cd "$REPO" || exit 1
exec > >(tee -a "$LOG") 2>&1
# shellcheck disable=SC1091
source "$REPO/setup_env.sh" >/dev/null 2>&1

say()  { echo; echo "[$(date -Is)] ===== $* ====="; }
note() { echo "[$(date -Is)] $*"; }
STATUS=()
ok()  { STATUS+=("PASS  $*"); note "PASS  $*"; }
bad() { STATUS+=("FAIL  $*"); note "FAIL  $*"; }

# Bracketed patterns throughout so this can never match its own command line and
# kill the chain — that happened repeatedly when done by hand.
kill_sim() {
  local g
  g=$(ps -eo pgid,comm | awk '$2=="ruby"{print $1; exit}')
  [ -n "$g" ] && { kill -TERM -"$g" 2>/dev/null; sleep 5; kill -9 -"$g" 2>/dev/null; }
  pkill -f 'full_world_rob[o]'        2>/dev/null
  pkill -f 'move_grou[p]'             2>/dev/null
  pkill -f 'parameter_brid[g]e'       2>/dev/null
  pkill -f 'robot_state_publishe[r]'  2>/dev/null
  sleep 3
}

start_sim() {  # $1 grade, $2 logfile
  kill_sim
  setsid ros2 launch gazebo_models full_world_robo.launch.py \
      gui:=false "ramp_angle:=$1" > "$2" 2>&1 &
  sleep 3
  local i
  for i in $(seq 1 75); do
    ros2 topic list 2>/dev/null | grep -qx /diff_drive_controller/odom && break
    sleep 2
  done
  sleep 10
  ros2 topic list 2>/dev/null | grep -qx /imu
}

set_min_lin() {  # $1 value
  python3 - "$1" <<'PY'
import re, sys
p = '/home/gautham/ros2_ws/src/coco-robot-ros2/coco_rl/coco_rl/ramp_env.py'
s = open(p).read()
s2 = re.sub(r'^MIN_LIN = [0-9.]+$', f'MIN_LIN = {sys.argv[1]}', s, count=1, flags=re.M)
assert s2 != s or f'MIN_LIN = {sys.argv[1]}' in s, 'MIN_LIN substitution failed'
open(p, 'w').write(s2)
PY
  ( cd "$WS" && colcon build --packages-select coco_rl ) >/dev/null 2>&1
  # shellcheck disable=SC1091
  source "$WS/install/setup.bash" >/dev/null 2>&1
  note "MIN_LIN set to $1 and rebuilt"
}

# ── 1. wait for the in-flight curriculum ─────────────────────────────────────
say "1. waiting for $(basename "$BASE_RUN")"
while true; do
  [ -f "$BASE_RUN/DONE" ] && { ok "base curriculum finished"; break; }
  if ! pgrep -f 'train_curriculu[m].sh' >/dev/null 2>&1; then
    bad "base curriculum runner vanished without writing DONE"; break
  fi
  sleep 60
done
kill_sim

FINAL_MODEL=$(ls -t "$BASE_RUN"/phase*deg_s*.zip 2>/dev/null \
              | grep -vE '_[0-9]+_steps\.zip$|_interrupted' | head -1)
note "final policy: ${FINAL_MODEL:-NONE FOUND}"

say "2. rebuild (picks up the M0 pick_place changes)"
( cd "$WS" && colcon build --packages-select coco_rl coco_moveit_config gazebo_models coco_config ) \
  >/dev/null 2>&1 && ok "colcon build" || bad "colcon build"
# shellcheck disable=SC1091
source "$WS/install/setup.bash" >/dev/null 2>&1

# ── 3. EXPERIMENT A: MIN_LIN A/B ─────────────────────────────────────────────
say "3. EXPERIMENT A — does MIN_LIN 0.15 -> 0.20 fix the deterministic stall?"
if [ -z "$FINAL_MODEL" ]; then
  bad "no final policy — MIN_LIN A/B skipped"
else
  for v in 0.15 0.20 0.25; do
    set_min_lin "$v"
    if start_sim 12 "/tmp/chain_minlin_${v}.log"; then
      timeout 2400 python3 -u -m coco_rl.evaluate "$FINAL_MODEL" --episodes 10 \
          > "$RUNS/eval_minlin_${v}.log" 2>&1
      rate=$(grep -m1 -o 'success rate: [0-9]*/[0-9]* ([0-9]*%)' "$RUNS/eval_minlin_${v}.log")
      far=$(grep -oE 'return +[-0-9.]+' "$RUNS/eval_minlin_${v}.log" | awk '{print $2}' \
            | sort -g | tail -1)
      note "  MIN_LIN=$v -> ${rate:-no summary}   best return ${far:-?}"
      [ -n "$rate" ] && ok "MIN_LIN=$v evaluated: $rate" || bad "MIN_LIN=$v eval"
    else
      bad "MIN_LIN=$v — sim did not come up"
    fi
  done
  # Leave the winner in place: prefer the smallest value that scored >0.
  BEST=0.15
  for v in 0.25 0.20 0.15; do
    g=$(grep -m1 -o 'success rate: \([0-9]*\)/' "$RUNS/eval_minlin_${v}.log" 2>/dev/null \
        | tr -dc '0-9')
    [ -n "$g" ] && [ "$g" -gt 0 ] && BEST=$v
  done
  set_min_lin "$BEST"
  note "left MIN_LIN at $BEST"
fi
kill_sim

# ── 4. EXPERIMENT B: the M0 grasp gate ───────────────────────────────────────
say "4. EXPERIMENT B — M0 grasp gate (joint tolerance 0.02 -> 0.003)"
if start_sim 18 /tmp/chain_grasp_sim.log; then
  setsid ros2 launch coco_moveit_config move_group.launch.py \
      > /tmp/chain_movegroup.log 2>&1 &
  sleep 30
  if ros2 node list 2>/dev/null | grep -q move_group; then
    ok "move_group up"
    : > "$RUNS/m0_grasp_gate.log"
    # (a) the four points that measured 0/5 in docs/RESULTS.md:106-112
    for pt in "0.150 0.130" "0.145 0.128" "0.152 0.135" "0.140 0.150"; do
      echo "=== --target $pt ===" >> "$RUNS/m0_grasp_gate.log"
      timeout 300 ros2 run coco_moveit_config pick_place.py --target $pt \
        >> "$RUNS/m0_grasp_gate.log" 2>&1
      echo "  exit=$?" >> "$RUNS/m0_grasp_gate.log"
    done
    # (b) the number the mission actually depends on: a +/-10 mm box
    python3 - "$RUNS/m0_box_points.txt" <<'PY'
import random
random.seed(0)
with open(__import__('sys').argv[1], 'w') as f:
    for _ in range(10):
        f.write(f'{0.152 + random.uniform(-0.010, 0.010):.4f} '
                f'{0.128 + random.uniform(-0.010, 0.010):.4f}\n')
PY
    while read -r x z; do
      echo "=== box --target $x $z ===" >> "$RUNS/m0_grasp_gate.log"
      timeout 300 ros2 run coco_moveit_config pick_place.py --target "$x" "$z" \
        >> "$RUNS/m0_grasp_gate.log" 2>&1
      echo "  exit=$?" >> "$RUNS/m0_grasp_gate.log"
    done < "$RUNS/m0_box_points.txt"
    comp=$(grep -c 'Pick-and-place sequence complete' "$RUNS/m0_grasp_gate.log")
    ok "grasp gate ran; $comp/14 sequences completed"
  else
    bad "move_group never came up"
  fi
else
  bad "grasp gate — sim did not come up"
fi
kill_sim

# ── 5. report ────────────────────────────────────────────────────────────────
say "5. writing $REPORT"
{
  echo "# Overnight report — $(date -Is)"
  echo
  echo "## Chain steps"
  printf -- '- %s\n' "${STATUS[@]}"
  echo
  echo "## Base curriculum: $(basename "$BASE_RUN")"
  [ -f "$BASE_RUN/SUMMARY.md" ] && sed -n '1,50p' "$BASE_RUN/SUMMARY.md"
  echo
  echo "## Experiment A — MIN_LIN A/B (deterministic eval of the final policy)"
  echo
  echo "| MIN_LIN | success rate |"
  echo "|---|---|"
  for v in 0.15 0.20 0.25; do
    r=$(grep -m1 -o 'success rate: [0-9]*/[0-9]* ([0-9]*%)' \
        "$RUNS/eval_minlin_${v}.log" 2>/dev/null)
    echo "| $v | ${r:-not run} |"
  done
  echo
  echo "Hypothesis: ramp_env.py:110 records 0.10 m/s timing out at x=4.38 and"
  echo "0.17 m/s reaching the goal 2/2; the policy stalls at 4.34 m."
  echo
  echo "## Experiment B — M0 grasp gate"
  echo
  echo "Joint tolerance 0.02 -> 0.003 rad (6.47 mm -> 0.97 mm of pinch error"
  echo "against a 5.88-7.76 mm clearance budget). Baseline was 0/5."
  echo
  if [ -f "$RUNS/m0_grasp_gate.log" ]; then
    echo "- sequences completed: $(grep -c 'Pick-and-place sequence complete' "$RUNS/m0_grasp_gate.log")/14"
    echo "- aborted: $(grep -c 'aborting demo' "$RUNS/m0_grasp_gate.log")"
    echo
    echo '```'
    grep -E '^=== |Pick-and-place sequence complete|aborting demo' \
      "$RUNS/m0_grasp_gate.log" | head -60
    echo '```'
  else
    echo "- not run"
  fi
  echo
  echo "## Gate decision"
  echo "- >=3/4 on the RESULTS points AND >=8/10 on the box -> proceed as planned"
  echo "- box passes but RESULTS points do not -> proceed with identical objects"
  echo "- both fail -> switch to the detachable_joint magnet grasp"
} > "$REPORT"

say "CHAIN COMPLETE"
printf -- '  %s\n' "${STATUS[@]}"
echo "report: $REPORT"
