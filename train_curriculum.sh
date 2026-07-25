#!/usr/bin/env bash
# Copyright 2026 Gautham Anil
#
# Licensed under the Apache License, Version 2.0 (the "License").
#
# train_curriculum.sh — run the 12° → 18° → 24° PPO curriculum unattended.
#
# Each phase relaunches the simulator at its own grade, trains, then
# evaluates the resulting policy on that same grade. Phase N+1 starts from
# phase N's weights (train_ppo.py --resume), which is the whole point of a
# curriculum: the easy grade teaches "drive forward and stay upright", the
# steeper ones only have to refine it.
#
#   ./train_curriculum.sh                       # 3 phases x 60k steps (~7 h)
#   ./train_curriculum.sh --steps 30000         # shorter (~3.5 h)
#   ./train_curriculum.sh --grades 18           # single grade, no curriculum
#   ./train_curriculum.sh --randomize           # vary spawn offset/yaw
#   ./train_curriculum.sh --eval-episodes 0     # skip the evaluations
#
# Leave it running with:
#   nohup ./train_curriculum.sh > /dev/null 2>&1 &
# (all output is tee'd into the run directory regardless, so nothing is lost)
#
# Progress:  tail -f ~/coco_rl_runs/<run>/curriculum.log
# Status:    cat  ~/coco_rl_runs/<run>/STATUS
# Finished:  the file  ~/coco_rl_runs/<run>/DONE  exists, and SUMMARY.md is
#            written with the per-phase success rates.
#
# NOTE: deliberately no `set -u` — ROS 2's setup.bash references unbound
# variables (AMENT_TRACE_SETUP_FILES) and nounset kills the script on the
# first source. No `set -e` either: a failed phase is recorded and handled,
# not allowed to silently abort a night of compute.
set -o pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$(cd "$REPO/../.." 2>/dev/null && pwd || echo "$REPO")"
[ -d "$WS/src" ] || WS="$REPO"

# ── options ──────────────────────────────────────────────────────────────────
STEPS=60000
GRADES=(12 18 24)
EVAL_EPISODES=10
RANDOMIZE=""
DO_INHIBIT=1
# Worst-case throughput assumed when arming the per-phase timeout. Measured
# throughput on this machine is ~7.6 env-steps/s, so 3.0 leaves 2.5x margin;
# the timeout exists only to stop a wedged simulator eating the whole night.
MIN_RATE=3.0
SEED=0

# Kept because the parse loop below shifts "$@" empty, and this script
# re-execs itself under systemd-inhibit further down. Passing "$@" there
# would hand the new process nothing and silently run the defaults — a
# --steps 600 smoke test quietly became a 180k-step, seven-hour run.
ORIG_ARGS=("$@")

while [ $# -gt 0 ]; do
  case "$1" in
    --steps)          STEPS="$2"; shift 2 ;;
    --grades)         read -r -a GRADES <<< "$2"; shift 2 ;;
    --eval-episodes)  EVAL_EPISODES="$2"; shift 2 ;;
    --seed)           SEED="$2"; shift 2 ;;
    --min-rate)       MIN_RATE="$2"; shift 2 ;;
    --randomize)      RANDOMIZE="--randomize"; shift ;;
    --no-inhibit)     DO_INHIBIT=0; shift ;;
    -h|--help)        sed -n '6,32p' "$0"; exit 0 ;;
    *) echo "unknown arg: $1" >&2; exit 2 ;;
  esac
done

# ── keep the machine awake ───────────────────────────────────────────────────
# This box suspends after 5 minutes idle and on lid close. A suspend freezes
# the simulator but not the wall clock, so on resume every _spin_sim() in
# ramp_env.py blows its wall-clock deadline at once and the run degenerates
# into a stream of 'sim_stalled' truncations — hours of compute spent
# training on nothing. Re-exec the whole script under an inhibitor lock
# rather than trusting the lid to stay open.
RUN_DIR="${COCO_RUN_DIR:-$HOME/coco_rl_runs/curriculum_$(date +%Y%m%d_%H%M%S)}"
export COCO_RUN_DIR="$RUN_DIR"
if [ "$DO_INHIBIT" -eq 1 ] && [ -z "$COCO_INHIBITED" ] \
   && command -v systemd-inhibit >/dev/null 2>&1; then
  export COCO_INHIBITED=1
  exec systemd-inhibit \
      --what=idle:sleep:handle-lid-switch \
      --who="coco curriculum" --why="PPO ramp curriculum training" \
      --mode=block "$0" "${ORIG_ARGS[@]}"
fi

mkdir -p "$RUN_DIR" || { echo "cannot create $RUN_DIR" >&2; exit 1; }
exec > >(tee -a "$RUN_DIR/curriculum.log") 2>&1

SIM_PID=""
declare -a PHASE_REPORT=()

# ── process hygiene ──────────────────────────────────────────────────────────
# A `ros2 launch` tree outlives `pkill gz sim`: the bridge, robot_state_publisher
# and controller spawners survive and keep publishing TF stamped with the OLD
# sim clock. The next phase's sim restarts its clock at 0, tf2 sees a jump back
# in time, and everything downstream misbehaves. Launches therefore run under
# setsid (own process group) and the group is killed, then stragglers are swept
# by name.
stop_all() {
  [ -n "$SIM_PID" ] && kill -TERM -- "-$SIM_PID" 2>/dev/null
  SIM_PID=""
  pkill -f 'coco_rl.train_ppo'          2>/dev/null
  pkill -f 'coco_rl.evaluate'           2>/dev/null
  pkill -f 'full_world_robo.launch.py'  2>/dev/null
  pkill -f 'parameter_bridge'           2>/dev/null
  pkill -f 'robot_state_publisher'      2>/dev/null
  pkill -f 'controller_manager'         2>/dev/null
  pkill -f 'gz sim'                     2>/dev/null
  sleep 3
  pkill -9 -f 'gz sim'                  2>/dev/null
  sleep 2
}
trap stop_all EXIT
# An EXIT trap does NOT run when bash is killed by a signal, so `kill <pid>`
# on this script left the simulator and a running train_ppo orphaned —
# they then fought the next run for the gz transport topics. Handle the
# signals explicitly. SIGINT is deliberately NOT trapped: Ctrl-C reaches the
# whole foreground process group, and train_ppo catches it to save
# <out>_interrupted.zip. Killing it from here would race that save.
trap 'echo "signal received — tearing down"; stop_all; exit 143' TERM HUP

launch_bg() {  # $1 logfile, rest: ros2 launch args
  local log="$1"; shift
  setsid ros2 launch "$@" >"$log" 2>&1 &
  echo $!
}

wait_for_topic() {  # $1 topic, $2 timeout_s
  local t=$(( $(date +%s) + ${2:-120} ))
  while [ "$(date +%s)" -lt "$t" ]; do
    ros2 topic list 2>/dev/null | grep -qx "$1" && return 0
    sleep 2
  done
  return 1
}

status() { echo "$*" > "$RUN_DIR/STATUS"; }
step()   { echo; echo "======== $* ========"; }
hms()    { printf '%dh%02dm' $(( $1 / 3600 )) $(( ($1 % 3600) / 60 )); }

# A phase can die with the model unsaved (a simulator crash raises out of
# env.reset()). Checkpoints land every 25k steps and Ctrl-C writes
# <out>_interrupted.zip, so there is usually still something worth carrying
# into the next phase. Newest wins.
newest_artifact() {  # $1 out prefix (absolute, no extension)
  ls -t "$1".zip "$1"_interrupted.zip "$1"_*_steps.zip 2>/dev/null | head -1
}

# ── preflight ────────────────────────────────────────────────────────────────
step "preflight"
# shellcheck disable=SC1091
source "$REPO/setup_env.sh" || { echo "could not source setup_env.sh"; exit 1; }
for tool in ros2 gz python3; do
  command -v "$tool" >/dev/null \
    || { echo "FATAL: '$tool' not on PATH after sourcing setup_env.sh"; exit 1; }
done
# Fail here, in five seconds, rather than two hours into phase 1.
python3 - <<'PY' || { echo "FATAL: training deps not importable"; exit 1; }
import stable_baselines3, gymnasium, torch
from coco_rl.train_ppo import main            # noqa: F401
from coco_rl.ramp_env import GOAL_SUMMIT
print(f'sb3 {stable_baselines3.__version__}, torch {torch.__version__}, '
      f'goal x={GOAL_SUMMIT:.2f} m from spawn')
PY

# Validate every requested grade now. The launch file raises on a missing
# wedge mesh, but that failure only shows up as a 150s topic-wait timeout
# inside the phase — so `--grades "12 20 24"` would otherwise burn two hours
# reaching phase 2 before telling you 20° was never generated.
MESH_DIR="$(ros2 pkg prefix gazebo_models 2>/dev/null)/share/gazebo_models/meshes"
for deg in "${GRADES[@]}"; do
  if [ ! -f "$MESH_DIR/ramp_wedge_${deg}.stl" ]; then
    echo "FATAL: no wedge mesh for ${deg}° (looked for"
    echo "       $MESH_DIR/ramp_wedge_${deg}.stl)"
    echo "       installed grades:" \
         "$(ls "$MESH_DIR"/ramp_wedge_*.stl 2>/dev/null \
            | sed 's/.*ramp_wedge_//; s/\.stl//' | tr '\n' ' ')"
    echo "       generate it with:"
    echo "         python3 gazebo_models/scripts/gen_ramp.py --angle-deg ${deg} \\"
    echo "             --run 2.5 --width 2.0 \\"
    echo "             --out gazebo_models/meshes/ramp_wedge_${deg}.stl"
    echo "       then rebuild (colcon build --packages-select gazebo_models)."
    echo "       Note the tip-over terminator fires at 0.6 rad (~34°), so"
    echo "       grades much past 30° are not climbable by design."
    exit 1
  fi
done

TOTAL_STEPS=$(( STEPS * ${#GRADES[@]} ))
ETA=$(python3 -c "print(int($TOTAL_STEPS / 7.6))")   # 7.6 steps/s measured
PHASE_TIMEOUT=$(python3 -c "print(max(1800, int($STEPS / $MIN_RATE)))")
echo "run dir      : $RUN_DIR"
echo "grades       : ${GRADES[*]} deg"
echo "steps/phase  : $STEPS  (total $TOTAL_STEPS)"
echo "eval/phase   : $EVAL_EPISODES episodes"
echo "randomize    : ${RANDOMIZE:-off}"
echo "phase timeout: $(hms "$PHASE_TIMEOUT") each (assumes >= $MIN_RATE steps/s)"
echo "rough ETA    : $(hms "$ETA") of training + evaluation on top"
echo "sleep inhibit: ${COCO_INHIBITED:+active (idle, sleep, lid close blocked)}"
df -h "$HOME" | tail -1
git -C "$REPO" log --oneline -1 2>/dev/null

RUN_START=$(date +%s)
PREV_MODEL=""
FAILED=0

# ── the curriculum ───────────────────────────────────────────────────────────
for i in "${!GRADES[@]}"; do
  deg="${GRADES[$i]}"
  n=$(( i + 1 ))
  OUT="$RUN_DIR/phase${n}_${deg}deg"
  P_START=$(date +%s)

  step "phase $n/${#GRADES[@]} — ${deg}° wedge, $STEPS steps"
  status "phase $n/${#GRADES[@]} (${deg}°): launching sim — started $(date -Is)"

  stop_all   # never train on top of a previous phase's nodes
  SIM_PID=$(launch_bg "$RUN_DIR/sim_phase${n}.log" \
                      gazebo_models full_world_robo.launch.py \
                      gui:=false "ramp_angle:=${deg}")
  # Gate on BOTH odometry sources: /model/coco/odometry (the gz plugin) shows
  # up well before ros2_control finishes activating, and the env needs the
  # controller's odom too.
  if wait_for_topic /model/coco/odometry 150 \
     && wait_for_topic /diff_drive_controller/odom 150; then
    sleep 8   # let the controllers settle before the first teleport
    echo "sim up at ${deg}°"
  else
    echo "FAIL: sim never came up for phase $n (see $RUN_DIR/sim_phase${n}.log)"
    PHASE_REPORT+=("phase $n (${deg}°): SIM FAILED TO START")
    FAILED=1
    continue
  fi

  status "phase $n/${#GRADES[@]} (${deg}°): training $STEPS steps — started $(date -Is)"
  RESUME=""
  [ -n "$PREV_MODEL" ] && RESUME="--resume $PREV_MODEL"
  echo "+ train_ppo --steps $STEPS --ramp-angle $deg $RESUME $RANDOMIZE"
  # --steps is *additional* steps when resuming: train_ppo passes
  # reset_num_timesteps=False, so SB3 adds them to the inherited counter.
  # -u matters here: piped into tee, python block-buffers stdout, so a
  # `tail -f` of the log sits silent for minutes at a time and an unattended
  # run looks hung when it is fine.
  timeout "$PHASE_TIMEOUT" python3 -u -m coco_rl.train_ppo \
      --fast --steps "$STEPS" --seed "$SEED" --ramp-angle "$deg" \
      --out "$OUT" $RESUME $RANDOMIZE 2>&1 | tee "$RUN_DIR/train_phase${n}.log"
  trc=${PIPESTATUS[0]}

  MODEL=$(newest_artifact "$OUT")
  if [ "$trc" -eq 0 ] && [ -f "$OUT.zip" ]; then
    verdict="ok"
    MODEL="$OUT.zip"
  elif [ -n "$MODEL" ]; then
    # Partial, but a partial policy still seeds the next grade better than
    # random weights. Recorded as PARTIAL so the summary cannot overstate it.
    verdict="PARTIAL (exit $trc, carried $(basename "$MODEL"))"
    echo "WARNING: phase $n did not finish cleanly (exit $trc); " \
         "continuing from $MODEL"
    FAILED=1
  else
    echo "FAIL: phase $n produced no model at all (exit $trc)"
    PHASE_REPORT+=("phase $n (${deg}°): NO MODEL (exit $trc)")
    FAILED=1
    continue
  fi
  PREV_MODEL="$MODEL"

  # ── evaluate on this grade ─────────────────────────────────────────────────
  rate="not evaluated"
  if [ "$EVAL_EPISODES" -gt 0 ]; then
    status "phase $n/${#GRADES[@]} (${deg}°): evaluating $EVAL_EPISODES episodes"
    echo "--- evaluating $(basename "$MODEL") on ${deg}° ---"
    timeout 3600 python3 -u -m coco_rl.evaluate "$MODEL" \
        --episodes "$EVAL_EPISODES" --fast $RANDOMIZE \
        2>&1 | tee "$RUN_DIR/eval_phase${n}.log"
    rate=$(grep -o 'success rate: [0-9]*/[0-9]* ([0-9]*%)' \
             "$RUN_DIR/eval_phase${n}.log" | tail -1)
    [ -z "$rate" ] && rate="evaluation produced no summary"
  fi

  P_ELAPSED=$(( $(date +%s) - P_START ))
  PHASE_REPORT+=("phase $n (${deg}°): $verdict — $rate — $(hms "$P_ELAPSED")")
  echo "phase $n done in $(hms "$P_ELAPSED"): $verdict — $rate"
done

stop_all

# ── summary ──────────────────────────────────────────────────────────────────
step "summary"
ELAPSED=$(( $(date +%s) - RUN_START ))

{
  echo "# Curriculum run — $(date -Is)"
  echo
  echo "- grades: $(printf '%s° ' "${GRADES[@]}")— $STEPS steps per phase, seed $SEED,"
  echo "  randomize ${RANDOMIZE:-off}"
  echo "- wall clock: $(hms "$ELAPSED")"
  echo "- commit: $(git -C "$REPO" rev-parse --short HEAD 2>/dev/null)"
  echo
  echo "## Phases"
  printf -- '- %s\n' "${PHASE_REPORT[@]}"
  echo
  echo "## Episodes per phase (from the Monitor CSVs)"
  echo
} > "$RUN_DIR/SUMMARY.md"

python3 - "$RUN_DIR" <<'PY' | tee -a "$RUN_DIR/SUMMARY.md"
import csv, glob, os, sys
run = sys.argv[1]
print('| phase | episodes | mean len | mean return | best return |')
print('|---|---|---|---|---|')
for path in sorted(glob.glob(os.path.join(run, 'phase*.monitor.csv'))):
    with open(path) as f:
        f.readline()          # Monitor opens with a '#{"t_start": ...}' line
        rows = list(csv.DictReader(f))
    name = os.path.basename(path).replace('.monitor.csv', '')
    if not rows:
        print(f'| {name} | 0 | — | — | — |')
        continue
    ls = [int(r['l']) for r in rows]
    rs = [float(r['r']) for r in rows]
    print(f'| {name} | {len(rs)} | {sum(ls)/len(ls):.1f} | '
          f'{sum(rs)/len(rs):.2f} | {max(rs):.2f} |')
PY

# Learning curve across the whole curriculum, phases side by side.
csvs=$(ls "$RUN_DIR"/phase*.monitor.csv 2>/dev/null)
if [ -n "$csvs" ]; then
  # shellcheck disable=SC2086
  python3 -m coco_rl.plot_curve $csvs -o "$RUN_DIR/curriculum_curve.png" \
    && echo "curve -> $RUN_DIR/curriculum_curve.png"
fi

echo
printf -- '  %s\n' "${PHASE_REPORT[@]}"
echo
echo "total wall clock: $(hms "$ELAPSED")"
echo "artifacts       : $RUN_DIR"
echo "final model     : ${PREV_MODEL:-none produced}"
echo "summary         : $RUN_DIR/SUMMARY.md"

date -Is > "$RUN_DIR/DONE"
if [ "$FAILED" -eq 0 ]; then
  status "DONE — all ${#GRADES[@]} phases completed, $(hms "$ELAPSED")"
  echo "CURRICULUM COMPLETE"
  exit 0
fi
status "DONE with problems — see SUMMARY.md ($(hms "$ELAPSED"))"
echo "CURRICULUM FINISHED WITH PROBLEMS — see $RUN_DIR/SUMMARY.md"
exit 1
