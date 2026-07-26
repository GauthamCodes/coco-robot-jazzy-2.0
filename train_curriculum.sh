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
# Stages are "<grade_deg>:<start_progress_m>". The default walks the start line
# back toward spawn *before* raising the grade, because the measured blocker was
# never the grade: a constant action summits in ~150 steps, but PPO rarely
# survived long enough to bank a single goal. Starting 2.5 m along shortens the
# episode that has to be survived, so goals appear early and the dense progress
# reward has something to sharpen. Distance first, then steepness.
STAGES=(12:2.5 12:1.0 12:0.0 18:0.0 24:0.0)
EVAL_EPISODES=10
RANDOMIZE=""
DO_INHIBIT=1
# Worst-case throughput assumed when arming the per-phase timeout. Measured
# throughput on this machine is ~7.6 env-steps/s, so 3.0 leaves 2.5x margin;
# the timeout exists only to stop a wedged simulator eating the whole night.
MIN_RATE=3.0
SEED=0
RETRIES=2       # extra attempts per phase before giving up on it
RESUME_RUN=""   # continue an existing run directory instead of starting one
DO_AUTORESUME=1 # install a login hook that resumes after a reboot

# Kept because the parse loop below shifts "$@" empty, and this script
# re-execs itself under systemd-inhibit further down. Passing "$@" there
# would hand the new process nothing and silently run the defaults — a
# --steps 600 smoke test quietly became a 180k-step, seven-hour run.
ORIG_ARGS=("$@")

while [ $# -gt 0 ]; do
  case "$1" in
    --steps)          STEPS="$2"; shift 2 ;;
    --grades)         read -r -a GRADES <<< "$2"; shift 2 ;;
    --stages)         read -r -a STAGES <<< "$2"; shift 2 ;;
    --eval-episodes)  EVAL_EPISODES="$2"; shift 2 ;;
    --seed)           SEED="$2"; shift 2 ;;
    --min-rate)       MIN_RATE="$2"; shift 2 ;;
    --retries)        RETRIES="$2"; shift 2 ;;
    --randomize)      RANDOMIZE="--randomize"; shift ;;
    --no-inhibit)     DO_INHIBIT=0; shift ;;
    --resume-run)     RESUME_RUN="$2"; shift 2 ;;
    --resume-latest)  RESUME_RUN="latest"; shift ;;
    --no-autoresume)  DO_AUTORESUME=0; shift ;;
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
RUNS_ROOT="$HOME/coco_rl_runs"
if [ -n "$RESUME_RUN" ] && [ -z "$COCO_RUN_DIR" ]; then
  if [ "$RESUME_RUN" = latest ]; then
    RESUME_RUN="$(ls -d "$RUNS_ROOT"/curriculum_* 2>/dev/null | tail -1)"
  fi
  [ -d "$RESUME_RUN" ] || { echo "no such run dir: $RESUME_RUN" >&2; exit 1; }
  if [ -f "$RESUME_RUN/DONE" ]; then
    echo "$RESUME_RUN already finished (DONE exists) — nothing to resume."
    exit 0
  fi
  COCO_RUN_DIR="$RESUME_RUN"
  echo "resuming $COCO_RUN_DIR"
fi
RUN_DIR="${COCO_RUN_DIR:-$RUNS_ROOT/curriculum_$(date +%Y%m%d_%H%M%S)}"
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

SIM_PID=""; TRAIN_PID=""
declare -a PHASE_REPORT=()

# ── process hygiene ──────────────────────────────────────────────────────────
# A `ros2 launch` tree outlives `pkill gz sim`: the bridge, robot_state_publisher
# and controller spawners survive and keep publishing TF stamped with the OLD
# sim clock. The next phase's sim restarts its clock at 0, tf2 sees a jump back
# in time, and everything downstream misbehaves. Launches therefore run under
# setsid (own process group) and the group is killed, then stragglers are swept
# by name.
stop_all() {
  # SIGINT first so train_ppo can save <out>_interrupted.zip before it dies;
  # it catches ExternalShutdownException and writes the checkpoint itself.
  if [ -n "$TRAIN_PID" ]; then
    kill -INT "$TRAIN_PID" 2>/dev/null
    for _ in 1 2 3 4 5 6 7 8 9 10; do
      kill -0 "$TRAIN_PID" 2>/dev/null || break
      sleep 1
    done
    kill -TERM "$TRAIN_PID" 2>/dev/null
    TRAIN_PID=""
  fi
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

# Steps already trained for a phase, summed over every Monitor CSV it has —
# the live one plus any .partN kept from an earlier, interrupted attempt.
# This is what makes a resumed run ask for the *remaining* steps instead of a
# full phase, so a reboot at 55k does not restart at 0.
steps_done() {  # $1 out prefix
  python3 - "$1" <<'PY'
import csv, glob, sys
total = 0
for path in sorted(glob.glob(sys.argv[1] + '.monitor.csv*')):
    try:
        with open(path) as f:
            f.readline()          # '#{"t_start": ...}' comment line
            for row in csv.DictReader(f):
                try:
                    total += int(row['l'])
                except (TypeError, ValueError, KeyError):
                    pass          # torn final line of an interrupted run
    except OSError:
        pass
print(total)
PY
}

# SB3's Monitor opens its CSV with mode 'wt', so resuming a phase would
# truncate the episode history that steps_done() and the learning curve both
# depend on. Park it as .partN first.
rotate_csv() {  # $1 out prefix
  local live="$1.monitor.csv" k=1
  [ -s "$live" ] || return 0
  while [ -e "$1.monitor.csv.part$k" ]; do k=$(( k + 1 )); done
  mv "$live" "$1.monitor.csv.part$k"
  echo "kept previous episodes as $(basename "$1.monitor.csv.part$k")"
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
for spec in "${STAGES[@]}"; do
  deg="${spec%%:*}"
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

# Power. The systemd-inhibit lock blocks *idle* and lid-close suspend, but it
# cannot stop a flat battery: GNOME's sleep-inactive-battery-type is `suspend`
# and the critical-battery action shuts down regardless of any inhibitor. A
# multi-hour run at 100% CPU on battery does not finish, so say so up front.
AC_ONLINE=0
for ps in /sys/class/power_supply/*; do
  [ "$(cat "$ps/type" 2>/dev/null)" = "Mains" ] \
    && [ "$(cat "$ps/online" 2>/dev/null)" = "1" ] && AC_ONLINE=1
done
if [ "$AC_ONLINE" -eq 1 ]; then
  echo "power        : on AC (inhibitor blocks idle/lid suspend)"
else
  cap="$(cat /sys/class/power_supply/BAT*/capacity 2>/dev/null | head -1)"
  echo "power        : *** ON BATTERY (${cap:-?}%) ***"
  echo "               A ${TOTAL_STEPS}-step run is hours at 100% CPU. No"
  echo "               inhibitor survives a critical battery, so plug in."
  echo "               Continuing in 20s — Ctrl-C to abort and plug in first."
  sleep 20
fi

TOTAL_STEPS=$(( STEPS * ${#STAGES[@]} ))
ETA=$(python3 -c "print(int($TOTAL_STEPS / 7.6))")   # 7.6 steps/s measured
PHASE_TIMEOUT=$(python3 -c "print(max(1800, int($STEPS / $MIN_RATE)))")
echo "run dir      : $RUN_DIR"
echo "stages       : ${STAGES[*]}  (grade_deg:start_progress_m)"
echo "steps/phase  : $STEPS  (total $TOTAL_STEPS)"
echo "eval/phase   : $EVAL_EPISODES episodes"
echo "randomize    : ${RANDOMIZE:-off}"
echo "phase timeout: $(hms "$PHASE_TIMEOUT") each (assumes >= $MIN_RATE steps/s)"
echo "retries      : $RETRIES extra attempt(s) per phase"
echo "rough ETA    : $(hms "$ETA") of training + evaluation on top"
echo "sleep inhibit: ${COCO_INHIBITED:+active (idle, sleep, lid close blocked)}"
df -h "$HOME" | tail -1
git -C "$REPO" log --oneline -1 2>/dev/null

RUN_START=$(date +%s)
PREV_MODEL=""
FAILED=0

# ── survive a reboot ─────────────────────────────────────────────────────────
# The inhibitor lock stops idle/lid suspend, and suspend itself is now
# harmless (ramp_env's deadlines use time.monotonic(), which does not tick
# while suspended, so the run simply pauses and continues). What neither
# handles is the machine actually going down: power loss, a panic, or the user
# shutting it down. So opt this run into a login hook that resumes it from the
# newest checkpoint. resume_curriculum.sh removes the hook once the run is
# DONE, and refuses to act if a curriculum is already running.
AUTORESUME_HOOK="$HOME/.config/autostart/coco-curriculum-resume.desktop"
if [ "$DO_AUTORESUME" -eq 1 ]; then
  : > "$RUN_DIR/AUTORESUME"
  mkdir -p "$(dirname "$AUTORESUME_HOOK")"
  cat > "$AUTORESUME_HOOK" <<EOF
[Desktop Entry]
Type=Application
Name=Coco RL curriculum — resume interrupted run
Comment=Restarts an interrupted PPO curriculum from its newest checkpoint
Exec=$REPO/resume_curriculum.sh
X-GNOME-Autostart-enabled=true
NoDisplay=true
EOF
  echo "auto-resume  : installed login hook -> $AUTORESUME_HOOK"
else
  rm -f "$RUN_DIR/AUTORESUME"
  echo "auto-resume  : disabled (--no-autoresume)"
fi

# ── the curriculum ───────────────────────────────────────────────────────────
for i in "${!STAGES[@]}"; do
  spec="${STAGES[$i]}"
  deg="${spec%%:*}"
  start="${spec#*:}"
  [ "$start" = "$spec" ] && start=0.0     # bare "18" means start at spawn
  n=$(( i + 1 ))
  # Start distance is part of the identity: two stages can share a grade, and
  # sharing an --out prefix would make the second overwrite the first's model
  # and monitor CSV.
  OUT="$RUN_DIR/phase${n}_${deg}deg_s${start}"
  P_START=$(date +%s)

  step "phase $n/${#STAGES[@]} — ${deg}° wedge, start +${start} m, $STEPS steps"

  # Already finished (this is a resumed run): carry its model and move on.
  if [ -f "$OUT.zip" ]; then
    echo "phase $n already complete ($(basename "$OUT.zip")) — skipping"
    PREV_MODEL="$OUT.zip"
    rate="not evaluated"
    if [ -s "$RUN_DIR/eval_phase${n}.log" ]; then
      rate="$(grep -o 'success rate: [0-9]*/[0-9]* ([0-9]*%)' \
                "$RUN_DIR/eval_phase${n}.log" | tail -1)"
    fi
    PHASE_REPORT+=("phase $n (${deg}°): ok (from earlier run) — ${rate:-n/a}")
    continue
  fi

  # Remaining steps for this phase. On a fresh run this is the full amount;
  # on a resume it is whatever a reboot or crash left unfinished.
  DONE_STEPS="$(steps_done "$OUT")"
  PHASE_STEPS=$(( STEPS - DONE_STEPS ))
  if [ "$PHASE_STEPS" -lt 512 ]; then
    # Under one PPO rollout left; do one so the phase ends with a saved model.
    PHASE_STEPS=512
  fi
  if [ "$DONE_STEPS" -gt 0 ]; then
    echo "phase $n resuming: $DONE_STEPS steps already done, $PHASE_STEPS to go"
  fi

  # Retry the whole phase, sim included. The first curriculum attempt lost all
  # three phases to a transient gz-transport miss on the per-episode set_pose
  # (fixed properly in ramp_env.gz_service, which now retries) — and because
  # each died before the 25k checkpoint, there was nothing to carry forward.
  # A crash that early is exactly the case a phase-level retry recovers, so
  # keep it as the backstop for whatever the next transient turns out to be.
  MODEL=""; verdict=""
  for try in $(seq 1 $(( RETRIES + 1 ))); do
    if [ "$try" -gt 1 ]; then
      echo
      echo "--- phase $n: retry $(( try - 1 ))/$RETRIES ---"
    fi
    status "phase $n/${#STAGES[@]} (${deg}°): launching sim (try $try) — $(date -Is)"

    stop_all   # never train on top of a previous attempt's nodes
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
      echo "FAIL: sim never came up (see $RUN_DIR/sim_phase${n}.log)"
      verdict="SIM FAILED TO START"
      continue
    fi

    # Prefer this phase's own newest artifact: on a retry after a partial run
    # that is further along than the previous phase's model.
    R_MODEL="$(newest_artifact "$OUT")"
    [ -z "$R_MODEL" ] && R_MODEL="$PREV_MODEL"
    RESUME=""
    [ -n "$R_MODEL" ] && RESUME="--resume $R_MODEL"

    rotate_csv "$OUT"   # never let SB3's Monitor truncate earlier episodes
    status "phase $n/${#STAGES[@]} (${deg}°): training $PHASE_STEPS steps (try $try) — $(date -Is)"
    echo "+ train_ppo --steps $PHASE_STEPS --ramp-angle $deg --start-progress $start $RESUME $RANDOMIZE"
    # --steps is *additional* steps when resuming: train_ppo passes
    # reset_num_timesteps=False, so SB3 adds them to the inherited counter.
    # -u matters here: piped into tee, python block-buffers stdout, so a
    # `tail -f` of the log sits silent for minutes at a time and an unattended
    # run looks hung when it is fine.
    # Backgrounded + `wait` rather than run in the foreground on purpose.
    # Bash defers a trap until the current foreground command returns, so with
    # the trainer in the foreground a SIGTERM to this script did nothing for
    # hours — the TERM/HUP trap only ran once training finished, which is
    # exactly when it is no longer needed. `wait` IS interruptible, so the
    # trap fires immediately and stop_all can tear the phase down.
    timeout "$PHASE_TIMEOUT" python3 -u -m coco_rl.train_ppo \
        --fast --steps "$PHASE_STEPS" --seed "$SEED" --ramp-angle "$deg" \
        --start-progress "$start" \
        --out "$OUT" $RESUME $RANDOMIZE \
        > >(tee "$RUN_DIR/train_phase${n}_try${try}.log") 2>&1 &
    TRAIN_PID=$!
    wait "$TRAIN_PID"; trc=$?
    TRAIN_PID=""

    if [ "$trc" -eq 0 ] && [ -f "$OUT.zip" ]; then
      MODEL="$OUT.zip"; verdict="ok"
      break
    fi
    echo "phase $n attempt $try did not finish cleanly (exit $trc)"
    verdict="FAILED (exit $trc)"
    tail -3 "$RUN_DIR/train_phase${n}_try${try}.log" | sed 's/^/    /'
  done

  if [ -z "$MODEL" ]; then
    # Nothing clean; fall back to any checkpoint the attempts left behind.
    MODEL="$(newest_artifact "$OUT")"
    if [ -n "$MODEL" ]; then
      # A partial policy still seeds the next grade better than random
      # weights. Recorded as PARTIAL so the summary cannot overstate it.
      verdict="PARTIAL ($verdict, carried $(basename "$MODEL"))"
      echo "WARNING: phase $n unfinished; continuing from $MODEL"
    else
      echo "FAIL: phase $n produced no model after $(( RETRIES + 1 )) attempts"
      PHASE_REPORT+=("phase $n (${deg}°): NO MODEL — $verdict")
      FAILED=1
      continue
    fi
    FAILED=1
  fi
  PREV_MODEL="$MODEL"

  # ── evaluate on this grade ─────────────────────────────────────────────────
  rate="not evaluated"
  if [ "$EVAL_EPISODES" -gt 0 ]; then
    status "phase $n/${#STAGES[@]} (${deg}°): evaluating $EVAL_EPISODES episodes"
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
  echo "- stages: ${STAGES[*]} (deg:start_m) — $STEPS steps per stage, seed $SEED,"
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
import csv, glob, os, re, sys
run = sys.argv[1]


def rows_for(prefix):
    """All episodes for a phase, including .partN files kept across a resume."""
    out = []
    parts = sorted(glob.glob(prefix + '.monitor.csv.part*'),
                   key=lambda p: int(p.rsplit('part', 1)[-1] or 0))
    for path in parts + [prefix + '.monitor.csv']:
        try:
            with open(path) as f:
                f.readline()   # Monitor opens with a '#{"t_start": ...}' line
                out.extend(list(csv.DictReader(f)))
        except OSError:
            pass
    return out


prefixes = sorted({re.sub(r'\.monitor\.csv.*$', '', p)
                   for p in glob.glob(os.path.join(run, 'phase*.monitor.csv*'))})
print('| phase | episodes | mean len | mean return | best return |')
print('|---|---|---|---|---|')
for prefix in prefixes:
    rows = rows_for(prefix)
    name = os.path.basename(prefix)
    ls, rs = [], []
    for r in rows:
        try:
            ls.append(int(r['l']))
            rs.append(float(r['r']))
        except (TypeError, ValueError, KeyError):
            pass
    if not rs:
        print(f'| {name} | 0 | — | — | — |')
        continue
    print(f'| {name} | {len(rs)} | {sum(ls)/len(ls):.1f} | '
          f'{sum(rs)/len(rs):.2f} | {max(rs):.2f} |')
PY

# Learning curve across the whole curriculum, phases side by side.
# plot_curve concatenates in argument order and accumulates the step axis, so
# the list has to be strictly chronological: per phase, .partN before the live
# CSV. Globbing parts and live files separately would put every phase's tail
# after every phase's head and produce a meaningless curve.
csvs=""
for gi in "${!STAGES[@]}"; do
  gspec="${STAGES[$gi]}"; gdeg="${gspec%%:*}"; gst="${gspec#*:}"
  [ "$gst" = "$gspec" ] && gst=0.0
  pfx="$RUN_DIR/phase$(( gi + 1 ))_${gdeg}deg_s${gst}"
  for f in $(ls -v "$pfx".monitor.csv.part* 2>/dev/null) "$pfx.monitor.csv"; do
    [ -s "$f" ] && csvs="$csvs $f"
  done
done
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
# The run is over, so the login hook has no further job. Leaving it installed
# would mean every future login inspects this directory forever.
rm -f "$AUTORESUME_HOOK" "$RUN_DIR/AUTORESUME"
if [ "$FAILED" -eq 0 ]; then
  status "DONE — all ${#STAGES[@]} phases completed, $(hms "$ELAPSED")"
  echo "CURRICULUM COMPLETE"
  exit 0
fi
status "DONE with problems — see SUMMARY.md ($(hms "$ELAPSED"))"
echo "CURRICULUM FINISHED WITH PROBLEMS — see $RUN_DIR/SUMMARY.md"
exit 1
