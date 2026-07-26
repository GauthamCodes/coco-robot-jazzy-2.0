#!/usr/bin/env bash
# Copyright 2026 Gautham Anil
#
# Licensed under the Apache License, Version 2.0 (the "License").
#
# overnight_chain.sh — run the whole follow-up programme unattended.
#
# Written so that a night of wall clock costs ONE agent wake-up instead of five.
# Everything here is local compute, which is free; agent invocations are not.
# So this waits for the in-flight curriculum, then does every follow-up step by
# itself and writes a single consolidated REPORT.md at the end.
#
# Steps, in order (each needs the simulator, so they are strictly serial —
# only one Gazebo instance can run on this machine):
#
#   1. wait for the in-flight curriculum to finish (or die)
#   2. swap in train_curriculum.sh.next (adds --init-model) and rebuild
#   3. re-run the two evaluations that were skipped/mismatched, with the
#      start distance each stage actually trained on
#   4. build and verify the traverse world (traverse:=true)
#   5. launch the --randomize curriculum, seeded from the finished policy
#   6. write REPORT.md
#
# Every step records PASS/FAIL and the chain continues, so one failure does not
# cost the whole night. Progress: tail -f ~/coco_rl_runs/overnight_chain.log
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
ok()   { STATUS+=("PASS  $*"); note "PASS  $*"; }
bad()  { STATUS+=("FAIL  $*"); note "FAIL  $*"; }

# Only ever one sim. Bracket the patterns so this never matches its own
# command line and kills the chain (that happened repeatedly by hand).
kill_sim() {
  local g
  g=$(ps -eo pgid,comm | awk '$2=="ruby"{print $1; exit}')
  [ -n "$g" ] && { kill -TERM -"$g" 2>/dev/null; sleep 5; kill -9 -"$g" 2>/dev/null; }
  pkill -f 'full_world_rob[o]'     2>/dev/null
  pkill -f 'parameter_brid[g]e'    2>/dev/null
  pkill -f 'robot_state_publishe[r]' 2>/dev/null
  sleep 3
}

start_sim() {  # $1 grade, $2 logfile, $3 extra launch args
  kill_sim
  # shellcheck disable=SC2086
  setsid ros2 launch gazebo_models full_world_robo.launch.py \
      gui:=false "ramp_angle:=$1" $3 > "$2" 2>&1 &
  sleep 3
  local i
  for i in $(seq 1 75); do
    ros2 topic list 2>/dev/null | grep -qx /diff_drive_controller/odom && break
    sleep 2
  done
  sleep 10
  ros2 topic list 2>/dev/null | grep -qx /imu
}

# ── 1. wait for the in-flight run ────────────────────────────────────────────
say "1. waiting for $BASE_RUN"
while true; do
  [ -f "$BASE_RUN/DONE" ] && { ok "base curriculum finished"; break; }
  if ! pgrep -f 'train_curriculu[m].sh' >/dev/null 2>&1; then
    bad "base curriculum runner vanished without writing DONE"
    break
  fi
  sleep 60
done
kill_sim

# The final policy of the base run: newest stage .zip that is not a checkpoint.
FINAL_MODEL=$(ls -t "$BASE_RUN"/phase*deg_s*.zip 2>/dev/null \
              | grep -vE '_[0-9]+_steps\.zip$|_interrupted' | head -1)
note "final policy: ${FINAL_MODEL:-NONE FOUND}"

# ── 2. swap in the patched runner and rebuild ────────────────────────────────
say "2. swap in --init-model support and rebuild"
# mv is atomic and makes a new inode, so this is safe even if anything still
# holds the old file open.
if [ -f "$REPO/train_curriculum.sh.next" ]; then
  mv "$REPO/train_curriculum.sh.next" "$REPO/train_curriculum.sh" \
    && chmod +x "$REPO/train_curriculum.sh" && ok "runner patched" || bad "runner patch"
fi
( cd "$WS" && colcon build --packages-select coco_rl gazebo_models coco_config ) \
  >/dev/null 2>&1 && ok "colcon build" || bad "colcon build"
# shellcheck disable=SC1091
source "$WS/install/setup.bash" >/dev/null 2>&1

# ── 3. the evaluations that were skipped or mismatched ───────────────────────
# Stage 1 was skipped entirely (it completed, then a filename bug made the
# runner think it failed; on resume it was correctly detected as done, and
# skipping also skips the eval). Stages 1 and 2 were additionally scored on the
# FULL task while they trained from +2.5 m and +1.0 m, which understates them.
say "3. re-run stage 1 and 2 evals at their own start distances"
for spec in "1:2.5:s2p5" "2:1.0:s1p0"; do
  n=${spec%%:*}; rest=${spec#*:}; start=${rest%%:*}; tag=${rest#*:}
  model=$(ls "$BASE_RUN"/phase${n}_12deg_${tag}.zip 2>/dev/null | head -1)
  if [ -z "$model" ]; then bad "stage $n model missing"; continue; fi
  if start_sim 12 /tmp/chain_eval${n}.log ""; then
    timeout 1800 python3 -u -m coco_rl.evaluate "$model" --episodes 10 \
        --start-progress "$start" \
        > "$BASE_RUN/eval_phase${n}_matched.log" 2>&1 \
      && ok "stage $n eval at start +${start} m" || bad "stage $n eval"
    grep -m1 'success rate' "$BASE_RUN/eval_phase${n}_matched.log" | sed 's/^/    /'
  else
    bad "stage $n eval — sim did not come up"
  fi
done

# ── 4. traverse world ────────────────────────────────────────────────────────
say "4. verify the traverse world (traverse:=true)"
if start_sim 18 /tmp/chain_traverse.log "traverse:=true"; then
  ros2 topic list >/dev/null 2>&1
  # climb_check only drives forward, so on the traverse it should still crest.
  timeout 300 python3 "$REPO/gazebo_models/scripts/climb_check.py" --duration 90 \
      > /tmp/chain_climb.log 2>&1
  tail -2 /tmp/chain_climb.log | sed 's/^/    /'
  grep -q '^PASS' /tmp/chain_climb.log \
    && ok "traverse world: robot still crests" || bad "traverse world climb"
  # And prove the far side is drivable rather than a cliff.
  timeout 600 python3 -u - > /tmp/chain_descend.log 2>&1 <<'PY'
import math, numpy as np
from coco_rl.ramp_env import CocoRampEnv
env = CocoRampEnv()
try:
    obs, _ = env.reset()
    peak = 0.0; maxx = -9.0
    for k in range(1, 601):
        obs, r, term, trunc, info = env.step(np.array([1.0, 0.0], dtype=np.float32))
        peak = max(peak, abs(obs[7])); maxx = max(maxx, obs[0])
        if term or trunc:
            break
    print(f'outcome={info.get("outcome")} steps={k} max_progress={maxx:.2f} '
          f'peak_pitch={math.degrees(peak):.1f}')
finally:
    env.close()
PY
  cat /tmp/chain_descend.log | sed 's/^/    /'
  ok "traverse descent probe recorded"
else
  bad "traverse world — sim did not come up"
fi
kill_sim

# ── 5. the randomize curriculum ──────────────────────────────────────────────
# The base run had randomize off, so spawn was identical every episode and a
# constant action provably solves it. --randomize varies spawn lateral offset
# +/-0.5 m and yaw +/-0.4 rad, so the policy must actually use y and yaw to
# steer onto a 2 m wide ramp. Distance stages are dropped: the seeded policy
# already covers the full distance, only start-pose variation is new.
say "5. launch the --randomize curriculum, seeded from the finished policy"
if [ -n "$FINAL_MODEL" ]; then
  setsid nohup "$REPO/train_curriculum.sh" \
      --stages "12:0.0 18:0.0 24:0.0" --steps 40000 --randomize \
      --init-model "$FINAL_MODEL" --no-autoresume \
      > /dev/null 2>&1 < /dev/null &
  sleep 90
  NEW_RUN=$(ls -dt "$RUNS"/curriculum_* | head -1)
  if [ "$NEW_RUN" != "$BASE_RUN" ] && [ -f "$NEW_RUN/curriculum.log" ]; then
    ok "randomize run started: $NEW_RUN"
  else
    bad "randomize run did not start"
  fi
else
  bad "no final policy to seed from — randomize run skipped"
fi

# ── 6. report ────────────────────────────────────────────────────────────────
say "6. writing $REPORT"
{
  echo "# Overnight report — $(date -Is)"
  echo
  echo "## Chain steps"
  printf -- '- %s\n' "${STATUS[@]}"
  echo
  echo "## Base curriculum: $(basename "$BASE_RUN")"
  echo
  [ -f "$BASE_RUN/SUMMARY.md" ] && sed -n '1,60p' "$BASE_RUN/SUMMARY.md"
  echo
  echo "## Evaluations at matched start distances"
  for n in 1 2; do
    f="$BASE_RUN/eval_phase${n}_matched.log"
    [ -f "$f" ] && echo "- stage $n: $(grep -m1 'success rate' "$f")"
  done
  echo
  echo "## Traverse world"
  echo '```'
  tail -2 /tmp/chain_climb.log 2>/dev/null
  cat /tmp/chain_descend.log 2>/dev/null
  echo '```'
  echo
  echo "## Randomize run"
  NEW_RUN=$(ls -dt "$RUNS"/curriculum_* | head -1)
  if [ "$NEW_RUN" != "$BASE_RUN" ]; then
    echo "- started: $NEW_RUN"
    echo "- watch: ./watch_training.py"
    [ -f "$NEW_RUN/STATUS" ] && echo "- status: $(cat "$NEW_RUN/STATUS")"
  else
    echo "- not started"
  fi
} > "$REPORT"

say "CHAIN COMPLETE"
printf -- '  %s\n' "${STATUS[@]}"
echo "report: $REPORT"
