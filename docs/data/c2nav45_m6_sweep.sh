#!/usr/bin/env bash
# C2-NAV.45 live validation: three fresh executive-driven GREEN M6 missions
# on the fixed command path, using C2-NAV.44's runner unchanged so the two
# sprints are directly comparable. Green is the lane that aborted 3/3 under
# the old gate, so it is the lane that tests the fix.
#
# Fresh simulator per run, torn down by process name between runs, headless,
# never --fast, depth fusion off, no Nav2 parameter/goal/safety change.
set +u
WT="/home/gautham/ros2_ws(personal)/src/coco-robot-ros2/.claude/worktrees/c2nav43-integration"
ROOT="${1:-/home/gautham/coco_nav_runs/c2nav45_m6}"
mkdir -p "$ROOT"

for i in 1 2 3; do
    RUN="$ROOT/r0${i}_green"
    if [ -f "$RUN/final_state.txt" ]; then
        echo "=== r0$i already has a result, skipping"
        continue
    fi
    echo "=== C2-NAV.45 run r0$i (green) starting $(date -u +%H:%M:%SZ)"
    bash "$WT/docs/data/c2nav44_m6_run.sh" "$RUN" green
    echo "=== r0$i runner exit $? at $(date -u +%H:%M:%SZ)"
    echo "--- final state:"; cat "$RUN/final_state.txt" 2>/dev/null
    sleep 20
done
echo "=== C2-NAV.45 sweep done $(date -u +%H:%M:%SZ)"
