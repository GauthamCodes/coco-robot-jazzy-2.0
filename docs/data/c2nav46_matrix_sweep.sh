#!/usr/bin/env bash
# C2-NAV.46: complete the M6 fetch colour matrix on the fixed arrival gate.
#
# Green already has three valid C2-NAV.45 runs and is NOT re-run here; this
# sweep collects three fresh executive-driven missions for each of red, blue
# and yellow, using C2-NAV.44's runner unchanged so every run in the matrix --
# C2-NAV.44's, C2-NAV.45's and these -- is directly comparable.
#
# Colours are INTERLEAVED by round rather than grouped, so machine-state drift
# over a two-hour sweep is spread across the three lanes instead of being
# confounded with colour, and so an early stop still leaves one run of every
# colour rather than three of one.
#
# Fresh simulator per run, torn down by process name between runs, headless,
# never --fast, depth fusion off, no Nav2 parameter / goal / planner /
# controller / safety change.
set +u
# Self-locating, like c2nav44_m6_run.sh: this file lives in <repo>/docs/data,
# so the repo root is two directories up. It used to hardcode the C2-NAV.43
# worktree, which stops existing the moment that branch merges (C2-NAV.47).
WT="${COCO_WT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)}"
ROOT="${1:-$HOME/coco_nav_runs/c2nav46_m6}"
mkdir -p "$ROOT"

for round in 1 2 3; do
    for colour in red blue yellow; do
        RUN="$ROOT/r${round}_${colour}"
        if [ -f "$RUN/final_state.txt" ]; then
            echo "=== r${round}_${colour} already has a result, skipping"
            continue
        fi
        echo "=== C2-NAV.46 round $round, colour $colour starting $(date -u +%H:%M:%SZ)"
        bash "$WT/docs/data/c2nav44_m6_run.sh" "$RUN" "$colour"
        echo "=== r${round}_${colour} runner exit $? at $(date -u +%H:%M:%SZ)"
        echo "--- final state:"; cat "$RUN/final_state.txt" 2>/dev/null
        sleep 20
    done
done
echo "=== C2-NAV.46 sweep done $(date -u +%H:%M:%SZ)"
