#!/usr/bin/env bash
# C2-NAV.49: re-measure the full M6 fetch colour matrix on robot_radius 0.25.
#
# C2-NAV.46's 11-of-12 is NOT a control for the corrected configuration: it was
# measured on local_costmap.robot_radius 0.20, the value C2-NAV.48 identified as
# the cause of the r2_blue return-leg PolygonStop deadlock. This sweep replaces
# it with twelve fresh missions on 0.25.
#
# Unlike c2nav46_matrix_sweep.sh this runs ALL FOUR colours. C2-NAV.46 carried
# green over from C2-NAV.45 rather than re-running it; on a changed costmap
# parameter no lane may be carried over, so green is measured here too.
#
# Colours are INTERLEAVED by round rather than grouped, for C2-NAV.46's reason:
# machine-state drift over a multi-hour sweep is spread across the four lanes
# instead of being confounded with colour, and an early stop still leaves one
# run of every colour rather than three of one.
#
# Fresh simulator per run, torn down by process name between runs, headless,
# never --fast, depth fusion off, no Nav2 parameter / goal / planner /
# controller / safety / PolygonStop / arrival-gate change. The runner is
# C2-NAV.44's, unchanged, so every run since C2-NAV.44 stays comparable.
#
# The overlay: <ws>/install cannot launch Gazebo (half-installed turtlebot3
# prefixes make ros_gz_sim's GazeboRosPaths.get_paths() throw; C2-NAV.48 §8,
# re-measured at 2 unresolvable entries here), so COCO_WS must point at an
# isolated prefix built from this tree:
#
#   COCO_WT=<this worktree> COCO_WS=$HOME/c2nav49_overlay \
#     bash docs/data/c2nav49_matrix_sweep.sh ~/coco_nav_runs/c2nav49_matrix
set +u
WT="${COCO_WT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)}"
ROOT="${1:-$HOME/coco_nav_runs/c2nav49_matrix}"
mkdir -p "$ROOT"

for round in 1 2 3; do
    for colour in red green blue yellow; do
        RUN="$ROOT/r${round}_${colour}"
        if [ -f "$RUN/final_state.txt" ]; then
            echo "=== r${round}_${colour} already has a result, skipping"
            continue
        fi
        echo "=== C2-NAV.49 round $round, colour $colour starting $(date -u +%H:%M:%SZ)"
        bash "$WT/docs/data/c2nav44_m6_run.sh" "$RUN" "$colour"
        echo "=== r${round}_${colour} runner exit $? at $(date -u +%H:%M:%SZ)"
        echo "--- final state:"; cat "$RUN/final_state.txt" 2>/dev/null
        sleep 20
    done
done
echo "=== C2-NAV.49 sweep done $(date -u +%H:%M:%SZ)"
