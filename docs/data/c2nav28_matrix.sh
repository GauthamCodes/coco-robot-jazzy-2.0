#!/usr/bin/env bash
# C2-NAV.28 live matrix: FOUR fresh trials and NOTHING CHANGED.
#
# This session added three columns to nav_bench.py's per-leg trace and
# nothing else. It is not a parameter experiment and there is no
# candidate arm: every run below uses the SAME frozen parameter file
# C2-NAV.26 most recently tested,
#
#   docs/data/c2nav25_slow_params.yaml   sha256 4c15893e...
#   PolygonSlow.slowdown_ratio = 1.0
#
# and the same route, timeouts and through-poses C2-NAV.21/.25/.26 ran.
# Nothing here re-opens C2-NAV.26's rejection of that configuration as a
# FINAL configuration; it is used because it is the configuration whose
# behaviour the open question is about.
#
#   1  topology A  c2n28_a_r1       full seven-leg tour
#   2  topology A  c2n28_a_r2       full seven-leg tour
#   3  topology B  c2n28_b_r1       full seven-leg tour, via cmd_vel_arbiter
#   4  topology A  c2n28_focus_r1   open_space -> wall_adjacent only
#
# Run 4 is the "reproduce the problematic wall-adjacent scenario" run the
# brief allows. It is NOT wall_adjacent in isolation: legs chain, and a
# leg starts wherever the previous one stopped, so open_space runs first
# to put the robot on the same approach every earlier tour used. It
# passes no --goal / --leg-timeout / --through-pose because all three of
# those name `enclosure_entry`, which this run does not contain, and
# nav_bench.py rejects a spec naming a scenario absent from the tour.
# None of the three would have altered either leg that does run.
#
# One Gazebo at a time, a fresh simulator per run, ros_clean between --
# all inherited from the runners, none of it re-implemented here.
#
# Naming: no string in this file contains ros_clean.sh's 'nav[2]_'
# pattern. "c2nav28_" is "nav2" followed by "8", not "nav2_".
HERE="$(cd "$(dirname "$0")" && pwd)"
WT="$(cd "$HERE/../.." && pwd)"
NB="$WT/.navbench"
PARAMS="$HERE/c2nav25_slow_params.yaml"

echo "=== C2-NAV.28 matrix  params $PARAMS"
sha256sum "$PARAMS"
grep -A6 '^    PolygonSlow:' "$PARAMS" | grep slowdown_ratio

bash "$NB/c2n21_matrix.sh" \
    "c2n28_a_r1:${PARAMS}:A" \
    "c2n28_a_r2:${PARAMS}:A" \
    "c2n28_b_r1:${PARAMS}:B"

if [ -f "$NB/results/c2n28_focus_r1.done" ]; then
    echo "=== SKIP c2n28_focus_r1 (already done) ==="
else
    echo "=== C2-NAV.28 matrix: c2n28_focus_r1  topology A  $(date -u +%H:%M:%S) UTC"
    bash "$NB/c2n14_run.sh" "$PARAMS" c2n28_focus_r1 \
        open_space,wall_adjacent 75 \
        > "$NB/logs/c2n28_focus_r1.out" 2>&1
    echo "=== c2n28_focus_r1 finished rc=$?  $(date -u +%H:%M:%S) UTC"
    grep -E '^\[nav_bench\]   (SUCCEEDED|TIMEOUT|FAILED|ABORTED)' \
        "$NB/logs/c2n28_focus_r1.out" | tail -8
    grep -E 'TELEMETRY' "$NB/logs/c2n28_focus_r1.out" | tail -1
fi
echo "=== C2-NAV.28 matrix complete $(date -u +%H:%M:%S) UTC ==="
