#!/usr/bin/env bash
# C2-NAV.30 live matrix: ONE fresh run, and NOTHING CHANGED.
#
# This session added one subscription and one column group to
# nav_bench.py. It is not a parameter experiment and there is no
# candidate arm. The run uses the SAME frozen parameter file C2-NAV.25,
# C2-NAV.26 and C2-NAV.28 all used,
#
#   docs/data/c2nav25_slow_params.yaml
#   PolygonSlow.slowdown_ratio = 1.0
#
# and the SAME route, timeouts and topology C2-NAV.28's own focus run
# used. Nothing here reopens C2-NAV.26's rejection of that configuration
# as a FINAL configuration; it is used because it is the configuration
# whose behaviour the open question is about, and because every earlier
# wall-adjacent measurement was taken under it.
#
#   1  topology A  c2n30_focus_r1   open_space -> wall_adjacent
#
# WHY THOSE TWO LEGS AND NOT wall_adjacent ALONE. Legs chain -- a leg
# starts wherever the previous one stopped -- so open_space runs first to
# put the robot on the same approach every earlier tour used. It is also
# TOUR's own designated control case ("goal 1.15 m from anything"), which
# makes it the LESS-BIASED REGION the analysis compares against, inside
# the SAME simulator instance. That is stronger than comparing across
# runs, because it controls for the instance.
#
# It passes no --goal / --leg-timeout / --through-pose: all three of
# those name `enclosure_entry` in the committed specs, which this run
# does not contain, and nav_bench.py rejects a spec naming a scenario
# absent from the tour. None would have altered either leg that runs.
#
# One Gazebo at a time, a fresh simulator, ros_clean before -- all
# inherited from c2n14_run.sh, none of it re-implemented here.
#
# Naming: no string in this file contains ros_clean.sh's 'nav[2]_'
# pattern. "c2nav30_" is "nav3" followed by "0"; "c2nav25_slow_params"
# is "nav2" followed by "5".
HERE="$(cd "$(dirname "$0")" && pwd)"
WT="$(cd "$HERE/../.." && pwd)"
NB="$WT/.navbench"
PARAMS="$HERE/c2nav25_slow_params.yaml"
TAG="${1:-c2n30_focus_r1}"

echo "=== C2-NAV.30 matrix  params $PARAMS"
sha256sum "$PARAMS"
grep -A6 '^    PolygonSlow:' "$PARAMS" | grep slowdown_ratio
echo "=== AMCL parameters as shipped into this run (NOT tuned here) ==="
sed -n '/^amcl:/,/^amcl_map_client:/p' "$PARAMS" \
    | grep -E 'particles|update_min|resample|alpha|laser_model|sigma_hit|z_hit|z_rand'

if [ -f "$NB/results/${TAG}.done" ]; then
    echo "=== SKIP ${TAG} (already done) ==="
    exit 0
fi

echo "=== C2-NAV.30 ${TAG}  topology A  $(date -u +%H:%M:%S) UTC"
bash "$NB/c2n14_run.sh" "$PARAMS" "$TAG" \
    open_space,wall_adjacent 75 \
    > "$NB/logs/${TAG}.out" 2>&1
echo "=== ${TAG} finished rc=$?  $(date -u +%H:%M:%S) UTC"

# The blindness guard, echoed where a reader will see it. An empty
# particle-cloud column must never be reported as a measurement.
grep -E '^\[nav_bench\] particle cloud:' "$NB/logs/${TAG}.out"
grep -E '^\[nav_bench\]   (SUCCEEDED|TIMEOUT|FAILED|ABORTED)' \
    "$NB/logs/${TAG}.out" | tail -8
grep -E '^\[nav_bench\]   wrote .*particle_cloud' "$NB/logs/${TAG}.out"
grep -E 'TELEMETRY' "$NB/logs/${TAG}.out" | tail -1
echo "=== C2-NAV.30 matrix complete $(date -u +%H:%M:%S) UTC ==="
