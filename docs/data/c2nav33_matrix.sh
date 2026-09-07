#!/usr/bin/env bash
# C2-NAV.33 live matrix: ONE fresh run, ONE behavioural leaf moved.
#
#   amcl.resample_interval: 1 -> 2      and nothing else, either way.
#
# The route, the timeouts, the topology and the runner are C2-NAV.30's
# EXACTLY -- open_space then wall_adjacent, 75 s default, topology A,
# c2n14_run.sh -- so the only thing that differs between this run and
# C2-NAV.30's is the one leaf. The parameter file is
# c2nav33_ri2_params.yaml, which `c2nav33_weights.py paramdiff` proves
# differs from c2nav25_slow_params.yaml in that leaf and no other, over
# all 323 leaves.
#
#   1  topology A  c2n33_focus_r1   open_space -> wall_adjacent
#
# WHY THOSE TWO LEGS. C2-NAV.30's reasoning, unchanged: legs chain, so
# open_space runs first to put the robot on the same approach every
# earlier tour used, and it is TOUR's own designated control case, which
# makes it the less-biased region the analysis compares against INSIDE
# THE SAME simulator instance.
#
# THIS IS NOT A TUNING ARM. resample_interval is moved to open a
# diagnostic window on information that resampling destroys before
# publication. Nothing here proposes interval 2 as a configuration, and
# one run is an observation, not a rate.
#
# The live readback runs ALONGSIDE the bench rather than after it,
# because /amcl is torn down when the run ends and a parameter that can
# no longer be read is not a parameter that was verified. It starts no
# node and writes nothing.
#
# One Gazebo at a time, a fresh simulator, ros_clean before -- all
# inherited from c2n14_run.sh, none of it re-implemented here.
#
# Naming: no string in this file contains ros_clean.sh's 'nav[2]_'
# pattern. "c2nav33_" is "nav3" followed by "3"; "c2nav25_slow_params"
# is "nav2" followed by "5" and is only mentioned in comments.
HERE="$(cd "$(dirname "$0")" && pwd)"
WT="$(cd "$HERE/../.." && pwd)"
NB="$WT/.navbench"
PARAMS="$HERE/c2nav33_ri2_params.yaml"
TAG="${1:-c2n33_focus_r1}"

echo "=== C2-NAV.33 matrix  params $PARAMS"
sha256sum "$PARAMS"
echo "=== the one leaf, as it sits in the file that will be loaded ==="
grep -n 'resample_interval' "$PARAMS"
echo "=== AMCL parameters as shipped into this run (NOT tuned here) ==="
sed -n '/^amcl:/,/^amcl_map_client:/p' "$PARAMS" \
    | grep -E 'particles|update_min|resample|alpha|laser_model|sigma_hit|z_hit|z_rand'
echo "=== the one-leaf proof, re-run here so the run carries it ==="
python3 "$HERE/c2nav33_weights.py" paramdiff || {
    echo "REFUSING: paramdiff did not report ONE LEAF"; exit 1; }

if [ -f "$NB/results/${TAG}.done" ]; then
    echo "=== SKIP ${TAG} (already done) ==="
    exit 0
fi

echo "=== C2-NAV.33 ${TAG}  topology A  $(date -u +%H:%M:%S) UTC"
bash "$NB/c2n14_run.sh" "$PARAMS" "$TAG" \
    open_space,wall_adjacent 75 \
    > "$NB/logs/${TAG}.out" 2>&1 &
BENCH=$!

# The readback races the bench deliberately: it waits for /amcl to go
# active, reads, and exits. It is not a monitor and does not persist.
bash "$HERE/c2nav33_liveparam.sh" "$TAG" \
    > "$NB/logs/${TAG}_amcl_live.out" 2>&1 &
LIVE=$!

wait "$BENCH"; rc=$?
echo "=== ${TAG} finished rc=$rc  $(date -u +%H:%M:%S) UTC"
wait "$LIVE" 2>/dev/null

echo "=== THE ONE VARIABLE, READ OFF THE RUNNING /amcl ==="
sed -n '1,6p' "$NB/results/${TAG}_amcl_live.txt" 2>/dev/null \
    || echo "NO READBACK FILE -- the parameter was NOT verified live"

# The blindness guard, echoed where a reader will see it. An empty
# particle-cloud column must never be reported as a measurement.
grep -E '^\[nav_bench\] particle cloud:' "$NB/logs/${TAG}.out"
grep -E '^\[nav_bench\]   (SUCCEEDED|TIMEOUT|FAILED|ABORTED)' \
    "$NB/logs/${TAG}.out" | tail -8
grep -E '^\[nav_bench\]   wrote .*particle_cloud' "$NB/logs/${TAG}.out"
grep -E 'TELEMETRY' "$NB/logs/${TAG}.out" | tail -1
echo "=== C2-NAV.33 matrix complete $(date -u +%H:%M:%S) UTC ==="
