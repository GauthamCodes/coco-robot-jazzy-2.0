#!/usr/bin/env bash
# C2-NAV.21 live matrix driver. Runs the candidate arms SEQUENTIALLY --
# one Gazebo at a time, a fresh simulator per run -- and INTERLEAVED, so
# baseline and candidate sample the same stretch of machine state rather
# than one arm getting the first hour and the other the second.
#
#   c2n21_matrix.sh <spec> [<spec> ...]
#     spec = <tag>:<abs params path>[:A|B]
#
# The route is the C2-NAV.11/.14 enclosure approach, unchanged and
# byte-identical to C2-NAV.18/.19: corridor_gate -> (-3.00, 0.625) ->
# (-3.40, 1.35) -> (-3.575, 2.95) as ONE NavigateThroughPoses request,
# inside a complete seven-leg tour. The BAD case reproduces roughly one
# tour in four, so a single run decides nothing and the driver exists to
# make several cheap.
#
# Naming: no string here contains "nav2_".
HERE="$(cd "$(dirname "$0")" && pwd)"
mkdir -p "$HERE/logs" "$HERE/results"

GOAL="enclosure_entry:-3.575,2.95"
LEGTO="enclosure_entry:200"
THROUGH="enclosure_entry:-3.00,0.625;enclosure_entry:-3.40,1.35"

for spec in "$@"; do
    TAG="${spec%%:*}"
    rest="${spec#*:}"
    case "$rest" in
        *:A|*:B) PARAMS="${rest%:*}"; TOPO="${rest##*:}";;
        *)       PARAMS="$rest";      TOPO="A";;
    esac
    if [ -f "$HERE/results/${TAG}.done" ]; then
        echo "=== SKIP $TAG (already done) ==="
        continue
    fi
    case "$TOPO" in
        A) RUNNER="$HERE/c2n14_run.sh";;
        B) RUNNER="$HERE/c2n21_run_b.sh";;
        *) echo "bad topology '$TOPO' in spec '$spec'"; exit 2;;
    esac
    echo "=== C2-NAV.21 matrix: $TAG  topology $TOPO  $(date -u +%H:%M:%S) UTC"
    echo "    params $PARAMS"
    bash "$RUNNER" "$PARAMS" "$TAG" ALL 75 "$GOAL" "$LEGTO" "$THROUGH" \
        > "$HERE/logs/${TAG}.out" 2>&1
    rc=$?
    echo "=== $TAG finished rc=$rc  $(date -u +%H:%M:%S) UTC"
    grep -E '^\[nav_bench\]   (SUCCEEDED|TIMEOUT|FAILED|ABORTED)' \
        "$HERE/logs/${TAG}.out" | tail -8
    grep -E 'TELEMETRY' "$HERE/logs/${TAG}.out" | tail -1
done
echo "=== C2-NAV.21 matrix complete $(date -u +%H:%M:%S) UTC ==="
