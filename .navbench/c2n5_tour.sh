#!/usr/bin/env bash
# C2-NAV.5 stage 2: the other six tour legs, both conditions.
#
# The seven legs of nav_bench.py's TOUR are a CHAIN -- leg N starts where
# leg N-1 stopped -- so the only way to measure wall_adjacent and
# wall_parallel as C2-NAV.0 and C2-NAV.1 measured them is to drive the
# whole tour in order. Running `--only wall_adjacent` from the spawn would
# be a different leg with a different start pose, and its numbers would
# not belong in the same table.
#
# 75 s per leg, both conditions: that is the budget C2-NAV.0 and C2-NAV.1
# used for the tour, and their committed tables are the comparison.
# Interleaved for the same reason stage 1 is.
#
# One fresh simulator per TOUR, not per leg. Every leg inside a tour is a
# fresh approach to ITS goal, but the tour is one chain, so three tours
# give three independent samples of each leg -- not twenty-one.
HERE="$(cd "$(dirname "$0")" && pwd)"
D="$(cd "$HERE/../docs/data" && pwd)"
mkdir -p "$HERE/logs" "$HERE/results"
N="${1:-3}"

BASE="$D/c2nav3_baseline_params.yaml"
CAND="$D/c2nav4_csf65_params.yaml"

for i in $(seq 1 "$N"); do
    for cond in base csf65; do
        [ "$cond" = base ] && P="$BASE" || P="$CAND"
        TAG="c2n5_tour_${cond}_r${i}"
        echo "######## $TAG  $(date -u +%H:%M:%S) ########"
        bash "$HERE/c2n5_run.sh" "$P" "$TAG" ALL 75 \
            > "$HERE/logs/$TAG.out" 2>&1
        echo "   exit $?  -> logs/$TAG.out"
        grep -E '^\[nav_bench\]|cost_scaling|bench exit' \
            "$HERE/logs/$TAG.out" | tail -10
    done
done
echo "######## TOUR STAGE DONE $(date -u +%H:%M:%S) ########"
