#!/usr/bin/env bash
# C2-NAV.5 stage 3: the cost field at both conditions, fresh simulators.
#
# The baseline probe is not decoration. C2-NAV.3 measured the field at the
# baseline stall and C2-NAV.4 replayed it, but both did so on runs from
# earlier sessions; reading it again in THIS session, on the same machine
# and the same day as the candidate probes, is what makes the two
# columns comparable without an argument about drift.
HERE="$(cd "$(dirname "$0")" && pwd)"
D="$(cd "$HERE/../docs/data" && pwd)"
mkdir -p "$HERE/logs" "$HERE/results"

run () {   # run <params> <tag>
    echo "######## $2  $(date -u +%H:%M:%S) ########"
    bash "$HERE/c2n5_costprobe.sh" "$1" "$2" > "$HERE/logs/$2.out" 2>&1
    echo "   exit $?  -> logs/$2.out"
    grep -E '^\[costprobe\]' "$HERE/logs/$2.out" | tail -12
}

run "$D/c2nav3_baseline_params.yaml" c2n5_cost_base_r1
run "$D/c2nav4_csf65_params.yaml"    c2n5_cost_csf65_r2
echo "######## COST STAGE DONE $(date -u +%H:%M:%S) ########"
