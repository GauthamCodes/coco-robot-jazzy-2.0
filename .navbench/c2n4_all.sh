#!/usr/bin/env bash
# C2-NAV.4: the live runs, in order, one fresh simulator each.
#
# Every candidate gets TWO fresh approaches, never a repeat:
#   *_bench    nav_bench.py --repeats 1 --only enclosure_entry -- the
#              behavioural leg, the same instrument C2-NAV.0/.1/.2 used
#   *_cap      c2nav3_capture.py -- the mechanism: the cost field, the
#              transformed plan and DWB's whole /evaluation at whatever
#              pose the robot ends at
#
# Not driven, deliberately: CSF 2.5, 10, 15 and 20. All four FAIL the
# pre-drive falsifier -- the replayed DWB decision still picks vx 0 --
# and C2-NAV.4's rule is that a candidate that cannot move the decision
# statically does not get simulator time.
HERE="$(cd "$(dirname "$0")" && pwd)"
D="$(cd "$HERE/../docs/data" && pwd)"
mkdir -p "$HERE/logs" "$HERE/results"

run () {   # run <script> <params> <tag>
    echo "############ $3  $(date -u +%H:%M:%S) ############"
    bash "$HERE/$1" "$2" "$3" > "$HERE/logs/$3.out" 2>&1
    echo "   exit $?  -> logs/$3.out"
    grep -E '^\[capture\]|^\[nav_bench\]|capture exit|bench exit' \
        "$HERE/logs/$3.out" | tail -8
}

run c2n4_bench.sh   "$D/c2nav3_baseline_params.yaml" base_bench
run c2n4_bench.sh   "$D/c2nav4_csf22_params.yaml"    csf22_bench
run c2n4_capture.sh "$D/c2nav4_csf22_params.yaml"    csf22_cap
run c2n4_bench.sh   "$D/c2nav4_csf30_params.yaml"    csf30_bench
run c2n4_capture.sh "$D/c2nav4_csf30_params.yaml"    csf30_cap
run c2n4_bench.sh   "$D/c2nav4_csf65_params.yaml"    csf65_bench
run c2n4_capture.sh "$D/c2nav4_csf65_params.yaml"    csf65_cap
echo "############ ALL DONE $(date -u +%H:%M:%S) ############"
