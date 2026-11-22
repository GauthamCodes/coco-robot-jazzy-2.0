#!/usr/bin/env bash
# C2-NAV.4, second stage: the SAME leg on a 150 s budget.
#
# Why this exists. The 75 s legs above all report TIMEOUT, but the CSF 30
# capture -- which has a 150 s budget because it stops on a stall rather
# than on a clock -- shows the robot pausing at ~0.93 m from t=19 s to
# t=47 s and then breaking through, reaching the 0.25 m goal tolerance at
# t=56.3 s. Two runs at the same setting, two different outcomes, and the
# difference is the budget.
#
# That makes "TIMEOUT at 75 s" an ambiguous measurement, and it makes the
# CSF 30 capture an UNFAIR comparison: the capture aborts the instant it
# detects a stall, so the baseline and CSF 22 captures stopped at t=30 s
# and t=20 s and never had the 150 s that CSF 30 got. The only way to
# attribute a traverse to the knob is to give every configuration the
# same clock.
#
# One fresh simulator per run, --repeats 1, nothing else changed.
HERE="$(cd "$(dirname "$0")" && pwd)"
D="$(cd "$HERE/../docs/data" && pwd)"
mkdir -p "$HERE/logs" "$HERE/results"

run () {   # run <params> <tag>
    echo "############ $2  $(date -u +%H:%M:%S) ############"
    bash "$HERE/c2n4_bench_long.sh" "$1" "$2" > "$HERE/logs/$2.out" 2>&1
    echo "   exit $?  -> logs/$2.out"
    grep -E '^\[nav_bench\]   ' "$HERE/logs/$2.out" | tail -3
}

run "$D/c2nav3_baseline_params.yaml" base_long
run "$D/c2nav4_csf22_params.yaml"    csf22_long
run "$D/c2nav4_csf30_params.yaml"    csf30_long
run "$D/c2nav4_csf65_params.yaml"    csf65_long
echo "############ LONG DONE $(date -u +%H:%M:%S) ############"
