#!/usr/bin/env bash
# C2-NAV.15: ONE fresh seven-leg tour, byte-identical config to C2-NAV.14
# (same params file, same goal, same heading-correcting pose + waypoint),
# reusing c2n14_run.sh UNCHANGED (per this brief's own convention: C2-NAV.12
# reused c2n11_run.sh directly with a new tag when the run mechanics did
# not change). The only thing that changed this session is nav_bench.py's
# plan-window capture (observation only, see its own docstring) -- the
# orchestration script itself needs no edits.
HERE="$(cd "$(dirname "$0")" && pwd)"
WT="$(cd "$HERE/.." && pwd)"
bash "$HERE/c2n14_run.sh" "$WT/docs/data/c2nav11_ntp_params.yaml" c2n15_tour_r1 ALL 75 "enclosure_entry:-3.575,2.95" "enclosure_entry:200" "enclosure_entry:-3.00,0.625;enclosure_entry:-3.40,1.35" > "$HERE/logs/c2n15_tour_r1_driver.log" 2>&1
echo "EXIT=$?" >> "$HERE/logs/c2n15_tour_r1_driver.log"
