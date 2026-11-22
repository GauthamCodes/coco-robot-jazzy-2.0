#!/usr/bin/env bash
# C2-NAV.16 run 1: byte-identical config to C2-NAV.14/.15 (same params file,
# same goal, same heading-correcting pose + waypoint), reusing c2n14_run.sh
# UNCHANGED -- same convention C2-NAV.15 itself used against C2-NAV.14's
# runner. Purpose: capture ONE fresh SW-corner failure with full plan-window
# instrumentation, to compare against the committed c2n15_tour_r1 GOOD run.
HERE="$(cd "$(dirname "$0")" && pwd)"
WT="$(cd "$HERE/.." && pwd)"
bash "$HERE/c2n14_run.sh" "$WT/docs/data/c2nav11_ntp_params.yaml" c2n16_tour_r1 ALL 75 "enclosure_entry:-3.575,2.95" "enclosure_entry:200" "enclosure_entry:-3.00,0.625;enclosure_entry:-3.40,1.35" > "$HERE/logs/c2n16_tour_r1_driver.log" 2>&1
echo "EXIT=$?" >> "$HERE/logs/c2n16_tour_r1_driver.log"
