#!/usr/bin/env bash
HERE="$(cd "$(dirname "$0")" && pwd)"
WT="$(cd "$HERE/.." && pwd)"
bash "$HERE/c2n14_run.sh" "$WT/docs/data/c2nav11_ntp_params.yaml" c2n14_tour_r1 ALL 75 "enclosure_entry:-3.575,2.95" "enclosure_entry:200" "enclosure_entry:-3.00,0.625;enclosure_entry:-3.40,1.35" > "$HERE/logs/c2n14_tour_r1_driver.log" 2>&1
echo "EXIT=$?" >> "$HERE/logs/c2n14_tour_r1_driver.log"
