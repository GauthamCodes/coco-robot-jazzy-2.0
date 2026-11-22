#!/usr/bin/env bash
# Does ros_clean.sh's pattern list match any C2-NAV.5 command line?
#
# C2-NAV.4's c2n4_bracketcheck.sh, re-pointed at the strings C2-NAV.5
# actually puts on the wire. The hazard is not hypothetical: C2-NAV.2's
# run died at exit 144 because 'nav2_' matched its own helper's name.
# Bracketing fixed a pattern matching its OWN text; it did NOT stop a
# pattern matching another process whose command line contains the
# substring.
HERE="$(cd "$(dirname "$0")" && pwd)"
WT="$(cd "$HERE/.." && pwd)"
CLEAN="$WT/gazebo_models/scripts/ros_clean.sh"

CMDS=(
  "bash $HERE/c2n5_enclosure.sh 5"
  "bash $HERE/c2n5_tour.sh 3"
  "bash $HERE/c2n5_run.sh $WT/docs/data/c2nav3_baseline_params.yaml c2n5_enc_base_r1 enclosure_entry 150"
  "bash $HERE/c2n5_run.sh $WT/docs/data/c2nav4_csf65_params.yaml c2n5_tour_csf65_r1 ALL 75"
  "bash $HERE/c2n4_up_nav.sh $WT/docs/data/c2nav4_csf65_params.yaml c2n5_enc_csf65_r1"
  "bash $HERE/c2n5_verify.sh c2n5_enc_csf65_r1"
  "bash $HERE/c2n5_costprobe.sh $WT/docs/data/c2nav4_csf65_params.yaml c2n5_cost_csf65_r1"
  "python3 $WT/docs/data/c2nav5_costprobe.py $HERE/results/c2n5_cost_csf65_r1"
  "python3 $WT/gazebo_models/scripts/nav_bench.py --tag c2n5_enc_base_r1 --repeats 1 --timeout 150 --only enclosure_entry --out $HERE/results"
  "python3 $WT/docs/data/c2nav5_report.py enclosure $HERE/results"
  "bash $CLEAN"
)

# The pattern array, read out of the script itself so this cannot drift
# from what actually runs.
mapfile -t PATS < <(sed -n "/^PATTERNS=(/,/^)/p" "$CLEAN" \
    | grep -oE "^  '[^']+'" | tr -d " '")

echo "patterns: ${#PATS[@]}"
bad=0
for c in "${CMDS[@]}"; do
    hits=()
    for p in "${PATS[@]}"; do
        if printf '%s' "$c" | grep -qE "$p"; then hits+=("$p"); fi
    done
    if [ ${#hits[@]} -eq 0 ]; then
        printf 'SAFE   %s\n' "$c"
    else
        printf 'MATCH  %s   <-- %s\n' "$c" "${hits[*]}"
        bad=1
    fi
done
# And the one that SHOULD match, as the positive control: a check whose
# success condition is "we saw nothing" must first prove it can see
# something.
ctl="ros2 launch gazebo_models nav.launch.py arbiter:=false"
for p in "${PATS[@]}"; do
    printf '%s' "$ctl" | grep -qE "$p" && { echo "CONTROL ok: '$p' matches the real nav launch"; ok=1; }
done
[ "${ok:-0}" = 1 ] || { echo "CONTROL FAILED: the matcher matches nothing at all"; bad=1; }
exit $bad
