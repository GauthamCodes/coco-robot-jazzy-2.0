#!/usr/bin/env bash
# Does ros_clean.sh's pattern list match any C2-NAV.8 command line?
#
# C2-NAV.7's c2n7_bracketcheck.sh re-pointed at the strings C2-NAV.8
# actually puts on the wire. Two positive controls, not one: the real
# nav launch, and the probe itself -- an orphaned probe is a CORRUPTED
# MEASUREMENT (it would splice one tour's recording into the next) rather
# than merely a stray process, so it MUST match.
#
# The hazard is not hypothetical: C2-NAV.2's run died at exit 144 because
# 'nav2_' matched its own helper's name, and C2-NAV.6 lost a status check
# to 'g[z] sim' the same way. Bracketing stops a pattern matching its OWN
# text; it does NOT stop it matching another process whose command line
# contains the substring.
HERE="$(cd "$(dirname "$0")" && pwd)"
WT="$(cd "$HERE/.." && pwd)"
CLEAN="$WT/gazebo_models/scripts/ros_clean.sh"

P="$WT/docs/data/c2nav4_csf65_params.yaml"
G='enclosure_entry:-3.575,2.95'
L='enclosure_entry:200'

CMDS=(
  "bash $HERE/c2n8_all.sh"
  "bash $HERE/c2n8_run.sh $P c2n8_tour_r1 ALL 75 $G $L"
  "bash $HERE/c2n8_run.sh $P c2n8_tour_r2 ALL 75 $G $L"
  "bash $HERE/c2n8_run.sh $P c2n8_tour_r3 ALL 75 $G $L"
  "bash $HERE/c2n4_up_nav.sh $P c2n8_tour_r1"
  "bash $HERE/c2n6_verify.sh c2n8_tour_r1"
  "bash $HERE/c2n8_status.sh"
  "python3 $WT/gazebo_models/scripts/nav_bench.py --tag c2n8_tour_r1 --repeats 1 --timeout 75 --out $HERE/results --goal $G --leg-timeout $L"
  "python3 $WT/docs/data/c2nav8_report.py collect $HERE/results"
  "python3 $WT/docs/data/c2nav8_report.py legs"
  "python3 $WT/docs/data/c2nav8_report.py stop $HERE/results"
  "python3 $WT/docs/data/c2nav7_geom.py track $HERE/results/c2n8_tour_r1_stop.csv C2-NAV.8-r1"
  "python3 $HERE/c2n8/offline_test.py"
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

declare -A CTL=(
  ["the real nav launch"]="ros2 launch gazebo_models nav.launch.py arbiter:=false"
  ["the C2-NAV.8 probe"]="python3 $WT/docs/data/c2nav6_stopprobe.py $HERE/results/c2n8_tour_r1 950"
  ["the simulator"]="ros2 launch gazebo_models full_world_robo.launch.py gui:=false"
)
for name in "${!CTL[@]}"; do
    ok=0
    for p in "${PATS[@]}"; do
        printf '%s' "${CTL[$name]}" | grep -qE "$p" && {
            echo "CONTROL ok: '$p' matches $name"; ok=1; }
    done
    [ "$ok" = 1 ] || { echo "CONTROL FAILED: nothing matches $name"; bad=1; }
done
exit $bad
