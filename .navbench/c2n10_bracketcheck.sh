#!/usr/bin/env bash
# Does ros_clean.sh's pattern list match any C2-NAV.10 command line?
#
# C2-NAV.8's c2n8_bracketcheck.sh re-pointed at the strings C2-NAV.10
# actually puts on the wire. No new process or launch file is added --
# --waypoint only changes what nav_bench.py itself sends, so this is
# mainly a regression check that the existing patterns still cover
# every C2-NAV.10 command line, plus the same two positive controls:
# the real nav launch, and the probe itself (an orphaned probe is a
# CORRUPTED MEASUREMENT, not merely a stray process).
HERE="$(cd "$(dirname "$0")" && pwd)"
WT="$(cd "$HERE/.." && pwd)"
CLEAN="$WT/gazebo_models/scripts/ros_clean.sh"

P="$WT/docs/data/c2nav4_csf65_params.yaml"
G='enclosure_entry:-3.575,2.95'
L='enclosure_entry:200'
WP='enclosure_entry:-3.40,1.35'

CMDS=(
  "bash $HERE/c2n10_all.sh"
  "bash $HERE/c2n10_run.sh $P c2n10_tour_r1 ALL 75 $G $L $WP"
  "bash $HERE/c2n10_run.sh $P c2n10_tour_r2 ALL 75 $G $L $WP"
  "bash $HERE/c2n10_run.sh $P c2n10_tour_r3 ALL 75 $G $L $WP"
  "bash $HERE/c2n4_up_nav.sh $P c2n10_tour_r1"
  "bash $HERE/c2n6_verify.sh c2n10_tour_r1"
  "python3 $WT/gazebo_models/scripts/nav_bench.py --tag c2n10_tour_r1 --repeats 1 --timeout 75 --out $HERE/results --goal $G --leg-timeout $L --waypoint $WP"
  "bash $CLEAN"
)

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
  ["the C2-NAV.10 probe"]="python3 $WT/docs/data/c2nav6_stopprobe.py $HERE/results/c2n10_tour_r1 950"
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
