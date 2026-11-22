#!/usr/bin/env bash
# Does ros_clean.sh's pattern list match any C2-NAV.6 command line?
#
# C2-NAV.5's c2n5_bracketcheck.sh, re-pointed at the strings C2-NAV.6
# actually puts on the wire, with one addition. C2-NAV.6 adds a node --
# c2nav6_stopprobe.py -- and the repo rule is that anything that outlives
# a Ctrl-C gets a ros_clean.sh pattern. So this run has TWO positive
# controls, not one: the real nav launch, and the probe itself, which
# MUST match or an orphan of it would splice a baseline recording into
# the candidate run.
#
# The hazard is not hypothetical: C2-NAV.2's run died at exit 144 because
# 'nav2_' matched its own helper's name. Bracketing fixed a pattern
# matching its OWN text; it did NOT stop a pattern matching another
# process whose command line contains the substring.
HERE="$(cd "$(dirname "$0")" && pwd)"
WT="$(cd "$HERE/.." && pwd)"
CLEAN="$WT/gazebo_models/scripts/ros_clean.sh"

CMDS=(
  "bash $HERE/c2n6_all.sh"
  "bash $HERE/c2n6_run.sh $WT/docs/data/c2nav4_csf65_params.yaml c2n6_base_r1 enclosure_entry,enclosure_exit 150"
  "bash $HERE/c2n6_run.sh $WT/docs/data/c2nav6_minpts7_params.yaml c2n6_cand_r1 enclosure_entry,enclosure_exit 150"
  "bash $HERE/c2n4_up_nav.sh $WT/docs/data/c2nav6_minpts7_params.yaml c2n6_cand_r1"
  "bash $HERE/c2n6_verify.sh c2n6_cand_r1"
  "python3 $WT/gazebo_models/scripts/nav_bench.py --tag c2n6_base_r1 --repeats 1 --timeout 150 --only enclosure_entry,enclosure_exit --out $HERE/results"
  "python3 $WT/docs/data/c2nav6_report.py stop $HERE/results"
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

# The two that SHOULD match. A check whose success condition is "we saw
# nothing" must first prove it can see something -- and the second
# control is also a requirement in its own right, because an orphaned
# probe is a corrupted measurement rather than merely a stray process.
declare -A CTL=(
  ["the real nav launch"]="ros2 launch gazebo_models nav.launch.py arbiter:=false"
  ["the C2-NAV.6 probe"]="python3 $WT/docs/data/c2nav6_stopprobe.py $HERE/results/c2n6_base_r1 420"
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
