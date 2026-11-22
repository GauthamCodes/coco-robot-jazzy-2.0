#!/usr/bin/env bash
# C2-NAV.10 PRIMARY EXPERIMENT: N fresh REDUCED-LEG approaches --
# corridor_gate -> enclosure_entry_waypoint -> enclosure_entry -- one
# fresh simulator each. Brief section 13 is explicit: do not run the
# whole seven-leg tour until this establishes whether the waypoint
# changes the approach outcome at all. (The full-tour validation script,
# c2n10_all.sh, is ready for that as "the next experiment" and is not
# invoked this session.)
#
#   c2n10_approach_all.sh [n_runs] [first_index]
#
# Mirrors C2-NAV.7's own precedent of a two-named-leg run via nav_bench's
# --only (c2n7_run.sh); the waypoint insertion turns "corridor_gate,
# enclosure_entry" into three actual NavigateToPose calls:
# corridor_gate -> enclosure_entry_waypoint -> enclosure_entry.
#
# Every run starts at the world SPAWN (nav_bench chains from wherever the
# previous leg stopped; this is the first leg), same as C2-NAV.7's own
# two-leg runs -- NOT the pose the full seven-leg tour reaches
# corridor_gate from. That caveat is C2-NAV.7's, repeated here rather
# than re-discovered: a reduced run is not a stand-in for the tour.
HERE="$(cd "$(dirname "$0")" && pwd)"
source "$HERE/env.sh"
WT="$(cd "$HERE/.." && pwd)"

PARAMS="$WT/docs/data/c2nav4_csf65_params.yaml"
GOAL='enclosure_entry:-3.575,2.95'
LEGTO='enclosure_entry:200'
WAYPOINT='enclosure_entry:-3.40,1.35'
ONLY='corridor_gate,enclosure_entry'
TO=75
N="${1:-3}"
FIRST="${2:-1}"

echo "############################################################"
echo "# C2-NAV.10 APPROACH-ONLY  $N fresh runs, from r$FIRST"
echo "#   params    $PARAMS"
sha256sum "$PARAMS" | sed 's/^/#   sha256    /'
echo "#   legs      $ONLY (waypoint inserted -> 3 actual NavigateToPose calls)"
echo "#   goal      $GOAL"
echo "#   caps      ${TO}s per leg, $LEGTO"
echo "#   waypoint  $WAYPOINT"
echo "#   started   $(date -u +%Y-%m-%dT%H:%M:%SZ)"
echo "############################################################"

for i in $(seq "$FIRST" $((FIRST + N - 1))); do
    TAG="c2n10_appr_r${i}"
    echo
    echo "########## RUN $i  ($TAG)  $(date -u +%H:%M:%S) UTC"
    bash "$HERE/c2n10_run.sh" "$PARAMS" "$TAG" "$ONLY" "$TO" "$GOAL" "$LEGTO" "$WAYPOINT" \
        2>&1 | tee "$HERE/logs/${TAG}.log"
    echo "########## RUN $i done rc=${PIPESTATUS[0]}  $(date -u +%H:%M:%S) UTC"
done

echo
echo "########## ros_clean after the last run"
bash "$WT/gazebo_models/scripts/ros_clean.sh" >/dev/null 2>&1
echo "ALL C2-NAV.10 APPROACH RUNS DONE $(date -u +%Y-%m-%dT%H:%M:%SZ)"
