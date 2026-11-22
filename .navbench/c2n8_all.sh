#!/usr/bin/env bash
# C2-NAV.8: N complete seven-leg tours, one FRESH SIMULATOR each.
#
#   c2n8_all.sh [n_tours] [first_index]
#
# A repeat is not a trial. nav_bench --repeats 3 runs one fresh approach
# followed by two ESCAPE probes from wherever the first stopped, so every
# tour here is --repeats 1 behind a full ros_clean + fresh Gazebo + fresh
# Nav2, and N samples means calling c2n8_run.sh N times.
#
# ONE VARIABLE against C2-NAV.5's committed tour, and it is not a
# navigation parameter: the enclosure_entry goal, (-3.45, 2.95) ->
# (-3.575, 2.95). CSF stays 65.0 / 5.0, inflation_radius 0.5,
# BaseObstacle.scale 8.0, SimpleGoalChecker, PolygonStop radius 0.25 and
# min_points 4 -- all read back off the LIVE nodes on every tour.
HERE="$(cd "$(dirname "$0")" && pwd)"
source "$HERE/env.sh"
WT="$(cd "$HERE/.." && pwd)"

PARAMS="$WT/docs/data/c2nav4_csf65_params.yaml"
GOAL='enclosure_entry:-3.575,2.95'
LEGTO='enclosure_entry:200'
TO=75
N="${1:-3}"
FIRST="${2:-1}"

echo "############################################################"
echo "# C2-NAV.8  $N fresh seven-leg tours, from r$FIRST"
echo "#   params  $PARAMS"
sha256sum "$PARAMS" | sed 's/^/#   sha256  /'
echo "#   goal    $GOAL"
echo "#   caps    ${TO}s per leg, $LEGTO"
echo "#   started $(date -u +%Y-%m-%dT%H:%M:%SZ)"
echo "############################################################"

for i in $(seq "$FIRST" $((FIRST + N - 1))); do
    TAG="c2n8_tour_r${i}"
    echo
    echo "########## TOUR $i  ($TAG)  $(date -u +%H:%M:%S) UTC"
    bash "$HERE/c2n8_run.sh" "$PARAMS" "$TAG" ALL "$TO" "$GOAL" "$LEGTO" \
        2>&1 | tee "$HERE/logs/${TAG}.log"
    echo "########## TOUR $i done rc=${PIPESTATUS[0]}  $(date -u +%H:%M:%S) UTC"
done

echo
echo "########## ros_clean after the last tour"
bash "$WT/gazebo_models/scripts/ros_clean.sh" >/dev/null 2>&1
echo "ALL TOURS DONE $(date -u +%Y-%m-%dT%H:%M:%SZ)"
