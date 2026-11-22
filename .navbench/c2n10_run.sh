#!/usr/bin/env bash
# C2-NAV.10 one run = ONE FRESH SIMULATOR = ONE COMPLETE SEVEN-LEG TOUR,
# with ONE intermediate waypoint inserted into the enclosure_entry leg.
#
#   c2n10_run.sh <abs params path> <tag> <scenarios|ALL> <default_to_s> \
#                [goal_spec] [leg_timeout_spec] [waypoint_spec]
#
# C2-NAV.8's c2n8_run.sh with one addition: an optional --waypoint
# pass-through, for the reason nav_bench's apply_waypoint_insert
# docstring gives. C2-NAV.9 found a PolygonStop-free route from every
# observed corridor_gate exit to the current enclosure_entry goal with a
# 326 mm bottleneck, but found nothing in the local cost field (CSF 65
# reaches cost 0 at 291 mm) that would make DWB prefer it over the
# 257-260 mm route past box_obstacle_1's SW corner that deadlocked
# C2-NAV.8's r1 tour. This inserts one explicit global-plan waypoint,
# derived from C2-NAV.9's committed clearance grid, to test whether that
# alone changes which route DWB samples as cheapest.
#
# Everything else is unchanged from C2-NAV.8 by design -- ros_clean,
# fresh Gazebo, fresh Nav2 against an EXPLICIT parameter file, live
# read-back before the bench starts, the C2-NAV.6 PolygonStop probe
# riding alongside UNMODIFIED (so a count here is comparable to a count
# there by construction), --repeats 1.
#
# The parameter file is c2nav4_csf65_params.yaml: the C2-NAV.5 VALIDATED
# configuration, PolygonStop.min_points 4 and radius 0.25, CSF 65/5,
# inflation_radius 0.5, BaseObstacle.scale 8. C2-NAV.10 changes NO
# navigation parameter at all -- the only difference from C2-NAV.8 is the
# inserted waypoint leg.
#
# Naming: ros_clean.sh's 'nav[2]_' pattern matches any command line
# CONTAINING "nav2_". Every C2-NAV.10 helper is c2n10_*, the probe is
# c2nav6_stopprobe.py, and the parameter file is c2nav4_csf65_params.yaml.
# c2n10_bracketcheck.sh asserts this.
HERE="$(cd "$(dirname "$0")" && pwd)"
source "$HERE/env.sh"
WT="$(cd "$HERE/.." && pwd)"
PARAMS="$1"
TAG="${2:-c2n10}"
ONLY="${3:-ALL}"
TO="${4:-75}"
GOAL="${5:-}"
LEGTO="${6:-}"
WAYPOINT="${7:-}"
mkdir -p "$HERE/results" "$HERE/logs"

[ -f "$PARAMS" ] || { echo "no such params file: $PARAMS"; exit 1; }
case "$PARAMS" in *nav2_*) echo "REFUSING: 'nav2_' in params path"; exit 1;; esac

PREFIX="$HERE/results/${TAG}"
rm -f "${PREFIX}.done" "${PREFIX}_stop.csv" "${PREFIX}_stop.json"

echo "=== c2n10 run $TAG  scenarios=$ONLY  default_timeout=${TO}s"
echo "    goal='${GOAL:-<committed TOUR>}'  leg_timeout='${LEGTO:-<none>}'"
echo "    waypoint='${WAYPOINT:-<none>}'"
echo "    $(date -u +%H:%M:%S) UTC ==="
bash "$WT/gazebo_models/scripts/ros_clean.sh" >/dev/null 2>&1
sleep 5
bash "$HERE/up_sim.sh" || { echo "SIM FAILED"; exit 1; }
sleep 3
bash "$HERE/c2n4_up_nav.sh" "$PARAMS" "$TAG" || { echo "NAV FAILED"; exit 1; }
bash "$HERE/wait_nav.sh" | head -1
sleep 3
bash "$HERE/c2n6_verify.sh" "$TAG" \
    | grep -E 'PolygonStop|PolygonSlow|PolygonLimit|polygons|cost_scaling|inflation_radius|BaseObstacle.scale|goal_checker.plugin|xy_goal_tolerance|yaw_goal_tolerance|max_vel_x|vx_samples|vtheta_samples|base_frame_id|source_timeout'
echo "=== ground truth at spawn ==="
timeout 10 ros2 topic echo /model/coco/odometry --once --field pose.pose.position 2>&1 | head -5

# Probe budget: the tour is seven legs, six at TO and one extended, plus
# bringup slack -- plus one more leg at TO if --waypoint inserted an
# eighth. C2-NAV.7's 2*TO+120 was sized for a two-leg run and would cut
# the recording off less than halfway through a tour -- which would
# present as "the probe saw no STOP frames on the exit".
EXT=0
case "$LEGTO" in *:*) EXT="${LEGTO##*:}";; esac
WPLEG=0
[ -n "$WAYPOINT" ] && WPLEG=1
BUDGET=$(python3 -c "print(int((6+$WPLEG)*$TO + max($TO, ${EXT:-0}) + 300))")
echo "=== stop probe up, budget ${BUDGET}s ==="
python3 "$WT/docs/data/c2nav6_stopprobe.py" "$PREFIX" "$BUDGET" \
    > "$HERE/logs/stopprobe_${TAG}.log" 2>&1 &
PROBE=$!
for i in $(seq 1 90); do
    grep -q 'stopprobe. alive' "$HERE/logs/stopprobe_${TAG}.log" && break
    grep -q 'INSTRUMENT NOT ALIVE' "$HERE/logs/stopprobe_${TAG}.log" && {
        echo "PROBE NOT ALIVE"; cat "$HERE/logs/stopprobe_${TAG}.log"; exit 1; }
    kill -0 "$PROBE" 2>/dev/null || { echo "PROBE DIED";
        cat "$HERE/logs/stopprobe_${TAG}.log"; exit 1; }
    sleep 1
done
grep 'stopprobe. alive' "$HERE/logs/stopprobe_${TAG}.log" || {
    echo "PROBE DID NOT REPORT ALIVE"; cat "$HERE/logs/stopprobe_${TAG}.log";
    exit 1; }

echo "=== bench ==="
# ALL means the whole seven-leg tour, i.e. NO --only. Passing the literal
# "ALL" to --only matches no scenario and silently runs zero legs, which
# looks like a bench that finished instantly rather than one that never
# started.
ARGS=(--tag "$TAG" --repeats 1 --timeout "$TO" --out "$HERE/results")
[ "$ONLY" = "ALL" ] || ARGS+=(--only "$ONLY")
[ -n "$GOAL" ] && ARGS+=(--goal "$GOAL")
[ -n "$LEGTO" ] && ARGS+=(--leg-timeout "$LEGTO")
[ -n "$WAYPOINT" ] && ARGS+=(--waypoint "$WAYPOINT")
python3 "$WT/gazebo_models/scripts/nav_bench.py" "${ARGS[@]}"
rc=$?
echo "bench exit $rc"
touch "${PREFIX}.done"
wait "$PROBE"
prc=$?
echo "probe exit $prc"
tail -3 "$HERE/logs/stopprobe_${TAG}.log"
case "$prc" in
  0) echo "TELEMETRY OK" ;;
  2) echo "TELEMETRY INVALID: probe recorded no rows" ;;
  3) echo "TELEMETRY INVALID: probe never saw the monitor or the wheels" ;;
  *) echo "TELEMETRY INVALID: probe exit $prc" ;;
esac
cp -f "$HERE/logs/c2n6_params_${TAG}.txt" \
      "$HERE/results/${TAG}_params_live.txt" 2>/dev/null
exit $rc
