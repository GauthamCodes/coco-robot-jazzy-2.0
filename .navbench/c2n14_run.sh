#!/usr/bin/env bash
# C2-NAV.14 one run = ONE FRESH SIMULATOR = ONE COMPLETE SEVEN-LEG TOUR,
# with enclosure_entry driven as ONE NavigateThroughPoses request carrying
# TWO through-poses ahead of its own goal: a heading-correcting pose,
# THEN the existing C2-NAV.9/.11 waypoint (-3.40, 1.35), unchanged.
#
#   c2n14_run.sh <abs params path> <tag> <scenarios|ALL> <default_to_s> \
#                [goal_spec] [leg_timeout_spec] [through_pose_spec_list]
#
# Identical to c2n11_run.sh (C2-NAV.11's own runner) except ONE thing:
# through_pose_spec_list may contain MULTIPLE "BEFORE:X,Y" specs
# separated by ';', each becoming its own --through-pose flag, so
# nav_bench.py's apply_through_poses() appends them in order into ONE
# scenario's via-pose list (see that function's docstring -- order is
# preserved, the scenario's own goal is always appended last by
# send_multi_leg's caller in main()). Everything else -- ros_clean,
# fresh Gazebo, fresh Nav2 against c2nav11_ntp_params.yaml (the ONLY
# params file with the corrected default_nav_through_poses_bt_xml),
# the C2-NAV.6 PolygonStop probe, --repeats 1 -- is byte-identical to
# c2n11_run.sh.
#
# Naming: ros_clean.sh's 'nav[2]_' pattern matches any command line
# CONTAINING "nav2_". Every C2-NAV.14 helper is c2n14_*, the probe is
# still c2nav6_stopprobe.py, and the parameter file is
# c2nav11_ntp_params.yaml (reused, not copied) -- none of those strings
# contain "nav2_".
HERE="$(cd "$(dirname "$0")" && pwd)"
source "$HERE/env.sh"
WT="$(cd "$HERE/.." && pwd)"
PARAMS="$1"
TAG="${2:-c2n14}"
ONLY="${3:-ALL}"
TO="${4:-75}"
GOAL="${5:-}"
LEGTO="${6:-}"
THROUGH="${7:-}"          # ';'-separated list of BEFORE:X,Y specs
mkdir -p "$HERE/results" "$HERE/logs"

[ -f "$PARAMS" ] || { echo "no such params file: $PARAMS"; exit 1; }
case "$PARAMS" in *nav2_*) echo "REFUSING: 'nav2_' in params path"; exit 1;; esac

PREFIX="$HERE/results/${TAG}"
rm -f "${PREFIX}.done" "${PREFIX}_stop.csv" "${PREFIX}_stop.json"

echo "=== c2n14 run $TAG  scenarios=$ONLY  default_timeout=${TO}s"
echo "    goal='${GOAL:-<committed TOUR>}'  leg_timeout='${LEGTO:-<none>}'"
echo "    through-pose list='${THROUGH:-<none>}'"
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
echo "=== bt_navigator through-poses BT (the C2-NAV.11 wiring fix, reused) ==="
ros2 param get /bt_navigator default_nav_through_poses_bt_xml 2>&1
echo "=== action server present? ==="
timeout 10 ros2 action list -t 2>&1 | grep -i navigate
echo "=== ground truth at spawn ==="
timeout 10 ros2 topic echo /model/coco/odometry --once --field pose.pose.position 2>&1 | head -5

EXT=0
case "$LEGTO" in *:*) EXT="${LEGTO##*:}";; esac
if [ "$ONLY" = "ALL" ]; then NLEGS=7; else NLEGS=$(( $(echo "$ONLY" | tr ',' '\n' | wc -l) )); fi
BUDGET=$(python3 -c "print(int($NLEGS*$TO + max($TO, ${EXT:-0}) + 300))")
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
ARGS=(--tag "$TAG" --repeats 1 --timeout "$TO" --out "$HERE/results")
[ "$ONLY" = "ALL" ] || ARGS+=(--only "$ONLY")
[ -n "$GOAL" ] && ARGS+=(--goal "$GOAL")
[ -n "$LEGTO" ] && ARGS+=(--leg-timeout "$LEGTO")
if [ -n "$THROUGH" ]; then
    IFS=';' read -ra TSPECS <<< "$THROUGH"
    for spec in "${TSPECS[@]}"; do
        ARGS+=(--through-pose "$spec")
    done
fi
echo "=== resolved nav_bench.py args: ${ARGS[*]} ==="
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
