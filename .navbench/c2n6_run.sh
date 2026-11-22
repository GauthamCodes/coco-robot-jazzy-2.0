#!/usr/bin/env bash
# C2-NAV.6 one run = ONE FRESH SIMULATOR.
#
#   c2n6_run.sh <abs params path> <tag> <scenarios> <timeout_s>
#
# C2-NAV.5's c2n5_run.sh with one addition: the PolygonStop probe rides
# alongside nav_bench.py for the whole run. Everything else -- ros_clean,
# fresh Gazebo, fresh Nav2 against an EXPLICIT parameter file, live
# read-back before the bench starts, --repeats 1 -- is unchanged, because
# a repeat is not a trial and the second approach of a --repeats 3 starts
# from wherever the first one stopped.
#
# The scenario list here is "enclosure_entry,enclosure_exit" rather than
# ALL: enclosure_exit is a leg that starts wherever enclosure_entry left
# the robot, and the failure under diagnosis only exists when the entry
# SUCCEEDED first. C2-NAV.5's baseline enclosure_exit was 3/3 precisely
# because its entry always failed, so the exit never began inside the
# pocket. Running the two legs back to back is what makes the exit a real
# test, and it is the cheapest way to reach the state.
#
# Naming: ros_clean.sh's 'nav[2]_' pattern matches any command line
# CONTAINING "nav2_" -- bracketing stops a pattern matching its own text,
# not other processes. So every C2-NAV.6 helper is c2n6_*, the probe is
# c2nav6_stopprobe.py, and the parameter files are c2nav4_csf65_params.yaml
# and c2nav6_minpts*_params.yaml. c2n6_bracketcheck.sh asserts this
# against the command lines this experiment actually puts on the wire.
HERE="$(cd "$(dirname "$0")" && pwd)"
source "$HERE/env.sh"
WT="$(cd "$HERE/.." && pwd)"
PARAMS="$1"
TAG="${2:-c2n6}"
ONLY="${3:-enclosure_entry,enclosure_exit}"
TO="${4:-150}"
mkdir -p "$HERE/results" "$HERE/logs"

[ -f "$PARAMS" ] || { echo "no such params file: $PARAMS"; exit 1; }
case "$PARAMS" in *nav2_*) echo "REFUSING: 'nav2_' in params path"; exit 1;; esac

PREFIX="$HERE/results/${TAG}"
rm -f "${PREFIX}.done" "${PREFIX}_stop.csv" "${PREFIX}_stop.json"

echo "=== c2n6 run $TAG  scenarios=$ONLY  timeout=${TO}s  $(date -u +%H:%M:%S) ==="
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

echo "=== stop probe up ==="
BUDGET=$(python3 -c "print(2*$TO + 120)")
python3 "$WT/docs/data/c2nav6_stopprobe.py" "$PREFIX" "$BUDGET" \
    > "$HERE/logs/stopprobe_${TAG}.log" 2>&1 &
PROBE=$!
# The probe refuses to record until it has seen the scan, the monitor,
# ground truth and the TF. Wait for that line before driving, so a bench
# never runs against an instrument that was not yet alive.
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
python3 "$WT/gazebo_models/scripts/nav_bench.py" \
    --tag "$TAG" --repeats 1 --timeout "$TO" --only "$ONLY" \
    --out "$HERE/results"
rc=$?
echo "bench exit $rc"
touch "${PREFIX}.done"
wait "$PROBE"
prc=$?
echo "probe exit $prc"
tail -3 "$HERE/logs/stopprobe_${TAG}.log"
# 0 ok, 2 = no rows, 3 = the monitor or the wheels were never seen. A run
# whose telemetry failed is discarded, not reinterpreted, so say which.
case "$prc" in
  0) echo "TELEMETRY OK" ;;
  2) echo "TELEMETRY INVALID: probe recorded no rows" ;;
  3) echo "TELEMETRY INVALID: probe never saw the monitor or the wheels" ;;
  *) echo "TELEMETRY INVALID: probe exit $prc" ;;
esac
exit $rc
