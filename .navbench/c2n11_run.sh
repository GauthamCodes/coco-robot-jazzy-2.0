#!/usr/bin/env bash
# C2-NAV.11 one run = ONE FRESH SIMULATOR = ONE COMPLETE SEVEN-LEG TOUR
# (or a --only subset), with the enclosure_entry leg driven as ONE
# NavigateThroughPoses request through C2-NAV.9's waypoint instead of
# C2-NAV.10's separate inserted leg.
#
#   c2n11_run.sh <abs params path> <tag> <scenarios|ALL> <default_to_s> \
#                [goal_spec] [leg_timeout_spec] [through_pose_spec]
#
# C2-NAV.10's c2n10_run.sh with two changes: --through-pose replaces
# --waypoint (nav_bench's apply_through_poses, not apply_waypoint_insert
# -- see that function's docstring for why), and the probe budget no
# longer adds an eighth leg, because attaching a through-pose to a
# scenario does not add one: the tour keeps exactly the same leg names
# and count as C2-NAV.8 ran, whether or not --through-pose is set.
#
# C2-NAV.10 found that an inserted waypoint LEG reliably threads the wide
# corridor (3/3) but the re-plan boundary right after it does not carry
# that preference into the segment where box_obstacle_1's SW corner
# actually lives (2/3 deadlocked there). This tests the other shape of
# intervention C2-NAV.10's own "Exact next experiment" names: the SAME
# waypoint (-3.40, 1.35), sent as part of ONE continuous
# NavigateThroughPoses request together with enclosure_entry's own goal,
# so there is no re-plan boundary for DWB's local sampling to reset at.
#
# The parameter file MUST be c2nav11_ntp_params.yaml, not
# c2nav4_csf65_params.yaml: it is byte-identical to the C2-NAV.5/8/9/10
# validated baseline except ONE line, bt_navigator.
# default_nav_through_poses_bt_xml, corrected from a pre-existing
# misconfiguration (it pointed at the SINGLE-pose replanning BT, which
# only reads the {goal} blackboard key -- ComputePathToPose, not
# ComputePathThroughPoses -- so NavigateThroughPoses as shipped in this
# repo would silently collapse to "plan straight to the last pose",
# discarding every intermediate pose without erroring). See
# docs/RESULTS.md "C2-NAV.11" for the full derivation and the two BT XML
# files compared byte-for-byte. No CSF/inflation/BaseObstacle/
# PolygonStop/goal-checker/DWB/footprint/collision-monitor value differs
# from c2nav4_csf65_params.yaml -- diff the two files to confirm before
# trusting a result.
#
# Everything else unchanged from C2-NAV.8/10 by design -- ros_clean,
# fresh Gazebo, fresh Nav2 against an EXPLICIT parameter file, live
# read-back before the bench starts, the C2-NAV.6 PolygonStop probe
# riding alongside UNMODIFIED (so a count here is comparable to C2-NAV.8
# and C2-NAV.10 by construction), --repeats 1.
#
# Naming: ros_clean.sh's 'nav[2]_' pattern matches any command line
# CONTAINING "nav2_". Every C2-NAV.11 helper is c2n11_*, the probe is
# still c2nav6_stopprobe.py, and the parameter file is
# c2nav11_ntp_params.yaml -- none of those strings contain "nav2_".
HERE="$(cd "$(dirname "$0")" && pwd)"
source "$HERE/env.sh"
WT="$(cd "$HERE/.." && pwd)"
PARAMS="$1"
TAG="${2:-c2n11}"
ONLY="${3:-ALL}"
TO="${4:-75}"
GOAL="${5:-}"
LEGTO="${6:-}"
THROUGH="${7:-}"
mkdir -p "$HERE/results" "$HERE/logs"

[ -f "$PARAMS" ] || { echo "no such params file: $PARAMS"; exit 1; }
case "$PARAMS" in *nav2_*) echo "REFUSING: 'nav2_' in params path"; exit 1;; esac

PREFIX="$HERE/results/${TAG}"
rm -f "${PREFIX}.done" "${PREFIX}_stop.csv" "${PREFIX}_stop.json"

echo "=== c2n11 run $TAG  scenarios=$ONLY  default_timeout=${TO}s"
echo "    goal='${GOAL:-<committed TOUR>}'  leg_timeout='${LEGTO:-<none>}'"
echo "    through-pose='${THROUGH:-<none>}'"
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
echo "=== bt_navigator through-poses BT (the C2-NAV.11 wiring fix) ==="
ros2 param get /bt_navigator default_nav_through_poses_bt_xml 2>&1
echo "=== action server present? ==="
timeout 10 ros2 action list -t 2>&1 | grep -i navigate
echo "=== ground truth at spawn ==="
timeout 10 ros2 topic echo /model/coco/odometry --once --field pose.pose.position 2>&1 | head -5

# Probe budget: unlike c2n10_run.sh, --through-pose never adds a leg, so
# the tour is exactly as many legs as --only selects (or 7 for ALL) at
# TO, one of them possibly extended by --leg-timeout, plus bringup slack.
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
[ -n "$THROUGH" ] && ARGS+=(--through-pose "$THROUGH")
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
