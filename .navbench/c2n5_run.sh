#!/usr/bin/env bash
# C2-NAV.5 one run = ONE FRESH SIMULATOR.
#
#   c2n5_run.sh <abs params path> <tag> <scenarios|ALL> <timeout_s>
#
# The whole point of C2-NAV.5 is that a repeat is not a trial. C2-NAV.2
# and C2-NAV.4 both recorded that `nav_bench.py --repeats 3` runs one
# fresh approach followed by two ESCAPE probes from wherever the first
# one stopped, which at the baseline is the stall itself. So every run
# here is --repeats 1 behind a full ros_clean + fresh Gazebo, and the
# only way to get N samples is to call this N times.
#
# Naming: ros_clean.sh's 'nav[2]_' pattern matches any command line
# CONTAINING "nav2_" -- bracketing stops a pattern matching its own text,
# not other processes. So every C2-NAV.5 helper is c2n5_* and the two
# parameter files it names are c2nav3_baseline_params.yaml and
# c2nav4_csf65_params.yaml, neither of which contains that substring.
# c2n5_bracketcheck.sh asserts this against the real command lines.
HERE="$(cd "$(dirname "$0")" && pwd)"
source "$HERE/env.sh"
WT="$(cd "$HERE/.." && pwd)"
PARAMS="$1"
TAG="${2:-c2n5}"
ONLY="${3:-ALL}"
TO="${4:-150}"
mkdir -p "$HERE/results" "$HERE/logs"

[ -f "$PARAMS" ] || { echo "no such params file: $PARAMS"; exit 1; }
case "$PARAMS" in *nav2_*) echo "REFUSING: 'nav2_' in params path"; exit 1;; esac

echo "=== c2n5 run $TAG  scenarios=$ONLY  timeout=${TO}s  $(date -u +%H:%M:%S) ==="
bash "$WT/gazebo_models/scripts/ros_clean.sh" >/dev/null 2>&1
sleep 5
bash "$HERE/up_sim.sh" || { echo "SIM FAILED"; exit 1; }
sleep 3
bash "$HERE/c2n4_up_nav.sh" "$PARAMS" "$TAG" || { echo "NAV FAILED"; exit 1; }
bash "$HERE/wait_nav.sh" | head -1
sleep 3
bash "$HERE/c2n5_verify.sh" "$TAG" \
    | grep -E 'cost_scaling|inflation_radius|BaseObstacle.scale|goal_checker.plugin|xy_goal_tolerance|yaw_goal_tolerance|prune_distance|max_vel_x|min_vel_x|vx_samples|vtheta_samples|sim_time|short_circuit|GridBased.plugin|forward_point'
echo "=== ground truth at spawn ==="
timeout 10 ros2 topic echo /model/coco/odometry --once --field pose.pose.position 2>&1 | head -5
echo "=== bench ==="
if [ "$ONLY" = "ALL" ]; then
    python3 "$WT/gazebo_models/scripts/nav_bench.py" \
        --tag "$TAG" --repeats 1 --timeout "$TO" --out "$HERE/results"
else
    python3 "$WT/gazebo_models/scripts/nav_bench.py" \
        --tag "$TAG" --repeats 1 --timeout "$TO" --only "$ONLY" \
        --out "$HERE/results"
fi
rc=$?
echo "bench exit $rc"
exit $rc
