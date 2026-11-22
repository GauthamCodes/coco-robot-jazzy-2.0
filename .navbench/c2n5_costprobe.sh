#!/usr/bin/env bash
# C2-NAV.5 stage 3: the cost field, on a fresh simulator, at both
# conditions.
#
#   c2n5_costprobe.sh <abs params path> <tag>
#
# One fresh approach and nothing else, for the reason C2-NAV.2 recorded:
# `--repeats 3` is one fresh approach plus two escape probes from the
# already-stalled state, and those are not trials.
#
# Runs c2nav5_costprobe.py rather than C2-NAV.3's c2nav3_capture.py,
# because the latter triggers on a STALL and the candidate does not
# stall. Both read the same grids through the same code; only the trigger
# differs. See the docstring in docs/data/c2nav5_costprobe.py.
HERE="$(cd "$(dirname "$0")" && pwd)"
source "$HERE/env.sh"
WT="$(cd "$HERE/.." && pwd)"
PARAMS="$1"
TAG="${2:-c2n5cost}"
OUT="$HERE/results/$TAG"
mkdir -p "$(dirname "$OUT")" "$HERE/logs"

[ -f "$PARAMS" ] || { echo "no such params file: $PARAMS"; exit 1; }
case "$PARAMS" in *nav2_*) echo "REFUSING: 'nav2_' in params path"; exit 1;; esac

bash "$WT/gazebo_models/scripts/ros_clean.sh" >/dev/null 2>&1
sleep 5
bash "$HERE/up_sim.sh" || { echo "SIM FAILED"; exit 1; }
sleep 3
bash "$HERE/c2n4_up_nav.sh" "$PARAMS" "$TAG" || { echo "NAV FAILED"; exit 1; }
bash "$HERE/wait_nav.sh" | head -1
sleep 3
bash "$HERE/c2n5_verify.sh" "$TAG" \
    | grep -E 'cost_scaling|inflation_radius|BaseObstacle.scale|goal_checker.plugin'
echo "=== ground truth at spawn ==="
timeout 10 ros2 topic echo /model/coco/odometry --once --field pose.pose.position 2>&1 | head -5
echo "=== costprobe ==="
python3 "$WT/docs/data/c2nav5_costprobe.py" "$OUT"
echo "costprobe exit $?"
