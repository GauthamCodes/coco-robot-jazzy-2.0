#!/usr/bin/env bash
# C2-NAV.4 mechanism run: ONE fresh enclosure-entry approach on a fresh
# simulator, with the C2-NAV.3 capture unchanged so the numbers compare
# to C2-NAV.3's directly.
#
#   c2n4_capture.sh <abs params path> <tag>
#
# One fresh approach and nothing else: C2-NAV.2 established that
# `--only enclosure_entry --repeats 3` is one fresh approach plus two
# ESCAPE probes from the already-stalled state, and they are not trials.
#
# ros_clean.sh is taken from THIS WORKTREE, not the trunk: the trunk copy
# predates the C2-NAV.3 bracketing fix (commit 323471f / 4f981b4).
HERE="$(cd "$(dirname "$0")" && pwd)"
source "$HERE/env.sh"
WT="$(cd "$HERE/.." && pwd)"
PARAMS="$1"
TAG="${2:-c2n4}"
OUT="$HERE/results/$TAG"
mkdir -p "$(dirname "$OUT")" "$HERE/logs"

bash "$WT/gazebo_models/scripts/ros_clean.sh" >/dev/null 2>&1
sleep 5
bash "$HERE/up_sim.sh" || exit 1
sleep 3
bash "$HERE/c2n4_up_nav.sh" "$PARAMS" "$TAG" || exit 1
bash "$HERE/wait_nav.sh" | head -1
sleep 3
bash "$HERE/c2n4_verify.sh" "$TAG" | grep -E 'cost_scaling|BaseObstacle.scale|goal_checker.plugin|inflation_radius'
echo "=== ground truth at spawn ==="
timeout 10 ros2 topic echo /model/coco/odometry --once --field pose.pose.position 2>&1 | head -5
echo "=== capture ==="
python3 "$WT/docs/data/c2nav3_capture.py" "$OUT"
echo "capture exit $?"
