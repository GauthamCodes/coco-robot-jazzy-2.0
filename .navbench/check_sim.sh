#!/usr/bin/env bash
HERE="$(cd "$(dirname "$0")" && pwd)"
source "$HERE/env.sh"
echo "=== clock rate ==="
timeout 6 ros2 topic hz /clock 2>&1 | head -3
echo "=== scan rate ==="
timeout 6 ros2 topic hz /scan 2>&1 | head -3
echo "=== ground truth pose ==="
timeout 6 ros2 topic echo /model/coco/odometry --once --field pose.pose 2>&1 | head -12
echo "=== controllers ==="
timeout 10 ros2 control list_controllers 2>&1 | head -10
