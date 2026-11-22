#!/usr/bin/env bash
# Which of the probe's four preconditions is not being met right now.
HERE="$(cd "$(dirname "$0")" && pwd)"
source "$HERE/env.sh"
for t in /scan /collision_monitor_state /model/coco/odometry /cmd_vel_smoothed /cmd_vel_nav; do
    printf '%-32s ' "$t"
    timeout 6 ros2 topic echo "$t" --once >/dev/null 2>&1 \
        && echo "PUBLISHING" || echo "silent"
done
echo "--- tf ---"
timeout 8 ros2 run tf2_ros tf2_echo base_footprint lidar_link 2>&1 | head -8
