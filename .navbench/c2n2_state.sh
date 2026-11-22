#!/usr/bin/env bash
# What is running right now, before C2-NAV.2 touches anything.
HERE="$(cd "$(dirname "$0")" && pwd)"
source "$HERE/env.sh"
echo "ROS_DISTRO=$ROS_DISTRO"
echo "=== sim/nav processes ==="
ps aux | grep -E '[g]z sim|[r]uby .*gz|[c]ontroller_server|[b]t_navigator|[p]arameter_bridge|[r]obot_state_pub|[c]md_vel' | awk '{print $2, $11, $12, $13}'
echo "=== node list ==="
timeout 15 ros2 node list 2>&1 | head -30
