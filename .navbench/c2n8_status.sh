#!/usr/bin/env bash
# C2-NAV.8 pre-flight: machine and ROS-graph state before anything starts.
# Bracketed patterns throughout -- this script's own command line must not
# be matched by ros_clean.sh, and its own text must not match itself.
HERE="$(cd "$(dirname "$0")" && pwd)"
source "$HERE/env.sh"
echo "=== ROS ==="
echo "ROS_DISTRO=$ROS_DISTRO  ROS_DOMAIN_ID=$ROS_DOMAIN_ID  DISPLAY=$DISPLAY"
echo "=== live nodes ==="
timeout 15 ros2 node list 2>/dev/null | head -30
echo "=== stray processes ==="
pgrep -af 'g[z] sim' | head -5
pgrep -af 'full_world_rob[o]' | head -5
pgrep -af 'nav_benc[h]' | head -5
pgrep -af 'c2nav6_stopprob[e]' | head -5
echo "=== machine ==="
free -g | head -2
df -h /home | tail -1
nvidia-smi --query-gpu=memory.used,memory.total --format=csv,noheader 2>/dev/null | head -2
echo "=== installed collision monitor ==="
dpkg -l | grep -E 'ros-jazzy-nav2-collision-monitor' | awk '{print "  "$2" "$3}'
