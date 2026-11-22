#!/usr/bin/env bash
# C2-NAV.2 bringup: full teardown, fresh headless sim, then Nav2 pointed
# at the worktree params. Topology A (arbiter:=false), exactly as the
# C2-NAV.0 baseline and the C2-NAV.1 experiment ran it.
HERE="$(cd "$(dirname "$0")" && pwd)"
source "$HERE/env.sh"
bash /home/gautham/ros2_ws/src/coco-robot-ros2/gazebo_models/scripts/ros_clean.sh >/dev/null 2>&1
sleep 5
bash "$HERE/up_sim.sh" || exit 1
sleep 3
bash "$HERE/up_nav_c2nav2.sh" || exit 1
bash "$HERE/wait_nav.sh" | tail -12
echo "=== robot at spawn? (ground truth) ==="
timeout 10 ros2 topic echo /model/coco/odometry --once --field pose.pose.position 2>&1 | head -5
