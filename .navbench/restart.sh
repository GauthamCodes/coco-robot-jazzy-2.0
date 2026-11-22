#!/usr/bin/env bash
# restart.sh [arbiter]  -- full teardown then a fresh sim + nav stack.
HERE="$(cd "$(dirname "$0")" && pwd)"
source "$HERE/env.sh"
bash /home/gautham/ros2_ws/src/coco-robot-ros2/gazebo_models/scripts/ros_clean.sh >/dev/null 2>&1
sleep 5
bash "$HERE/up_sim.sh" || exit 1
sleep 3
bash "$HERE/up_nav.sh" "${1:-false}" >/dev/null 2>&1
bash "$HERE/wait_nav.sh" | tail -12
