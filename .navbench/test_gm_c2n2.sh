#!/usr/bin/env bash
# gazebo_models unit tests for C2-NAV.2. Same invocation C2-NAV.1 used:
# from INSIDE the package dir, on a clean graph, test_integration ignored.
HERE=/home/gautham/ros2_ws/src/coco-robot-ros2/.claude/worktrees/c2nav0-diagnosis
source "$HERE/.navbench/env.sh"
bash "$HERE/gazebo_models/scripts/ros_clean.sh" >/dev/null 2>&1
sleep 3
cd "$HERE/gazebo_models" || exit 1
python3 -m pytest test -q --ignore=test/test_integration 2>&1 | tail -12
