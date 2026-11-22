#!/usr/bin/env bash
# gazebo_models unit tests, from INSIDE the package dir, clean graph,
# with test_integration ignored (CLAUDE.md: without the ignore, pytest
# dies importing test_sim_bringup.launch.py during collection and
# silently reports 0 tests instead of failing loudly).
HERE=/home/gautham/ros2_ws/src/coco-robot-ros2/.claude/worktrees/c2nav0-diagnosis
source "$HERE/.navbench/env.sh"
cd "$HERE/gazebo_models" || exit 1
python3 -m pytest test -q --ignore=test/test_integration 2>&1 | tail -12
