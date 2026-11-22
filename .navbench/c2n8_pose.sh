#!/usr/bin/env bash
# Where is the robot right now, in world coordinates, and what is the
# nearest world-file collision face? Read-only: subscribes, publishes
# nothing, sends no goal.
HERE="$(cd "$(dirname "$0")" && pwd)"
source "$HERE/env.sh"
timeout 10 ros2 topic echo /model/coco/odometry --once --field pose.pose.position 2>&1 | head -4
