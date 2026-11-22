#!/usr/bin/env bash
source /home/gautham/ros2_ws/src/coco-robot-ros2/setup_env.sh
echo "source rc=$?"
echo "ros2: $(which ros2)"
echo "AMENT head: ${AMENT_PREFIX_PATH:0:200}"
