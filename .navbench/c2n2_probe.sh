#!/usr/bin/env bash
# C2-NAV.2 evidence probe on a FRESH simulator, so the robot drives into
# the pinch from the spawn exactly as repeat 0 did, rather than starting
# already stalled. Same params file, same topology A, nothing changed.
HERE="$(cd "$(dirname "$0")" && pwd)"
source "$HERE/env.sh"
bash /home/gautham/ros2_ws/src/coco-robot-ros2/gazebo_models/scripts/ros_clean.sh >/dev/null 2>&1
sleep 5
bash "$HERE/up_sim.sh" || exit 1
sleep 3
bash "$HERE/up_nav_c2nav2.sh" | tail -3 || exit 1
bash "$HERE/wait_nav.sh" | head -1
sleep 3
python3 "$HERE/c2n2_evalprobe.py" "$HERE/results/c2n2_eval.json"
