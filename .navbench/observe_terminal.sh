#!/usr/bin/env bash
# C2-NAV.1 section 12, the other half: WATCH the two legs whose terminal
# phase the change was supposed to delete. Baseline open_space drove
# 2.2 m in 9 s and then stood still for 5.5 s; baseline wall_adjacent
# spent 73.0 s of a 77.3 s leg settling and failed 2 of 3. Fresh sim, so
# the tour starts at the spawn as it does in the measured run.
HERE=/home/gautham/ros2_ws/src/coco-robot-ros2/.claude/worktrees/c2nav0-diagnosis/.navbench
source "$HERE/env.sh"
SHOT="$HERE/observe_terminal"
rm -rf "$SHOT"
mkdir -p "$SHOT"

bash "$HERE/../gazebo_models/scripts/ros_clean.sh" > /dev/null 2>&1
sleep 5
bash "$HERE/up_sim.sh" || exit 1
sleep 3
bash "$HERE/up_nav_c2nav1.sh" > "$HERE/logs/nav_obs.log" 2>&1
tail -2 "$HERE/logs/nav_obs.log"
bash "$HERE/wait_nav.sh" | head -1

rviz2 -d /opt/ros/jazzy/share/nav2_bringup/rviz/nav2_default_view.rviz \
    --ros-args -p use_sim_time:=true > "$HERE/logs/rviz2.log" 2>&1 &
RV=$!
sleep 30

ffmpeg -y -f x11grab -framerate 10 -video_size 1920x1080 -i :1 \
    -t 45 -vf fps=1 "$SHOT/frame_%03d.png" \
    > "$HERE/logs/ffmpeg3.log" 2>&1 &
FF=$!
sleep 2

python3 "$HERE/../gazebo_models/scripts/nav_bench.py" \
    --tag observe_terminal --repeats 1 --timeout 75 \
    --only open_space,wall_adjacent --out "$HERE/results" || true

wait $FF 2>/dev/null
kill $RV 2>/dev/null
ls "$SHOT" | wc -l
