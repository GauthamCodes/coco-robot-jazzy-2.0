#!/usr/bin/env bash
# C2-NAV.1 section 12: RViz on the nav2 default view, which draws the
# local costmap, the global plan, the footprint and the scan. That is
# the view that can answer "stuck despite visible free space"; the
# Gazebo GUI camera, zoomed to the whole arena, cannot.
HERE=/home/gautham/ros2_ws/src/coco-robot-ros2/.claude/worktrees/c2nav0-diagnosis/.navbench
source "$HERE/env.sh"
SHOT="$HERE/observe_rviz"
rm -rf "$SHOT"
mkdir -p "$SHOT"

rviz2 -d /opt/ros/jazzy/share/nav2_bringup/rviz/nav2_default_view.rviz \
    --ros-args -p use_sim_time:=true > "$HERE/logs/rviz.log" 2>&1 &
RV=$!
echo "rviz pid $RV"
sleep 35

ffmpeg -y -f x11grab -framerate 10 -video_size 1920x1080 -i :1 \
    -t 95 -vf fps=1/3 "$SHOT/frame_%03d.png" \
    > "$HERE/logs/ffmpeg2.log" 2>&1 &
FF=$!
sleep 2

python3 "$HERE/../gazebo_models/scripts/nav_bench.py" \
    --tag observe_rviz --repeats 1 --timeout 75 \
    --only enclosure_entry --out "$HERE/results" || true

wait $FF 2>/dev/null
kill $RV 2>/dev/null
ls "$SHOT" | wc -l
