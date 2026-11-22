#!/usr/bin/env bash
# C2-NAV.2: nav.launch.py pointed at the WORKTREE nav2_params.yaml.
# install/gazebo_models/share/gazebo_models/config/nav2_params.yaml is a
# SYMLINK to the trunk checkout, which still holds BaseObstacle.scale 8.0
# and SimpleGoalChecker -- i.e. the C2-NAV.0 baseline. Without
# params_file:= this experiment would silently re-run the baseline.
HERE="$(cd "$(dirname "$0")" && pwd)"
source "$HERE/env.sh"
mkdir -p "$HERE/logs"
PARAMS="$(cd "$HERE/../gazebo_models/config" && pwd)/nav2_params.yaml"
echo "params_file: $PARAMS"
echo "--- what that file says ---"
grep -n 'GoalChecker\|BaseObstacle.scale:\|yaw_goal_tolerance:\|xy_goal_tolerance:' "$PARAMS"
echo "--- what the INSTALLED default would have said ---"
readlink -f /home/gautham/ros2_ws/install/gazebo_models/share/gazebo_models/config/nav2_params.yaml
grep -n 'GoalChecker\|BaseObstacle.scale:' \
    /home/gautham/ros2_ws/install/gazebo_models/share/gazebo_models/config/nav2_params.yaml
echo "---"
ros2 launch gazebo_models nav.launch.py arbiter:=false \
    params_file:="$PARAMS" > "$HERE/logs/nav_c2nav2.log" 2>&1 &
echo "nav pid $!"
for i in $(seq 1 150); do
    st=$(timeout 5 ros2 lifecycle get /bt_navigator 2>/dev/null)
    if echo "$st" | grep -q active; then echo "nav2 active after ${i}s"; exit 0; fi
    sleep 1
done
echo "NAV2 DID NOT ACTIVATE"; tail -40 "$HERE/logs/nav_c2nav2.log"; exit 1
