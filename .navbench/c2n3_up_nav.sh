#!/usr/bin/env bash
# C2-NAV.3: nav.launch.py pointed at the EXPLICIT C2-NAV.0 baseline params.
#
# ~/ros2_ws/install/gazebo_models symlinks to the TRUNK checkout, so
# `ros2 launch gazebo_models nav.launch.py` with no params_file would read
# the trunk's nav2_params.yaml. That file happens to BE the C2-NAV.0
# baseline (sha256 dbcee9ca...), but "happens to be" is not evidence.
# docs/data/c2nav3_baseline_params.yaml in this worktree is a verbatim copy
# of nav2_params.yaml at commit 8f05c45 -- the C2-NAV.0 commit -- and is
# passed explicitly so the record names one exact file. Both hashes are
# printed below every run.
#
# It is deliberately NOT named *nav2_params.yaml: an unbracketed 'nav2_'
# in ros_clean.sh matched any command line containing that substring.
HERE="$(cd "$(dirname "$0")" && pwd)"
source "$HERE/env.sh"
mkdir -p "$HERE/logs"
PARAMS="$(cd "$HERE/../docs/data" && pwd)/c2nav3_baseline_params.yaml"
echo "params_file: $PARAMS"
sha256sum "$PARAMS"
echo "--- the values that define the C2-NAV.0 baseline ---"
grep -n 'GoalChecker\|BaseObstacle.scale:\|PathAlign.scale:\|PathDist.scale:\|GoalAlign.scale:\|GoalDist.scale:\|RotateToGoal.scale:\|forward_point_distance\|yaw_goal_tolerance:\|vx_samples\|vtheta_samples\|sim_time:\|max_vel_x\|min_vel_x\|robot_radius\|inflation_radius\|short_circuit' "$PARAMS"
echo "--- what the INSTALLED default would have been ---"
readlink -f /home/gautham/ros2_ws/install/gazebo_models/share/gazebo_models/config/nav2_params.yaml
sha256sum /home/gautham/ros2_ws/install/gazebo_models/share/gazebo_models/config/nav2_params.yaml
echo "---"
ros2 launch gazebo_models nav.launch.py arbiter:=false \
    params_file:="$PARAMS" > "$HERE/logs/nav_c2n3.log" 2>&1 &
echo "nav pid $!"
for i in $(seq 1 150); do
    st=$(timeout 5 ros2 lifecycle get /bt_navigator 2>/dev/null)
    if echo "$st" | grep -q active; then echo "nav2 active after ${i}s"; exit 0; fi
    sleep 1
done
echo "NAV2 DID NOT ACTIVATE"; tail -40 "$HERE/logs/nav_c2n3.log"; exit 1
