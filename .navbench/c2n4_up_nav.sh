#!/usr/bin/env bash
# C2-NAV.4: nav.launch.py pointed at an EXPLICIT parameter file.
#
#   c2n4_up_nav.sh <abs params path> <log suffix>
#
# ~/ros2_ws/install/gazebo_models symlinks to the TRUNK checkout, so a
# launch without params_file:= reads the trunk's nav2_params.yaml. Worse,
# the WORKTREE's gazebo_models/config/nav2_params.yaml still carries
# C2-NAV.2's REJECTED BaseObstacle.scale 2.0, left there deliberately as
# that experiment's record. Neither is the C2-NAV.0 baseline this
# experiment needs, so the file is always named explicitly and hashed.
#
# Naming: ros_clean.sh's 'nav[2]_' pattern matches any command line
# CONTAINING that substring -- bracketing stops it matching its own text,
# not other processes. So every C2-NAV.4 helper is c2n4_* and every
# parameter file is c2nav4_*_params.yaml, never *nav2_params.yaml.
HERE="$(cd "$(dirname "$0")" && pwd)"
source "$HERE/env.sh"
mkdir -p "$HERE/logs"
PARAMS="$1"
SUF="${2:-c2n4}"
[ -f "$PARAMS" ] || { echo "no such params file: $PARAMS"; exit 1; }
case "$PARAMS" in *nav2_*) echo "REFUSING: params path contains 'nav2_', ros_clean.sh would kill the launch"; exit 1;; esac

echo "params_file: $PARAMS"
sha256sum "$PARAMS"
echo "--- the single C2-NAV.4 variable, and the baseline it sits in ---"
sed -n '190,207p' "$PARAMS" | grep -n 'local_costmap\|robot_radius\|inflation_radius\|cost_scaling_factor'
echo "--- global_costmap inflation (MUST stay 5.0: one variable) ---"
sed -n '296,300p' "$PARAMS"
echo "--- the C2-NAV.0 behavioural baseline ---"
grep -n 'GoalChecker\|BaseObstacle.scale:\|PathAlign.scale:\|PathDist.scale:\|GoalAlign.scale:\|GoalDist.scale:\|RotateToGoal.scale:\|forward_point_distance\|yaw_goal_tolerance:\|vx_samples\|vtheta_samples\|sim_time:\|max_vel_x\|min_vel_x\|short_circuit' "$PARAMS"
echo "--- what the INSTALLED default would have been ---"
readlink -f /home/gautham/ros2_ws/install/gazebo_models/share/gazebo_models/config/nav2_params.yaml
sha256sum /home/gautham/ros2_ws/install/gazebo_models/share/gazebo_models/config/nav2_params.yaml
echo "---"
ros2 launch gazebo_models nav.launch.py arbiter:=false \
    params_file:="$PARAMS" > "$HERE/logs/nav_${SUF}.log" 2>&1 &
echo "nav pid $!"
for i in $(seq 1 150); do
    st=$(timeout 5 ros2 lifecycle get /bt_navigator 2>/dev/null)
    if echo "$st" | grep -q active; then echo "nav2 active after ${i}s"; exit 0; fi
    sleep 1
done
echo "NAV2 DID NOT ACTIVATE"; tail -40 "$HERE/logs/nav_${SUF}.log"; exit 1
