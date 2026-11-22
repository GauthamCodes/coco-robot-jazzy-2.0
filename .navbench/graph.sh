#!/usr/bin/env bash
HERE="$(cd "$(dirname "$0")" && pwd)"
source "$HERE/env.sh"
for t in /cmd_vel_nav /cmd_vel_smoothed /cmd_vel /diff_drive_controller/cmd_vel \
         /cmd_vel_teleop /cmd_vel_rl /cmd_vel_approach /collision_monitor_state \
         /plan /local_plan /evaluation /scan; do
    echo "############ $t"
    timeout 15 ros2 topic info "$t" --verbose 2>&1 \
        | grep -E "^Type|^Publisher count|^Subscription count|Node name|Topic type" \
        | sed 's/^ *//'
done
echo "############ dwb-ish topics"
ros2 topic list | grep -iE "eval|traj|cost_cloud|marker|global_plan|local_plan|transformed"
