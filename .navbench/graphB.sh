#!/usr/bin/env bash
HERE="$(cd "$(dirname "$0")" && pwd)"
source "$HERE/env.sh"
for t in /cmd_vel_nav /cmd_vel_smoothed /cmd_vel /diff_drive_controller/cmd_vel; do
    echo "##### $t"
    timeout 20 ros2 topic info "$t" --verbose 2>&1 \
        | grep -E "^Publisher count|^Subscription count|Node name" \
        | sed 's/^ *//'
done
