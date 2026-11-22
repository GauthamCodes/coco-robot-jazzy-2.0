#!/usr/bin/env bash
# Where does the command stop? Walk the chain from DWB to the wheels.
HERE="$(cd "$(dirname "$0")" && pwd)"
source "$HERE/env.sh"
echo "=== controllers ==="
timeout 20 ros2 control list_controllers 2>&1 | head -10
echo
echo "=== topic wiring ==="
for t in /cmd_vel_nav /cmd_vel /diff_drive_controller/cmd_vel /cmd_vel_smoothed; do
    echo "--- $t"
    timeout 12 ros2 topic info "$t" 2>&1 | head -5
done
echo
echo "=== is anything on the wire? (3 s each) ==="
for t in /cmd_vel_nav /diff_drive_controller/cmd_vel; do
    n=$(timeout 6 ros2 topic hz "$t" 2>&1 | head -2 | tr '\n' ' ')
    printf '  %-34s %s\n' "$t" "$n"
done
echo
echo "=== collision monitor state ==="
timeout 8 ros2 topic echo /collision_monitor_state --once 2>&1 | head -5
echo
echo "=== nav log: controller / smoother / monitor complaints ==="
grep -iE 'collision|smoother|no valid control|failed to make progress|aborting|stuck|Invalid path|goal checker' "$HERE/logs/nav_c2n3.log" | tail -25
