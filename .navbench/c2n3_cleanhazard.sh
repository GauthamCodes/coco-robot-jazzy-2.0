#!/usr/bin/env bash
# Does the ros_clean.sh self-kill hazard still exist? Section 12 of the
# C2-NAV.3 brief asks this to be checked, not assumed.
#
# The hazard: ros_clean.sh brackets its patterns so a pattern cannot match
# the pkill that runs it -- EXCEPT for a few. An unbracketed 'nav2_' also
# matches ANY process whose command line contains the substring "nav2_",
# including a helper script named c2nav2_up.sh. Nothing here kills
# anything; it only reports which patterns are unbracketed and whether a
# would-be helper name matches them.
CLEAN=/home/gautham/ros2_ws/src/coco-robot-ros2/.claude/worktrees/c2nav0-diagnosis/gazebo_models/scripts/ros_clean.sh

echo "=== patterns with NO bracket (a pattern that can match itself) ==="
sed -n "/^PATTERNS=(/,/^)/p" "$CLEAN" \
  | grep -oE "^  '[^']+'" | tr -d " '" \
  | grep -v '\[' | sed 's/^/  /'

echo
echo "=== do those patterns match plausible C2-NAV helper names? ==="
for name in c2nav2_up.sh c2nav3_up.sh c2n3_up.sh c2n3_capture.py \
            c2nav3_baseline_params.yaml; do
  for pat in 'nav2_' 'ros2_control_node' 'rosbridge'; do
    if echo "$name" | grep -qE "$pat"; then
      echo "  MATCH  pattern '$pat'  vs  $name"
    fi
  done
done
echo "  (no MATCH line above for a name = that name is safe)"

echo
echo "=== does the bracketed form still match the real nav2 nodes? ==="
for node in nav2_controller nav2_planner nav2_bt_navigator nav2_amcl \
            nav2_map_server nav2_lifecycle_manager nav2_velocity_smoother \
            nav2_collision_monitor nav2_behaviors ros2_control_node \
            rosbridge_websocket; do
  b=$(echo "$node" | grep -cE 'nav[2]_|ros2_control_nod[e]|rosbridg[e]')
  printf '  %-28s bracketed-pattern match: %s\n' "$node" "$b"
done

echo
echo "=== and the bracketed pattern text must NOT match itself ==="
for pat in 'nav[2]_' 'ros2_control_nod[e]' 'rosbridg[e]'; do
  m=$(echo "$pat" | grep -cE "$pat")
  printf '  pattern %-24s matches its own text: %s (must be 0)\n' "$pat" "$m"
done
