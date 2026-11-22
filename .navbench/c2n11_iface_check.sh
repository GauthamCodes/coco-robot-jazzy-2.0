#!/usr/bin/env bash
# C2-NAV.11 static/interface verification (no simulator, no live ROS graph
# needed for most of this -- ros2 interface show reads message definitions
# from the installed package, not a running node).
set -e
. /home/gautham/ros2_ws/src/coco-robot-ros2/setup_env.sh >/dev/null 2>&1
WT=/home/gautham/ros2_ws/src/coco-robot-ros2/.claude/worktrees/c2nav0-diagnosis

echo "=== 1. nav2_msgs and nav2_bt_navigator installed? ==="
ros2 pkg list 2>/dev/null | grep -E '^nav2_msgs$|^nav2_bt_navigator$|^nav2_waypoint_follower$'

echo
echo "=== 2. NavigateThroughPoses action definition ==="
ros2 interface show nav2_msgs/action/NavigateThroughPoses

echo
echo "=== 3. bt_navigator config in the validated C2-NAV.5/8/9/10 baseline (BEFORE fix) ==="
grep -n "bt_navigator" -A 17 "$WT/docs/data/c2nav4_csf65_params.yaml"

echo
echo "=== 4. bt_navigator config in the C2-NAV.11 params file (AFTER fix) ==="
grep -n "bt_navigator" -A 17 "$WT/docs/data/c2nav11_ntp_params.yaml"

echo
echo "=== 5. one-line diff between baseline and C2-NAV.11 params ==="
diff "$WT/docs/data/c2nav4_csf65_params.yaml" "$WT/docs/data/c2nav11_ntp_params.yaml" || true

echo
echo "=== 6. sha256 of both params files ==="
sha256sum "$WT/docs/data/c2nav4_csf65_params.yaml" "$WT/docs/data/c2nav11_ntp_params.yaml"

echo
echo "=== 7. the two candidate BT XML files, byte diff ==="
diff /opt/ros/jazzy/share/nav2_bt_navigator/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml \
     /opt/ros/jazzy/share/nav2_bt_navigator/behavior_trees/navigate_through_poses_w_replanning_and_recovery.xml || true

echo
echo "=== 8. key structural difference: does each BT read {goal} or {goals}? ==="
echo "-- navigate_to_pose (single-pose) BT ComputePath node:"
grep -o '<ComputePath[A-Za-z]*[^/]*/>' /opt/ros/jazzy/share/nav2_bt_navigator/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml
echo "-- navigate_through_poses (multi-pose) BT ComputePath node:"
grep -o '<ComputePath[A-Za-z]*[^/]*/>' /opt/ros/jazzy/share/nav2_bt_navigator/behavior_trees/navigate_through_poses_w_replanning_and_recovery.xml
