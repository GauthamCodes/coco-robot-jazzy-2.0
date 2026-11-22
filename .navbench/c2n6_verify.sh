#!/usr/bin/env bash
# C2-NAV.6: read the parameters back OFF THE LIVE NODES.
#
# C2-NAV.5's c2n5_verify.sh with the collision monitor promoted from a
# "must not have moved" spot-check to the thing under test: every field
# of all three action polygons, plus the monitor's frames and source
# wiring. C2-NAV.6's single variable is PolygonStop.min_points, so
# PolygonStop.min_points has to be read back off /collision_monitor, and
# so does PolygonStop.radius -- which must NOT have moved -- and the two
# other polygons' min_points, which share the value 4 in the file and are
# exactly why a global substitution on "min_points: 4" would have been
# wrong.
#
# A parameter file that was edited and a parameter file that was LOADED
# are different claims. This asserts the second one, and it is the only
# evidence that the run under way is the run intended.
HERE="$(cd "$(dirname "$0")" && pwd)"
source "$HERE/env.sh"
mkdir -p "$HERE/logs"
OUT="$HERE/logs/c2n6_params_${1:-run}.txt"
{
echo "=== THE SINGLE C2-NAV.6 VARIABLE, live ==="
for p in PolygonStop.min_points; do
    printf '  collision_monitor %-38s ' "$p"
    timeout 8 ros2 param get /collision_monitor "$p" 2>&1 | tail -1
done
echo "=== the PolygonStop geometry, which must NOT have moved ==="
for p in PolygonStop.type PolygonStop.radius PolygonStop.action_type \
         PolygonStop.enabled PolygonStop.visualize; do
    printf '  collision_monitor %-38s ' "$p"
    timeout 8 ros2 param get /collision_monitor "$p" 2>&1 | tail -1
done
echo "=== the other zones, which must NOT have moved (min_points stays 4/4/6) ==="
for p in polygons PolygonSlow.min_points PolygonSlow.action_type \
         PolygonSlow.slowdown_ratio PolygonLimit.min_points \
         PolygonLimit.action_type PolygonLimit.linear_limit \
         PolygonLimit.angular_limit FootprintApproach.min_points \
         FootprintApproach.action_type; do
    printf '  collision_monitor %-38s ' "$p"
    timeout 8 ros2 param get /collision_monitor "$p" 2>&1 | tail -1
done
echo "=== monitor frames and source wiring, which must NOT have moved ==="
for p in base_frame_id odom_frame_id cmd_vel_in_topic cmd_vel_out_topic \
         base_shift_correction transform_tolerance source_timeout \
         observation_sources scan.topic scan.source_timeout; do
    printf '  collision_monitor %-38s ' "$p"
    timeout 8 ros2 param get /collision_monitor "$p" 2>&1 | tail -1
done
echo "=== the C2-NAV.4/.5 candidate baseline, live ==="
for p in inflation_layer.cost_scaling_factor inflation_layer.inflation_radius \
         robot_radius footprint_padding; do
    printf '  local_costmap  %-42s ' "$p"
    timeout 8 ros2 param get /local_costmap/local_costmap "$p" 2>&1 | tail -1
done
for p in inflation_layer.cost_scaling_factor inflation_layer.inflation_radius \
         robot_radius footprint_padding; do
    printf '  global_costmap %-42s ' "$p"
    timeout 8 ros2 param get /global_costmap/global_costmap "$p" 2>&1 | tail -1
done
echo "=== the C2-NAV.0 controller baseline, live ==="
for p in FollowPath.BaseObstacle.scale FollowPath.BaseObstacle.sum_scores \
         FollowPath.GoalDist.scale FollowPath.GoalAlign.scale \
         FollowPath.PathDist.scale FollowPath.PathAlign.scale \
         FollowPath.RotateToGoal.scale \
         FollowPath.sim_time FollowPath.vx_samples FollowPath.vtheta_samples \
         FollowPath.max_vel_x FollowPath.min_vel_x \
         goal_checker.plugin goal_checker.xy_goal_tolerance \
         goal_checker.yaw_goal_tolerance controller_frequency; do
    printf '  controller_server %-52s ' "$p"
    timeout 8 ros2 param get /controller_server "$p" 2>&1 | tail -1
done
echo "=== planner ==="
timeout 8 ros2 param get /planner_server GridBased.plugin 2>&1 | tail -1
echo "=== the scan the monitor actually reads ==="
timeout 10 ros2 topic echo /scan --once --field angle_min 2>&1 | head -2
timeout 10 ros2 topic echo /scan --once --field angle_increment 2>&1 | head -2
timeout 10 ros2 topic echo /scan --once --field range_min 2>&1 | head -2
timeout 10 ros2 topic echo /scan --once --field range_max 2>&1 | head -2
timeout 10 ros2 topic echo /scan --once --field header.frame_id 2>&1 | head -2
echo "=== installed collision monitor ==="
dpkg -l | grep -E 'ros-jazzy-nav2-collision-monitor' | awk '{print "  "$2" "$3}'
} 2>&1 | tee "$OUT"
echo "wrote $OUT"
