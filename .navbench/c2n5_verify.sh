#!/usr/bin/env bash
# C2-NAV.5: read the parameters back OFF THE LIVE NODES.
#
# Body is C2-NAV.4's c2n4_verify.sh with three additions -- the two
# MapGrid forward_point_distances, prune_plan/prune_distance, and the
# collision monitor -- because the C2-NAV.5 brief names all of them as
# things that must not have moved. Only the output filename changes, so a
# C2-NAV.5 read-back cannot be confused with a C2-NAV.4 one sitting in
# the same logs/ directory.
#
# A parameter file that was edited and a parameter file that was LOADED
# are different claims. This asserts the second one, and it is the only
# evidence that the run under way is the run intended.
HERE="$(cd "$(dirname "$0")" && pwd)"
source "$HERE/env.sh"
mkdir -p "$HERE/logs"
OUT="$HERE/logs/c2n5_params_${1:-run}.txt"
{
echo "=== THE SINGLE VARIABLE, live ==="
for p in inflation_layer.cost_scaling_factor inflation_layer.inflation_radius \
         robot_radius footprint_padding resolution width height; do
    printf '  local_costmap  %-42s ' "$p"
    timeout 8 ros2 param get /local_costmap/local_costmap "$p" 2>&1 | tail -1
done
echo "=== the global costmap, which must NOT have moved ==="
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
         FollowPath.PathAlign.forward_point_distance \
         FollowPath.GoalAlign.forward_point_distance \
         FollowPath.sim_time FollowPath.vx_samples FollowPath.vtheta_samples \
         FollowPath.max_vel_x FollowPath.min_vel_x \
         FollowPath.short_circuit_trajectory_evaluation \
         FollowPath.prune_plan FollowPath.prune_distance \
         FollowPath.publish_cost_grid_pc \
         goal_checker.plugin goal_checker.xy_goal_tolerance \
         goal_checker.yaw_goal_tolerance goal_checker.stateful \
         controller_frequency; do
    printf '  controller_server %-52s ' "$p"
    timeout 8 ros2 param get /controller_server "$p" 2>&1 | tail -1
done
echo "=== planner ==="
timeout 8 ros2 param get /planner_server GridBased.plugin 2>&1 | tail -1
echo "=== collision monitor, which must NOT have moved ==="
for p in polygons PolygonStop.radius PolygonSlow.action_type \
         PolygonSlow.slowdown_ratio PolygonLimit.action_type \
         PolygonLimit.linear_limit FootprintApproach.action_type; do
    printf '  collision_monitor %-32s ' "$p"
    timeout 8 ros2 param get /collision_monitor "$p" 2>&1 | tail -1
done
echo "=== installed dwb_critics / costmap ==="
dpkg -l | grep -E 'ros-jazzy-(dwb-critics|nav2-costmap-2d|nav2-dwb-controller)' \
    | awk '{print "  "$2" "$3}'
} 2>&1 | tee "$OUT"
echo "wrote $OUT"
