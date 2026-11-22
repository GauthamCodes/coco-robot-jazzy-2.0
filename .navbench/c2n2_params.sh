#!/usr/bin/env bash
# The parameters the RUNNING controller actually holds. The file is not
# evidence; the live node is. C2-NAV.1 verified single-variable this way
# and this reruns the identical list so the two are comparable, plus the
# critic scales, which are what C2-NAV.2 moves.
HERE="$(cd "$(dirname "$0")" && pwd)"
source "$HERE/env.sh"
echo "=== controller_server, live ==="
for p in FollowPath.BaseObstacle.scale FollowPath.BaseObstacle.sum_scores \
         FollowPath.PathAlign.scale FollowPath.PathDist.scale \
         FollowPath.GoalAlign.scale FollowPath.GoalDist.scale \
         FollowPath.RotateToGoal.scale FollowPath.RotateToGoal.slowing_factor \
         FollowPath.RotateToGoal.lookahead_time \
         FollowPath.Oscillation.scale \
         FollowPath.xy_goal_tolerance FollowPath.trans_stopped_velocity \
         FollowPath.vx_samples FollowPath.vtheta_samples \
         FollowPath.sim_time FollowPath.max_vel_x FollowPath.max_vel_theta \
         FollowPath.acc_lim_x FollowPath.decel_lim_x \
         FollowPath.linear_granularity FollowPath.angular_granularity \
         FollowPath.short_circuit_trajectory_evaluation \
         FollowPath.critics \
         goal_checker.plugin goal_checker.xy_goal_tolerance \
         goal_checker.yaw_goal_tolerance goal_checker.stateful \
         progress_checker.required_movement_radius \
         progress_checker.movement_time_allowance \
         controller_frequency failure_tolerance min_x_velocity_threshold; do
    v=$(timeout 10 ros2 param get /controller_server "$p" 2>&1 | tail -1)
    printf '%-52s %s\n' "$p" "$v"
done
echo "=== local_costmap, live ==="
for p in robot_radius inflation_layer.inflation_radius \
         inflation_layer.cost_scaling_factor width height resolution \
         update_frequency publish_frequency; do
    v=$(timeout 10 ros2 param get /local_costmap/local_costmap "$p" 2>&1 | tail -1)
    printf '%-52s %s\n' "$p" "$v"
done
echo "=== collision_monitor, live ==="
for p in PolygonStop.action_type PolygonSlow.action_type \
         PolygonSlow.slowdown_ratio PolygonLimit.action_type; do
    v=$(timeout 10 ros2 param get /collision_monitor "$p" 2>&1 | tail -1)
    printf '%-52s %s\n' "$p" "$v"
done
echo "=== the goal checker the node says it created ==="
grep -i 'goal checker\|Created goal\|progress checker' "$HERE/logs/nav_c2nav2.log" | head -6
