#!/usr/bin/env bash
# The parameters the RUNNING controller actually holds. The file is not
# evidence; the live node is. Same list C2-NAV.1 and C2-NAV.2 used, so the
# four runs are comparable line for line, plus the trajectory-generator
# and MapGrid values C2-NAV.3 depends on.
HERE="$(cd "$(dirname "$0")" && pwd)"
source "$HERE/env.sh"
echo "=== controller_server, live ==="
for p in FollowPath.BaseObstacle.scale FollowPath.BaseObstacle.sum_scores \
         FollowPath.PathAlign.scale FollowPath.PathDist.scale \
         FollowPath.GoalAlign.scale FollowPath.GoalDist.scale \
         FollowPath.PathAlign.forward_point_distance \
         FollowPath.GoalAlign.forward_point_distance \
         FollowPath.PathDist.aggregation_type \
         FollowPath.GoalDist.aggregation_type \
         FollowPath.RotateToGoal.scale FollowPath.RotateToGoal.slowing_factor \
         FollowPath.RotateToGoal.lookahead_time \
         FollowPath.Oscillation.scale \
         FollowPath.xy_goal_tolerance FollowPath.trans_stopped_velocity \
         FollowPath.vx_samples FollowPath.vtheta_samples \
         FollowPath.sim_time FollowPath.max_vel_x FollowPath.min_vel_x \
         FollowPath.max_vel_theta \
         FollowPath.acc_lim_x FollowPath.decel_lim_x \
         FollowPath.acc_lim_theta FollowPath.decel_lim_theta \
         FollowPath.linear_granularity FollowPath.angular_granularity \
         FollowPath.discretize_by_time FollowPath.include_last_point \
         FollowPath.prune_plan FollowPath.prune_distance \
         FollowPath.forward_prune_distance \
         FollowPath.shorten_transformed_plan \
         FollowPath.short_circuit_trajectory_evaluation \
         FollowPath.publish_cost_grid_pc FollowPath.publish_transformed_plan \
         FollowPath.critics \
         goal_checker.plugin goal_checker.xy_goal_tolerance \
         goal_checker.yaw_goal_tolerance goal_checker.stateful \
         progress_checker.required_movement_radius \
         progress_checker.movement_time_allowance \
         controller_frequency failure_tolerance min_x_velocity_threshold; do
    v=$(timeout 10 ros2 param get /controller_server "$p" 2>&1 | tail -1)
    printf '%-56s %s\n' "$p" "$v"
done
echo "=== local_costmap, live ==="
for p in robot_radius inflation_layer.inflation_radius \
         inflation_layer.cost_scaling_factor width height resolution \
         update_frequency publish_frequency global_frame \
         always_send_full_costmap; do
    v=$(timeout 10 ros2 param get /local_costmap/local_costmap "$p" 2>&1 | tail -1)
    printf '%-56s %s\n' "$p" "$v"
done
echo "=== installed nav2 dwb version ==="
grep -m1 '<version>' /opt/ros/jazzy/share/dwb_critics/package.xml
