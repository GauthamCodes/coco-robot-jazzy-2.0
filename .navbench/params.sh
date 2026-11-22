#!/usr/bin/env bash
# The parameters the running controller actually holds, as opposed to the
# ones the yaml file appears to set.
HERE="$(cd "$(dirname "$0")" && pwd)"
source "$HERE/env.sh"
for p in FollowPath.xy_goal_tolerance FollowPath.yaw_goal_tolerance \
         FollowPath.trans_stopped_velocity FollowPath.RotateToGoal.scale \
         FollowPath.RotateToGoal.slowing_factor \
         FollowPath.RotateToGoal.lookahead_time \
         FollowPath.BaseObstacle.scale FollowPath.BaseObstacle.sum_scores \
         FollowPath.Oscillation.scale \
         FollowPath.Oscillation.oscillation_reset_dist \
         FollowPath.Oscillation.oscillation_reset_angle \
         FollowPath.Oscillation.x_only_threshold \
         FollowPath.publish_evaluation FollowPath.publish_trajectories \
         FollowPath.vx_samples FollowPath.vtheta_samples \
         FollowPath.sim_time FollowPath.max_vel_x \
         goal_checker.xy_goal_tolerance goal_checker.yaw_goal_tolerance \
         goal_checker.stateful \
         progress_checker.required_movement_radius \
         progress_checker.movement_time_allowance \
         controller_frequency failure_tolerance; do
    v=$(timeout 10 ros2 param get /controller_server "$p" 2>&1 | tail -1)
    printf '%-46s %s\n' "$p" "$v"
done
