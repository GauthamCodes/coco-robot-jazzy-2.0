#!/usr/bin/env bash
# C2-NAV.23 live readback of the ONE experimental variable, off the
# running controller_server. Pure observation: it starts no node, writes
# no parameter and changes no behaviour. It exists because
# c2n6_verify.sh reads goal_checker.xy_goal_tolerance and NOT
# FollowPath.xy_goal_tolerance, so nothing in the existing runner proves
# the candidate file's single edited line was actually LOADED. A file
# that was edited and a file that was loaded are different claims.
#
#   c2n23_liveparam.sh <tag>
#
# Naming: no string here contains "nav2_", which ros_clean.sh's 'nav[2]_'
# pattern would match.
HERE="$(cd "$(dirname "$0")" && pwd)"
source "$HERE/env.sh"
TAG="${1:?tag}"
mkdir -p "$HERE/results"
OUT="$HERE/results/${TAG}_fpxy_live.txt"
for i in $(seq 1 600); do
    st=$(timeout 5 ros2 lifecycle get /controller_server 2>/dev/null)
    echo "$st" | grep -q active && break
    sleep 2
done
{
  echo "tag: $TAG"
  echo "read at: $(date -u +%Y-%m-%dT%H:%M:%SZ) UTC"
  echo "--- THE SINGLE C2-NAV.23 VARIABLE ---"
  printf '  controller_server FollowPath.xy_goal_tolerance                  '
  timeout 8 ros2 param get /controller_server FollowPath.xy_goal_tolerance 2>&1 | tail -1
  echo "--- what must NOT have moved ---"
  printf '  controller_server goal_checker.xy_goal_tolerance                '
  timeout 8 ros2 param get /controller_server goal_checker.xy_goal_tolerance 2>&1 | tail -1
  printf '  controller_server goal_checker.yaw_goal_tolerance               '
  timeout 8 ros2 param get /controller_server goal_checker.yaw_goal_tolerance 2>&1 | tail -1
  printf '  controller_server FollowPath.sim_time                           '
  timeout 8 ros2 param get /controller_server FollowPath.sim_time 2>&1 | tail -1
  printf '  controller_server FollowPath.PathAlign.forward_point_distance   '
  timeout 8 ros2 param get /controller_server FollowPath.PathAlign.forward_point_distance 2>&1 | tail -1
  printf '  controller_server FollowPath.GoalAlign.forward_point_distance   '
  timeout 8 ros2 param get /controller_server FollowPath.GoalAlign.forward_point_distance 2>&1 | tail -1
  printf '  controller_server FollowPath.critics                            '
  timeout 8 ros2 param get /controller_server FollowPath.critics 2>&1 | tail -1
  printf '  controller_server FollowPath.RotateToGoal.scale                 '
  timeout 8 ros2 param get /controller_server FollowPath.RotateToGoal.scale 2>&1 | tail -1
  printf '  controller_server FollowPath.max_vel_x                          '
  timeout 8 ros2 param get /controller_server FollowPath.max_vel_x 2>&1 | tail -1
  printf '  controller_server FollowPath.max_vel_theta                      '
  timeout 8 ros2 param get /controller_server FollowPath.max_vel_theta 2>&1 | tail -1
  printf '  controller_server FollowPath.vx_samples                         '
  timeout 8 ros2 param get /controller_server FollowPath.vx_samples 2>&1 | tail -1
  printf '  controller_server FollowPath.vtheta_samples                     '
  timeout 8 ros2 param get /controller_server FollowPath.vtheta_samples 2>&1 | tail -1
  printf '  collision_monitor PolygonStop.radius                            '
  timeout 8 ros2 param get /collision_monitor PolygonStop.radius 2>&1 | tail -1
  printf '  collision_monitor PolygonSlow.slowdown_ratio                    '
  timeout 8 ros2 param get /collision_monitor PolygonSlow.slowdown_ratio 2>&1 | tail -1
} | tee "$OUT"
echo "wrote $OUT"
