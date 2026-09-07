#!/usr/bin/env bash
# C2-NAV.33 live readback of the ONE experimental variable, off the
# running /amcl. Pure observation: it starts no node, writes no
# parameter and changes no behaviour.
#
# It exists for the reason c2n23_liveparam.sh exists, and the reason is
# stronger here. Nothing in the existing runner reads ANY amcl
# parameter -- c2n6_verify.sh covers collision_monitor, the costmaps,
# controller_server and planner_server, and stops there. A file that was
# edited and a file that was LOADED are different claims, and this
# session's entire diagnostic effect depends on the second one.
#
# It also guards a specific hazard found in the binary while writing the
# gate. `AmclNode::initParameters` contains a `movl $0x1, 0xac8(%r12)`
# -- a literal 1 written to resample_interval_ -- reached through a
# WARN-severity logging block, i.e. the fallback taken when the
# parameter is absent or unusable. A silent fall back to 1 would leave
# the published clouds flat and look exactly like "the weights are
# unbiased". This readback is what distinguishes those two, together
# with the alternation test in `c2nav33_weights.py phase`.
#
#   c2nav33_liveparam.sh <tag>
#
# Naming: no string here contains ros_clean.sh's 'nav[2]_' pattern --
# bracketed here for the same reason the pattern is bracketed there, so
# this comment is not itself a match. "c2nav33_" is "nav3" then "3".
HERE="$(cd "$(dirname "$0")" && pwd)"
WT="$(cd "$HERE/../.." && pwd)"
NB="$WT/.navbench"
source "$NB/env.sh"
TAG="${1:?tag}"
mkdir -p "$NB/results"
OUT="$NB/results/${TAG}_amcl_live.txt"

# Wait for /amcl to be active. The parameter server does not answer
# before configuration, and a timeout that reads nothing must not be
# mistaken for a parameter that reads wrong.
UP=no
for i in $(seq 1 600); do
    st=$(timeout 5 ros2 lifecycle get /amcl 2>/dev/null)
    echo "$st" | grep -q active && { UP=yes; break; }
    sleep 2
done

{
  echo "tag: $TAG"
  echo "read at: $(date -u +%Y-%m-%dT%H:%M:%SZ) UTC"
  echo "amcl reached active: $UP"
  if [ "$UP" != "yes" ]; then
      echo "NO READBACK -- /amcl never reported active."
      echo "Treat every line below as ABSENT, not as a measured value."
  fi
  echo "--- THE SINGLE C2-NAV.33 VARIABLE (expect: 2) ---"
  printf '  amcl resample_interval                    '
  timeout 8 ros2 param get /amcl resample_interval 2>&1 | tail -1
  echo "--- what must NOT have moved ---"
  for p in alpha1 alpha2 alpha3 alpha4 alpha5 \
           update_min_d update_min_a \
           laser_model_type laser_likelihood_max_dist laser_max_range \
           laser_min_range max_beams sigma_hit \
           z_hit z_rand z_max z_short lambda_short \
           do_beamskip beam_skip_distance beam_skip_error_threshold \
           beam_skip_threshold \
           min_particles max_particles pf_err pf_z \
           recovery_alpha_slow recovery_alpha_fast \
           robot_model_type transform_tolerance save_pose_rate \
           tf_broadcast base_frame_id odom_frame_id global_frame_id; do
      printf '  amcl %-40s ' "$p"
      timeout 8 ros2 param get /amcl "$p" 2>&1 | tail -1
  done
  echo "--- and the navigation leaves this session may not touch ---"
  printf '  collision_monitor PolygonSlow.slowdown_ratio   '
  timeout 8 ros2 param get /collision_monitor PolygonSlow.slowdown_ratio 2>&1 | tail -1
  printf '  collision_monitor PolygonStop.radius           '
  timeout 8 ros2 param get /collision_monitor PolygonStop.radius 2>&1 | tail -1
  printf '  controller_server FollowPath.xy_goal_tolerance '
  timeout 8 ros2 param get /controller_server FollowPath.xy_goal_tolerance 2>&1 | tail -1
} | tee "$OUT"
echo "wrote $OUT"
