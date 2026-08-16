#!/usr/bin/env bash
# Copyright 2026 Gautham Anil
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# ros_clean.sh — tear down everything a Coco sim run leaves behind.
#
# WHY THIS EXISTS, measured: three consecutive traverse runs in M5 failed
# with Nav2 rejecting every goal ("bt_navigator: Action server is inactive"),
# and Nav2 was not the fault. `ros2 launch gazebo_models
# full_world_robo.launch.py` spawns parameter_bridge, robot_state_publisher
# and cmd_vel_relay as SEPARATE processes whose command lines do not contain
# "full_world_robo", so killing the launch pattern leaves them running.
# Six orphaned bridges accumulated across those attempts, and a stale /clock
# publisher makes every consumer see time jump backwards:
#
#   tf2_buffer: Detected jump back in time. Clearing TF buffer.
#   global_costmap: 'map' and 'base_footprint' are not part of the same tree.
#   amcl: Message Filter dropping message ... queue is full
#
# AMCL then never updates, map->odom expires, the global costmap never
# finishes activating, bt_navigator is never activated, and it rejects goals
# — four layers away from the actual fault. Each run is WORSE than the last;
# that is the tell. Kill by PROCESS name, not by launch-file name.
#
# TWO RULES this file obeys, and so should anything that copies it:
#   1. Every pattern is bracketed ('full_world_rob[o]'), so a pattern can
#      never match the process doing the matching.
#   2. It is a FILE. Run from `bash -c`, the shell's own command line
#      contains the whole script text, and it kills itself mid-sweep.
#
# Usage:
#   ./ros_clean.sh          # sweep, then report what is left
#   ./ros_clean.sh --list   # show what WOULD be killed, kill nothing

set -o pipefail

LIST_ONLY=0
[ "${1:-}" = "--list" ] && LIST_ONLY=1

# Bracketed so none of these can match this script's own `pkill`.
PATTERNS=(
  # launch trees
  'full_world_rob[o].launch.py'
  'nav[.]launch.py'
  'mission[.]launch.py'
  'slam[.]launch.py'
  'web[.]launch.py'
  'perception[.]launch.py'
  'arbiter[.]launch.py'
  'move_group[.]launch.py'
  'teleop[.]launch.py'
  'rsp[.]launch.py'
  # the simulator itself; gz sim is a ruby launcher wrapping the server
  'g[z] sim'
  # the orphans that started all of this
  'parameter_bridg[e]'
  'robot_state_publishe[r]'
  'cmd_vel_rela[y]'
  'cmd_vel_arbite[r]'
  # ros2_control
  'controller_manage[r]'
  'ros2_control_node'
  'controller_manager[/ ]spawner'
  # nav2 / moveit / viz
  'component_container_isolate[d]'
  'nav2_'
  'move_grou[p]'
  'rviz[2]'
  # coco nodes and scripts
  'ramp_drive[r]'
  'target_finde[r]'
  'approach_serve[r]'
  'grasp_serve[r]'
  # mission_hud is launched by mission.launch.py but, like the bridges
  # above, its command line does not contain "mission.launch.py". It
  # survived a sweep the day it was added and two of them then published
  # /mission/hud at once, the older one winning often enough that a
  # fixed display field looked unfixed. Anything added to a launch file
  # has to be added here too.
  'mission_hu[d]'
  'magnet_releas[e]'
  'traverse_dem[o]'
  'pick_plac[e]'
  'verify_si[m]'
  'map_driv[e]'
  'coco_rl[.]train_ppo'
  'coco_rl[.]evaluate'
  # web stack
  'rosbridge'
  'web_video_serve[r]'
)

survivors() {
  local found=()
  for pat in "${PATTERNS[@]}"; do
    while read -r pid; do
      [ -n "$pid" ] && found+=("$pid")
    done < <(pgrep -f "$pat" 2>/dev/null)
  done
  printf '%s\n' "${found[@]}" | grep -v '^$' | sort -u
}

report() {
  local pids
  pids=$(survivors)
  [ -z "$pids" ] && return 0
  for pid in $pids; do
    printf '  %-8s %s\n' "$pid" "$(tr '\0' ' ' < "/proc/$pid/cmdline" 2>/dev/null | cut -c1-100)"
  done
}

if [ "$LIST_ONLY" = 1 ]; then
  echo "would kill:"
  report
  exit 0
fi

before=$(survivors | wc -l)

for sig in TERM TERM KILL; do
  for pat in "${PATTERNS[@]}"; do
    pkill "-$sig" -f "$pat" 2>/dev/null
  done
  # gz sim can take a moment to unwind its render thread; give each pass
  # time to land before escalating rather than SIGKILLing a healthy exit.
  sleep 2
done

left=$(survivors | wc -l)
echo "ros_clean: ${before} matched, ${left} still running"
if [ "$left" -gt 0 ]; then
  echo "still up:"
  report
  exit 1
fi
exit 0
