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
  'ros2_control_nod[e]'
  'controller_manager[/ ]spawner'
  # nav2 / moveit / viz
  'component_container_isolate[d]'
  # Bracketed like everything else, and it was not. Rule 1 in the header
  # says every pattern is bracketed so a pattern cannot match the process
  # doing the matching; 'nav2_', 'ros2_control_node' and 'rosbridge' were
  # the three exceptions. It bites more widely than self-matching: an
  # unbracketed 'nav2_' matches ANY command line containing that
  # substring, which includes a helper script named c2nav2_up.sh and
  # includes `ros2 launch ... params_file:=<...>/nav2_params.yaml`. A
  # C2-NAV.2 bringup helper was killed by the very sweep it invoked and
  # exited before the simulator started. '[2]' matches a literal '2', so
  # every real nav2_* node still matches exactly as before.
  'nav[2]_'
  'move_grou[p]'
  'rviz[2]'
  # coco nodes and scripts
  'ramp_drive[r]'
  'target_finde[r]'
  # target_pose_node (C2-M4.0). Same rule as mission_hud: two of
  # them both publish /perception/target_pose and
  # /perception/grasp_point, and a stale one winning the race is a
  # grasp aimed where the target used to be.
  'target_pose_nod[e]'
  'approach_serve[r]'
  'grasp_serve[r]'
  # mission_hud is launched by mission.launch.py but, like the bridges
  # above, its command line does not contain "mission.launch.py". It
  # survived a sweep the day it was added and two of them then published
  # /mission/hud at once, the older one winning often enough that a
  # fixed display field looked unfixed. Anything added to a launch file
  # has to be added here too.
  'mission_hu[d]'
  # mission_executive (C2-M3) is launched by mission.launch.py and, like
  # mission_hud, its command line does not contain "mission.launch.py".
  # Two of them would both publish /mission/mode, which the arbiter
  # latches -- an orphan asserting 'rl' while the live one asks for 'nav'
  # is a robot that stops for no visible reason.
  'mission_executiv[e]'
  # localization_monitor (C2-M5.1) is launched by mission.launch.py and
  # its command line does not contain "mission.launch.py" either. Two of
  # them would both publish /localization/health, and the executive acts
  # on that topic: an orphan still asserting degraded=1 from the last run
  # would safe-stop and spin a robot whose localization is fine.
  'localization_monito[r]'
  'magnet_releas[e]'
  'traverse_dem[o]'
  # pitch_probe is an operator diagnostic, not in any launch file, but it
  # is a node that outlives a Ctrl-C in the wrong terminal like any other.
  'pitch_prob[e]'
  # terrain_observer (C2-M2.0) is the same shape: publish-only, in no
  # launch file yet, and it survives a Ctrl-C exactly like mission_hud
  # did. Two of them would publish /terrain/state at once, and since the
  # traction bound only ever tightens, the stale one would look MORE
  # confident than the live one. Listed here before it is ever launched.
  'terrain_observe[r]'
  # c2m5_locrec (C2-M5.0) is a subscribe-only recorder in docs/data, in no
  # launch file. Listed for the same reason terrain_observer is: it is a
  # node, it outlives a Ctrl-C in the wrong terminal, and an orphan of it
  # holds a half-written CSV open and keeps appending to it across the
  # NEXT run — which would silently splice two experiments into one file.
  'c2m5_locre[c]'
  # c2m51_hrec (C2-M5.1) is the same shape as c2m5_locrec: a
  # subscribe-only recorder in docs/data, in no launch file, and an
  # orphan of it holds a half-written CSV open and keeps appending to it
  # across the NEXT run.
  'c2m51_hre[c]'
  # c2m51_inject (C2-M5.1) publishes /initialpose. An orphan of this one
  # is worse than a stale recorder: it re-injects a 3 m pose error into
  # the NEXT run, and the run looks like a spontaneous divergence.
  'c2m51_injec[t]'
  'pick_plac[e]'
  'verify_si[m]'
  'map_driv[e]'
  'coco_rl[.]train_ppo'
  'coco_rl[.]evaluate'
  # web stack
  'rosbridg[e]'
  'web_video_serve[r]'
  'rosapi_nod[e]'
  # The panel's static server. Matched on --directory rather than on
  # `http.server` alone, which would also kill an unrelated `python3 -m
  # http.server` the user happened to be running in another terminal.
  # It is an ExecuteProcess, so a SIGKILLed launch parent orphans it and
  # leaves :8000 bound — after which the next run's panel never serves.
  'http[.]server.*coco_we[b]'
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
