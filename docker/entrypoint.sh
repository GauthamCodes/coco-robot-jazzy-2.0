#!/usr/bin/env bash
# COCO 2.0 container entrypoint.
#
#   coco-entrypoint platform   the appliance: simulator + mission stack +
#                              web platform. This is the default.
#   coco-entrypoint shell      an interactive shell with the overlay
#                              sourced, for debugging.
#   coco-entrypoint <cmd...>   run anything else with the overlay sourced.
#
# Deliberately no `set -u`: /opt/ros/jazzy/setup.bash and the workspace
# overlay it chains into reference unbound variables (AMENT_TRACE_SETUP_
# FILES among them), so nounset kills this before a single node starts.
# `set -e` is also omitted -- this script's job when something dies is to
# say so and tear the rest down, not to vanish.
set -o pipefail

log() { echo "[coco] $*"; }
die() { echo "[coco] FATAL: $*" >&2; exit 1; }

COCO_WS="${COCO_WS:-/opt/coco_ws}"
HTTP_PORT="${COCO_HTTP_PORT:-8080}"
VIDEO_PORT="${COCO_VIDEO_PORT:-8081}"
TARGET_COLOUR="${COCO_TARGET_COLOUR:-blue}"
GUI="${COCO_GUI:-false}"
RVIZ="${COCO_RVIZ:-false}"

# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash || die "no ROS 2 Jazzy in this image"
# shellcheck disable=SC1091
source "${COCO_WS}/install/setup.bash" \
  || die "no workspace overlay at ${COCO_WS}/install — the image did not build"

SIM_PID=""
STACK_PID=""

# Tearing down a `ros2 launch` tree needs more than killing the launcher:
# the bridge, robot_state_publisher and the controller spawners survive
# and keep publishing TF stamped with the OLD sim clock. Each launch runs
# via setsid, in its own process group, and we kill the group.
stop_all() {
  log "shutting down"
  for pid in "$STACK_PID" "$SIM_PID"; do
    [ -n "$pid" ] && kill -TERM -- "-$pid" 2>/dev/null
  done
  sleep 2
  for pid in "$STACK_PID" "$SIM_PID"; do
    [ -n "$pid" ] && kill -9 -- "-$pid" 2>/dev/null
  done
}
trap 'stop_all; exit 0' TERM INT

# Run in this shell: command substitution would orphan the PID from wait.
launch_bg() {
  local logfile="$1"; shift
  setsid ros2 launch "$@" > "$logfile" 2>&1 &
  LAUNCHED_PID=$!
}

# Wait for a topic to have a publisher. Used to sequence the two launches:
# starting the mission stack before the simulator publishes a clock means
# every use_sim_time node waits on a /clock that is not there yet, and
# Nav2's lifecycle times out in a way that reads like a Nav2 bug.
#
# NOT `grep -q`. This script sets `pipefail`, and `grep -q` exits the
# instant it matches -- which closes the pipe, kills `ros2 topic info`
# with EPIPE (Python exits 120), and makes `pipefail` report the whole
# pipeline as FAILED **precisely when the pattern was found**. The check
# then never succeeds and the caller waits out its full timeout.
# Measured on the native path: exit 0 without pipefail, exit 120 with it,
# on the same matching input. Without `-q`, grep reads the stream to the
# end, so the writer never sees EPIPE.
wait_for_topic() {
  local topic="$1" timeout="${2:-180}" waited=0
  while [ "$waited" -lt "$timeout" ]; do
    if ros2 topic info "$topic" 2>/dev/null \
         | grep 'Publisher count: [1-9]' >/dev/null; then
      return 0
    fi
    sleep 2
    waited=$((waited + 2))
  done
  return 1
}

run_platform() {
  log "COCO 2.0 appliance starting"
  log "  workspace   ${COCO_WS}"
  log "  http        :${HTTP_PORT}   video :${VIDEO_PORT}"
  log "  gui=${GUI} rviz=${RVIZ} colour=${TARGET_COLOUR}"

  log "1/3 simulator (gazebo_models full_world_robo.launch.py)"
  launch_bg /tmp/coco_sim.log gazebo_models \
    full_world_robo.launch.py "gui:=${GUI}" traverse:=true
  SIM_PID=$LAUNCHED_PID

  # Both odometry sources, for the reason verify_all.sh gives: the gz
  # plugin's /model/coco/odometry appears well before ros2_control
  # finishes activating, so waiting only for it races the controllers.
  if ! wait_for_topic /model/coco/odometry 240; then
    log "simulator never published /model/coco/odometry; last log lines:"
    tail -40 /tmp/coco_sim.log >&2
    die "simulator did not come up"
  fi
  log "    simulator clock and model odometry are up"
  if ! wait_for_topic /diff_drive_controller/odom 180; then
    log "controllers never activated; last log lines:"
    tail -40 /tmp/coco_sim.log >&2
    die "ros2_control did not activate"
  fi
  log "    controllers active"

  log "2/3 mission stack + web platform (coco_mission mission.launch.py)"
  launch_bg /tmp/coco_stack.log coco_mission mission.launch.py \
    "rviz:=${RVIZ}" "target_colour:=${TARGET_COLOUR}" \
    platform:=true web:=true
  STACK_PID=$LAUNCHED_PID

  log "3/3 waiting for the platform to report ready on /healthz"
  # This does NOT gate the container's health -- HEALTHCHECK does, and it
  # is the single source of truth. This loop exists so the LOG says what
  # is missing instead of going quiet for three minutes.
  local waited=0
  while [ "$waited" -lt 240 ]; do
    if curl -fsS "http://127.0.0.1:${HTTP_PORT}/healthz" >/dev/null 2>&1; then
      log "READY — open http://localhost:${HTTP_PORT}"
      break
    fi
    local body
    body=$(curl -s "http://127.0.0.1:${HTTP_PORT}/healthz" 2>/dev/null)
    if [ -n "$body" ]; then
      log "    not ready yet: $(echo "$body" | tr -d '\n ' | head -c 200)"
    else
      log "    web platform has not bound :${HTTP_PORT} yet"
    fi
    sleep 5
    waited=$((waited + 5))
  done
  [ "$waited" -ge 240 ] && log "WARNING: still not ready after 240 s; \
the container stays up so you can inspect it (docker compose logs)"

  # Hold the container open on the stack. If the stack dies, so should we
  # -- a container that outlives its own robot is a container that looks
  # healthy in `docker ps` and answers nothing.
  wait "$STACK_PID"
  log "mission stack exited"
  stop_all
}

case "${1:-platform}" in
  platform) run_platform ;;
  shell)    exec /bin/bash ;;
  *)        exec "$@" ;;
esac
