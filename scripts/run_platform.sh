#!/usr/bin/env bash
# run_platform.sh — bring up the COCO platform with one command.
#
#   ./scripts/run_platform.sh            build if needed, then up
#   ./scripts/run_platform.sh --build    force a rebuild first
#   ./scripts/run_platform.sh --native   no Docker: run on this machine
#   ./scripts/run_platform.sh --down     stop and remove the container
#
# The Docker path is the documented one (docs/DOCKER.md). The native path
# exists because that is how this repo is actually developed, and because
# a developer who already has ROS 2 Jazzy and Gazebo Harmonic installed
# should not need a container to see the platform.
#
# No `set -u`: sourcing ROS trips nounset before anything runs.
set -o pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
HTTP_PORT="${COCO_HTTP_PORT:-8080}"
MODE=up
FORCE_BUILD=0

for arg in "$@"; do
  case "$arg" in
    --build)  FORCE_BUILD=1 ;;
    --native) MODE=native ;;
    --down)   MODE=down ;;
    -h|--help) sed -n '2,16p' "${BASH_SOURCE[0]}"; exit 0 ;;
    *) echo "unknown argument: $arg" >&2; exit 2 ;;
  esac
done

have_docker() {
  command -v docker >/dev/null 2>&1 && docker compose version >/dev/null 2>&1
}

case "$MODE" in
down)
  have_docker || { echo "docker not installed" >&2; exit 1; }
  cd "$REPO" && exec docker compose down
  ;;

native)
  # One Gazebo at a time on a machine, always. Refuse rather than start a
  # second one: a bare sweep for `gz sim` has killed an unrelated
  # project's simulator on this machine before, so this reports and stops
  # instead of cleaning up after someone else.
  if pgrep -f 'g[z] sim' >/dev/null 2>&1; then
    echo "A Gazebo simulator is already running on this machine." >&2
    echo "COCO needs a fresh one, and only one may run at a time." >&2
    echo "Stop it first (or use the Docker path, which is isolated):" >&2
    pgrep -af 'g[z] sim' | sed 's/^/  /' >&2
    exit 1
  fi
  # Start from a CLEAN package path. A login shell often already carries
  # AMENT_PREFIX_PATH entries for other workspaces, and ros_gz_sim's
  # GazeboRosPaths.get_paths() enumerates every package on it. One
  # half-installed package anywhere on that path -- an egg-link with no
  # package marker, which `colcon build` leaves behind after an
  # interrupted build -- makes every gz launch die with
  # "package 'X' not found", several layers from the cause. Measured on
  # the development machine, where a stray turtlebot3_teleop did exactly
  # that and cost two bring-ups before it was recognised.
  #
  # COCO_PRESERVE_PATH=1 opts out, for a deliberately layered overlay.
  if [ "${COCO_PRESERVE_PATH:-0}" != "1" ]; then
    unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH
    unset ROS_PACKAGE_PATH
  fi
  # shellcheck disable=SC1091
  source "$REPO/setup_env.sh" || exit 1
  echo "[coco] simulator…"
  setsid ros2 launch gazebo_models full_world_robo.launch.py \
      gui:=false traverse:=true > /tmp/coco_sim.log 2>&1 &
  SIM=$!
  # NOT `grep -q`. This script sets `pipefail`, and `grep -q` exits the
  # instant it matches -- closing the pipe, killing `ros2 topic info`
  # with EPIPE (Python exits 120), and making `pipefail` report the
  # pipeline as FAILED **exactly when the pattern was found**. The wait
  # then never breaks early and burns all 120 iterations, which on a
  # busy graph is over ten minutes of looking at "[coco] simulator…"
  # with a robot that came up long ago. Measured: exit 0 without
  # pipefail, 120 with it, on the same matching input.
  for _ in $(seq 1 120); do
    ros2 topic info /diff_drive_controller/odom 2>/dev/null \
      | grep 'Publisher count: [1-9]' >/dev/null && break
    sleep 2
  done
  echo "[coco] mission stack + web platform…"
  setsid ros2 launch coco_mission mission.launch.py \
      rviz:=false platform:=true > /tmp/coco_stack.log 2>&1 &
  STACK=$!
  trap 'kill -TERM -- "-$STACK" "-$SIM" 2>/dev/null' INT TERM
  echo "[coco] waiting for http://localhost:${HTTP_PORT}/healthz …"
  for _ in $(seq 1 60); do
    if curl -fsS "http://127.0.0.1:${HTTP_PORT}/healthz" >/dev/null 2>&1; then
      echo "[coco] drivable — open http://localhost:${HTTP_PORT}"
      break
    fi
    sleep 5
  done
  # /healthz 200 means the required components are up. It does NOT mean
  # AMCL has a pose, and a mission started in that window aborts
  # instantly with NAVIGATION_FAILED while bt_navigator logs "Initial
  # robot pose is not available" -- which reads as a mission bug and is
  # not one. Measured twice. Driving works throughout; only the mission
  # needs this, so it is reported rather than enforced.
  echo "[coco] waiting for localisation (map -> odom) …"
  for _ in $(seq 1 40); do
    # Same pipefail/`grep -q` trap as the wait above. tf2_echo streams
    # until its timeout, so -q would close the pipe on the first match.
    if timeout 6 ros2 run tf2_ros tf2_echo map odom 2>/dev/null \
         | grep 'Translation' >/dev/null; then
      echo "[coco] LOCALISED — missions can be started"
      break
    fi
  done
  wait "$STACK"
  ;;

up)
  if ! have_docker; then
    echo "docker (with the compose plugin) is not installed." >&2
    echo "Either install Docker, or run natively on a machine that" >&2
    echo "already has ROS 2 Jazzy + Gazebo Harmonic:" >&2
    echo "  ./scripts/run_platform.sh --native" >&2
    exit 1
  fi
  cd "$REPO" || exit 1
  if [ "$FORCE_BUILD" = 1 ] || ! docker image inspect coco-platform:jazzy \
       >/dev/null 2>&1; then
    echo "[coco] building the image (first build takes a while)…"
    docker compose build || exit 1
  fi
  echo "[coco] starting; the platform is ready when the container is"
  echo "[coco] healthy — watch with: docker compose ps"
  echo "[coco] then open http://localhost:${HTTP_PORT}"
  exec docker compose up
  ;;
esac
