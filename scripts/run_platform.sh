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
  # shellcheck disable=SC1091
  source "$REPO/setup_env.sh" || exit 1
  echo "[coco] simulator…"
  setsid ros2 launch gazebo_models full_world_robo.launch.py \
      gui:=false traverse:=true > /tmp/coco_sim.log 2>&1 &
  SIM=$!
  for _ in $(seq 1 120); do
    ros2 topic info /diff_drive_controller/odom 2>/dev/null \
      | grep -q 'Publisher count: [1-9]' && break
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
      echo "[coco] READY — open http://localhost:${HTTP_PORT}"
      break
    fi
    sleep 5
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
