#!/usr/bin/env bash
# envrun.sh <repo> <overlay_ws> <cmd...> -- run a command in the COCO env.
REPO="$1"; WS="$2"; shift 2
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH ROS_PACKAGE_PATH PYTHONPATH
export COCO_WS="$WS"
# shellcheck disable=SC1091
source "$REPO/setup_env.sh" >/dev/null 2>&1
exec "$@"
