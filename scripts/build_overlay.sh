#!/usr/bin/env bash
# build_overlay.sh — build a COCO-only colcon overlay, and nothing else.
#
#   ./scripts/build_overlay.sh                     into $HOME/coco_ws_build
#   ./scripts/build_overlay.sh DEST                into DEST
#   ./scripts/build_overlay.sh DEST --symlink-install    extra colcon args
#
# Then, in every terminal (a clean one needs nothing else first):
#
#   export COCO_WS=DEST
#   source <this repo>/setup_env.sh
#
# Why not `cd <ws> && colcon build`: that builds every package under
# <ws>/src into <ws>/install. On the development machine that included
# turtlebot3 sources, whose --symlink-install markers outlived a rename of
# the workspace directory as dangling symlinks, and ros_gz_sim then killed
# every gz launch with "package 'turtlebot3_teleop' not found". Here
# --base-paths confines discovery to this repository, the package path
# starts clean, and the default is a COPYING install, so the overlay does
# not break when a source tree is moved or renamed -- the failure that
# started this. Pass --symlink-install for a development overlay.
#
# No `set -u`: sourcing ROS trips nounset before anything runs.
set -o pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DEST="${1:-$HOME/coco_ws_build}"
[ "$#" -gt 0 ] && shift
mkdir -p "$DEST" || exit 1
DEST="$(cd "$DEST" && pwd)"

# Start from a clean package path: whatever a login shell layered on
# (~/.bashrc sourcing another workspace, say) would otherwise be frozen
# into this overlay's setup.bash as an underlay.
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH
unset ROS_PACKAGE_PATH PYTHONPATH
export COCO_WS="$DEST"
# shellcheck disable=SC1091
source "$REPO/setup_env.sh" || exit 1

echo "[build_overlay] $REPO -> $DEST"
cd "$DEST" || exit 1
exec colcon --log-base "$DEST/log" build \
    --base-paths "$REPO" \
    --build-base "$DEST/build" \
    --install-base "$DEST/install" \
    "$@"
