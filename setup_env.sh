# setup_env.sh — source this in every terminal before running anything.
#
#   source ~/ros2_ws/src/coco-robot-ros2/setup_env.sh
#
# Sets up: ROS 2 Jazzy + workspace overlay, CycloneDDS on loopback,
# Gazebo Harmonic, the user-space MoveIt/rosbridge prefix (if present),
# and a render-engine fallback for when the NVIDIA driver is not loaded.

# Locate the colcon workspace from this script's own path rather than
# assuming ~/ros2_ws: this file lives at <ws>/src/coco-robot-ros2/, so the
# workspace root is two directories up. Without this, a clone anywhere
# else sources ROS but silently never sources its own overlay, and every
# `ros2 launch` then fails with "package not found" for no visible reason.
# Overridable: C2-NAV.40 measured that this derivation is WRONG from a git
# worktree (it lands on .claude/), and C2-NAV.48 needs to point a run at an
# isolated overlay because <ws>/install carries stale turtlebot3 prefixes
# whose ament index markers are dangling symlinks, which makes ros_gz_sim's
# GazeboRosPaths.get_paths() enumeration throw and kills every gz launch.
# Export COCO_WS to choose the overlay; scripts/build_overlay.sh builds one.
_COCO_REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
_COCO_SRC_WS="$(cd "$_COCO_REPO/../.." && pwd)"
COCO_WS="${COCO_WS:-$_COCO_SRC_WS}"

if [ ! -f /opt/ros/jazzy/setup.bash ]; then
    echo "[setup_env] ERROR: /opt/ros/jazzy/setup.bash not found." >&2
    echo "[setup_env] This project targets ROS 2 Jazzy; install it with" >&2
    echo "[setup_env]   sudo apt install ros-jazzy-desktop" >&2
    echo "[setup_env] (see README.md prerequisites)." >&2
    return 1 2>/dev/null || exit 1
fi
source /opt/ros/jazzy/setup.bash

# The overlay's OWN packages: local_setup.bash, not setup.bash. colcon's
# setup.bash re-sources every underlay that was on the path when the
# overlay was BUILT, frozen into the file -- measured: <ws>/install's
# chained $HOME/ros2_ws/install, an unrelated workspace ~/.bashrc had on the
# path at build time, so even `bash --noprofile --norc` acquired it. ROS is
# sourced just above and the MoveIt prefix just below, so nothing COCO
# needs is lost; a deliberate underlay still works if sourced before this.
if [ -f "$COCO_WS/install/local_setup.bash" ]; then
    source "$COCO_WS/install/local_setup.bash"
else
    echo "[setup_env] note: no overlay at $COCO_WS/install — run colcon build" >&2
fi

export GZ_VERSION=harmonic

# DDS: CycloneDDS on loopback for single-machine sim
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo" multicast="true"/></Interfaces></General></Domain></CycloneDDS>'

# gz-transport on loopback too: without this, `gz service`/`gz topic`
# discovery binds the WiFi interface and every call times out the moment
# the network drops (single-machine sim should never depend on WiFi).
export GZ_IP=127.0.0.1

# Render engine: prefer the NVIDIA dGPU when its driver is loaded, else
# fall back to Mesa (Intel iGPU). Forcing the NVIDIA EGL vendor while the
# driver is down makes gz-sim segfault in driCreateNewScreen3.
if nvidia-smi >/dev/null 2>&1; then
    export __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json
else
    export __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/50_mesa.json
    echo "[setup_env] NVIDIA driver not loaded — using Mesa/iGPU rendering" >&2
fi

# Interpreter version for the site-packages paths below. Hardcoding
# python3.12 silently drops the pip --user installs (torch, SB3, tornado)
# off PYTHONPATH on any other interpreter, so they appear uninstalled.
COCO_PYVER="$(python3 -c 'import sys; print(f"python{sys.version_info.major}.{sys.version_info.minor}")')"

# User-space deb prefix for MoveIt2 + rosbridge + web_video_server
# (created because apt/sudo was unavailable; harmless if you have since
# installed the real ros-jazzy-moveit / ros-jazzy-rosbridge-suite debs,
# in which case you can delete <ws>/moveit_prefix entirely).
# It lives in the SOURCE workspace; an isolated overlay selected with
# COCO_WS has none, and without this fallback would run without MoveIt.
MV=""
for _ws in "$COCO_WS" "$_COCO_SRC_WS"; do
    if [ -d "$_ws/moveit_prefix/root/opt/ros/jazzy" ]; then
        MV="$_ws/moveit_prefix/root/opt/ros/jazzy"
        break
    fi
done
unset _ws
if [ -n "$MV" ]; then
    export AMENT_PREFIX_PATH="$MV:$AMENT_PREFIX_PATH"
    export CMAKE_PREFIX_PATH="$MV:$CMAKE_PREFIX_PATH"
    export LD_LIBRARY_PATH="$MV/lib:$MV/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH"
    export PYTHONPATH="$MV/lib/$COCO_PYVER/site-packages:$PYTHONPATH"
    export PATH="$MV/bin:$PATH"
fi

# pip --user packages (tornado/cbor2 for rosbridge, torch/sb3 for RL)
export PYTHONPATH="$HOME/.local/lib/$COCO_PYVER/site-packages:$PYTHONPATH"

# Preflight. A package LISTED on AMENT_PREFIX_PATH that cannot be RESOLVED
# kills every gz launch, naming a package COCO never asked for ("package
# 'turtlebot3_teleop' not found", four layers from the cause). Say so here,
# one line per package. Warns only; it never edits the path.
python3 "$_COCO_REPO/scripts/check_ament_path.py" || true
unset _COCO_REPO _COCO_SRC_WS
