# setup_env.sh — source this in every terminal before running anything.
#
#   source ~/ros2_ws/src/coco-robot-ros2/setup_env.sh
#
# Sets up: ROS 2 Jazzy + workspace overlay, CycloneDDS on loopback,
# Gazebo Harmonic, the user-space MoveIt/rosbridge prefix (if present),
# and a render-engine fallback for when the NVIDIA driver is not loaded.

source /opt/ros/jazzy/setup.bash
[ -f "$HOME/ros2_ws/install/setup.bash" ] && source "$HOME/ros2_ws/install/setup.bash"

export GZ_VERSION=harmonic

# DDS: CycloneDDS on loopback for single-machine sim
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo" multicast="true"/></Interfaces></General></Domain></CycloneDDS>'

# Render engine: prefer the NVIDIA dGPU when its driver is loaded, else
# fall back to Mesa (Intel iGPU). Forcing the NVIDIA EGL vendor while the
# driver is down makes gz-sim segfault in driCreateNewScreen3.
if nvidia-smi >/dev/null 2>&1; then
    export __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json
else
    export __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/50_mesa.json
    echo "[setup_env] NVIDIA driver not loaded — using Mesa/iGPU rendering" >&2
fi

# User-space deb prefix for MoveIt2 + rosbridge + web_video_server
# (created because apt/sudo was unavailable; harmless if you have since
# installed the real ros-jazzy-moveit / ros-jazzy-rosbridge-suite debs,
# in which case you can delete ~/ros2_ws/moveit_prefix entirely).
MV="$HOME/ros2_ws/moveit_prefix/root/opt/ros/jazzy"
if [ -d "$MV" ]; then
    export AMENT_PREFIX_PATH="$MV:$AMENT_PREFIX_PATH"
    export CMAKE_PREFIX_PATH="$MV:$CMAKE_PREFIX_PATH"
    export LD_LIBRARY_PATH="$MV/lib:$MV/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH"
    export PYTHONPATH="$MV/lib/python3.12/site-packages:$PYTHONPATH"
    export PATH="$MV/bin:$PATH"
fi

# pip --user packages (tornado/cbor2 for rosbridge, torch/sb3 for RL)
export PYTHONPATH="$HOME/.local/lib/python3.12/site-packages:$PYTHONPATH"
