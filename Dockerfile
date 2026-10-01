# Coco Robot — ROS 2 Jazzy + Gazebo Harmonic
#
# NOTE: this image is provided for reproducibility but has not been
# runtime-tested (development happened on a native Ubuntu 24.04 install —
# see docs/RUNNING.md). GUI/GPU passthrough is up to the host
# (e.g. rocker, or -e DISPLAY -v /tmp/.X11-unix).
FROM osrf/ros:jazzy-desktop

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-ros-gz \
    ros-jazzy-gz-ros2-control \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-xacro \
    ros-jazzy-slam-toolbox \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    ros-jazzy-moveit \
    ros-jazzy-rosbridge-suite \
    ros-jazzy-web-video-server \
    ros-jazzy-rmw-cyclonedds-cpp \
    python3-pip \
    git \
    xterm \
    && rm -rf /var/lib/apt/lists/*

# RL dependencies (CPU-only torch keeps the image small)
RUN pip3 install --break-system-packages \
    torch --index-url https://download.pytorch.org/whl/cpu && \
    pip3 install --break-system-packages stable-baselines3 gymnasium

WORKDIR /ros2_ws
COPY . src/coco-robot-ros2/

RUN . /opt/ros/jazzy/setup.sh && \
    colcon build --symlink-install \
      --packages-select gazebo_models custom_teleop coco_config \
                        coco_moveit_config coco_web coco_rl

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc && \
    echo "source /ros2_ws/install/setup.bash" >> /root/.bashrc

CMD ["/bin/bash"]
