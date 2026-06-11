# Dockerfile for Coco Robot ROS2 Development
FROM osrf/ros:humble-desktop

# Install dependencies
RUN apt-get update && apt-get install -y \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-gazebo-ros2-control \
    ros-humble-diff-drive-controller \
    ros-humble-joint-state-broadcaster \
    ros-humble-forward-command-controller \
    ros-humble-robot-state-publisher \
    ros-humble-xacro \
    python3-pip \
    git \
    vim \
    && rm -rf /var/lib/apt/lists/*

# Create workspace
WORKDIR /workspace
COPY . /workspace/

# Build workspace
RUN . /opt/ros/humble/setup.sh && \
    colcon build --symlink-install

# Source workspace in bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /workspace/install/setup.bash" >> ~/.bashrc

CMD ["/bin/bash"]
