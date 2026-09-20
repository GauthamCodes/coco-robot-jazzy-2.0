# COCO 2.0 — a local robotics simulation appliance.
#
#   docker compose build && docker compose up
#   then open http://localhost:8080
#
# One container runs the whole stack: Gazebo Harmonic, ROS 2 Jazzy, Nav2,
# the mission executive, and the coco.v1 web platform. That is deliberate,
# not a shortcut -- see docs/DOCKER.md. platform_server is itself a ROS 2
# node and has to share a DDS graph with the simulator; splitting it into
# its own container buys nothing for a single-user appliance and adds a
# whole class of discovery failures.
#
# HONESTY NOTE: this image is authored but NOT runtime-tested. Docker is
# not installed on the machine this was written on, so `docker build` has
# never been executed against this file. It is a careful translation of a
# working native install (see docs/DOCKER.md for exactly what is verified
# and what is not). Treat the first build as a bring-up, not a regression.

FROM osrf/ros:jazzy-desktop

# Container paths only. Nothing here may reference a developer's
# workspace: the host path this was developed in is literally
# `~/ros2_ws(personal)`, parentheses and all, which is exactly the kind of
# thing that must never end up baked into an image.
ENV COCO_WS=/opt/coco_ws
ENV DEBIAN_FRONTEND=noninteractive

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
      ros-jazzy-web-video-server \
      ros-jazzy-rmw-cyclonedds-cpp \
      ros-jazzy-rosbridge-suite \
      ros-jazzy-rosapi \
      python3-tornado \
      # coco_web encodes JPEG for the binary sensor frames (P0.2). The
      # jazzy-desktop base does carry OpenCV, but naming it here means
      # the image does not depend on which base variant is used.
      python3-opencv \
      python3-numpy \
      python3-pip \
      curl \
      tini \
    && rm -rf /var/lib/apt/lists/*

# CPU-only torch: the ramp climb is a PPO policy, so the mission genuinely
# needs it, but the CUDA wheels would add gigabytes for a simulator that
# runs the policy in inference only.
RUN pip3 install --break-system-packages --no-cache-dir \
      torch --index-url https://download.pytorch.org/whl/cpu \
 && pip3 install --break-system-packages --no-cache-dir \
      stable-baselines3 gymnasium

WORKDIR ${COCO_WS}
COPY . src/coco-robot-ros2/

# All NINE packages. The image this replaces built six and silently left
# out coco_mission, coco_perception and coco_sim -- so the mission system
# was not in the "reproducible" image at all, and `docker compose up`
# could never have run a fetch.
RUN . /opt/ros/jazzy/setup.sh && \
    colcon build --symlink-install --packages-select \
      coco_config coco_mission coco_moveit_config coco_perception \
      coco_rl coco_sim coco_web custom_teleop gazebo_models

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# Single-machine sim: keep DDS and gz-transport on loopback. Without this
# discovery binds whatever interface the container happens to get and
# every `gz service` call waits for a timeout.
ENV CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name='lo' multicast='true'/></Interfaces></General></Domain></CycloneDDS>"
ENV GZ_IP=127.0.0.1
ENV GZ_VERSION=harmonic
# No GPU is assumed. Gazebo's sensor rendering falls back to software,
# which is slow but correct; a host with a GPU can override this.
ENV LIBGL_ALWAYS_SOFTWARE=1
ENV COCO_HTTP_PORT=8080
ENV COCO_VIDEO_PORT=8081
ENV COCO_TARGET_COLOUR=blue
ENV COCO_GUI=false
ENV COCO_RVIZ=false

RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc && \
    echo "source ${COCO_WS}/install/setup.bash" >> /root/.bashrc

COPY docker/entrypoint.sh /usr/local/bin/coco-entrypoint
RUN chmod +x /usr/local/bin/coco-entrypoint

EXPOSE 8080 8081

# /healthz answers 503 until every REQUIRED component is up, so this goes
# green when the stack has actually converged rather than when the port
# binds. start-period covers the Gazebo spawn, which is the slow part and
# is slower still on software rendering.
HEALTHCHECK --interval=10s --timeout=5s --start-period=180s --retries=30 \
  CMD curl -fsS "http://127.0.0.1:${COCO_HTTP_PORT}/healthz" || exit 1

# tini reaps the launch trees. Without an init, killing the container
# leaves gz sim and the component containers as zombies inside it, and
# the next `docker compose up` starts a second simulator on top.
ENTRYPOINT ["/usr/bin/tini", "--", "/usr/local/bin/coco-entrypoint"]
CMD ["platform"]
