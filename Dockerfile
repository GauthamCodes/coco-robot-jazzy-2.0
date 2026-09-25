# COCO 2.0 -- one image for the local appliance, the test suite and
# headless RL.
#
#   docker compose build                    # the image
#   docker compose up                       # the appliance, http://localhost:8080
#   docker compose run --rm coco test       # every package's test suite
#   docker compose run --rm coco info       # what this image was built from
#
# or, for an image whose content is a function of a commit and nothing
# else (clean-room export, metadata filled in):
#
#   scripts/container/build.sh --ref HEAD
#
# One container runs the whole stack: Gazebo Harmonic, ROS 2 Jazzy, Nav2,
# the mission executive, and the coco.v1 web platform. That is deliberate,
# not a shortcut -- see docs/DOCKER.md. platform_server is itself a ROS 2
# node and has to share a DDS graph with the simulator; splitting it into
# its own container buys nothing for a single-user appliance and adds a
# whole class of discovery failures.
#
# ONE image, not runtime/test/dev stages, and that is measured rather than
# assumed (docs/DOCKER.md, "Why one image"): everything the test suite
# needs beyond the appliance is MuJoCo and Node, a rounding error on this
# image, and one image means the digest CI tested is the digest that runs.
# A builder stage would save nothing either: the colcon output is under
# 5 MB and the compilers live in the base layers.

# The base is pinned by DIGEST. The tag moves on every OSRF rebuild, and
# with it every ROS package the base carries; the digest is what the
# measurements in docs/DOCKER.md were taken on. Refresh it deliberately:
# change this line, rebuild, re-run scripts/container/validate.sh.
ARG BASE_IMAGE=osrf/ros:jazzy-desktop@sha256:2f520187e84304fffb60d19c9a7d8e2e79d563a6e579a090427e5994eb1b8c45
FROM ${BASE_IMAGE}
ARG BASE_IMAGE

# Container paths only. Nothing here may reference a developer's
# workspace: the host path this was developed in is literally
# `~/ros2_ws(personal)`, parentheses and all, which is exactly the kind of
# thing that must never end up baked into an image.
ENV COCO_WS=/opt/coco_ws
ENV DEBIAN_FRONTEND=noninteractive

# apt. The debs are kept in a BuildKit cache mount, NOT in the image, so a
# changed package list re-downloads only what changed: measured, the cold
# download of this layer is the single longest step of a clean build. The
# base's docker-clean hook would delete them, so it is set aside for the
# install and put back after it.
#
# Not version-pinned, deliberately: packages.ros.org keeps only the latest
# sync, so an exact-version pin stops resolving the day ROS syncs again.
# What was installed is recorded instead (/opt/coco/manifest/dpkg.txt) so
# drift between two builds is a diff, not a mystery.
#
# The list is what `rosdep check --from-paths src` needs plus what nothing
# declares; scripts/container/validate.sh re-runs that check in the built
# image and fails on anything unsatisfied.
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt/lists,sharing=locked \
    mv /etc/apt/apt.conf.d/docker-clean /etc/apt/docker-clean.off && \
    echo 'Binary::apt::APT::Keep-Downloaded-Packages "true";' \
      > /etc/apt/apt.conf.d/keep-cache && \
    apt-get update && apt-get install -y --no-install-recommends \
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
      # gazebo_models exec_depends, used by nav.launch.py depth_cloud:=true.
      # The image this replaces left them out; rosdep check caught it.
      ros-jazzy-depth-image-proc \
      ros-jazzy-image-proc \
      # custom_teleop's declared exec_depend (its launch file opens one).
      xterm \
      python3-tornado \
      # coco_web encodes JPEG for the binary sensor frames (P0.2). The
      # jazzy-desktop base does carry OpenCV, but naming it here means
      # the image does not depend on which base variant is used.
      python3-opencv \
      python3-numpy \
      # localization_monitor builds its likelihood field with
      # scipy.ndimage on the first /map. No package.xml declares it; the
      # desktop base happens to carry it. Named so a slimmer base cannot
      # silently drop the localization health check.
      python3-scipy \
      python3-pip \
      # coco_web's Node decoder test (a declared test_depend). Noble ships
      # 18.x; the test needs >= 18.
      nodejs \
      # `ss`, which docs/DOCKER.md's port check uses.
      iproute2 \
      # A virtual display for the tools that insist on one. The mission
      # runner the colour matrix is measured with forces rviz:=true, and
      # RViz's OGRE needs a GLX context, which Qt's offscreen platform does
      # not give it. `xvfb-run -a <cmd>`; see scripts/container/.
      xvfb \
      xauth \
      curl \
      tini && \
    rm /etc/apt/apt.conf.d/keep-cache && \
    mv /etc/apt/docker-clean.off /etc/apt/apt.conf.d/docker-clean

# pip: exactly docker/pip-constraints.txt, then prove it. torch comes from
# the CPU index -- the mission runs the ramp policy in inference only, and
# the CUDA wheels would add gigabytes. The build FAILS if the installed pip
# set differs from the lock by a name or a version.
COPY docker/pip-constraints.txt docker/check_pip_lock.py /opt/coco/pip/
RUN --mount=type=cache,target=/root/.cache/pip \
    pip3 install --break-system-packages \
      --index-url https://download.pytorch.org/whl/cpu \
      -c /opt/coco/pip/pip-constraints.txt torch \
 && pip3 install --break-system-packages \
      -c /opt/coco/pip/pip-constraints.txt \
      stable-baselines3 gymnasium cloudpickle mujoco tornado \
 && python3 /opt/coco/pip/check_pip_lock.py /opt/coco/pip/pip-constraints.txt

# What the image contains, recorded in the image. Reproducibility metadata:
# two images with equal manifests carry the same apt and pip versions.
RUN mkdir -p /opt/coco/manifest && \
    dpkg-query -W -f='${Package}=${Version}\n' | sort \
      > /opt/coco/manifest/dpkg.txt && \
    pip3 list --format=freeze 2>/dev/null | sort \
      > /opt/coco/manifest/pip.txt

# Least privilege. The runtime user owns its home and nothing else: the
# workspace stays root-owned, so the running stack cannot modify its own
# code. uid 1000 so a bind-mounted evidence directory is writable by the
# usual first user on a Linux host. Noble's base image ships a user
# `ubuntu` holding uid 1000; it is removed first.
RUN (userdel -r ubuntu 2>/dev/null || true) && \
    groupadd --gid 1000 coco && \
    useradd --uid 1000 --gid 1000 --create-home --shell /bin/bash coco

WORKDIR ${COCO_WS}
COPY . src/coco-robot-ros2/

# All NINE packages. The image this replaces built six and silently left
# out coco_mission, coco_perception and coco_sim -- so the mission system
# was not in the "reproducible" image at all, and `docker compose up`
# could never have run a fetch. --symlink-install is required, not a
# preference: coco_sim's yard.py finds worlds/yard_params.yaml relative
# to its own source file (scripts/build_overlay.sh has the measurement).
# log/ is colcon's timestamped build log: dropped so it is neither dead
# weight nor a source of layer-to-layer nondeterminism.
RUN . /opt/ros/jazzy/setup.sh && \
    colcon build --symlink-install --packages-select \
      coco_config coco_mission coco_moveit_config coco_perception \
      coco_rl coco_sim coco_web custom_teleop gazebo_models && \
    rm -rf log

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
# The source tree is read-only to the runtime user; do not try to write
# __pycache__ into it.
ENV PYTHONDONTWRITEBYTECODE=1
ENV COCO_HTTP_PORT=8080
ENV COCO_VIDEO_PORT=8081
ENV COCO_TARGET_COLOUR=blue
ENV COCO_GUI=false
ENV COCO_RVIZ=false

RUN echo "source /opt/ros/jazzy/setup.bash" >> /etc/bash.bashrc && \
    echo "source ${COCO_WS}/install/setup.bash" >> /etc/bash.bashrc

COPY docker/entrypoint.sh /usr/local/bin/coco-entrypoint
RUN chmod 0755 /usr/local/bin/coco-entrypoint

# Identity, last so that it invalidates nothing above it.
#
# REPRODUCIBILITY metadata goes in a file, in a layer: what the image was
# built FROM. For one commit and one base digest it is the same bytes on
# every build. scripts/container/build.sh fills these in from git; a bare
# `docker compose build` leaves them "unknown", which is the honest value.
#
# BUILD metadata -- the branch name, and when and where the build ran --
# does not decide the content, so it lives in the image config (labels and
# env), never in a layer, and wall-clock time lives outside the image
# altogether (build.sh writes it next to the evidence).
ARG COCO_VERSION=2.0
ARG COCO_GIT_SHA=unknown
ARG COCO_GIT_DIRTY=unknown
ARG COCO_SOURCE_DATE_EPOCH=0
ARG COCO_INFRA_SHA=unknown
ARG COCO_GIT_BRANCH=unknown
RUN python3 -c "import json,sys; a=sys.argv[1:]; json.dump(dict(zip(a[0::2], a[1::2])), open('/opt/coco/build-info.json', 'w'), indent=1, sort_keys=True)" \
      coco_version "${COCO_VERSION}" \
      git_sha "${COCO_GIT_SHA}" \
      git_dirty "${COCO_GIT_DIRTY}" \
      infra_sha "${COCO_INFRA_SHA}" \
      source_date_epoch "${COCO_SOURCE_DATE_EPOCH}" \
      base_image "${BASE_IMAGE}" \
      ros_distro jazzy \
      rmw rmw_cyclonedds_cpp \
      simulator "gazebo-harmonic (gz sim 8), headless, software rendering by default" \
      packages "coco_config coco_mission coco_moveit_config coco_perception coco_rl coco_sim coco_web custom_teleop gazebo_models"
ENV COCO_VERSION=${COCO_VERSION} \
    COCO_GIT_SHA=${COCO_GIT_SHA} \
    COCO_BUILD_BRANCH=${COCO_GIT_BRANCH}
LABEL org.opencontainers.image.title="COCO 2.0" \
      org.opencontainers.image.description="ROS 2 Jazzy + Gazebo Harmonic mobile manipulator: simulator, mission stack and coco.v1 web platform" \
      org.opencontainers.image.source="https://github.com/GauthamCodes/coco-robot-jazzy-2.0" \
      org.opencontainers.image.version="${COCO_VERSION}" \
      org.opencontainers.image.revision="${COCO_GIT_SHA}" \
      org.opencontainers.image.base.name="${BASE_IMAGE}" \
      org.opencontainers.image.ref.name="${COCO_GIT_BRANCH}"

USER coco

# 8080 only: the UI, /ws, /healthz and the MJPEG view at /video/<alias>.
# web_video_server (8081) listens on the container's loopback for the
# platform alone; see docker-compose.yml.
EXPOSE 8080

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
