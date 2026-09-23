#!/usr/bin/env bash
# Isaac Sim (clean env, bundled Humble libs) <-> ROS 2 Jazzy (system,
# /opt/ros/jazzy only) on a PRIVATE domain, so no COCO graph is touched.
TMP=/home/gautham/.claude/jobs/d213ad33/tmp
VENV=/home/gautham/isaac-sim/venv
BRIDGE=$VENV/lib/python3.10/site-packages/isaacsim/exts/isaacsim.ros2.bridge
DOMAIN=77
rm -f "$TMP/isaac_publishing.marker" "$TMP"/jazzy_*.txt
: > "$TMP/isaac_smoke5.log"

# ── Isaac side: nothing inherited but what is named here ──
env -i HOME=/home/gautham PATH=/usr/bin:/bin \
    ROS_DISTRO=humble RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    ROS_DOMAIN_ID=$DOMAIN LD_LIBRARY_PATH="$BRIDGE/humble/lib" \
    timeout 600 "$VENV/bin/python" -u "$TMP/isaac_smoke5.py" \
    >> "$TMP/isaac_smoke5.log" 2>&1 &
ISAAC=$!
echo "isaac pid $ISAAC"

# wait (bounded) for the publisher loop to start
for _ in $(seq 1 150); do
  [ -f "$TMP/isaac_publishing.marker" ] && break
  kill -0 "$ISAAC" 2>/dev/null || break
  sleep 1
done
if [ ! -f "$TMP/isaac_publishing.marker" ]; then
  echo "Isaac never reached the publishing loop"
  kill "$ISAAC" 2>/dev/null; wait "$ISAAC"; echo "isaac exit=$?"; exit 3
fi
echo "marker seen at $(date +%T)"

# ── Jazzy side: system ROS only, same private domain ──
(
  # shellcheck disable=SC1091
  source /opt/ros/jazzy/setup.bash
  export ROS_DOMAIN_ID=$DOMAIN RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  echo "jazzy: ROS_DISTRO=$ROS_DISTRO domain=$ROS_DOMAIN_ID"
  ros2 topic list --no-daemon > "$TMP/jazzy_topics.txt" 2>&1
  timeout 30 ros2 topic echo /isaac/cube_z std_msgs/msg/Float64 --once \
    > "$TMP/jazzy_cubez.txt" 2>&1; echo "cube_z echo exit=$?"
  timeout 30 ros2 topic pub -r 2 /coco_ping std_msgs/msg/String \
    "{data: 'ping from jazzy'}" > "$TMP/jazzy_pub.txt" 2>&1 &
  PUB=$!
  timeout 30 ros2 topic echo /isaac_pong std_msgs/msg/String --once \
    > "$TMP/jazzy_pong.txt" 2>&1; echo "pong echo exit=$?"
  kill "$PUB" 2>/dev/null; wait "$PUB" 2>/dev/null
)

wait "$ISAAC"; echo "isaac exit=$?"
