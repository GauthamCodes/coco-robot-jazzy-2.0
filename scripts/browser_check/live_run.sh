#!/usr/bin/env bash
# live_run.sh <repo> <overlay_ws> <outdir> [colour]
#
# One live end-to-end: fresh simulator, mission stack + platform, a ROS
# recorder, the headless-browser scenario, a socket safety probe, then
# teardown of OUR process groups only. Refuses to start if any Gazebo is
# already running -- it never kills a simulator it did not start.
set -o pipefail
REPO="$1"; WS="$2"; OUT="$3"; COLOUR="${4:-green}"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
mkdir -p "$OUT"

if pgrep -f 'g[z] sim' >/dev/null 2>&1; then
  echo "REFUSING: a Gazebo simulator is already running:" >&2
  pgrep -af 'g[z] sim' >&2
  exit 3
fi

unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH ROS_PACKAGE_PATH PYTHONPATH
export COCO_WS="$WS"
# A dedicated ROS domain unless the caller chose one. The P0.2 release
# pass's GUI run, on the shared default domain 0, received a /mission/mode
# `nav` and a Nav2 goal to (2.50, 2.00) that no process of the run and no
# browser sent -- a measured run must not be reachable by whatever else is
# using domain 0 on this machine or LAN.
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-61}"
# shellcheck disable=SC1091
source "$REPO/setup_env.sh" >/dev/null 2>&1

log() { echo "[live $(date +%H:%M:%S)] $*" | tee -a "$OUT/run.log"; }
SIM=""; STACK=""; REC=""; MET=""
teardown() {
  log "teardown"
  for pid in "$MET" "$REC" "$STACK" "$SIM"; do
    [ -n "$pid" ] && kill -INT -- "-$pid" 2>/dev/null
  done
  sleep 6
  for pid in "$MET" "$REC" "$STACK" "$SIM"; do
    [ -n "$pid" ] && kill -KILL -- "-$pid" 2>/dev/null
  done
  # Then the SESSION each setsid created. With gui:=true, gz forks
  # `gz sim server` and `gz sim gui` into process groups of their own
  # (measured: PGID = their own PID, same session), and neither carries
  # the world path, so the group kills above can miss them and
  # ros_clean.sh cannot recognise them. The release pass's GUI run left
  # the server orphaned exactly this way. -s is a session match, not -f,
  # so this shell (a different session) can never match itself.
  for pid in "$MET" "$REC" "$STACK" "$SIM"; do
    [ -n "$pid" ] && pkill -INT -s "$pid" 2>/dev/null
  done
  sleep 3
  for pid in "$MET" "$REC" "$STACK" "$SIM"; do
    [ -n "$pid" ] && pkill -KILL -s "$pid" 2>/dev/null
  done
}
trap 'teardown; exit 130' INT TERM

# COCO_LIVE_GUI=true runs the same scenario with the Gazebo GUI, for the
# GUI-vs-headless comparison; everything else is identical.
GUI="${COCO_LIVE_GUI:-false}"
log "overlay $WS ; colour $COLOUR ; gui $GUI"
setsid ros2 launch gazebo_models full_world_robo.launch.py \
  gui:="$GUI" traverse:=true > "$OUT/sim.log" 2>&1 &
SIM=$!
for _ in $(seq 1 120); do
  ros2 topic info /diff_drive_controller/odom 2>/dev/null \
    | grep 'Publisher count: [1-9]' >/dev/null && break
  sleep 2
done
log "controllers up"

setsid python3 "$HERE/wheel_recorder.py" "$OUT/recorder.jsonl" \
  > "$OUT/recorder.log" 2>&1 &
REC=$!

setsid ros2 launch coco_mission mission.launch.py rviz:=false \
  platform:=true > "$OUT/stack.log" 2>&1 &
STACK=$!

# Sample the platform's own measured numbers every 5 s.
setsid python3 "$HERE/metrics_sampler.py" "$OUT/metrics.jsonl" \
  > /dev/null 2>&1 &
MET=$!

for _ in $(seq 1 90); do
  curl -fsS http://127.0.0.1:8080/healthz >/dev/null 2>&1 && break
  sleep 2
done
log "healthz: $(curl -s -o /dev/null -w '%{http_code}' http://127.0.0.1:8080/healthz)"
curl -s http://127.0.0.1:8080/healthz > "$OUT/healthz_ready.json"
# Where web_video_server actually listens: loopback only since the
# release pass (the page reaches MJPEG through :8080/video/<alias>).
ss -ltn '( sport = :8081 )' > "$OUT/video_listen.txt" 2>&1

log "browser scenario"
python3 "$HERE/live.py" http://127.0.0.1:8080/ "$OUT" "$OUT/recorder.jsonl" \
  "$COLOUR" > "$OUT/live.log" 2>&1
log "browser scenario exit $?"

log "safety probe"
ros2 topic info /diff_drive_controller/cmd_vel -v > "$OUT/wheel_topic_before_probe.txt" 2>&1
python3 "$HERE/safety_probe.py" ws://127.0.0.1:8080/ws > "$OUT/safety_probe.json" 2>&1
ros2 topic info /diff_drive_controller/cmd_vel -v > "$OUT/wheel_topic_after_probe.txt" 2>&1
ros2 node info /coco_web_platform > "$OUT/platform_node_info.txt" 2>&1
curl -s http://127.0.0.1:8080/healthz > "$OUT/healthz_end.json"
curl -s http://127.0.0.1:8080/api/session > "$OUT/session_end.json"
curl -s http://127.0.0.1:8080/api/metrics > "$OUT/metrics_end.json"

teardown
log "done"
