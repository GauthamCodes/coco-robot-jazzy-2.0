#!/usr/bin/env bash
# C2-NAV.21 TOPOLOGY B bringup: nav.launch.py with arbiter:=true, plus the
# arbiter itself -- the cmd_vel path `mission.launch.py` actually runs.
#
#   c2n21_up_nav_b.sh <abs params path> <log suffix>
#
# Topology A (every C2-NAV experiment from .0 to .21 so far) is
# `nav.launch.py` alone: cmd_vel_relay publishes straight to
# /diff_drive_controller/cmd_vel. Topology B routes Nav2 through
# /cmd_vel_nav into cmd_vel_arbiter, which then owns the controller
# topic. C2-NAV.0 measured the difference and it is not cosmetic: 14/21
# legs against 16/21, 0.155 m/s against 0.208, DWB at 7.97 Hz against
# 8.76.
#
# ros_clean.sh already carries 'arbiter[.]launch.py' and
# 'cmd_vel_arbite[r]', so nothing new needs adding for this file.
#
# Naming: every helper is c2n21_* and every parameter file is
# c2nav21_*_params.yaml, so no command line here contains "nav2_", which
# ros_clean.sh's 'nav[2]_' pattern would match.
HERE="$(cd "$(dirname "$0")" && pwd)"
source "$HERE/env.sh"
mkdir -p "$HERE/logs"
PARAMS="$1"
SUF="${2:-c2n21b}"
[ -f "$PARAMS" ] || { echo "no such params file: $PARAMS"; exit 1; }
case "$PARAMS" in *nav2_*) echo "REFUSING: params path contains 'nav2_'"; exit 1;; esac

echo "params_file: $PARAMS"
sha256sum "$PARAMS"
echo "--- the C2-NAV.21 variables, and the baseline they sit in ---"
grep -n 'forward_point_distance\|sim_time:\|cost_scaling_factor\|inflation_radius\|BaseObstacle.scale:\|GoalChecker\|vx_samples\|vtheta_samples\|aggregation_type\|default_nav_through_poses_bt_xml' "$PARAMS"

echo "--- arbiter up FIRST: nav.launch.py arbiter:=true only redirects the"
echo "    relay to /cmd_vel_nav; without cmd_vel_arbiter nothing reaches"
echo "    the wheels at all. ---"
# initial_mode:=nav is LOAD-BEARING, not tidiness. cmd_vel_arbiter
# defaults to 'idle', where only teleop can move the robot, and it waits
# for /mission/mode to say otherwise -- which mission.launch.py provides
# from the mission executive and a bare Nav2 benchmark does not. Without
# it every leg reports TIMEOUT having moved 0.000 m with DWB commanding
# normally, which reads exactly like a navigation failure and is not one.
# The node's own docstring and arbiter.launch.py's usage line both say
# to pass it; this script did not, and three tours were spent finding
# that out.
ros2 launch custom_teleop arbiter.launch.py initial_mode:=nav \
    > "$HERE/logs/arb_${SUF}.log" 2>&1 &
echo "arbiter pid $!"
for i in $(seq 1 60); do
    if ros2 node list 2>/dev/null | grep -q cmd_vel_arbiter; then
        echo "cmd_vel_arbiter up after ${i}s"; break; fi
    sleep 1
done
ros2 node list 2>/dev/null | grep -q cmd_vel_arbiter || {
    echo "ARBITER DID NOT COME UP"; tail -30 "$HERE/logs/arb_${SUF}.log";
    exit 1; }

# PROVE the arbiter will actually pass Nav2 through, rather than assume
# the launch argument took. An arbiter sitting in 'idle' forwards
# nothing and reports no error.
MODE=$(timeout 10 ros2 param get /cmd_vel_arbiter initial_mode 2>&1)
echo "arbiter initial_mode: $MODE"
case "$MODE" in
  *nav*) : ;;
  *) echo "REFUSING: cmd_vel_arbiter is not in nav mode ($MODE)."
     echo "In 'idle' every leg moves 0.000 m and reports TIMEOUT."
     exit 1;;
esac
grep -m1 'mode=' "$HERE/logs/arb_${SUF}.log" || true

ros2 launch gazebo_models nav.launch.py arbiter:=true \
    params_file:="$PARAMS" > "$HERE/logs/nav_${SUF}.log" 2>&1 &
echo "nav pid $!"
for i in $(seq 1 150); do
    st=$(timeout 5 ros2 lifecycle get /bt_navigator 2>/dev/null)
    if echo "$st" | grep -q active; then echo "nav2 active after ${i}s"; break; fi
    sleep 1
done
timeout 5 ros2 lifecycle get /bt_navigator 2>/dev/null | grep -q active || {
    echo "NAV2 DID NOT ACTIVATE"; tail -40 "$HERE/logs/nav_${SUF}.log";
    exit 1; }

# PROVE the topology is actually B, rather than assuming the flag took.
# The tell is the publisher set on the controller topic: in topology A
# cmd_vel_relay owns it, in topology B cmd_vel_arbiter does.
echo "=== topology proof: who publishes /diff_drive_controller/cmd_vel ==="
timeout 15 ros2 topic info -v /diff_drive_controller/cmd_vel 2>&1 \
    | grep -A3 'Publisher count\|Node name' | head -20
echo "=== and /cmd_vel_nav must now have BOTH a Nav2 publisher and the"
echo "    arbiter subscribed ==="
timeout 15 ros2 topic info -v /cmd_vel_nav 2>&1 \
    | grep -E 'Publisher count|Subscription count|Node name' | head -20
exit 0
