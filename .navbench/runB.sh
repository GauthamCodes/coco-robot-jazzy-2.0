#!/usr/bin/env bash
# runB.sh <repeats> -- topology B: nav.launch.py arbiter:=true + the
# cmd_vel_arbiter, i.e. exactly what coco_mission/launch/mission.launch.py
# starts. The arbiter defaults to 'idle', so /mission/mode must assert
# 'nav' or nothing moves; the mission executive does this at runtime.
HERE="$(cd "$(dirname "$0")" && pwd)"
source "$HERE/env.sh"
REP="${1:-3}"; shift
ros2 topic pub -r 2 /mission/mode std_msgs/msg/String "{data: nav}" \
    > "$HERE/logs/mode.log" 2>&1 &
MODEPID=$!
echo "mode publisher pid $MODEPID"
sleep 4
timeout 6 ros2 topic echo /cmd_vel_arbiter/status --once 2>&1 | head -3
python3 "$HERE/../gazebo_models/scripts/nav_bench.py" \
    --tag baselineB --repeats "$REP" --out "$HERE/results" "$@"
kill $MODEPID 2>/dev/null
