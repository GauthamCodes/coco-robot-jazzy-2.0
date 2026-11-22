#!/usr/bin/env bash
# Bring up the simulator (headless) and wait until it is publishing.
HERE="$(cd "$(dirname "$0")" && pwd)"
source "$HERE/env.sh"
mkdir -p "$HERE/logs"
ros2 launch gazebo_models full_world_robo.launch.py gui:=false \
    > "$HERE/logs/sim.log" 2>&1 &
echo "sim pid $!"
for i in $(seq 1 120); do
    if ros2 topic list 2>/dev/null | grep -q '/model/coco/odometry'; then
        n=$(timeout 5 ros2 topic echo /scan --once 2>/dev/null | wc -l)
        if [ "${n:-0}" -gt 5 ]; then echo "sim up after ${i}s"; exit 0; fi
    fi
    sleep 1
done
echo "SIM DID NOT COME UP"; tail -40 "$HERE/logs/sim.log"; exit 1
