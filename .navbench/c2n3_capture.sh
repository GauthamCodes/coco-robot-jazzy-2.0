#!/usr/bin/env bash
# C2-NAV.3: one FRESH enclosure-entry approach on a fresh simulator, so the
# robot drives into the pinch from the spawn rather than starting already
# stalled. C2-NAV.2 established that `--only enclosure_entry --repeats 3`
# gives one fresh approach and two escape probes, not three trials; this
# takes the one fresh approach and nothing else.
#
# Topology A (arbiter:=false), C2-NAV.0 baseline params, nothing changed.
HERE="$(cd "$(dirname "$0")" && pwd)"
source "$HERE/env.sh"
OUT="${1:-$HERE/results/c2n3}"
mkdir -p "$(dirname "$OUT")" "$HERE/logs"

bash /home/gautham/ros2_ws/src/coco-robot-ros2/gazebo_models/scripts/ros_clean.sh >/dev/null 2>&1
sleep 5
bash "$HERE/up_sim.sh" || exit 1
sleep 3
bash "$HERE/c2n3_up_nav.sh" || exit 1
bash "$HERE/wait_nav.sh" | head -1
sleep 3
echo "=== ground truth at spawn ==="
timeout 10 ros2 topic echo /model/coco/odometry --once --field pose.pose.position 2>&1 | head -5
echo "=== capture ==="
python3 "$HERE/../docs/data/c2nav3_capture.py" "$OUT"
echo "capture exit $?"
