#!/usr/bin/env bash
# Read-only: how does the 4.5 ROS 2 bridge pick its ROS libraries?
E=/home/gautham/isaac-sim/venv/lib/python3.10/site-packages/isaacsim/exts
B=$E/isaacsim.ros2.bridge
echo "=== clock / sim-time node files ==="
find "$B" -name '*PublishClock*' | head -3
find "$E/isaacsim.core.nodes" -name '*ReadSimulationTime*' | head -3
echo "=== humble/ contents ==="
ls "$B/humble"
echo "rclpy present: $(ls -d "$B/humble/rclpy" 2>/dev/null || echo no)"
echo "=== distro-selection logic (python) ==="
grep -rn "ROS_DISTRO\|LD_LIBRARY_PATH\|humble\|jazzy\|RMW_IMPLEMENTATION" \
  "$B/isaacsim/ros2/bridge/"*.py 2>/dev/null | head -30
echo "=== extension.toml [settings] ==="
sed -n '/\[settings\]/,/^\[/p' "$B/config/extension.toml" | head -20
