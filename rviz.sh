#!/bin/bash
# rviz.sh - Launch RViz with Coco Robot configuration
# Usage: ./rviz.sh

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$SCRIPT_DIR"

echo "╔════════════════════════════════════════════════════════════════╗"
echo "║              Launching RViz - Coco Robot                       ║"
echo "╚════════════════════════════════════════════════════════════════╝"
echo ""

# Check if ROS2 is sourced
if [ -z "$ROS_DISTRO" ]; then
    echo "❌ ROS2 not sourced! Please source your ROS2 installation first:"
    echo "   source /opt/ros/humble/setup.bash"
    exit 1
fi

# Check if workspace is built
if [ ! -d "$WS_DIR/install" ]; then
    echo "❌ Workspace not built! Run ./build.sh first"
    exit 1
fi

# Source the workspace
source "$WS_DIR/install/setup.bash"

# Get the RViz config path
RVIZ_CONFIG=$(ros2 pkg prefix gazebo_models)/share/gazebo_models/rviz/coco_robot.rviz

if [ ! -f "$RVIZ_CONFIG" ]; then
    echo "⚠️  RViz config not found at: $RVIZ_CONFIG"
    echo "   Launching RViz with default configuration..."
    rviz2
else
    echo "✓ Using config: $RVIZ_CONFIG"
    echo ""
    echo "🎨 Launching RViz2..."
    rviz2 -d "$RVIZ_CONFIG"
fi
