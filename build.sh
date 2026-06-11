#!/bin/bash
# build.sh - Build the coco-robot-ros2 workspace
# Usage: ./build.sh [clean]

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$SCRIPT_DIR"

echo "╔════════════════════════════════════════════════════════════════╗"
echo "║         Building Coco Robot ROS2 Workspace                    ║"
echo "╚════════════════════════════════════════════════════════════════╝"
echo ""

# Check if ROS2 is sourced
if [ -z "$ROS_DISTRO" ]; then
    echo "❌ ROS2 not sourced! Please source your ROS2 installation first:"
    echo "   source /opt/ros/humble/setup.bash"
    exit 1
fi

echo "✓ ROS2 Distro: $ROS_DISTRO"
echo "✓ Workspace: $WS_DIR"
echo ""

# Clean build if requested
if [ "$1" == "clean" ]; then
    echo "🧹 Cleaning build artifacts..."
    rm -rf build/ install/ log/
    echo "✓ Clean complete"
    echo ""
fi

# Build the workspace
echo "🔨 Building packages..."
cd "$WS_DIR"

colcon build \
    --symlink-install \
    --packages-select gazebo_models custom_teleop coco_config \
    --cmake-args -DCMAKE_BUILD_TYPE=Release

if [ $? -eq 0 ]; then
    echo ""
    echo "╔════════════════════════════════════════════════════════════════╗"
    echo "║                  ✅ BUILD SUCCESSFUL                           ║"
    echo "╚════════════════════════════════════════════════════════════════╝"
    echo ""
    echo "Next steps:"
    echo "  1. Source the workspace: source install/setup.bash"
    echo "  2. Or use: ./run.sh"
    echo ""
else
    echo ""
    echo "❌ Build failed! Check errors above."
    exit 1
fi
