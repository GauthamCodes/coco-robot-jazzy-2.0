#!/bin/bash
# teleop.sh - Launch teleop nodes for the Coco Robot
# Usage: ./teleop.sh [wheels|arm|both]

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$SCRIPT_DIR"

MODE="${1:-both}"

echo "╔════════════════════════════════════════════════════════════════╗"
echo "║              Coco Robot Teleoperation                          ║"
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

case "$MODE" in
    wheels)
        echo "🎮 Starting WHEELS teleop..."
        echo ""
        echo "Controls:"
        echo "  w/s - Forward/Backward"
        echo "  a/d - Turn left/right"
        echo "  x   - Stop"
        echo "  q   - Quit"
        echo ""
        ros2 run custom_teleop teleop_wheels_node
        ;;
    arm)
        echo "🎮 Starting ARM teleop..."
        echo ""
        echo "Controls:"
        echo "  w/s     - Shoulder +/-"
        echo "  e/d     - Elbow +/-"
        echo "  r/f     - Gripper open/close"
        echo "  SPACE   - Reset to home"
        echo "  h       - Help"
        echo "  q       - Quit"
        echo ""
        ros2 run custom_teleop teleop_arm_node
        ;;
    both)
        echo "🎮 Starting BOTH teleop nodes in separate terminals..."
        echo ""
        echo "NOTE: This requires tmux or separate terminal windows"
        echo ""
        echo "Run these commands in separate terminals:"
        echo "  Terminal 1: ./teleop.sh wheels"
        echo "  Terminal 2: ./teleop.sh arm"
        echo ""
        exit 0
        ;;
    *)
        echo "❌ Invalid mode: $MODE"
        echo "Usage: ./teleop.sh [wheels|arm|both]"
        exit 1
        ;;
esac
