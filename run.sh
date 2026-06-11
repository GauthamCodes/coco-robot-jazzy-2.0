#!/bin/bash
# run.sh - Launch the Coco Robot simulation
# Usage: ./run.sh [gui:=false] [use_sim_time:=true]

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$SCRIPT_DIR"

echo "╔════════════════════════════════════════════════════════════════╗"
echo "║         Launching Coco Robot Gazebo Simulation                ║"
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
echo "✓ Sourcing workspace..."
source "$WS_DIR/install/setup.bash"

# Parse arguments (default: gui=true)
LAUNCH_ARGS="$@"
if [ -z "$LAUNCH_ARGS" ]; then
    LAUNCH_ARGS="gui:=true"
fi

echo "✓ Launch arguments: $LAUNCH_ARGS"
echo ""
echo "🚀 Starting Gazebo simulation..."
echo ""

# Launch the simulation
ros2 launch gazebo_models full_world_robo.launch.py $LAUNCH_ARGS

# Note: This script will block until the simulation is closed
