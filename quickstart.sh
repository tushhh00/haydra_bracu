#!/bin/bash
# Quick start script for BRACU Hydra Robot Pick-and-Place System

set -e

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$WORKSPACE_DIR"

echo "================================"
echo "BRACU Hydra Robot - Quick Start"
echo "================================"
echo ""

# Check if workspace is built
if [ ! -d "install" ]; then
    echo "❌ Workspace not built. Building now..."
    source /opt/ros/jazzy/setup.bash
    colcon build --symlink-install
    echo "✓ Build complete"
    echo ""
fi

# Source workspace
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# Function to check if camera is connected
check_camera() {
    if timeout 2 rs-enumerate-devices 2>/dev/null | grep -q "Intel RealSense"; then
        return 0
    else
        return 1
    fi
}

# Check camera
echo "Checking camera connection..."
if check_camera; then
    echo "✓ Camera detected"
else
    echo "⚠ Camera not detected!"
    echo "  Please ensure RealSense D435i is connected via USB 3.0"
    echo "  You may need to unplug and replug the camera"
    read -p "Continue anyway? (y/N): " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

echo ""
echo "Select mode:"
echo "  1) Launch complete system (MoveIt + Camera + YOLO)"
echo "  2) Test camera only"
echo "  3) Test IK solver"
echo "  4) Run pick script"
echo "  5) View TF tree"
echo ""
read -p "Enter choice [1-5]: " choice

case $choice in
    1)
        echo ""
        echo "Launching complete pick-and-place system..."
        echo "This will start:"
        echo "  - Robot hardware interface"
        echo "  - MoveIt2 motion planning"
        echo "  - RealSense camera"
        echo "  - YOLO object detection"
        echo ""
        echo "Press Ctrl+C to stop"
        echo ""
        sleep 2
        ros2 launch robot_moveit_config pick_place_system.launch.py publish_static_camera_tf:=true
        ;;
    2)
        echo ""
        echo "Testing camera..."
        ros2 launch realsense2_camera rs_launch.py \
            enable_depth:=true \
            enable_color:=true \
            pointcloud.enable:=true
        ;;
    3)
        echo ""
        echo "Testing IK solver..."
        echo "Enter positions as: x y z"
        echo "Example: 0.0 0.4 0.6"
        python3 src/robot_moveit_config/scripts/robot_ik_solver.py
        ;;
    4)
        echo ""
        echo "Make sure system is running in another terminal!"
        echo ""
        echo "Select pick script:"
        echo "  1) IK-based (Cartesian)"
        echo "  2) Joint-space (more reliable)"
        read -p "Enter choice [1-2]: " pick_choice
        
        if [ "$pick_choice" = "1" ]; then
            python3 src/robot_moveit_config/scripts/pick_with_ik.py
        else
            python3 src/robot_moveit_config/scripts/pick_detected_object.py
        fi
        ;;
    5)
        echo ""
        echo "Generating TF tree visualization..."
        ros2 run tf2_tools view_frames
        if [ -f frames.pdf ]; then
            evince frames.pdf 2>/dev/null &
            echo "✓ TF tree saved to frames.pdf"
        else
            echo "⚠ Failed to generate TF tree"
        fi
        ;;
    *)
        echo "Invalid choice"
        exit 1
        ;;
esac
