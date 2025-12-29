#!/bin/bash
#
# IKFast Generator Script for 4-DOF Robot
# 
# This script helps generate an IKFast analytical IK solver.
# 
# Prerequisites:
#   sudo apt install ros-jazzy-moveit-kinematics
#   pip install openravepy  # Or use Docker
#
# For 4-DOF robots, we use "translation3d" IK type (position only, no orientation)
#

set -e

ROBOT_NAME="robot"
PLANNING_GROUP="manipulator"
BASE_LINK="base_link"
EE_LINK="ee_link"
WORKSPACE="/home/peru0002/ws_moveit"

echo "=============================================="
echo "IKFast Generator for 4-DOF Robot"
echo "=============================================="
echo ""
echo "Your robot: $ROBOT_NAME"
echo "Planning group: $PLANNING_GROUP"
echo "Base link: $BASE_LINK"
echo "End effector: $EE_LINK"
echo ""

# Step 1: Check if OpenRAVE is available
echo "Step 1: Checking OpenRAVE..."
if command -v openrave &> /dev/null; then
    echo "  OpenRAVE found!"
else
    echo "  OpenRAVE NOT found."
    echo ""
    echo "  To install OpenRAVE (Ubuntu), try:"
    echo "    # Option 1: Docker (recommended)"
    echo "    docker pull openrave/openrave:latest"
    echo ""
    echo "    # Option 2: Build from source (complex)"
    echo "    # See: https://github.com/rdiankov/openrave"
    echo ""
    echo "  Alternatively, use the MoveIt Setup Assistant IKFast plugin."
fi

# Step 2: Export URDF
echo ""
echo "Step 2: Robot URDF location..."
URDF_FILE="$WORKSPACE/src/robot_description/urdf/robot.urdf"
if [ -f "$URDF_FILE" ]; then
    echo "  Found: $URDF_FILE"
else
    URDF_FILE="$WORKSPACE/src/robot_description/urdf/robot.xacro"
    if [ -f "$URDF_FILE" ]; then
        echo "  Found xacro: $URDF_FILE"
        echo "  Convert to URDF with:"
        echo "    xacro $URDF_FILE > /tmp/robot.urdf"
    else
        echo "  URDF not found at expected location"
    fi
fi

echo ""
echo "=============================================="
echo "IKFast Generation Options"
echo "=============================================="
echo ""
echo "For a 4-DOF robot, you have these options:"
echo ""
echo "1. translation3d - Position only (x, y, z)"
echo "   Best for SCARA-like arms, pick & place"
echo "   Command: --iktype=translation3d"
echo ""
echo "2. translationxy - Position in XY plane only"
echo "   For planar robots"
echo "   Command: --iktype=translationxy"
echo ""
echo "=============================================="
echo "Manual Generation Steps"
echo "=============================================="
echo ""
echo "# 1. Convert URDF to COLLADA (DAE)"
echo "rosrun collada_urdf urdf_to_collada robot.urdf robot.dae"
echo ""
echo "# 2. Generate IKFast (using Docker)"
echo "docker run --rm -v \$(pwd):/workspace openrave/openrave \\"
echo "  python /usr/local/lib/python2.7/dist-packages/openravepy/_openravepy_/ikfast.py \\"
echo "  --robot=/workspace/robot.dae \\"
echo "  --iktype=translation3d \\"
echo "  --baselink=0 --eelink=4 \\"
echo "  --savefile=/workspace/ikfast_output.cpp"
echo ""
echo "# 3. Create MoveIt IKFast plugin"
echo "ros2 run moveit_kinematics create_ikfast_moveit_plugin.py \\"
echo "  --robot_name=$ROBOT_NAME \\"
echo "  --moveit_config_pkg=robot_moveit_config \\"
echo "  --ikfast_plugin_pkg=robot_ikfast_plugin \\"
echo "  --base_link=$BASE_LINK \\"
echo "  --eef_link=$EE_LINK \\"
echo "  --ikfast_output_file=ikfast_output.cpp"
echo ""
