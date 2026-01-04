#!/bin/bash
# Launch script for MoveIt demo with YOLO + RealSense that avoids snap library conflicts

# Capture all command line arguments to pass to ros2 launch
LAUNCH_ARGS="$@"

WORKSPACE_DIR="/home/peru0002/hydra_ws"

env -i \
    HOME=$HOME \
    DISPLAY=$DISPLAY \
    PATH=/opt/ros/jazzy/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
    XAUTHORITY=$XAUTHORITY \
    USER=$USER \
    LANG=${LANG:-en_US.UTF-8} \
    bash -c "
        cd $WORKSPACE_DIR
        
        # Source ROS2 base
        source /opt/ros/jazzy/setup.bash
        
        # Set up paths for hydra_ws packages
        export AMENT_PREFIX_PATH=$WORKSPACE_DIR/install/robot_moveit_config:$WORKSPACE_DIR/install/yolo:$WORKSPACE_DIR/install/robot_description:\$AMENT_PREFIX_PATH
        export PYTHONPATH=$WORKSPACE_DIR/install/yolo/lib/python3.12/site-packages:\$PYTHONPATH
        
        # Source packages
        source $WORKSPACE_DIR/install/robot_moveit_config/share/robot_moveit_config/local_setup.bash 2>/dev/null || true
        source $WORKSPACE_DIR/install/yolo/share/yolo/local_setup.bash 2>/dev/null || true
        source $WORKSPACE_DIR/install/robot_description/share/robot_description/local_setup.bash 2>/dev/null || true
        
        echo '================================'
        echo 'BRACU Hydra Robot - MoveIt Demo'
        echo '================================'
        echo 'Args: $LAUNCH_ARGS'
        echo ''
        
        ros2 launch robot_moveit_config pick_place_system.launch.py $LAUNCH_ARGS
    "
