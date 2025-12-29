#!/bin/bash
# Launch script for MoveIt demo with YOLO + RealSense that avoids snap library conflicts

# Capture all command line arguments to pass to ros2 launch
LAUNCH_ARGS="$@"

env -i \
    HOME=$HOME \
    DISPLAY=$DISPLAY \
    PATH=/opt/ros/jazzy/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
    XAUTHORITY=$XAUTHORITY \
    bash -c "cd /home/peru0002/ws_moveit && \
             export AMENT_PREFIX_PATH=/home/peru0002/ws_moveit/install/robot_moveit_config && \
             source install/setup.bash && \
             ros2 launch robot_moveit_config pick_place_system.launch.py $LAUNCH_ARGS"
