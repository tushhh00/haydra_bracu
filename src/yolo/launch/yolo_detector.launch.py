#!/usr/bin/env python3
"""
Launch file for YOLO detector node (subscribes to camera topic)
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package share directory
    pkg_share = get_package_share_directory('yolo')
    
    # Declare launch arguments
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value='/home/peru0002/ws_moveit/src/yolo/best.pt',
        description='Path to YOLO model weights'
    )
    
    confidence_arg = DeclareLaunchArgument(
        'confidence_threshold',
        default_value='0.25',
        description='Detection confidence threshold'
    )
    
    device_arg = DeclareLaunchArgument(
        'device',
        default_value='cuda',
        description='Device to run inference on (cuda or cpu)'
    )
    
    input_topic_arg = DeclareLaunchArgument(
        'input_topic',
        default_value='/camera/image_raw',
        description='Input image topic'
    )
    
    # YOLO detector node
    yolo_detector_node = Node(
        package='yolo',
        executable='yolo_detector',
        name='yolo_detector',
        parameters=[{
            'model_path': LaunchConfiguration('model_path'),
            'confidence_threshold': LaunchConfiguration('confidence_threshold'),
            'device': LaunchConfiguration('device'),
            'input_topic': LaunchConfiguration('input_topic'),
            'output_topic': '/yolo/detections',
            'output_image_topic': '/yolo/image_detections',
            'publish_annotated_image': True,
        }],
        output='screen'
    )
    
    return LaunchDescription([
        model_path_arg,
        confidence_arg,
        device_arg,
        input_topic_arg,
        yolo_detector_node,
    ])
