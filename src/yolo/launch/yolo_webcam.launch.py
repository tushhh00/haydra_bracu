#!/usr/bin/env python3
"""
Launch file for YOLO webcam node (captures from webcam and publishes)
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
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
    
    camera_id_arg = DeclareLaunchArgument(
        'camera_id',
        default_value='0',
        description='Camera device ID'
    )
    
    frame_rate_arg = DeclareLaunchArgument(
        'frame_rate',
        default_value='30.0',
        description='Frame rate for capture'
    )
    
    # YOLO webcam node
    yolo_webcam_node = Node(
        package='yolo',
        executable='yolo_webcam',
        name='yolo_webcam',
        parameters=[{
            'model_path': LaunchConfiguration('model_path'),
            'confidence_threshold': LaunchConfiguration('confidence_threshold'),
            'device': LaunchConfiguration('device'),
            'camera_id': LaunchConfiguration('camera_id'),
            'frame_rate': LaunchConfiguration('frame_rate'),
            'output_topic': '/yolo/detections',
            'output_image_topic': '/yolo/image_raw',
            'output_annotated_topic': '/yolo/image_detections',
        }],
        output='screen'
    )
    
    return LaunchDescription([
        model_path_arg,
        confidence_arg,
        device_arg,
        camera_id_arg,
        frame_rate_arg,
        yolo_webcam_node,
    ])
