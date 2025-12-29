from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package share directory
    yolo_pkg = get_package_share_directory('yolo')
    
    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(yolo_pkg, 'config', 'object_3d_detector.yaml'),
        description='Path to the configuration file'
    )
    
    target_class_arg = DeclareLaunchArgument(
        'target_class',
        default_value='',
        description='Target class to detect (empty = all classes)'
    )
    
    confidence_arg = DeclareLaunchArgument(
        'confidence',
        default_value='0.5',
        description='Detection confidence threshold'
    )
    
    # 3D Object Detector Node
    object_3d_detector_node = Node(
        package='yolo',
        executable='object_3d_detector_node.py',
        name='object_3d_detector',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'target_class': LaunchConfiguration('target_class'),
                'confidence_threshold': LaunchConfiguration('confidence'),
            }
        ],
    )
    
    return LaunchDescription([
        config_file_arg,
        target_class_arg,
        confidence_arg,
        object_3d_detector_node,
    ])
