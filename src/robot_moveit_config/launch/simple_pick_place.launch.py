"""
Simple Pick and Place Launch File

Launches the simple_pick_place node that receives detected object poses
and executes pick and place operations using MoveIt.

Usage:
  ros2 launch robot_moveit_config simple_pick_place.launch.py

To trigger pick and place:
  ros2 service call /simple_pick_place/pick_and_place std_srvs/srv/Trigger
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument('arm_group', default_value='manipulator',
                            description='Name of the arm planning group'),
        DeclareLaunchArgument('gripper_group', default_value='gripper',
                            description='Name of the gripper planning group'),
        DeclareLaunchArgument('place_x', default_value='0.2',
                            description='X position to place object'),
        DeclareLaunchArgument('place_y', default_value='0.2',
                            description='Y position to place object'),
        DeclareLaunchArgument('place_z', default_value='0.15',
                            description='Z position to place object'),
        
        # Simple Pick Place Node
        Node(
            package='robot_moveit_config',
            executable='simple_pick_place',
            name='simple_pick_place',
            output='screen',
            parameters=[{
                'arm_group': LaunchConfiguration('arm_group'),
                'gripper_group': LaunchConfiguration('gripper_group'),
                'world_frame': 'world',
                'ee_frame': 'ee_link',
                'place_x': LaunchConfiguration('place_x'),
                'place_y': LaunchConfiguration('place_y'),
                'place_z': LaunchConfiguration('place_z'),
                'approach_distance': 0.1,
                'lift_distance': 0.1,
            }],
        ),
    ])
