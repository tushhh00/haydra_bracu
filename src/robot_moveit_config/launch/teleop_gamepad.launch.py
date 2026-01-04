#!/usr/bin/env python3
"""
Launch file for Gamepad Teleoperation with MoveIt Servo
Butter-smooth control for 4-DOF robotic arm
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package paths
    pkg_share = get_package_share_directory('robot_moveit_config')
    
    # Servo config
    servo_config = os.path.join(pkg_share, 'config', 'servo_config.yaml')
    
    # Declare arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Joy node - reads gamepad input
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'device_id': 0,
            'deadzone': 0.1,
            'autorepeat_rate': 50.0,  # Higher for smoother input
            'coalesce_interval_ms': 1,  # Minimize input delay
        }],
        output='screen',
    )
    
    # MoveIt Servo node
    servo_node = Node(
        package='moveit_servo',
        executable='servo_node',
        name='servo_node',
        parameters=[
            servo_config,
            {'use_sim_time': use_sim_time},
        ],
        output='screen',
    )
    
    # Gamepad teleop node (our custom smooth controller)
    gamepad_teleop = Node(
        package='robot_moveit_config',
        executable='gamepad_teleop.py',
        name='gamepad_teleop',
        parameters=[{
            'linear_speed': 0.15,      # m/s - lower = smoother
            'angular_speed': 0.3,      # rad/s
            'joint_speed': 0.3,        # rad/s
            'smoothing_alpha': 0.25,   # 0.1-0.5 (lower = smoother)
            'deadzone': 0.12,          # Stick deadzone
            'frame_id': 'base_link',
        }],
        output='screen',
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        joy_node,
        servo_node,
        gamepad_teleop,
    ])
