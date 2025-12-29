from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package directories
    robot_moveit_config_pkg = get_package_share_directory('robot_moveit_config')
    robot_description_pkg = get_package_share_directory('robot_description')
    
    # Declare arguments
    place_x_arg = DeclareLaunchArgument('place_x', default_value='0.3')
    place_y_arg = DeclareLaunchArgument('place_y', default_value='0.3')
    place_z_arg = DeclareLaunchArgument('place_z', default_value='0.1')
    
    # Build MoveIt config
    # Note: You may need to adjust this based on your robot configuration
    moveit_config = {
        'robot_description': '',  # Will be loaded from topic
        'robot_description_semantic': open(
            os.path.join(robot_moveit_config_pkg, 'srdf', 'robot.srdf')
        ).read(),
    }
    
    # MTC Pick Place node
    mtc_pick_place_node = Node(
        package='robot_moveit_config',
        executable='mtc_pick_place',
        name='mtc_pick_place',
        output='screen',
        parameters=[
            moveit_config,
            os.path.join(robot_moveit_config_pkg, 'config', 'kinematics.yaml'),
            os.path.join(robot_moveit_config_pkg, 'config', 'ompl_planning.yaml'),
            {
                'arm_group': 'manipulator',
                'gripper_group': 'gripper',
                'ee_frame': 'ee_link',
                'world_frame': 'world',
                'object_radius': 0.025,
                'object_height': 0.1,
                'place_x': LaunchConfiguration('place_x'),
                'place_y': LaunchConfiguration('place_y'),
                'place_z': LaunchConfiguration('place_z'),
            }
        ],
    )

    return LaunchDescription([
        place_x_arg,
        place_y_arg,
        place_z_arg,
        mtc_pick_place_node,
    ])
