from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os
import xacro
import yaml


def load_file(path):
    with open(path, 'r') as f:
        return f.read()


def load_yaml(path):
    with open(path, 'r') as f:
        return yaml.safe_load(f)


def generate_launch_description():
    robot_description_pkg = get_package_share_directory('robot_description')
    moveit_config_pkg = get_package_share_directory('robot_moveit_config')

    urdf_path = os.path.join(robot_description_pkg, 'urdf', 'robot.xacro')
    srdf_path = os.path.join(moveit_config_pkg, 'srdf', 'robot.srdf')

    # Process xacro to get robot_description
    robot_description_config = xacro.process_file(urdf_path).toxml()

    robot_description = {'robot_description': robot_description_config}
    robot_description_semantic = {'robot_description_semantic': load_file(srdf_path)}

    kinematics_yaml = load_yaml(os.path.join(moveit_config_pkg, 'config', 'kinematics.yaml'))
    ompl_yaml = load_yaml(os.path.join(moveit_config_pkg, 'config', 'ompl_planning.yaml'))
    move_group_yaml = load_yaml(os.path.join(moveit_config_pkg, 'config', 'move_group.yaml'))
    controllers_yaml = load_yaml(os.path.join(moveit_config_pkg, 'config', 'moveit_controllers.yaml'))
    joint_limits_yaml = load_yaml(os.path.join(moveit_config_pkg, 'config', 'joint_limits.yaml'))

    rviz_config_file = os.path.join(moveit_config_pkg, 'config', 'moveit.rviz')

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[robot_description]
    )

    # Joint State Publisher (publishes fake joint states for planning)
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[robot_description],
    )

    # Move Group
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            ompl_yaml,
            move_group_yaml,
            controllers_yaml,
            joint_limits_yaml,
        ],
    )

    # RViz2 with MoveIt MotionPlanning plugin
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[robot_description, robot_description_semantic, kinematics_yaml]
    )

    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher,
        move_group_node,
        rviz_node,
    ])
