from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import xacro
import yaml
import tempfile


def load_file(path):
    with open(path, 'r') as f:
        return f.read()


def load_yaml(path):
    with open(path, 'r') as f:
        return yaml.safe_load(f)


def generate_rviz_config(rviz_template_path, image_topic, enable_image_display):
    """Generate RViz config with dynamic image topic"""
    with open(rviz_template_path, 'r') as f:
        rviz_config = yaml.safe_load(f)
    
    # Find and update the YOLO Detections display
    if rviz_config and 'Visualization Manager' in rviz_config:
        displays = rviz_config['Visualization Manager'].get('Displays', [])
        
        # Find or create the image display
        image_display_found = False
        for display in displays:
            if display.get('Name') == 'YOLO Detections' or display.get('Class') == 'rviz_default_plugins/Image':
                display['Topic']['Value'] = image_topic
                display['Enabled'] = enable_image_display
                image_display_found = True
                break
        
        # If no image display exists and we want to enable it, add one
        if not image_display_found and enable_image_display:
            image_display = {
                'Class': 'rviz_default_plugins/Image',
                'Enabled': True,
                'Max Value': 1,
                'Median window': 5,
                'Min Value': 0,
                'Name': 'YOLO Detections',
                'Normalize Range': True,
                'Topic': {
                    'Depth': 5,
                    'Durability Policy': 'Volatile',
                    'History Policy': 'Keep Last',
                    'Reliability Policy': 'Best Effort',
                    'Value': image_topic
                }
            }
            # Insert after Grid display
            displays.insert(1, image_display)
    
    # Write to a temporary file
    temp_dir = tempfile.gettempdir()
    temp_rviz_path = os.path.join(temp_dir, 'moveit_dynamic.rviz')
    with open(temp_rviz_path, 'w') as f:
        yaml.dump(rviz_config, f, default_flow_style=False)
    
    return temp_rviz_path


def generate_launch_description():
    # Declare launch arguments for image topic
    image_topic_arg = DeclareLaunchArgument(
        'image_topic',
        default_value='/yolo/image_detections',
        description='Image topic to display in RViz'
    )
    
    enable_image_display_arg = DeclareLaunchArgument(
        'enable_image_display',
        default_value='true',
        description='Enable/disable image display in RViz'
    )

    # Paths
    robot_description_pkg = get_package_share_directory('robot_description')
    moveit_config_pkg = get_package_share_directory('robot_moveit_config')

    urdf_path = os.path.join(robot_description_pkg, 'urdf', 'robot.xacro')
    srdf_path = os.path.join(moveit_config_pkg, 'srdf', 'robot.srdf')
    rviz_config_file = os.path.join(moveit_config_pkg, 'config', 'moveit.rviz')

    # Process xacro to get robot_description
    robot_description_config = xacro.process_file(urdf_path).toxml()

    # Load configs
    robot_description = {'robot_description': robot_description_config}
    robot_description_semantic = {'robot_description_semantic': load_file(srdf_path)}
    
    kinematics_yaml = load_yaml(os.path.join(moveit_config_pkg, 'config', 'kinematics.yaml'))
    ompl_yaml = load_yaml(os.path.join(moveit_config_pkg, 'config', 'ompl_planning.yaml'))
    joint_limits_yaml = load_yaml(os.path.join(moveit_config_pkg, 'config', 'joint_limits.yaml'))
    controllers_yaml = load_yaml(os.path.join(moveit_config_pkg, 'config', 'moveit_controllers.yaml'))

    # Planning configuration
    planning_pipelines_config = {
        'default_planning_pipeline': 'ompl',
        'planning_pipelines': ['ompl'],
        'ompl': {
            'planning_plugins': ['ompl_interface/OMPLPlanner'],
            'request_adapters': [
                'default_planning_request_adapters/ResolveConstraintFrames',
                'default_planning_request_adapters/ValidateWorkspaceBounds',
                'default_planning_request_adapters/CheckStartStateBounds',
                'default_planning_request_adapters/CheckStartStateCollision',
            ],
            'response_adapters': [
                'default_planning_response_adapters/AddTimeOptimalParameterization',
                'default_planning_response_adapters/ValidateSolution',
                'default_planning_response_adapters/DisplayMotionPath',
            ],
        },
    }

    # Trajectory execution configuration - fake execution for demo
    trajectory_execution = {
        'moveit_manage_controllers': True,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01,
        'publish_monitored_planning_scene': True,
        'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager',
    }

    # Fake trajectory execution for demo mode
    fake_execution = {
        'fake_execution_type': 'interpolate',
    }

    # Planning scene monitor config
    planning_scene_monitor_parameters = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }

    # Move group capabilities
    move_group_capabilities = {
        'capabilities': '',
    }

    # Kinematics config with proper namespace
    robot_description_kinematics = {'robot_description_kinematics': kinematics_yaml}

    # Joint limits config with proper namespace
    robot_description_planning = {'robot_description_planning': joint_limits_yaml}

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )

    # Static TF for virtual joint (world -> base_link)
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='log',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'base_link'],
    )

    # Publish fake joint states for demo (without GUI for cleaner startup)
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[
            robot_description,
            {'source_list': ['/move_group/fake_controller_joint_states']},
        ],
    )

    # Move Group Node
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            ompl_yaml,
            planning_pipelines_config,
            trajectory_execution,
            planning_scene_monitor_parameters,
            move_group_capabilities,
            controllers_yaml,
            fake_execution,
            {'use_sim_time': False},
        ],
    )

    # RViz2 with MoveIt MotionPlanning plugin
    # Use OpaqueFunction to resolve launch arguments at runtime
    def launch_rviz(context):
        image_topic = LaunchConfiguration('image_topic').perform(context)
        enable_image_str = LaunchConfiguration('enable_image_display').perform(context)
        enable_image = enable_image_str.lower() in ('true', '1', 'yes')
        
        # Generate dynamic RViz config
        dynamic_rviz_config = generate_rviz_config(rviz_config_file, image_topic, enable_image)
        
        return [Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', dynamic_rviz_config],
            parameters=[
                robot_description,
                robot_description_semantic,
                robot_description_kinematics,
                robot_description_planning,
                ompl_yaml,
            ],
        )]

    rviz_node = OpaqueFunction(function=launch_rviz)

    # Fake trajectory server for demo execution
    fake_trajectory_server = Node(
        package='robot_moveit_config',
        executable='fake_trajectory_server.py',
        name='fake_trajectory_server',
        output='screen',
    )

    return LaunchDescription([
        image_topic_arg,
        enable_image_display_arg,
        robot_state_publisher,
        static_tf,
        joint_state_publisher,
        move_group_node,
        fake_trajectory_server,
        rviz_node,
    ])
