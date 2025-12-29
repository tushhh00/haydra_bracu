"""
Complete Pick and Place System Launch File

This launches:
1. RealSense camera (with aligned depth)
2. Static TF for camera (if table-mounted)
3. 3D Object detector (YOLO + depth)
4. MoveIt demo environment
5. MTC Pick and Place node (optional)

IMPORTANT: For accurate pick & place:
- Measure camera position relative to robot base
- Update static_camera_tf values OR attach camera to robot URDF
- Verify TF chain: ros2 run tf2_tools view_frames
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package directories
    yolo_pkg = get_package_share_directory('yolo')
    robot_moveit_config_pkg = get_package_share_directory('robot_moveit_config')
    
    # Declare arguments
    target_class_arg = DeclareLaunchArgument(
        'target_class',
        default_value='',
        description='Target class to detect (empty = all classes)'
    )
    
    launch_mtc_arg = DeclareLaunchArgument(
        'launch_mtc',
        default_value='false',
        description='Whether to launch MTC pick and place node'
    )
    
    # Static camera TF arguments (for table-mounted cameras)
    # Set publish_static_camera_tf:=true and update these values!
    publish_static_tf_arg = DeclareLaunchArgument(
        'publish_static_camera_tf',
        default_value='false',
        description='Publish static TF from base_link to camera (for table-mounted cameras)'
    )
    
    # Camera position relative to robot base (MEASURE THIS PHYSICALLY!)
    camera_x_arg = DeclareLaunchArgument('camera_x', default_value='0.5',
        description='Camera X position relative to base_link (meters)')
    camera_y_arg = DeclareLaunchArgument('camera_y', default_value='0.0',
        description='Camera Y position relative to base_link (meters)')
    camera_z_arg = DeclareLaunchArgument('camera_z', default_value='0.8',
        description='Camera Z position relative to base_link (meters)')
    # Camera orientation (roll, pitch, yaw in radians)
    camera_roll_arg = DeclareLaunchArgument('camera_roll', default_value='0.0',
        description='Camera roll angle (radians)')
    camera_pitch_arg = DeclareLaunchArgument('camera_pitch', default_value='0.0',
        description='Camera pitch angle (radians) - positive looks down')
    camera_yaw_arg = DeclareLaunchArgument('camera_yaw', default_value='0.0',
        description='Camera yaw angle (radians)')
    
    # RealSense camera launch
    # CRITICAL: align_depth MUST be enabled for accurate 3D detection!
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('realsense2_camera'),
            '/launch/rs_launch.py'
        ]),
        launch_arguments={
            'depth_module.depth_profile': '848x480x30',
            'rgb_camera.color_profile': '848x480x30',
            'align_depth.enable': 'true',
            'pointcloud.enable': 'false',  # Disable if not needed (saves CPU)
            'initial_reset': 'true',  # Reset camera on startup to fix connection issues
            'enable_infra1': 'false',  # Disable infrared to reduce USB bandwidth
            'enable_infra2': 'false',
            'enable_gyro': 'false',  # Disable IMU to reduce CPU load
            'enable_accel': 'false',
        }.items()
    )
    
    # Static TF publisher for table-mounted camera
    # Only launched if publish_static_camera_tf:=true
    static_camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_static_tf',
        arguments=[
            '--x', LaunchConfiguration('camera_x'),
            '--y', LaunchConfiguration('camera_y'),
            '--z', LaunchConfiguration('camera_z'),
            '--roll', LaunchConfiguration('camera_roll'),
            '--pitch', LaunchConfiguration('camera_pitch'),
            '--yaw', LaunchConfiguration('camera_yaw'),
            '--frame-id', 'base_link',
            '--child-frame-id', 'camera_link',
        ],
        condition=IfCondition(LaunchConfiguration('publish_static_camera_tf'))
    )
    
    # 3D Object detector launch (delayed to allow camera to start)
    object_3d_detector = TimerAction(
        period=10.0,  # Wait 10 seconds for camera to fully initialize (especially after reset)
        actions=[
            Node(
                package='yolo',
                executable='object_3d_detector',
                name='object_3d_detector',
                output='screen',
                parameters=[
                    os.path.join(yolo_pkg, 'config', 'object_3d_detector.yaml'),
                    {'target_class': LaunchConfiguration('target_class')},
                ],
            )
        ]
    )
    
    # MoveIt demo launch (delayed)
    moveit_demo = TimerAction(
        period=2.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    robot_moveit_config_pkg,
                    '/launch/demo.launch.py'
                ]),
                launch_arguments={
                    'image_topic': '/yolo/image_3d_detections',
                }.items()
            )
        ]
    )
    
    return LaunchDescription([
        target_class_arg,
        launch_mtc_arg,
        publish_static_tf_arg,
        camera_x_arg,
        camera_y_arg,
        camera_z_arg,
        camera_roll_arg,
        camera_pitch_arg,
        camera_yaw_arg,
        realsense_launch,
        static_camera_tf,
        object_3d_detector,
        moveit_demo,
    ])
