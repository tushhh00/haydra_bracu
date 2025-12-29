#!/home/peru0002/ws_moveit/src/yolo/.venv/bin/python
"""
3D Object Detection Node
Combines YOLO 2D detections with RealSense depth to get 3D object positions.

Key features:
- Uses aligned depth image for accurate 3D projection
- Proper camera intrinsics from CameraInfo
- TF transformation to base_link for MoveIt compatibility
- Median depth sampling for noise robustness
"""

import sys
sys.path.insert(0, '/home/peru0002/ws_moveit/src/yolo/.venv/lib/python3.12/site-packages')

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.duration import Duration
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, PoseArray, Point, Pose
from std_msgs.msg import Header
from vision_msgs.msg import Detection2DArray
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
import os
from ament_index_python.packages import get_package_share_directory
import message_filters
from typing import Optional, Tuple, List, Dict

# TF2 imports for coordinate transformation
import tf2_ros
import tf2_geometry_msgs


class Object3DDetectorNode(Node):
    def __init__(self):
        super().__init__('object_3d_detector')
        
        # Declare parameters
        self.declare_parameter('model_path', '')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('device', 'cuda')
        self.declare_parameter('color_topic', '/camera/camera/color/image_raw')
        # IMPORTANT: Use aligned depth for accurate 3D projection
        self.declare_parameter('depth_topic', '/camera/camera/aligned_depth_to_color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera/color/camera_info')
        self.declare_parameter('output_pose_topic', '/detected_object/pose')
        self.declare_parameter('output_pose_camera_topic', '/detected_object/pose_camera')
        self.declare_parameter('output_poses_topic', '/detected_objects/poses')
        self.declare_parameter('output_image_topic', '/yolo/image_3d_detections')
        # Camera optical frame (poses are first computed here)
        self.declare_parameter('camera_frame', 'camera_color_optical_frame')
        # Robot base frame (poses transformed here for MoveIt)
        self.declare_parameter('robot_base_frame', 'base_link')
        self.declare_parameter('target_class', '')  # Empty means detect all
        self.declare_parameter('depth_scale', 0.001)  # RealSense depth is in mm
        self.declare_parameter('min_depth', 0.1)  # Minimum valid depth in meters
        self.declare_parameter('max_depth', 2.0)  # Maximum valid depth in meters
        self.declare_parameter('depth_window_size', 5)  # Window for median depth sampling
        self.declare_parameter('tf_timeout', 0.5)  # TF lookup timeout in seconds
        
        # Get parameters
        model_path = self.get_parameter('model_path').value
        self.conf_threshold = self.get_parameter('confidence_threshold').value
        self.device = self.get_parameter('device').value
        color_topic = self.get_parameter('color_topic').value
        depth_topic = self.get_parameter('depth_topic').value
        camera_info_topic = self.get_parameter('camera_info_topic').value
        output_pose_topic = self.get_parameter('output_pose_topic').value
        output_pose_camera_topic = self.get_parameter('output_pose_camera_topic').value
        output_poses_topic = self.get_parameter('output_poses_topic').value
        output_image_topic = self.get_parameter('output_image_topic').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.robot_base_frame = self.get_parameter('robot_base_frame').value
        self.target_class = self.get_parameter('target_class').value
        self.depth_scale = self.get_parameter('depth_scale').value
        self.min_depth = self.get_parameter('min_depth').value
        self.max_depth = self.get_parameter('max_depth').value
        self.depth_window_size = self.get_parameter('depth_window_size').value
        self.tf_timeout = self.get_parameter('tf_timeout').value
        
        # Find model path
        if not model_path:
            try:
                pkg_share = get_package_share_directory('yolo')
                model_path = os.path.join(pkg_share, 'models', 'best.pt')
            except Exception:
                model_path = os.path.join(
                    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                    'best.pt'
                )
        
        if not os.path.exists(model_path):
            model_path = '/home/peru0002/ws_moveit/src/yolo/best.pt'
        
        self.get_logger().info(f'Loading YOLO model from: {model_path}')
        self.get_logger().info(f'Using device: {self.device}')
        
        # Load YOLO model
        self.model = YOLO(model_path)
        self.model.to(self.device)
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # Camera intrinsics (will be updated from camera_info)
        self.camera_matrix = None
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None
        
        # TF2 buffer and listener for coordinate transformation
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # QoS profile
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Subscribe to camera info (reliable QoS)
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            camera_info_topic,
            self.camera_info_callback,
            10
        )
        
        # Synchronized subscribers for color and depth
        self.color_sub = message_filters.Subscriber(
            self, Image, color_topic, qos_profile=qos_profile)
        self.depth_sub = message_filters.Subscriber(
            self, Image, depth_topic, qos_profile=qos_profile)
        
        # Time synchronizer
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.color_sub, self.depth_sub],
            queue_size=10,
            slop=0.1
        )
        self.ts.registerCallback(self.synced_callback)
        
        # Publishers
        # Pose in robot base frame (for MoveIt)
        self.pose_pub = self.create_publisher(PoseStamped, output_pose_topic, 10)
        # Pose in camera frame (for debugging)
        self.pose_camera_pub = self.create_publisher(PoseStamped, output_pose_camera_topic, 10)
        self.poses_pub = self.create_publisher(PoseArray, output_poses_topic, 10)
        self.image_pub = self.create_publisher(Image, output_image_topic, 10)
        
        # Store latest detection for external queries
        self.latest_detection = None
        self.latest_detections = []
        self.tf_available = False
        
        self.get_logger().info('3D Object Detector initialized!')
        self.get_logger().info(f'  Color topic: {color_topic}')
        self.get_logger().info(f'  Depth topic: {depth_topic}')
        self.get_logger().info(f'  Output pose topic (base_link): {output_pose_topic}')
        self.get_logger().info(f'  Output pose topic (camera): {output_pose_camera_topic}')
        self.get_logger().info(f'  Camera frame: {self.camera_frame}')
        self.get_logger().info(f'  Robot base frame: {self.robot_base_frame}')
        self.get_logger().info(f'  Target class: {self.target_class if self.target_class else "ALL"}')
    
    def camera_info_callback(self, msg: CameraInfo):
        """Extract camera intrinsics from camera info"""
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.fx = self.camera_matrix[0, 0]
            self.fy = self.camera_matrix[1, 1]
            self.cx = self.camera_matrix[0, 2]
            self.cy = self.camera_matrix[1, 2]
            self.get_logger().info(f'Camera intrinsics received: fx={self.fx:.2f}, fy={self.fy:.2f}, cx={self.cx:.2f}, cy={self.cy:.2f}')
    
    def transform_pose_to_base(self, pose_camera: PoseStamped) -> Optional[PoseStamped]:
        """
        Transform a pose from camera frame to robot base frame.
        
        This is CRITICAL for MoveIt - all planning happens in base_link frame.
        
        Args:
            pose_camera: PoseStamped in camera_color_optical_frame
            
        Returns:
            PoseStamped in base_link frame, or None if transform failed
        """
        try:
            # Check if transform is available
            if not self.tf_buffer.can_transform(
                self.robot_base_frame,
                pose_camera.header.frame_id,
                rclpy.time.Time(),
                timeout=Duration(seconds=self.tf_timeout)
            ):
                if not self.tf_available:
                    self.get_logger().warn(
                        f'TF not available: {pose_camera.header.frame_id} -> {self.robot_base_frame}. '
                        f'Make sure camera is connected to robot TF tree!'
                    )
                return None
            
            # Transform pose
            pose_base = self.tf_buffer.transform(
                pose_camera,
                self.robot_base_frame,
                timeout=Duration(seconds=self.tf_timeout)
            )
            
            if not self.tf_available:
                self.tf_available = True
                self.get_logger().info(
                    f'TF available: {pose_camera.header.frame_id} -> {self.robot_base_frame}'
                )
            
            return pose_base
            
        except tf2_ros.TransformException as e:
            self.get_logger().warn(f'TF transform failed: {str(e)}')
            return None
    
    def pixel_to_3d(self, u: int, v: int, depth: float) -> Optional[Tuple[float, float, float]]:
        """
        Convert pixel coordinates and depth to 3D point in camera frame.
        
        Args:
            u: pixel x coordinate
            v: pixel y coordinate
            depth: depth value in meters
        
        Returns:
            (x, y, z) in camera frame, or None if invalid
        """
        if self.fx is None:
            return None
        
        if depth < self.min_depth or depth > self.max_depth:
            return None
        
        # Convert from pixel to camera coordinates
        x = (u - self.cx) * depth / self.fx
        y = (v - self.cy) * depth / self.fy
        z = depth
        
        return (x, y, z)
    
    def get_depth_at_point(self, depth_image: np.ndarray, u: int, v: int, 
                           window_size: int = None) -> Optional[float]:
        """
        Get depth at a point using a window to handle noise.
        
        Using median depth in a window is CRITICAL for stable 3D positions.
        Single-pixel depth is too noisy for reliable grasping.
        
        Args:
            depth_image: depth image in raw format
            u: pixel x coordinate
            v: pixel y coordinate
            window_size: size of window to sample depth (uses param if None)
        
        Returns:
            Median depth in meters, or None if invalid
        """
        if window_size is None:
            window_size = self.depth_window_size
            
        h, w = depth_image.shape[:2]
        
        # Clamp coordinates to image bounds
        u = np.clip(u, 0, w - 1)
        v = np.clip(v, 0, h - 1)
        
        # Create window bounds
        half = window_size // 2
        u_min = max(0, u - half)
        u_max = min(w, u + half + 1)
        v_min = max(0, v - half)
        v_max = min(h, v + half + 1)
        
        # Extract window
        window = depth_image[v_min:v_max, u_min:u_max]
        
        # Get valid depth values
        valid_depths = window[window > 0]
        
        if len(valid_depths) == 0:
            return None
        
        # Use median for robustness
        depth = np.median(valid_depths) * self.depth_scale
        
        return depth
    
    def synced_callback(self, color_msg: Image, depth_msg: Image):
        """Process synchronized color and depth images"""
        if self.camera_matrix is None:
            self.get_logger().warn('Camera intrinsics not yet received, skipping...')
            return
        
        try:
            # Convert images
            color_image = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding='bgr8')
            
            # Handle different depth encodings
            if depth_msg.encoding == '16UC1':
                depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='16UC1')
            elif depth_msg.encoding == '32FC1':
                depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='32FC1')
                self.depth_scale = 1.0  # Already in meters
            else:
                depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
            
            # Run YOLO detection
            results = self.model.predict(
                color_image,
                conf=self.conf_threshold,
                verbose=False
            )
            
            # Process detections
            detections_3d = []
            annotated_frame = color_image.copy()
            
            for result in results:
                for box in result.boxes:
                    class_id = int(box.cls[0])
                    class_name = self.model.names[class_id]
                    confidence = float(box.conf[0])
                    
                    # Filter by target class if specified
                    if self.target_class and class_name != self.target_class:
                        continue
                    
                    # Get bounding box
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
                    
                    # Calculate center of bounding box
                    center_u = int((x1 + x2) / 2)
                    center_v = int((y1 + y2) / 2)
                    
                    # Get depth at center
                    depth = self.get_depth_at_point(depth_image, center_u, center_v)
                    
                    if depth is None:
                        self.get_logger().debug(f'No valid depth for {class_name} at ({center_u}, {center_v})')
                        continue
                    
                    # Convert to 3D
                    point_3d = self.pixel_to_3d(center_u, center_v, depth)
                    
                    if point_3d is None:
                        continue
                    
                    x, y, z = point_3d
                    
                    detection = {
                        'class_name': class_name,
                        'confidence': confidence,
                        'bbox': (x1, y1, x2, y2),
                        'center_2d': (center_u, center_v),
                        'position_3d': (x, y, z),
                        'depth': depth
                    }
                    detections_3d.append(detection)
                    
                    # Draw on image
                    cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    
                    # Add label with 3D position
                    label = f'{class_name}: ({x:.3f}, {y:.3f}, {z:.3f})m'
                    label_size, _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)
                    cv2.rectangle(annotated_frame, 
                                  (x1, y1 - label_size[1] - 10), 
                                  (x1 + label_size[0], y1), 
                                  (0, 255, 0), -1)
                    cv2.putText(annotated_frame, label, (x1, y1 - 5),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
                    
                    # Draw center point
                    cv2.circle(annotated_frame, (center_u, center_v), 5, (0, 0, 255), -1)
                    
                    self.get_logger().info(
                        f'Detected {class_name} at 3D position: x={x:.3f}, y={y:.3f}, z={z:.3f} m'
                    )
            
            # Publish results
            if detections_3d:
                # Publish single best detection (highest confidence)
                best_detection = max(detections_3d, key=lambda d: d['confidence'])
                
                # Create pose in camera frame
                pose_camera = PoseStamped()
                pose_camera.header.stamp = self.get_clock().now().to_msg()
                pose_camera.header.frame_id = self.camera_frame
                pose_camera.pose.position.x = best_detection['position_3d'][0]
                pose_camera.pose.position.y = best_detection['position_3d'][1]
                pose_camera.pose.position.z = best_detection['position_3d'][2]
                # Identity orientation - object orientation cannot be determined from 2D
                pose_camera.pose.orientation.w = 1.0
                
                # Publish pose in camera frame (for debugging)
                self.pose_camera_pub.publish(pose_camera)
                
                # Transform to robot base frame (CRITICAL for MoveIt)
                pose_base = self.transform_pose_to_base(pose_camera)
                
                if pose_base is not None:
                    # Publish pose in base_link frame (for MoveIt)
                    self.pose_pub.publish(pose_base)
                    
                    self.get_logger().info(
                        f'Detection in base_link: x={pose_base.pose.position.x:.3f}, '
                        f'y={pose_base.pose.position.y:.3f}, z={pose_base.pose.position.z:.3f} m'
                    )
                else:
                    # If TF not available, publish in camera frame with warning
                    self.pose_pub.publish(pose_camera)
                    self.get_logger().warn(
                        'TF not available - publishing pose in camera frame. '
                        'MoveIt planning may fail!'
                    )
                
                # Publish all detections in camera frame
                poses_msg = PoseArray()
                poses_msg.header = pose_camera.header
                for det in detections_3d:
                    pose = Pose()
                    pose.position.x = det['position_3d'][0]
                    pose.position.y = det['position_3d'][1]
                    pose.position.z = det['position_3d'][2]
                    pose.orientation.w = 1.0
                    poses_msg.poses.append(pose)
                self.poses_pub.publish(poses_msg)
                
                self.latest_detection = best_detection
                self.latest_detections = detections_3d
            
            # Publish annotated image
            annotated_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding='bgr8')
            annotated_msg.header = color_msg.header
            self.image_pub.publish(annotated_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error processing images: {str(e)}')
            import traceback
            self.get_logger().error(traceback.format_exc())


def main(args=None):
    rclpy.init(args=args)
    node = Object3DDetectorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
