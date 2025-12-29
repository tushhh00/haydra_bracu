#!/home/peru0002/ws_moveit/src/yolo/.venv/bin/python
"""
YOLOv11 Object Detection ROS2 Node
Subscribes to camera image topic and publishes detection results
"""

import sys
sys.path.insert(0, '/home/peru0002/ws_moveit/src/yolo/.venv/lib/python3.12/site-packages')

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from std_msgs.msg import Header
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
import os
from ament_index_python.packages import get_package_share_directory


class YoloDetectorNode(Node):
    def __init__(self):
        super().__init__('yolo_detector')
        
        # Declare parameters
        self.declare_parameter('model_path', '')
        self.declare_parameter('confidence_threshold', 0.25)
        self.declare_parameter('device', 'cuda')  # 'cuda' or 'cpu'
        self.declare_parameter('input_topic', '/camera/image_raw')
        self.declare_parameter('output_topic', '/yolo/detections')
        self.declare_parameter('output_image_topic', '/yolo/image_detections')
        self.declare_parameter('publish_annotated_image', True)
        
        # Get parameters
        model_path = self.get_parameter('model_path').value
        self.conf_threshold = self.get_parameter('confidence_threshold').value
        self.device = self.get_parameter('device').value
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        output_image_topic = self.get_parameter('output_image_topic').value
        self.publish_annotated = self.get_parameter('publish_annotated_image').value
        
        # Find model path
        if not model_path:
            # Try to find in package share directory
            try:
                pkg_share = get_package_share_directory('yolo')
                model_path = os.path.join(pkg_share, 'models', 'best.pt')
            except Exception:
                # Fallback to source directory
                model_path = os.path.join(
                    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                    'best.pt'
                )
        
        if not os.path.exists(model_path):
            # Try workspace source path
            model_path = '/home/peru0002/ws_moveit/src/yolo/best.pt'
        
        self.get_logger().info(f'Loading YOLO model from: {model_path}')
        self.get_logger().info(f'Using device: {self.device}')
        
        # Load YOLO model
        self.model = YOLO(model_path)
        self.model.to(self.device)
        
        # CV Bridge for image conversion
        self.bridge = CvBridge()
        
        # QoS profile for camera images
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Subscriber
        self.image_sub = self.create_subscription(
            Image,
            input_topic,
            self.image_callback,
            qos_profile
        )
        
        # Publishers
        self.detection_pub = self.create_publisher(
            Detection2DArray,
            output_topic,
            10
        )
        
        if self.publish_annotated:
            self.image_pub = self.create_publisher(
                Image,
                output_image_topic,
                10
            )
        
        self.get_logger().info(f'YOLO Detector initialized!')
        self.get_logger().info(f'  Subscribing to: {input_topic}')
        self.get_logger().info(f'  Publishing detections to: {output_topic}')
        if self.publish_annotated:
            self.get_logger().info(f'  Publishing annotated images to: {output_image_topic}')
    
    def image_callback(self, msg: Image):
        """Process incoming image and run YOLO detection"""
        try:
            # Convert ROS Image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Run YOLO detection
            results = self.model.predict(
                cv_image,
                conf=self.conf_threshold,
                verbose=False
            )
            
            # Create Detection2DArray message
            detection_array = Detection2DArray()
            detection_array.header = msg.header
            
            for result in results:
                for box in result.boxes:
                    detection = Detection2D()
                    detection.header = msg.header
                    
                    # Bounding box (center x, y, width, height)
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                    detection.bbox.center.position.x = float((x1 + x2) / 2)
                    detection.bbox.center.position.y = float((y1 + y2) / 2)
                    detection.bbox.size_x = float(x2 - x1)
                    detection.bbox.size_y = float(y2 - y1)
                    
                    # Class and confidence
                    hypothesis = ObjectHypothesisWithPose()
                    hypothesis.hypothesis.class_id = self.model.names[int(box.cls[0])]
                    hypothesis.hypothesis.score = float(box.conf[0])
                    detection.results.append(hypothesis)
                    
                    detection_array.detections.append(detection)
            
            # Publish detections
            self.detection_pub.publish(detection_array)
            
            # Publish annotated image
            if self.publish_annotated:
                annotated_frame = results[0].plot()
                annotated_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding='bgr8')
                annotated_msg.header = msg.header
                self.image_pub.publish(annotated_msg)
            
            # Log detections
            if len(detection_array.detections) > 0:
                self.get_logger().debug(f'Detected {len(detection_array.detections)} objects')
                
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
