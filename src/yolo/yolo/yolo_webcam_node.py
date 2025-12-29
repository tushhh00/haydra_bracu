#!/home/peru0002/ws_moveit/src/yolo/.venv/bin/python
"""
YOLOv11 Webcam Detection ROS2 Node
Publishes webcam feed with YOLO detections
"""

import sys
sys.path.insert(0, '/home/peru0002/ws_moveit/src/yolo/.venv/lib/python3.12/site-packages')

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import os


class YoloWebcamNode(Node):
    def __init__(self):
        super().__init__('yolo_webcam')
        
        # Declare parameters
        self.declare_parameter('model_path', '/home/peru0002/ws_moveit/src/yolo/best.pt')
        self.declare_parameter('confidence_threshold', 0.25)
        self.declare_parameter('device', 'cuda')
        self.declare_parameter('camera_id', 0)
        self.declare_parameter('frame_rate', 30.0)
        self.declare_parameter('output_topic', '/yolo/detections')
        self.declare_parameter('output_image_topic', '/yolo/image_raw')
        self.declare_parameter('output_annotated_topic', '/yolo/image_detections')
        
        # Get parameters
        model_path = self.get_parameter('model_path').value
        self.conf_threshold = self.get_parameter('confidence_threshold').value
        self.device = self.get_parameter('device').value
        camera_id = self.get_parameter('camera_id').value
        frame_rate = self.get_parameter('frame_rate').value
        output_topic = self.get_parameter('output_topic').value
        output_image_topic = self.get_parameter('output_image_topic').value
        output_annotated_topic = self.get_parameter('output_annotated_topic').value
        
        self.get_logger().info(f'Loading YOLO model from: {model_path}')
        self.get_logger().info(f'Using device: {self.device}')
        
        # Load YOLO model
        self.model = YOLO(model_path)
        self.model.to(self.device)
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # Open webcam
        self.cap = cv2.VideoCapture(camera_id)
        if not self.cap.isOpened():
            self.get_logger().error(f'Failed to open camera {camera_id}')
            return
        
        self.get_logger().info(f'Camera {camera_id} opened successfully')
        
        # Publishers
        self.detection_pub = self.create_publisher(
            Detection2DArray,
            output_topic,
            10
        )
        
        self.image_pub = self.create_publisher(
            Image,
            output_image_topic,
            10
        )
        
        self.annotated_pub = self.create_publisher(
            Image,
            output_annotated_topic,
            10
        )
        
        # Timer for frame capture
        timer_period = 1.0 / frame_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.get_logger().info(f'YOLO Webcam node started!')
        self.get_logger().info(f'  Publishing raw images to: {output_image_topic}')
        self.get_logger().info(f'  Publishing annotated images to: {output_annotated_topic}')
        self.get_logger().info(f'  Publishing detections to: {output_topic}')
    
    def timer_callback(self):
        """Capture frame, run detection, and publish"""
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Failed to capture frame')
            return
        
        # Get current timestamp
        stamp = self.get_clock().now().to_msg()
        
        # Publish raw image
        raw_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        raw_msg.header.stamp = stamp
        raw_msg.header.frame_id = 'camera_frame'
        self.image_pub.publish(raw_msg)
        
        # Run YOLO detection
        results = self.model.predict(
            frame,
            conf=self.conf_threshold,
            verbose=False
        )
        
        # Create Detection2DArray message
        detection_array = Detection2DArray()
        detection_array.header.stamp = stamp
        detection_array.header.frame_id = 'camera_frame'
        
        for result in results:
            for box in result.boxes:
                detection = Detection2D()
                detection.header.stamp = stamp
                detection.header.frame_id = 'camera_frame'
                
                # Bounding box
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
        annotated_frame = results[0].plot()
        annotated_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding='bgr8')
        annotated_msg.header.stamp = stamp
        annotated_msg.header.frame_id = 'camera_frame'
        self.annotated_pub.publish(annotated_msg)
    
    def destroy_node(self):
        """Clean up resources"""
        if self.cap is not None:
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = YoloWebcamNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
