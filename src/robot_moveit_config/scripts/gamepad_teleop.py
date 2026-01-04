#!/usr/bin/env python3
"""
Gamepad Teleop for 4-DOF Robotic Arm using MoveIt Servo
Optimized for butter-smooth control with input filtering
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import TwistStamped
from control_msgs.msg import JointJog
from std_srvs.srv import Trigger
import numpy as np
from collections import deque


class SmoothFilter:
    """Exponential moving average filter for smooth control"""
    def __init__(self, alpha=0.3, deadzone=0.1):
        self.alpha = alpha  # Lower = smoother (0.1-0.5 typical)
        self.deadzone = deadzone
        self.value = 0.0
        
    def update(self, raw_value):
        # Apply deadzone
        if abs(raw_value) < self.deadzone:
            raw_value = 0.0
        
        # Exponential smoothing
        self.value = self.alpha * raw_value + (1 - self.alpha) * self.value
        
        # Small value cutoff to prevent drift
        if abs(self.value) < 0.01:
            self.value = 0.0
            
        return self.value


class GamepadTeleop(Node):
    """
    Gamepad teleoperation with butter-smooth control
    
    Controls (Xbox layout):
    - Left Stick X/Y: Move end effector X/Y
    - Right Stick Y: Move end effector Z
    - Right Stick X: Rotate wrist (joint 4)
    - LB/RB: Switch between Cartesian and Joint mode
    - A: Enable/Disable servo
    - B: Emergency stop
    - D-Pad: Joint jog (in joint mode)
    """
    
    def __init__(self):
        super().__init__('gamepad_teleop')
        
        # Parameters for tuning
        self.declare_parameter('linear_speed', 0.15)  # m/s - slower = smoother
        self.declare_parameter('angular_speed', 0.3)  # rad/s
        self.declare_parameter('joint_speed', 0.3)    # rad/s
        self.declare_parameter('smoothing_alpha', 0.25)  # 0.1-0.5 (lower=smoother)
        self.declare_parameter('deadzone', 0.12)  # Stick deadzone
        self.declare_parameter('frame_id', 'base_link')
        
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.joint_speed = self.get_parameter('joint_speed').value
        self.smoothing_alpha = self.get_parameter('smoothing_alpha').value
        self.deadzone = self.get_parameter('deadzone').value
        self.frame_id = self.get_parameter('frame_id').value
        
        # Smooth filters for each axis
        self.filters = {
            'lx': SmoothFilter(self.smoothing_alpha, self.deadzone),
            'ly': SmoothFilter(self.smoothing_alpha, self.deadzone),
            'rx': SmoothFilter(self.smoothing_alpha, self.deadzone),
            'ry': SmoothFilter(self.smoothing_alpha, self.deadzone),
        }
        
        # State
        self.cartesian_mode = True  # Start in Cartesian mode
        self.servo_enabled = False
        self.last_buttons = []
        
        # Joint names for joint jog mode
        self.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4']
        
        # QoS for reliable communication
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        
        # Subscribe to joy
        self.joy_sub = self.create_subscription(
            Joy, '/joy', self.joy_callback, qos)
        
        # Publishers for MoveIt Servo (use correct topic names)
        self.twist_pub = self.create_publisher(
            TwistStamped, '/servo_node/delta_twist_cmds', qos)
        self.joint_pub = self.create_publisher(
            JointJog, '/servo_node/delta_joint_cmds', qos)
        
        # Service clients for servo control
        self.start_servo_client = self.create_client(
            Trigger, '/servo_node/start_servo')
        self.stop_servo_client = self.create_client(
            Trigger, '/servo_node/stop_servo')
        
        # Timer for publishing commands at fixed rate (smoother than event-driven)
        self.publish_rate = 50.0  # Hz
        self.timer = self.create_timer(1.0/self.publish_rate, self.publish_command)
        
        # Store latest joy state
        self.latest_joy = None
        
        self.get_logger().info('='*50)
        self.get_logger().info('GAMEPAD TELEOP - Butter Smooth Control')
        self.get_logger().info('='*50)
        self.get_logger().info('Controls:')
        self.get_logger().info('  Left Stick  : Move X/Y')
        self.get_logger().info('  Right Stick Y: Move Z')
        self.get_logger().info('  Right Stick X: Rotate wrist')
        self.get_logger().info('  A Button    : Start servo')
        self.get_logger().info('  B Button    : Stop servo')
        self.get_logger().info('  LB/RB       : Toggle Cartesian/Joint mode')
        self.get_logger().info('  D-Pad       : Joint jog (in joint mode)')
        self.get_logger().info('='*50)
        self.get_logger().info(f'Speed: linear={self.linear_speed}, angular={self.angular_speed}')
        self.get_logger().info(f'Smoothing: alpha={self.smoothing_alpha}, deadzone={self.deadzone}')
        
    def joy_callback(self, msg):
        """Store latest joy message"""
        self.latest_joy = msg
        
        # Handle button presses (edge detection)
        if len(self.last_buttons) == len(msg.buttons):
            # A button (index 0) - Start servo
            if msg.buttons[0] == 1 and self.last_buttons[0] == 0:
                self.start_servo()
            
            # B button (index 1) - Stop servo
            if msg.buttons[1] == 1 and self.last_buttons[1] == 0:
                self.stop_servo()
            
            # LB (index 4) or RB (index 5) - Toggle mode
            if (msg.buttons[4] == 1 and self.last_buttons[4] == 0) or \
               (msg.buttons[5] == 1 and self.last_buttons[5] == 0):
                self.cartesian_mode = not self.cartesian_mode
                mode = "CARTESIAN" if self.cartesian_mode else "JOINT"
                self.get_logger().info(f'Switched to {mode} mode')
        
        self.last_buttons = list(msg.buttons)
    
    def publish_command(self):
        """Publish smoothed commands at fixed rate"""
        if self.latest_joy is None or not self.servo_enabled:
            return
        
        msg = self.latest_joy
        
        # Get filtered axis values
        lx = self.filters['lx'].update(msg.axes[0] if len(msg.axes) > 0 else 0.0)
        ly = self.filters['ly'].update(msg.axes[1] if len(msg.axes) > 1 else 0.0)
        rx = self.filters['rx'].update(msg.axes[3] if len(msg.axes) > 3 else 0.0)
        ry = self.filters['ry'].update(msg.axes[4] if len(msg.axes) > 4 else 0.0)
        
        # D-Pad for joint mode
        dpad_x = msg.axes[6] if len(msg.axes) > 6 else 0.0
        dpad_y = msg.axes[7] if len(msg.axes) > 7 else 0.0
        
        if self.cartesian_mode:
            # Cartesian (Twist) control
            twist = TwistStamped()
            twist.header.stamp = self.get_clock().now().to_msg()
            twist.header.frame_id = self.frame_id
            
            # Linear motion (X, Y, Z)
            twist.twist.linear.x = ly * self.linear_speed  # Forward/back
            twist.twist.linear.y = lx * self.linear_speed  # Left/right
            twist.twist.linear.z = ry * self.linear_speed  # Up/down
            
            # Angular motion (wrist rotation)
            twist.twist.angular.z = rx * self.angular_speed
            
            # Only publish if there's actual motion
            if any([abs(twist.twist.linear.x) > 0.001,
                    abs(twist.twist.linear.y) > 0.001,
                    abs(twist.twist.linear.z) > 0.001,
                    abs(twist.twist.angular.z) > 0.001]):
                self.twist_pub.publish(twist)
        else:
            # Joint jog control
            joint_jog = JointJog()
            joint_jog.header.stamp = self.get_clock().now().to_msg()
            joint_jog.header.frame_id = self.frame_id
            
            # Use D-pad and sticks for joint control
            velocities = [
                lx * self.joint_speed,   # Joint 1 - base rotation
                ly * self.joint_speed,   # Joint 2 - shoulder
                ry * self.joint_speed,   # Joint 3 - elbow
                rx * self.joint_speed,   # Joint 4 - wrist
            ]
            
            joint_jog.joint_names = self.joint_names
            joint_jog.velocities = velocities
            
            if any(abs(v) > 0.001 for v in velocities):
                self.joint_pub.publish(joint_jog)
    
    def start_servo(self):
        """Start MoveIt Servo"""
        if not self.start_servo_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Start servo service not available')
            return
        
        future = self.start_servo_client.call_async(Trigger.Request())
        future.add_done_callback(self.start_servo_callback)
    
    def start_servo_callback(self, future):
        try:
            result = future.result()
            if result.success:
                self.servo_enabled = True
                self.get_logger().info('✓ Servo ENABLED - Ready to move!')
            else:
                self.get_logger().error(f'Failed to start servo: {result.message}')
        except Exception as e:
            self.get_logger().error(f'Start servo error: {e}')
    
    def stop_servo(self):
        """Stop MoveIt Servo"""
        if not self.stop_servo_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Stop servo service not available')
            return
        
        future = self.stop_servo_client.call_async(Trigger.Request())
        future.add_done_callback(self.stop_servo_callback)
    
    def stop_servo_callback(self, future):
        try:
            result = future.result()
            self.servo_enabled = False
            self.get_logger().info('✗ Servo DISABLED - Arm stopped')
            # Reset filters
            for f in self.filters.values():
                f.value = 0.0
        except Exception as e:
            self.get_logger().error(f'Stop servo error: {e}')


def main():
    rclpy.init()
    node = GamepadTeleop()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
