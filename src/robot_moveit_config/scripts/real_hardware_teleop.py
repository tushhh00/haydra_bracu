#!/usr/bin/env python3
"""
Real Hardware Teleop for 4-DOF Robotic Arm
Works with micro-ROS on ESP32-S3

Publishes joint commands to /arm_controller/commands
Subscribes to /joint_states for feedback
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Joy, JointState
from std_msgs.msg import Float64MultiArray
import numpy as np


class SmoothFilter:
    """Low-pass filter for smooth control"""
    def __init__(self, cutoff_hz=3.0, sample_rate=50.0):
        rc = 1.0 / (2.0 * np.pi * cutoff_hz)
        dt = 1.0 / sample_rate
        self.alpha = dt / (rc + dt)
        self.value = 0.0
        self.deadzone = 0.08
        
    def update(self, raw):
        if abs(raw) < self.deadzone:
            raw = 0.0
        self.value += self.alpha * (raw - self.value)
        if abs(self.value) < 0.005:
            self.value = 0.0
        return self.value
    
    def reset(self):
        self.value = 0.0


class RealHardwareTeleop(Node):
    """
    Gamepad teleop for real hardware via micro-ROS
    
    Controls (Xbox):
    - Left Stick X: Joint 1 (base rotation)
    - Left Stick Y: Joint 2 (shoulder)
    - Right Stick Y: Joint 3 (elbow)
    - Right Stick X: Joint 4 (wrist)
    - LT/RT: Speed control
    - A: Enable
    - B: Disable (emergency stop)
    - Start: Go home
    - Back: Calibrate (set current as zero)
    """
    
    def __init__(self):
        super().__init__('real_hardware_teleop')
        
        # Parameters
        self.declare_parameter('max_velocity', 0.3)  # rad/s
        self.declare_parameter('filter_cutoff', 3.0)  # Hz
        self.declare_parameter('publish_rate', 50.0)  # Hz
        
        self.max_vel = self.get_parameter('max_velocity').value
        filter_cutoff = self.get_parameter('filter_cutoff').value
        publish_rate = self.get_parameter('publish_rate').value
        
        # Joint configuration
        self.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4']
        self.joint_limits = {
            'joint_1': (-1.745, 1.745),
            'joint_2': (-0.873, 1.745),
            'joint_3': (-0.873, 1.745),
            'joint_4': (-1.571, 1.571),
        }
        
        # State
        self.current_positions = [0.0, 0.0, 0.0, 0.0]
        self.target_positions = [0.0, 0.0, 0.0, 0.0]
        self.enabled = False
        self.speed_mult = 1.0
        self.last_buttons = []
        self.connected = False
        
        # Filters
        self.filters = [
            SmoothFilter(filter_cutoff, publish_rate),
            SmoothFilter(filter_cutoff, publish_rate),
            SmoothFilter(filter_cutoff, publish_rate),
            SmoothFilter(filter_cutoff, publish_rate),
        ]
        
        # QoS for real-time
        qos = QoSProfile(
            depth=10, 
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )
        
        # Subscribers
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_cb, qos)
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_cb, qos)
        
        # Publisher for ESP32 commands
        self.cmd_pub = self.create_publisher(
            Float64MultiArray, '/arm_controller/commands', qos)
        
        # Control loop timer
        self.dt = 1.0 / publish_rate
        self.timer = self.create_timer(self.dt, self.control_loop)
        
        # Connection check timer
        self.last_joint_state_time = None
        self.connection_timer = self.create_timer(1.0, self.check_connection)
        
        self.print_banner()
    
    def print_banner(self):
        self.get_logger().info('='*55)
        self.get_logger().info('   REAL HARDWARE TELEOP - ESP32-S3 + micro-ROS')
        self.get_logger().info('='*55)
        self.get_logger().info('Controls:')
        self.get_logger().info('  Left Stick X  : Joint 1 (base)')
        self.get_logger().info('  Left Stick Y  : Joint 2 (shoulder)')
        self.get_logger().info('  Right Stick Y : Joint 3 (elbow)')
        self.get_logger().info('  Right Stick X : Joint 4 (wrist)')
        self.get_logger().info('  LT/RT         : Slow/Fast')
        self.get_logger().info('  A Button      : ENABLE')
        self.get_logger().info('  B Button      : EMERGENCY STOP')
        self.get_logger().info('  Start         : Go HOME')
        self.get_logger().info('  Back          : Set current as zero')
        self.get_logger().info('='*55)
        self.get_logger().info(f'Max velocity: {self.max_vel} rad/s')
        self.get_logger().info('')
        self.get_logger().warn('>>> Waiting for ESP32 connection... <<<')
    
    def check_connection(self):
        """Check if ESP32 is connected"""
        if self.last_joint_state_time is None:
            if not self.connected:
                self.get_logger().warn('No joint_states from ESP32. Is micro-ROS agent running?')
            return
        
        age = (self.get_clock().now() - self.last_joint_state_time).nanoseconds / 1e9
        
        if age > 1.0:
            if self.connected:
                self.get_logger().error('Lost connection to ESP32!')
                self.connected = False
                self.enabled = False
        else:
            if not self.connected:
                self.get_logger().info('✓ Connected to ESP32!')
                self.get_logger().info('>>> Press A to ENABLE <<<')
                self.connected = True
    
    def joint_state_cb(self, msg):
        """Update from ESP32"""
        self.last_joint_state_time = self.get_clock().now()
        
        for i, name in enumerate(self.joint_names):
            if name in msg.name:
                idx = msg.name.index(name)
                self.current_positions[i] = msg.position[idx]
    
    def joy_cb(self, msg):
        """Process gamepad input"""
        if not self.connected:
            return
        
        # Button edge detection
        if len(self.last_buttons) == len(msg.buttons):
            # A (0) - Enable
            if msg.buttons[0] == 1 and self.last_buttons[0] == 0:
                self.enabled = True
                self.target_positions = list(self.current_positions)
                self.get_logger().info('✓ ENABLED - Move carefully!')
            
            # B (1) - Emergency stop
            if msg.buttons[1] == 1 and self.last_buttons[1] == 0:
                self.emergency_stop()
            
            # Start (7) - Home
            if msg.buttons[7] == 1 and self.last_buttons[7] == 0:
                self.go_home()
            
            # Back (6) - Calibrate
            if msg.buttons[6] == 1 and self.last_buttons[6] == 0:
                self.calibrate()
        
        self.last_buttons = list(msg.buttons)
        
        if not self.enabled:
            return
        
        # Get axes
        axes = msg.axes if len(msg.axes) >= 6 else [0.0] * 6
        
        # Speed from triggers
        lt = (1.0 - axes[2]) / 2.0 if len(axes) > 2 else 0.0
        rt = (1.0 - axes[5]) / 2.0 if len(axes) > 5 else 0.0
        self.speed_mult = 0.2 + 0.8 * (rt - lt * 0.5)
        self.speed_mult = max(0.1, min(1.0, self.speed_mult))
        
        # Filter joystick inputs
        raw_inputs = [
            axes[0],  # Left X -> Joint 1
            axes[1],  # Left Y -> Joint 2
            axes[4] if len(axes) > 4 else 0.0,  # Right Y -> Joint 3
            axes[3] if len(axes) > 3 else 0.0,  # Right X -> Joint 4
        ]
        
        for i, raw in enumerate(raw_inputs):
            self.filters[i].update(raw)
    
    def control_loop(self):
        """Main control loop"""
        if not self.enabled or not self.connected:
            return
        
        # Calculate velocities
        velocities = [f.value * self.max_vel * self.speed_mult for f in self.filters]
        
        # Skip if no motion
        if all(abs(v) < 0.001 for v in velocities):
            return
        
        # Update targets
        for i in range(4):
            new_pos = self.target_positions[i] + velocities[i] * self.dt
            limits = self.joint_limits[self.joint_names[i]]
            new_pos = max(limits[0], min(limits[1], new_pos))
            self.target_positions[i] = new_pos
        
        # Publish to ESP32
        cmd = Float64MultiArray()
        cmd.data = self.target_positions
        self.cmd_pub.publish(cmd)
    
    def emergency_stop(self):
        """Emergency stop - hold current position"""
        self.get_logger().warn('!!! EMERGENCY STOP !!!')
        self.enabled = False
        
        # Send current position to hold
        cmd = Float64MultiArray()
        cmd.data = list(self.current_positions)
        self.cmd_pub.publish(cmd)
        
        self.target_positions = list(self.current_positions)
        for f in self.filters:
            f.reset()
    
    def go_home(self):
        """Go to home position"""
        self.get_logger().info('Going HOME...')
        self.enabled = False
        
        # Send home position
        cmd = Float64MultiArray()
        cmd.data = [0.0, 0.0, 0.0, 0.0]
        self.cmd_pub.publish(cmd)
        
        self.target_positions = [0.0, 0.0, 0.0, 0.0]
        for f in self.filters:
            f.reset()
        
        self.get_logger().info('Home command sent. Press A to re-enable.')
    
    def calibrate(self):
        """Set current position as zero (home)"""
        self.get_logger().warn('Calibrating - current position is now zero')
        self.enabled = False
        
        # This would need ESP32 firmware support to actually zero the encoders
        self.target_positions = [0.0, 0.0, 0.0, 0.0]
        for f in self.filters:
            f.reset()


def main():
    rclpy.init()
    node = RealHardwareTeleop()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        # Send stop command
        cmd = Float64MultiArray()
        cmd.data = list(node.current_positions)
        node.cmd_pub.publish(cmd)
        
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
