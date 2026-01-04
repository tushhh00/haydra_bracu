#!/usr/bin/env python3
"""
Smooth Gamepad Teleop for 4-DOF Robotic Arm
Direct joint control with butter-smooth motion using interpolation

This bypasses MoveIt Servo and sends directly to the arm controller
for maximum smoothness and minimal latency.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Joy, JointState
import numpy as np
import math


class SmoothFilter:
    """Butterworth-like low-pass filter for ultra-smooth control"""
    def __init__(self, cutoff_hz=5.0, sample_rate=50.0):
        # Calculate filter coefficient
        rc = 1.0 / (2.0 * np.pi * cutoff_hz)
        dt = 1.0 / sample_rate
        self.alpha = dt / (rc + dt)
        self.value = 0.0
        self.deadzone = 0.08
        
    def update(self, raw):
        # Apply deadzone
        if abs(raw) < self.deadzone:
            raw = 0.0
        # First-order IIR filter
        self.value += self.alpha * (raw - self.value)
        # Prevent drift
        if abs(self.value) < 0.005:
            self.value = 0.0
        return self.value
    
    def reset(self):
        self.value = 0.0


class SmoothGamepadTeleop(Node):
    """
    Butter-smooth gamepad teleoperation using direct trajectory control
    
    Controls (Xbox):
    - Left Stick X: Joint 1 (base rotation)
    - Left Stick Y: Joint 2 (shoulder)
    - Right Stick Y: Joint 3 (elbow)
    - Right Stick X: Joint 4 (wrist)
    - LT/RT: Speed multiplier
    - A: Enable control
    - B: Disable control (stop)
    - Start: Go to home position
    """
    
    def __init__(self):
        super().__init__('smooth_gamepad_teleop')
        
        # Parameters
        self.declare_parameter('max_velocity', 0.5)  # rad/s per joint
        self.declare_parameter('filter_cutoff', 3.0)  # Hz - lower = smoother
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
        
        # Current and target positions
        self.current_positions = [0.0, 0.0, 0.0, 0.0]
        self.target_positions = [0.0, 0.0, 0.0, 0.0]
        
        # Smooth filters for each joystick axis
        self.filters = [
            SmoothFilter(filter_cutoff, publish_rate),
            SmoothFilter(filter_cutoff, publish_rate),
            SmoothFilter(filter_cutoff, publish_rate),
            SmoothFilter(filter_cutoff, publish_rate),
        ]
        
        # State
        self.enabled = False
        self.speed_mult = 1.0
        self.last_buttons = []
        
        # QoS
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        
        # Subscribers
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_cb, qos)
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_cb, qos)
        
        # Publisher - to fake controller joint states (what MoveIt listens to)
        self.joint_state_pub = self.create_publisher(
            JointState, '/move_group/fake_controller_joint_states', qos)
        
        # Also publish to joint_states for direct feedback
        self.joint_state_direct_pub = self.create_publisher(
            JointState, '/joint_states', qos)
        
        # Timer for smooth publishing
        self.dt = 1.0 / publish_rate
        self.timer = self.create_timer(self.dt, self.control_loop)
        
        self.get_logger().info('='*55)
        self.get_logger().info('   SMOOTH GAMEPAD TELEOP - Butter Control')
        self.get_logger().info('='*55)
        self.get_logger().info('Controls:')
        self.get_logger().info('  Left Stick X  : Joint 1 (base rotation)')
        self.get_logger().info('  Left Stick Y  : Joint 2 (shoulder)')
        self.get_logger().info('  Right Stick Y : Joint 3 (elbow)')
        self.get_logger().info('  Right Stick X : Joint 4 (wrist)')
        self.get_logger().info('  LT/RT         : Slow/Fast speed')
        self.get_logger().info('  A Button      : ENABLE control')
        self.get_logger().info('  B Button      : DISABLE (stop)')
        self.get_logger().info('  Start         : Go HOME')
        self.get_logger().info('='*55)
        self.get_logger().info(f'Max velocity: {self.max_vel} rad/s')
        self.get_logger().info(f'Filter cutoff: {filter_cutoff} Hz')
        self.get_logger().info(f'Publish rate: {publish_rate} Hz')
        self.get_logger().info('')
        self.get_logger().info('>>> Press A to ENABLE <<<')
        
    def joint_state_cb(self, msg):
        """Update current joint positions"""
        for i, name in enumerate(self.joint_names):
            if name in msg.name:
                idx = msg.name.index(name)
                self.current_positions[i] = msg.position[idx]
    
    def joy_cb(self, msg):
        """Process joystick input"""
        # Button edge detection
        if len(self.last_buttons) == len(msg.buttons):
            # A (0) - Enable
            if msg.buttons[0] == 1 and self.last_buttons[0] == 0:
                self.enabled = True
                self.target_positions = list(self.current_positions)
                self.get_logger().info('✓ ENABLED - Move the sticks!')
            
            # B (1) - Disable
            if msg.buttons[1] == 1 and self.last_buttons[1] == 0:
                self.enabled = False
                for f in self.filters:
                    f.reset()
                self.get_logger().info('✗ DISABLED - Arm stopped')
            
            # Start (7) - Home
            if msg.buttons[7] == 1 and self.last_buttons[7] == 0:
                self.go_home()
        
        self.last_buttons = list(msg.buttons)
        
        if not self.enabled:
            return
        
        # Get axis values (with button mapping for different controllers)
        axes = msg.axes if len(msg.axes) >= 6 else [0.0] * 6
        
        # Speed multiplier from triggers (LT=slow, RT=fast)
        lt = (1.0 - axes[2]) / 2.0 if len(axes) > 2 else 0.0  # 0-1
        rt = (1.0 - axes[5]) / 2.0 if len(axes) > 5 else 0.0  # 0-1
        self.speed_mult = 0.3 + 0.7 * (rt - lt * 0.5)  # 0.3x to 1.0x
        self.speed_mult = max(0.2, min(1.5, self.speed_mult))
        
        # Apply filters to joystick axes
        raw_inputs = [
            axes[0],  # Left stick X -> Joint 1
            axes[1],  # Left stick Y -> Joint 2
            axes[4] if len(axes) > 4 else 0.0,  # Right stick Y -> Joint 3
            axes[3] if len(axes) > 3 else 0.0,  # Right stick X -> Joint 4
        ]
        
        # Update filtered values
        for i, raw in enumerate(raw_inputs):
            self.filters[i].update(raw)
    
    def control_loop(self):
        """Main control loop - publishes smooth joint states"""
        if not self.enabled:
            return
        
        # Calculate velocity commands from filtered inputs
        velocities = []
        for i, f in enumerate(self.filters):
            vel = f.value * self.max_vel * self.speed_mult
            velocities.append(vel)
        
        # Check if any motion requested
        if all(abs(v) < 0.001 for v in velocities):
            return
        
        # Update target positions with velocity integration
        new_targets = []
        for i in range(4):
            new_pos = self.target_positions[i] + velocities[i] * self.dt
            # Apply joint limits
            limits = self.joint_limits[self.joint_names[i]]
            new_pos = max(limits[0], min(limits[1], new_pos))
            new_targets.append(new_pos)
        
        self.target_positions = new_targets
        
        # Create and publish joint state
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = self.joint_names
        js.position = self.target_positions
        js.velocity = [0.0] * 4
        js.effort = [0.0] * 4
        
        # Publish to both topics for MoveIt and visualization
        self.joint_state_pub.publish(js)
        self.joint_state_direct_pub.publish(js)
    
    def go_home(self):
        """Smoothly go to home position"""
        self.get_logger().info('Going HOME...')
        self.enabled = False
        
        # Smoothly interpolate to home
        home_pos = [0.0, 0.0, 0.0, 0.0]
        steps = 50  # 1 second at 50Hz
        
        start_pos = list(self.target_positions)
        
        for i in range(steps):
            alpha = (i + 1) / steps
            interp_pos = [start_pos[j] + alpha * (home_pos[j] - start_pos[j]) for j in range(4)]
            
            js = JointState()
            js.header.stamp = self.get_clock().now().to_msg()
            js.name = self.joint_names
            js.position = interp_pos
            js.velocity = [0.0] * 4
            js.effort = [0.0] * 4
            
            self.joint_state_pub.publish(js)
            self.joint_state_direct_pub.publish(js)
            
            import time
            time.sleep(self.dt)
        
        self.target_positions = [0.0, 0.0, 0.0, 0.0]
        for f in self.filters:
            f.reset()
        
        self.get_logger().info('Home reached. Press A to re-enable.')


def main():
    rclpy.init()
    node = SmoothGamepadTeleop()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
