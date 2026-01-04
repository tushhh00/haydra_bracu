#!/usr/bin/env python3
"""
Serial Teleop for 4-DOF Robotic Arm + 2 Grippers with ESP32
Uses simple serial protocol (no micro-ROS needed)

ARCHITECTURE:
  This node bridges between two coordinate spaces:
  
  1. URDF/MoveIt Space (Simulation)
     - Joint limits defined in URDF (radians)
     - Used by RViz visualization and MoveIt planning
     - Unchanged and preserved for motion planning
  
  2. Servo Space (Real Robot)
     - Joint limits defined by physical servo constraints (degrees)
     - Applied only when sending commands to ESP32
     - Accounts for mechanical offsets and physical limitations

Protocol:
  Send: "J <base> <j1> <j2> <j3> <grip1> <grip2>\n" (degrees)
  Recv: "S <base> <j1> <j2> <j3> <grip1> <grip2>\n" (degrees)

Controls (Xbox Controller):
  Left Stick X  : Base servo (joint_1)
  Left Stick Y  : Joint 1 (joint_2)
  Right Stick Y : Joint 2 (joint_3)
  Right Stick X : Joint 3 (joint_4)
  LB / RB       : Gripper 1 CLOSE / OPEN
  LT / RT       : Gripper 2 CLOSE / OPEN
  A Button      : ENABLE control
  B Button      : EMERGENCY STOP
  Start         : Go HOME
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy, JointState
import serial
import serial.tools.list_ports
import numpy as np
import threading
import time


# =============================================================================
# JOINT LIMIT CONFIGURATION
# =============================================================================

# URDF Joint Limits (radians) - DO NOT MODIFY
# These match the URDF model and are used for RViz/MoveIt visualization
URDF_LIMITS = {
    'joint_1': (-1.745, 1.745),    # Base: ~±100°
    'joint_2': (-0.873, 1.745),    # Joint 1: ~-50° to +100°
    'joint_3': (-0.873, 1.745),    # Joint 2: ~-50° to +100°
    'joint_4': (-1.571, 1.571),    # Joint 3: ~±90°
}

# Real Robot Servo Limits (degrees) - Adjust for your hardware
# These are the physical constraints of your servo motors
SERVO_LIMITS = {
    'joint_1': (80, 130),     # Base servo: 80° to 130°
    'joint_2': (20, 85),      # Joint 1: 20° to 85°
    'joint_3': (60, 85),      # Joint 2: 60° to 85°
    'joint_4': (0, 180),      # Joint 3: 0° to 180°
    'gripper_1': (0, 60),     # Gripper 1: 0° to 60°
    'gripper_2': (0, 100),    # Gripper 2: 0° to 100°
}

# Real Robot Home Position (degrees) - Adjust for your hardware
# These are the servo angles when the robot is in "home" pose
SERVO_HOME = {
    'joint_1': 98,     # Base home
    'joint_2': 85,     # Joint 1 home
    'joint_3': 85,     # Joint 2 home
    'joint_4': 90,     # Joint 3 home
    'gripper_1': 0,    # Gripper 1 home (open)
    'gripper_2': 0,    # Gripper 2 home (open)
}

# =============================================================================
# COORDINATE SPACE CONVERSION FUNCTIONS
# =============================================================================

def servo_to_urdf(servo_angle: float, joint_name: str) -> float:
    """
    Convert servo angle (degrees) to URDF angle (radians).
    
    The servo HOME position maps to URDF 0 radians.
    Movement away from home is converted degree-for-degree to radians.
    
    Args:
        servo_angle: Current servo position in degrees
        joint_name: Name of the joint (e.g., 'joint_1')
    
    Returns:
        URDF joint angle in radians
    """
    if joint_name not in SERVO_HOME or joint_name not in URDF_LIMITS:
        return 0.0
    
    # Servo home position = URDF 0 radians
    servo_center = SERVO_HOME[joint_name]
    
    # Deviation from home in degrees, converted to radians
    urdf_angle = np.radians(servo_angle - servo_center)
    
    # Clamp to URDF limits
    urdf_min, urdf_max = URDF_LIMITS[joint_name]
    return float(np.clip(urdf_angle, urdf_min, urdf_max))


def urdf_to_servo(urdf_angle: float, joint_name: str) -> float:
    """
    Convert URDF angle (radians) to servo angle (degrees).
    
    URDF 0 radians maps to servo HOME position.
    
    Args:
        urdf_angle: URDF joint angle in radians
        joint_name: Name of the joint (e.g., 'joint_1')
    
    Returns:
        Servo angle in degrees
    """
    if joint_name not in SERVO_HOME or joint_name not in SERVO_LIMITS:
        return 0.0
    
    # URDF 0 = servo home, convert radians to degrees offset
    servo_center = SERVO_HOME[joint_name]
    servo_angle = servo_center + np.degrees(urdf_angle)
    
    # Clamp to servo limits
    servo_min, servo_max = SERVO_LIMITS[joint_name]
    return float(np.clip(servo_angle, servo_min, servo_max))


# =============================================================================
# HELPER CLASSES
# =============================================================================

class SmoothFilter:
    """Low-pass filter for smooth control"""
    def __init__(self, cutoff_hz=5.0, sample_rate=50.0):
        rc = 1.0 / (2.0 * np.pi * cutoff_hz)
        dt = 1.0 / sample_rate
        self.alpha = min(dt / (rc + dt) * 2.0, 0.5)
        self.value = 0.0
        self.deadzone = 0.06
        
    def update(self, raw):
        if abs(raw) < self.deadzone:
            raw = 0.0
        self.value += self.alpha * (raw - self.value)
        if abs(self.value) < 0.005:
            self.value = 0.0
        return self.value
    
    def reset(self):
        self.value = 0.0


# =============================================================================
# MAIN NODE
# =============================================================================

class SerialTeleop(Node):
    def __init__(self):
        super().__init__('serial_teleop')
        
        # Parameters
        self.declare_parameter('serial_port', '')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('max_velocity', 40.0)  # degrees/s in servo space
        self.declare_parameter('gripper_speed', 30.0)  # degrees/s for grippers
        self.declare_parameter('publish_rate', 50.0)
        
        self.baud_rate = self.get_parameter('baud_rate').value
        self.max_vel = self.get_parameter('max_velocity').value
        self.gripper_speed = self.get_parameter('gripper_speed').value
        publish_rate = self.get_parameter('publish_rate').value
        
        # Joint names (must match URDF)
        self.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'gripper_1', 'gripper_2']
        
        # Home positions in servo space (degrees)
        self.home_positions = [
            SERVO_HOME['joint_1'],
            SERVO_HOME['joint_2'],
            SERVO_HOME['joint_3'],
            SERVO_HOME['joint_4'],
            SERVO_HOME['gripper_1'],
            SERVO_HOME['gripper_2'],
        ]
        
        # State - ALL positions stored in SERVO SPACE (degrees)
        self.current_positions = list(self.home_positions)  # Actual servo positions
        self.target_positions = list(self.home_positions)   # Target servo positions
        
        self.enabled = False
        self.speed_mult = 1.0
        self.last_buttons = []
        self.connected = False
        
        # Filters for arm joints (not grippers)
        self.filters = [SmoothFilter(5.0, publish_rate) for _ in range(4)]
        
        # Serial connection
        self.serial_port = None
        self.serial_lock = threading.Lock()
        self.serial_thread = None
        self.running = True
        
        # Find and connect to ESP32
        port_name = self.get_parameter('serial_port').value
        self.connect_serial(port_name)
        
        # ROS2 subscribers
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_cb, 10)
        
        # Publishers for RViz/MoveIt visualization
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.fake_controller_pub = self.create_publisher(
            JointState, '/move_group/fake_controller_joint_states', 10)
        
        # Control loop timer
        self.dt = 1.0 / publish_rate
        self.timer = self.create_timer(self.dt, self.control_loop)
        
        self.print_banner()
    
    def find_esp32_port(self):
        """Auto-detect ESP32 serial port"""
        ports = list(serial.tools.list_ports.comports())
        for port in ports:
            if 'CP210' in port.description or 'CH340' in port.description or \
               'USB' in port.description or 'ttyUSB' in port.device or \
               'ttyACM' in port.device:
                return port.device
        
        import os
        for p in ['/dev/ttyUSB0', '/dev/ttyUSB1', '/dev/ttyACM0', '/dev/ttyACM1']:
            if os.path.exists(p):
                return p
        return None
    
    def connect_serial(self, port_name=''):
        """Connect to ESP32 via serial"""
        if not port_name:
            port_name = self.find_esp32_port()
            
        if not port_name:
            self.get_logger().error('No ESP32 found! Connect it via USB.')
            return False
        
        try:
            self.serial_port = serial.Serial(port_name, self.baud_rate, timeout=0.1)
            time.sleep(2)
            self.serial_port.reset_input_buffer()
            
            self.serial_port.write(b'PING\n')
            time.sleep(0.3)
            
            response = self.serial_port.readline().decode().strip()
            # Accept PONG, ESP32, READY, or a valid status message starting with "S "
            if 'PONG' in response or 'ESP32' in response or 'READY' in response or response.startswith('S '):
                self.get_logger().info(f'✓ Connected to ESP32 on {port_name}')
                self.connected = True
                
                # If we got a status message, parse it to get initial positions
                if response.startswith('S '):
                    self.parse_status(response)
                    self.target_positions = list(self.current_positions)
                
                self.serial_thread = threading.Thread(target=self.serial_reader, daemon=True)
                self.serial_thread.start()
                return True
            else:
                self.get_logger().warn(f'No valid response from {port_name}: "{response}"')
                self.serial_port.close()
                self.serial_port = None
                return False
                
        except Exception as e:
            self.get_logger().error(f'Serial error: {e}')
            return False
    
    def serial_reader(self):
        """Background thread to read serial data"""
        while self.running and self.serial_port:
            try:
                if self.serial_port.in_waiting:
                    line = self.serial_port.readline().decode().strip()
                    self.parse_status(line)
            except Exception as e:
                if self.running:
                    self.get_logger().warn(f'Serial read error: {e}')
                break
    
    def parse_status(self, line):
        """Parse status message from ESP32"""
        if line.startswith('S '):
            parts = line.split()
            if len(parts) >= 7:  # S + 6 values
                try:
                    with self.serial_lock:
                        for i in range(6):
                            self.current_positions[i] = float(parts[i + 1])
                except ValueError:
                    pass
    
    def send_command(self, positions):
        """Send joint positions to ESP32 (in degrees)"""
        if not self.serial_port or not self.connected:
            return
        
        try:
            cmd = f"J {positions[0]:.2f} {positions[1]:.2f} {positions[2]:.2f} {positions[3]:.2f} {positions[4]:.2f} {positions[5]:.2f}\n"
            with self.serial_lock:
                self.serial_port.write(cmd.encode())
        except Exception as e:
            self.get_logger().warn(f'Serial write error: {e}')
            self.connected = False
    
    def print_banner(self):
        self.get_logger().info('='*60)
        self.get_logger().info('   SERIAL TELEOP - 4-DOF Arm + 2 Grippers')
        self.get_logger().info('='*60)
        self.get_logger().info('ARCHITECTURE:')
        self.get_logger().info('  URDF/MoveIt uses its own joint limits (unchanged)')
        self.get_logger().info('  Real robot uses servo limits (below)')
        self.get_logger().info('-'*60)
        self.get_logger().info('SERVO LIMITS (Real Robot):')
        self.get_logger().info(f'  Base: {SERVO_LIMITS["joint_1"][0]}-{SERVO_LIMITS["joint_1"][1]}° | J1: {SERVO_LIMITS["joint_2"][0]}-{SERVO_LIMITS["joint_2"][1]}°')
        self.get_logger().info(f'  J2: {SERVO_LIMITS["joint_3"][0]}-{SERVO_LIMITS["joint_3"][1]}° | J3: {SERVO_LIMITS["joint_4"][0]}-{SERVO_LIMITS["joint_4"][1]}°')
        self.get_logger().info(f'  Grip1: {SERVO_LIMITS["gripper_1"][0]}-{SERVO_LIMITS["gripper_1"][1]}° | Grip2: {SERVO_LIMITS["gripper_2"][0]}-{SERVO_LIMITS["gripper_2"][1]}°')
        self.get_logger().info('-'*60)
        self.get_logger().info('CONTROLS:')
        self.get_logger().info('  Left Stick X/Y  : Base / Joint 1')
        self.get_logger().info('  Right Stick X/Y : Joint 3 / Joint 2')
        self.get_logger().info('  LB / RB         : Gripper 1 Close/Open')
        self.get_logger().info('  LT / RT         : Gripper 2 Close/Open')
        self.get_logger().info('  A=Enable  B=E-Stop  Start=Home')
        self.get_logger().info('='*60)
        self.get_logger().info(f'Home: Base={SERVO_HOME["joint_1"]}° J1={SERVO_HOME["joint_2"]}° J2={SERVO_HOME["joint_3"]}° J3={SERVO_HOME["joint_4"]}°')
        if self.connected:
            self.get_logger().info('>>> Press A to ENABLE <<<')
        else:
            self.get_logger().warn('>>> Connect ESP32 and restart <<<')
    
    def joy_cb(self, msg):
        """Process gamepad input"""
        # Button edge detection
        if len(self.last_buttons) == len(msg.buttons):
            # A (0) - Enable
            if msg.buttons[0] == 1 and self.last_buttons[0] == 0:
                if self.connected:
                    self.enabled = True
                    self.target_positions = list(self.current_positions)
                    for f in self.filters:
                        f.reset()
                    self.get_logger().info('✓ ENABLED - Move carefully!')
                else:
                    self.get_logger().warn('Cannot enable - ESP32 not connected')
            
            # B (1) - Emergency stop
            if msg.buttons[1] == 1 and self.last_buttons[1] == 0:
                self.emergency_stop()
            
            # Start (7) - Home
            if msg.buttons[7] == 1 and self.last_buttons[7] == 0:
                self.go_home()
        
        self.last_buttons = list(msg.buttons)
        
        if not self.enabled:
            return
        
        # ======= ARM CONTROL (Sticks) =======
        raw_inputs = [
            msg.axes[0],   # Left X -> Base
            msg.axes[1],   # Left Y -> Joint 1
            msg.axes[4],   # Right Y -> Joint 2
            msg.axes[3],   # Right X -> Joint 3
        ]
        
        # Apply filters
        filtered = [self.filters[i].update(raw_inputs[i]) for i in range(4)]
        
        # Update arm targets with speed multiplier (in servo space - degrees)
        vel = self.max_vel * self.speed_mult * self.dt
        for i in range(4):
            delta = filtered[i] * vel
            self.target_positions[i] += delta
            
            # Clamp to SERVO limits (not URDF limits)
            joint_name = self.joint_names[i]
            limits = SERVO_LIMITS[joint_name]
            self.target_positions[i] = np.clip(self.target_positions[i], limits[0], limits[1])
        
        # ======= GRIPPER 1 CONTROL (LB/RB - buttons 4/5) =======
        # LB (button 4) = Close gripper 1
        # RB (button 5) = Open gripper 1
        if len(msg.buttons) > 5:
            grip1_delta = 0
            if msg.buttons[4] == 1:  # LB pressed - Close gripper 1
                grip1_delta = -self.gripper_speed * self.dt
            elif msg.buttons[5] == 1:  # RB pressed - Open gripper 1
                grip1_delta = self.gripper_speed * self.dt
            
            self.target_positions[4] += grip1_delta
            limits = SERVO_LIMITS['gripper_1']
            self.target_positions[4] = np.clip(self.target_positions[4], limits[0], limits[1])
        
        # ======= GRIPPER 2 CONTROL (LT/RT - axes 2/5) =======
        # LT (axis 2): 1.0 (not pressed) to -1.0 (fully pressed) = Close gripper 2
        # RT (axis 5): 1.0 (not pressed) to -1.0 (fully pressed) = Open gripper 2
        if len(msg.axes) > 5:
            lt = msg.axes[2]  # LT trigger
            rt = msg.axes[5]  # RT trigger
            
            grip2_delta = 0
            if lt < 0.5:  # LT pressed (value goes from 1.0 to -1.0) - Close gripper 2
                grip2_delta = -self.gripper_speed * self.dt * (1.0 - lt) / 2.0
            if rt < 0.5:  # RT pressed (value goes from 1.0 to -1.0) - Open gripper 2
                grip2_delta = self.gripper_speed * self.dt * (1.0 - rt) / 2.0
            
            self.target_positions[5] += grip2_delta
            limits = SERVO_LIMITS['gripper_2']
            self.target_positions[5] = np.clip(self.target_positions[5], limits[0], limits[1])
    
    def emergency_stop(self):
        """Emergency stop - hold current position"""
        self.enabled = False
        self.target_positions = list(self.current_positions)
        for f in self.filters:
            f.reset()
        self.get_logger().warn('!!! EMERGENCY STOP !!!')
    
    def go_home(self):
        """Move to home position"""
        self.target_positions = list(self.home_positions)
        self.enabled = True
        self.send_command(self.target_positions)
        self.get_logger().info(f'Going HOME: {self.home_positions}')
    
    def control_loop(self):
        """Main control loop - bridges servo space and URDF space"""
        if not self.connected:
            return
        
        # =====================================================
        # SERVO SPACE: Send commands to real robot (degrees)
        # =====================================================
        if self.enabled:
            self.send_command(self.target_positions)
        
        # =====================================================
        # URDF SPACE: Publish joint states for RViz (radians)
        # =====================================================
        # Convert servo positions (degrees) → URDF positions (radians)
        # This maps the physical servo range to the URDF joint range
        positions_rad = [
            servo_to_urdf(self.current_positions[0], 'joint_1'),  # Base
            servo_to_urdf(self.current_positions[1], 'joint_2'),  # J1
            servo_to_urdf(self.current_positions[2], 'joint_3'),  # J2
            servo_to_urdf(self.current_positions[3], 'joint_4'),  # J3
        ]
        
        # Publish to RViz/MoveIt (only arm joints, 4 joints)
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names[:4]  # Only arm joints for URDF
        msg.position = positions_rad
        
        self.joint_state_pub.publish(msg)
        self.fake_controller_pub.publish(msg)
    
    def destroy_node(self):
        """Cleanup"""
        self.running = False
        if self.serial_port:
            self.serial_port.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SerialTeleop()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
