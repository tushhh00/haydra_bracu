#!/usr/bin/env python3
"""
Simple Pick Script - Uses MoveGroup action (no MoveItPy)
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint
from sensor_msgs.msg import JointState
import sys
import select
import math

class SimplePicker(Node):
    def __init__(self):
        super().__init__('simple_picker')
        
        # Subscribe to detection
        self.pose_sub = self.create_subscription(
            PoseStamped, '/detected_object/pose', self.pose_cb, 10)
        
        # Subscribe to joint states
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_cb, 10)
        
        # MoveGroup action client  
        self.move_client = ActionClient(self, MoveGroup, 'move_action')
        
        self.latest_pose = None
        self.current_joints = {}
        self.is_moving = False
        
        self.get_logger().info('Waiting for MoveGroup action...')
        if self.move_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().info('Connected to MoveGroup!')
        else:
            self.get_logger().error('MoveGroup not available!')
            
    def pose_cb(self, msg):
        self.latest_pose = msg
        
    def joint_cb(self, msg):
        for i, name in enumerate(msg.name):
            self.current_joints[name] = msg.position[i]
    
    def move_joints(self, joint_positions):
        """Move to specific joint positions"""
        if self.is_moving:
            self.get_logger().warn('Already moving!')
            return
            
        goal = MoveGroup.Goal()
        goal.request.group_name = 'manipulator'
        goal.request.num_planning_attempts = 10
        goal.request.allowed_planning_time = 5.0
        goal.request.max_velocity_scaling_factor = 0.3
        goal.request.max_acceleration_scaling_factor = 0.3
        
        # Create joint constraints
        constraints = Constraints()
        joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4']
        
        for name, position in zip(joint_names, joint_positions):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = position
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
        
        goal.request.goal_constraints.append(constraints)
        
        self.get_logger().info(f'Moving to joints: {[f"{p:.2f}" for p in joint_positions]}')
        self.is_moving = True
        
        future = self.move_client.send_goal_async(goal)
        future.add_done_callback(self.goal_response_cb)
        
    def goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected!')
            self.is_moving = False
            return
        self.get_logger().info('Executing...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_cb)
        
    def result_cb(self, future):
        result = future.result().result
        if result.error_code.val == 1:
            self.get_logger().info('SUCCESS!')
        else:
            self.get_logger().error(f'Failed: code {result.error_code.val}')
        self.is_moving = False
        
    def go_home(self):
        """Move to home position (all joints at 0)"""
        self.move_joints([0.0, 0.0, 0.0, 0.0])
        
    def go_ready(self):
        """Move to ready position"""
        self.move_joints([0.0, 0.5, -0.5, 0.0])
        
    def go_extended(self):
        """Move to extended position"""
        self.move_joints([0.0, 1.0, 0.5, 0.0])
        
    def pick_toward_object(self):
        """Calculate joint angles to point toward detected object"""
        if self.latest_pose is None:
            self.get_logger().warn('No detection!')
            return
            
        x = self.latest_pose.pose.position.x
        y = self.latest_pose.pose.position.y
        z = self.latest_pose.pose.position.z
        
        self.get_logger().info(f'Object at: x={x:.3f}, y={y:.3f}, z={z:.3f}')
        
        # Simple IK for 4-DOF SCARA-like arm
        # Joint 1: rotation toward object (atan2)
        j1 = math.atan2(y, x)
        
        # Clamp to joint limits
        j1 = max(-1.74, min(1.74, j1))
        
        # Joint 2 and 3: simple reach based on distance
        dist = math.sqrt(x**2 + y**2)
        
        # Heuristic for reach
        if dist < 0.15:
            j2, j3 = 0.3, 0.0  # Close position
        elif dist < 0.25:
            j2, j3 = 0.5, 0.2  # Medium position  
        elif dist < 0.35:
            j2, j3 = 0.7, 0.3  # Far position
        else:
            j2, j3 = 1.0, 0.5  # Extended position
            
        # Joint 4: keep at 0 (gripper orientation)
        j4 = 0.0
        
        self.get_logger().info(f'Calculated joints: j1={j1:.2f}, j2={j2:.2f}, j3={j3:.2f}, j4={j4:.2f}')
        self.move_joints([j1, j2, j3, j4])


def main():
    rclpy.init()
    node = SimplePicker()
    
    print('\n' + '='*50)
    print('SIMPLE PICK (Joint-based)')
    print('='*50)
    print('Commands:')
    print('  s     - Show detection')
    print('  p     - Pick (move toward object)')
    print('  home  - Go to home')
    print('  ready - Go to ready')
    print('  ext   - Go to extended')
    print('  q     - Quit')
    print('='*50 + '\n')
    
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            
            if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
                cmd = sys.stdin.readline().strip().lower()
                
                if cmd == 'q':
                    break
                elif cmd == 's':
                    if node.latest_pose:
                        p = node.latest_pose.pose.position
                        d = math.sqrt(p.x**2 + p.y**2)
                        print(f'Detection: x={p.x:.3f}, y={p.y:.3f}, z={p.z:.3f}')
                        print(f'  Distance: {d:.3f}m')
                    else:
                        print('No detection')
                elif cmd == 'p':
                    node.pick_toward_object()
                elif cmd == 'home':
                    node.go_home()
                elif cmd == 'ready':
                    node.go_ready()
                elif cmd == 'ext':
                    node.go_extended()
                elif cmd:
                    print(f'Unknown: {cmd}')
                    
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
