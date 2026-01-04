#!/usr/bin/env python3
"""
Simple Pick Script - Uses MoveIt2 services for motion planning
Synchronous motion - waits for completion before accepting new commands
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetMotionPlan
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import (
    Constraints, JointConstraint, RobotState, 
    MotionPlanRequest, WorkspaceParameters
)
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
        
        # Planning service
        self.plan_client = self.create_client(GetMotionPlan, '/plan_kinematic_path')
        
        # Execution action
        self.execute_client = ActionClient(self, ExecuteTrajectory, '/execute_trajectory')
        
        self.latest_pose = None
        self.current_joints = {}
        self.latest_joint_state = None
        self.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4']
        
        self.get_logger().info('Waiting for planning service...')
        if not self.plan_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('Planning service not available!')
        else:
            self.get_logger().info('Planning service ready!')
            
        self.get_logger().info('Waiting for execute action...')
        if not self.execute_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Execute action not available!')
        else:
            self.get_logger().info('Execute action ready!')
            
        # Wait for first joint state
        self.get_logger().info('Waiting for joint states...')
        for _ in range(50):  # 5 seconds
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.latest_joint_state is not None:
                break
        if self.latest_joint_state:
            self.get_logger().info('Got joint states!')
        else:
            self.get_logger().warn('No joint states yet')
            
    def pose_cb(self, msg):
        self.latest_pose = msg
        
    def joint_cb(self, msg):
        self.latest_joint_state = msg
        for i, name in enumerate(msg.name):
            self.current_joints[name] = msg.position[i]
    
    def move_joints(self, joint_positions, duration_sec=2.0):
        """Move to specific joint positions using MoveIt2 planning"""
        # Update joint state
        rclpy.spin_once(self, timeout_sec=0.1)
        
        if self.latest_joint_state is None:
            self.get_logger().error('No joint state received!')
            return False
        
        # Create motion plan request
        request = GetMotionPlan.Request()
        request.motion_plan_request.group_name = 'manipulator'
        request.motion_plan_request.num_planning_attempts = 10
        request.motion_plan_request.allowed_planning_time = 5.0
        request.motion_plan_request.max_velocity_scaling_factor = 0.3
        request.motion_plan_request.max_acceleration_scaling_factor = 0.3
        
        # Set start state
        request.motion_plan_request.start_state.joint_state = self.latest_joint_state
        
        # Workspace
        request.motion_plan_request.workspace_parameters.header.frame_id = 'base_link'
        request.motion_plan_request.workspace_parameters.min_corner.x = -1.0
        request.motion_plan_request.workspace_parameters.min_corner.y = -1.0
        request.motion_plan_request.workspace_parameters.min_corner.z = -0.5
        request.motion_plan_request.workspace_parameters.max_corner.x = 1.0
        request.motion_plan_request.workspace_parameters.max_corner.y = 1.0
        request.motion_plan_request.workspace_parameters.max_corner.z = 1.5
        
        # Goal constraints
        constraints = Constraints()
        for name, position in zip(self.joint_names, joint_positions):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = position
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
        request.motion_plan_request.goal_constraints.append(constraints)
        
        self.get_logger().info(f'Planning to joints: {[f"{p:.2f}" for p in joint_positions]}')
        
        # Call planning service
        future = self.plan_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        
        if future.result() is None:
            self.get_logger().error('Planning service call failed!')
            return False
            
        response = future.result()
        error_code = response.motion_plan_response.error_code.val
        
        if error_code != 1:  # Not SUCCESS
            error_names = {
                -1: 'FAILURE', -2: 'PLANNING_FAILED', -3: 'INVALID_MOTION_PLAN',
                -4: 'ENVIRONMENT_CHANGED', -5: 'CONTROL_FAILED', 
                -6: 'SENSOR_DATA', -7: 'TIMED_OUT', -10: 'PREEMPTED', 
                -12: 'START_STATE_IN_COLLISION', -13: 'GOAL_IN_COLLISION'
            }
            error_name = error_names.get(error_code, f'UNKNOWN({error_code})')
            self.get_logger().error(f'Planning failed: {error_name}')
            return False
            
        self.get_logger().info('Plan found! Executing...')
        
        # Execute the trajectory
        execute_goal = ExecuteTrajectory.Goal()
        execute_goal.trajectory = response.motion_plan_response.trajectory
        
        send_future = self.execute_client.send_goal_async(execute_goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=5.0)
        
        goal_handle = send_future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error('Execution rejected!')
            return False
        
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=60.0)
        
        if result_future.result() is None:
            self.get_logger().error('Execution timeout!')
            return False
            
        exec_result = result_future.result().result
        if exec_result.error_code.val == 1:
            self.get_logger().info('SUCCESS! Motion planned and executed.')
            return True
        else:
            self.get_logger().error(f'Execution failed: code {exec_result.error_code.val}')
            return False
        
    def go_home(self):
        """Move to home position (all joints at 0)"""
        self.move_joints([0.0, 0.0, 0.0, 0.0])
        
    def go_ready(self):
        """Move to ready position"""
        self.move_joints([0.0, 0.5, -0.5, 0.0])
        
    def go_extended(self):
        """Move to extended position"""
        self.move_joints([0.0, 1.0, 0.5, 0.0])
        
    def pick_toward_object(self, x=None, y=None, z=None):
        """Calculate joint angles to point toward detected object or given coordinates"""
        if x is None or y is None or z is None:
            if self.latest_pose is None:
                self.get_logger().warn('No detection!')
                return
            x = self.latest_pose.pose.position.x
            y = self.latest_pose.pose.position.y
            z = self.latest_pose.pose.position.z
        
        self.get_logger().info(f'Target: x={x:.3f}, y={y:.3f}, z={z:.3f}')
        
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
    print('  p     - Pick (move toward detected object)')
    print('  x=0.7 y=0.0 z=0.8 - Move to specific coords')
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
                elif cmd.startswith('x='):
                    # Parse x=0.7 y=0.0 z=0.8 format
                    try:
                        parts = cmd.split()
                        coords = {}
                        for part in parts:
                            if '=' in part:
                                key, val = part.split('=')
                                coords[key] = float(val)
                        if 'x' in coords and 'y' in coords and 'z' in coords:
                            node.pick_toward_object(coords['x'], coords['y'], coords['z'])
                        else:
                            print('Usage: x=0.7 y=0.0 z=0.8')
                    except Exception as e:
                        print(f'Parse error: {e}')
                elif cmd:
                    print(f'Unknown: {cmd}')
                    
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
