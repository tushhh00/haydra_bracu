#!/usr/bin/env python3
"""
Pick script using custom IK solver for 4-DOF robot
"""

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from rclpy.duration import Duration
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from rclpy.action import ActionClient
import numpy as np
import math


def rotation_z(theta):
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])

def rotation_x(theta):
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[1, 0, 0, 0], [0, c, -s, 0], [0, s, c, 0], [0, 0, 0, 1]])

def translation(x, y, z):
    return np.array([[1, 0, 0, x], [0, 1, 0, y], [0, 0, 1, z], [0, 0, 0, 1]])


class Robot4DOF_IK:
    """IK solver for 4-DOF robot"""
    
    def __init__(self):
        self.d0 = 0.3498
        self.d1 = 0.0962
        self.a1 = 0.0065
        self.d2 = 0.187
        self.a2 = -0.0123
        self.d3 = 0.1205
        self.a3 = 0.006
        self.d_ee = 0.225
        
        self.limits = np.array([
            [-1.745329, 1.745329],
            [-0.872665, 1.745329],
            [-0.872665, 1.745329],
            [-1.570796, 1.570796],
        ])
    
    def fk_position(self, joints):
        j1, j2, j3, j4 = joints
        T = np.eye(4)
        T = T @ translation(0, 0, self.d0)
        T = T @ rotation_z(j1)
        T = T @ translation(self.a1, 0, self.d1)
        T = T @ rotation_x(-j2)
        T = T @ translation(self.a2, 0, self.d2)
        T = T @ rotation_x(-j3)
        T = T @ translation(self.a3, 0, self.d3)
        T = T @ rotation_z(j4)
        T = T @ translation(0, 0, self.d_ee)
        return T[0, 3], T[1, 3], T[2, 3]
    
    def solve_ik(self, x, y, z, tolerance=0.02):
        """Solve IK using scipy with multiple initial guesses"""
        try:
            from scipy.optimize import minimize
        except ImportError:
            raise ImportError("scipy is required. Install: pip3 install scipy")
        
        target = np.array([x, y, z])
        j1_init = math.atan2(-x, y) if abs(x) > 0.001 or abs(y) > 0.001 else 0
        
        def cost(joints):
            try:
                pos = self.fk_position(joints)
                return (pos[0]-target[0])**2 + (pos[1]-target[1])**2 + (pos[2]-target[2])**2
            except:
                return float('inf')
        
        # Multiple initial guesses for different arm configurations
        guesses = [
            [j1_init, 0.5, 0.5, 0],
            [j1_init, 0.8, 0.5, 0],
            [j1_init, 1.0, 0.8, 0],
            [j1_init, 1.2, 1.0, 0],
            [j1_init, 0.3, 0.3, 0],
            [j1_init + 0.1, 0.5, 0.5, 0],
            [j1_init - 0.1, 0.5, 0.5, 0],
        ]
        
        best = None
        best_err = float('inf')
        
        for guess in guesses:
            try:
                result = minimize(cost, guess, method='SLSQP', bounds=self.limits,
                                options={'ftol': 1e-12, 'maxiter': 1000})
                err = math.sqrt(result.fun)
                if err < best_err:
                    best_err = err
                    best = result.x.tolist()
            except Exception:
                continue
        
        return best if best and best_err < tolerance else None


class PickNode(Node):
    def __init__(self):
        super().__init__('pick_with_ik')
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory'
        )
        
        self.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4']
        self.ik = Robot4DOF_IK()
        
        self.get_logger().info('Pick node with IK ready')
        
    def get_detection_pose(self, max_retries=5):
        """Get object pose from TF with retry logic"""
        for attempt in range(max_retries):
            try:
                trans = self.tf_buffer.lookup_transform(
                    'base_link', 'detected_object',
                    rclpy.time.Time(),
                    timeout=Duration(seconds=2.0)
                )
                t = trans.transform.translation
                
                # Validate position is reasonable
                if abs(t.x) > 1.0 or abs(t.y) > 1.0 or t.z < 0.0 or t.z > 1.5:
                    self.get_logger().warn(f'Unreasonable position: ({t.x:.3f}, {t.y:.3f}, {t.z:.3f})')
                    if attempt < max_retries - 1:
                        rclpy.spin_once(self, timeout_sec=0.5)
                        continue
                
                return t.x, t.y, t.z
            except Exception as e:
                if attempt < max_retries - 1:
                    self.get_logger().warn(f'TF lookup attempt {attempt+1}/{max_retries} failed: {e}')
                    rclpy.spin_once(self, timeout_sec=0.5)
                else:
                    self.get_logger().error(f'TF lookup failed after {max_retries} attempts: {e}')
        return None
    
    def move_to_joints(self, joints, duration=3.0, timeout_margin=5.0):
        """Move to joint positions with error handling"""
        if not self.action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Action server not available')
            return False
        
        # Validate joint values
        for i, (j, (j_min, j_max)) in enumerate(zip(joints, self.ik.limits)):
            if not (j_min <= j <= j_max):
                self.get_logger().error(f'Joint {i} value {j:.3f} exceeds limits [{j_min:.3f}, {j_max:.3f}]')
                return False
        
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.joint_names
        
        point = JointTrajectoryPoint()
        point.positions = joints
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration % 1) * 1e9)
        goal.trajectory.points.append(point)
        
        try:
            future = self.action_client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
            
            if not future.done():
                self.get_logger().error('Goal send timeout')
                return False
            
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().error('Goal rejected by action server')
                return False
            
            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future, timeout_sec=duration + timeout_margin)
            
            if not result_future.done():
                self.get_logger().error('Motion execution timeout')
                return False
            
            result = result_future.result().result
            if result.error_code != 0:
                self.get_logger().error(f'Motion failed with error code: {result.error_code}')
                return False
            
            return True
        except Exception as e:
            self.get_logger().error(f'Motion execution error: {e}')
            return False
    
    def move_to_position(self, x, y, z, approach_offset=0.15):
        """Move to XYZ position using IK"""
        self.get_logger().info(f'Target: ({x:.3f}, {y:.3f}, {z:.3f})')
        
        # First move to approach position (higher)
        approach_z = z + approach_offset
        joints_approach = self.ik.solve_ik(x, y, approach_z)
        
        if joints_approach is None:
            self.get_logger().warn(f'Approach position unreachable, trying target directly')
            joints_approach = None
        else:
            self.get_logger().info(f'Approach joints: {[f"{j:.3f}" for j in joints_approach]}')
        
        # Target position
        joints_target = self.ik.solve_ik(x, y, z)
        
        if joints_target is None:
            self.get_logger().error('Target position UNREACHABLE')
            return False
        
        self.get_logger().info(f'Target joints: {[f"{j:.3f}" for j in joints_target]}')
        
        # Verify with FK
        fk = self.ik.fk_position(joints_target)
        err = math.sqrt((fk[0]-x)**2 + (fk[1]-y)**2 + (fk[2]-z)**2)
        self.get_logger().info(f'FK verify: ({fk[0]:.3f}, {fk[1]:.3f}, {fk[2]:.3f}), err={err*1000:.1f}mm')
        
        # Execute motion
        if joints_approach:
            self.get_logger().info('Moving to approach...')
            if not self.move_to_joints(joints_approach, 3.0):
                return False
        
        self.get_logger().info('Moving to target...')
        return self.move_to_joints(joints_target, 2.0)
    
    def run(self):
        """Main execution with comprehensive error handling"""
        # Wait for TF system to stabilize
        self.get_logger().info('Waiting for detection and TF system...')
        for _ in range(3):
            rclpy.spin_once(self, timeout_sec=1.0)
        
        # Get detection with retries
        pose = self.get_detection_pose()
        if pose is None:
            self.get_logger().error('No detection found! Ensure:')
            self.get_logger().error('  1. Camera is running: ros2 topic echo /camera/camera/color/image_raw')
            self.get_logger().error('  2. YOLO is detecting: ros2 topic echo /detected_object/pose')
            self.get_logger().error('  3. TF is publishing: ros2 run tf2_ros tf2_echo base_link detected_object')
            return False
        
        x, y, z = pose
        self.get_logger().info(f'Detected object at: ({x:.3f}, {y:.3f}, {z:.3f})')
        
        # Workspace bounds check
        ik_workspace = {
            'x': (-0.3, 0.3),
            'y': (0.2, 0.5),
            'z': (0.3, 0.9)
        }
        
        if not (ik_workspace['x'][0] <= x <= ik_workspace['x'][1] and
                ik_workspace['y'][0] <= y <= ik_workspace['y'][1] and
                ik_workspace['z'][0] <= z <= ik_workspace['z'][1]):
            self.get_logger().error(f'Object outside reachable workspace!')
            self.get_logger().error(f'  X: {ik_workspace["x"]}, Y: {ik_workspace["y"]}, Z: {ik_workspace["z"]}')
            return False
        
        # Add offset for gripper (pick above object)
        pick_z = z + 0.05  # 5cm above object
        
        # Try with relaxed tolerance if first attempt fails
        for tolerance in [0.02, 0.03, 0.05]:
            self.get_logger().info(f'Attempting IK with tolerance={tolerance*1000:.0f}mm')
            
            # Solve IK with current tolerance
            joints_target = self.ik.solve_ik(x, y, pick_z, tolerance=tolerance)
            
            if joints_target is not None:
                break
        else:
            self.get_logger().error('Target position UNREACHABLE even with relaxed tolerance')
            self.get_logger().error('Consider using joint-space control: pick_detected_object.py')
            return False
        
        self.get_logger().info(f'IK solution: [{joints_target[0]:.3f}, {joints_target[1]:.3f}, {joints_target[2]:.3f}, {joints_target[3]:.3f}]')
        
        # Verify with FK
        fk = self.ik.fk_position(joints_target)
        err = math.sqrt((fk[0]-x)**2 + (fk[1]-y)**2 + (fk[2]-pick_z)**2)
        self.get_logger().info(f'FK verify: ({fk[0]:.3f}, {fk[1]:.3f}, {fk[2]:.3f}), error={err*1000:.1f}mm')
        
        if err > 0.05:  # 5cm error threshold
            self.get_logger().warn(f'Large FK error: {err*1000:.0f}mm - motion may be inaccurate')
        
        # Execute motion with approach (if possible)
        approach_z = pick_z + 0.15
        joints_approach = self.ik.solve_ik(x, y, approach_z, tolerance=0.03)
        
        if joints_approach:
            self.get_logger().info('Moving to approach position...')
            if not self.move_to_joints(joints_approach, 3.0):
                self.get_logger().warn('Approach motion failed, trying direct motion')
        
        self.get_logger().info('Moving to pick position...')
        success = self.move_to_joints(joints_target, 2.0)
        
        if success:
            self.get_logger().info('✓ Pick motion complete!')
            return True
        else:
            self.get_logger().error('✗ Pick motion failed!')
            return False


def main():
    """Main entry point with argument handling"""
    import sys
    import argparse
    
    parser = argparse.ArgumentParser(description='Pick with IK - Move robot to position')
    parser.add_argument('-x', type=float, help='X coordinate (meters)')
    parser.add_argument('-y', type=float, help='Y coordinate (meters)')
    parser.add_argument('-z', type=float, help='Z coordinate (meters)')
    parser.add_argument('--home', action='store_true', help='Go to home position')
    parser.add_argument('--ready', action='store_true', help='Go to ready position')
    parser.add_argument('--detect', action='store_true', help='Use detected object (default if no coords)')
    
    # Parse known args to allow ROS args
    args, _ = parser.parse_known_args()
    
    rclpy.init(args=sys.argv)
    node = PickNode()
    
    try:
        # Predefined positions
        home_joints = [0.0, 0.0, 0.0, 0.0]
        ready_joints = [0.0, 0.5, -0.5, 0.0]
        
        if args.home:
            node.get_logger().info('Moving to HOME position...')
            success = node.move_to_joints(home_joints, 3.0)
        elif args.ready:
            node.get_logger().info('Moving to READY position...')
            success = node.move_to_joints(ready_joints, 3.0)
        elif args.x is not None and args.y is not None and args.z is not None:
            # Manual coordinates
            x, y, z = args.x, args.y, args.z
            node.get_logger().info(f'Moving to manual position: ({x:.3f}, {y:.3f}, {z:.3f})')
            success = node.move_to_position(x, y, z)
        else:
            # Default: use detection
            success = node.run()
        
        if success:
            node.get_logger().info('✓ Motion complete!')
        else:
            node.get_logger().error('✗ Motion failed!')
        
        sys.exit(0 if success else 1)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user')
        sys.exit(130)
    except Exception as e:
        node.get_logger().error(f'Unexpected error: {e}')
        import traceback
        traceback.print_exc()
        sys.exit(1)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
