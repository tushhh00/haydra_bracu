#!/usr/bin/env python3
"""
Simple MoveIt Pick Example

This is a simpler example using moveit_py to plan motion to detected objects.
The /detected_object/pose topic provides poses already transformed to base_link.

Usage:
  # First, launch the pick_place_system
  ./launch_moveit_demo.sh publish_static_camera_tf:=true camera_x:=0.5 camera_z:=0.8 camera_pitch:=0.5
  
  # Then run this script
  python3 simple_moveit_pick.py
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from moveit.planning import MoveItPy
from moveit.core.robot_state import RobotState
import time
import copy


class SimplePick(Node):
    def __init__(self):
        super().__init__('simple_pick')
        
        # Subscribe to detected poses (already in base_link frame!)
        self.latest_pose = None
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/detected_object/pose',
            self.pose_callback,
            10
        )
        
        self.get_logger().info('Waiting for MoveItPy to initialize...')
        
        # Initialize MoveItPy
        self.moveit = MoveItPy(node_name="simple_pick_moveit")
        self.planning_component = self.moveit.get_planning_component("manipulator")
        
        self.get_logger().info('Simple Pick node ready!')
        self.get_logger().info('Listening for detections on /detected_object/pose')
        
    def pose_callback(self, msg: PoseStamped):
        """Store the latest detection"""
        self.latest_pose = msg
        
    def get_current_pose(self):
        """Get current end effector pose"""
        robot_state = self.moveit.get_robot_model().get_robot_state()
        return self.planning_component.get_start_state()
        
    def plan_to_pose(self, target_pose: PoseStamped, execute=False):
        """Plan (and optionally execute) motion to target pose"""
        
        # Set the goal
        self.planning_component.set_start_state_to_current_state()
        self.planning_component.set_goal_state(pose_stamped_msg=target_pose, 
                                                pose_link="link4")  # Adjust link name!
        
        # Plan
        self.get_logger().info('Planning motion...')
        plan_result = self.planning_component.plan()
        
        if plan_result:
            self.get_logger().info('Motion plan found!')
            
            if execute:
                self.get_logger().info('Executing motion...')
                self.moveit.execute(plan_result.trajectory, 
                                   controllers=[])  # Use default controllers
                self.get_logger().info('Motion complete!')
            return True
        else:
            self.get_logger().error('Motion planning failed!')
            return False
            
    def pick_detected_object(self, approach_height=0.1, execute=False):
        """
        Plan motion to pick up the detected object.
        
        Args:
            approach_height: Height above object to approach (meters)
            execute: If True, execute the motion. If False, just plan.
        """
        if self.latest_pose is None:
            self.get_logger().warn('No object detected yet! Waiting...')
            return False
            
        # Create approach pose (above the object)
        approach_pose = copy.deepcopy(self.latest_pose)
        approach_pose.pose.position.z += approach_height
        
        # Set end effector orientation (pointing down)
        # For a gripper pointing down: rotation of 180° around X axis
        approach_pose.pose.orientation.x = 1.0
        approach_pose.pose.orientation.y = 0.0
        approach_pose.pose.orientation.z = 0.0
        approach_pose.pose.orientation.w = 0.0
        
        pos = self.latest_pose.pose.position
        self.get_logger().info(f'Object detected at: x={pos.x:.3f}, y={pos.y:.3f}, z={pos.z:.3f}')
        self.get_logger().info(f'Approach pose: z={approach_pose.pose.position.z:.3f}')
        
        return self.plan_to_pose(approach_pose, execute=execute)


def main():
    rclpy.init()
    
    node = SimplePick()
    
    # Give time for detection to arrive
    print("\n" + "="*60)
    print("SIMPLE MOVEIT PICK EXAMPLE")
    print("="*60)
    print("\nThe system is receiving detections from YOLO.")
    print("Poses are automatically transformed to base_link frame.")
    print("\nOptions:")
    print("  1. Plan to detected object (no execution)")
    print("  2. Plan and execute motion to detected object")
    print("  3. Show latest detection")
    print("  4. Quit")
    print("="*60 + "\n")
    
    try:
        while rclpy.ok():
            # Process callbacks
            rclpy.spin_once(node, timeout_sec=0.1)
            
            # Simple menu
            try:
                choice = input("\nEnter choice (1-4): ").strip()
            except EOFError:
                break
                
            if choice == '1':
                node.pick_detected_object(approach_height=0.1, execute=False)
            elif choice == '2':
                confirm = input("Execute motion? (y/n): ").strip().lower()
                if confirm == 'y':
                    node.pick_detected_object(approach_height=0.1, execute=True)
            elif choice == '3':
                if node.latest_pose:
                    p = node.latest_pose.pose.position
                    print(f"\nLatest detection (base_link frame):")
                    print(f"  Position: x={p.x:.3f}, y={p.y:.3f}, z={p.z:.3f} meters")
                else:
                    print("\nNo detection received yet. Make sure objects are visible to camera.")
            elif choice == '4':
                break
                
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
