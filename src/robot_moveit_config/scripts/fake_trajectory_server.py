#!/usr/bin/env python3
"""
Fake trajectory action server for MoveIt demo.
This simulates a joint trajectory controller by receiving trajectory commands
and publishing joint states to move the robot visualization.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration
import time


class FakeTrajectoryServer(Node):
    def __init__(self):
        super().__init__('fake_trajectory_server')
        
        self.callback_group = ReentrantCallbackGroup()
        
        # Create the action server
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            'arm_controller/follow_joint_trajectory',
            self.execute_callback,
            callback_group=self.callback_group
        )
        
        # Publisher for joint states (MoveIt's fake controller joint states topic)
        self.joint_state_pub = self.create_publisher(
            JointState,
            '/move_group/fake_controller_joint_states',
            10
        )
        
        # Also publish to regular joint_states for direct feedback
        self.joint_state_direct_pub = self.create_publisher(
            JointState,
            '/joint_states',
            10
        )
        
        # Current joint positions
        self.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4']
        self.current_positions = [0.0, 0.0, 0.0, 0.0]
        
        self.get_logger().info('Fake trajectory server started on arm_controller/follow_joint_trajectory')
    
    def execute_callback(self, goal_handle):
        self.get_logger().info('Received trajectory execution request')
        
        trajectory = goal_handle.request.trajectory
        
        # Execute each point in the trajectory
        for i, point in enumerate(trajectory.points):
            # Check if goal was canceled
            if not goal_handle.is_active:
                self.get_logger().info('Goal canceled')
                goal_handle.canceled()
                result = FollowJointTrajectory.Result()
                result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
                return result
            
            # Update positions
            self.current_positions = list(point.positions)
            
            # Publish joint states
            js = JointState()
            js.header.stamp = self.get_clock().now().to_msg()
            js.name = trajectory.joint_names
            js.position = list(point.positions)
            js.velocity = list(point.velocities) if point.velocities else [0.0] * len(point.positions)
            js.effort = [0.0] * len(point.positions)
            
            self.joint_state_pub.publish(js)
            self.joint_state_direct_pub.publish(js)
            
            # Calculate time to sleep until next point
            if i < len(trajectory.points) - 1:
                current_time = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
                next_time = trajectory.points[i+1].time_from_start.sec + trajectory.points[i+1].time_from_start.nanosec * 1e-9
                sleep_time = max(0.01, next_time - current_time)
            else:
                sleep_time = 0.05
            
            time.sleep(sleep_time)
            
            # Send feedback
            feedback = FollowJointTrajectory.Feedback()
            feedback.header.stamp = self.get_clock().now().to_msg()
            feedback.joint_names = trajectory.joint_names
            feedback.actual.positions = list(point.positions)
            feedback.desired.positions = list(point.positions)
            feedback.error.positions = [0.0] * len(point.positions)
            goal_handle.publish_feedback(feedback)
        
        # Mark as succeeded
        goal_handle.succeed()
        
        self.get_logger().info('Trajectory execution completed successfully')
        
        result = FollowJointTrajectory.Result()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        return result


def main(args=None):
    rclpy.init(args=args)
    node = FakeTrajectoryServer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
