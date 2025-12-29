#!/usr/bin/env python3
"""
Test IK reachability for a position
"""
import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import PositionIKRequest, RobotState
from geometry_msgs.msg import PoseStamped
import sys

class IKTester(Node):
    def __init__(self):
        super().__init__('ik_tester')
        
        # Create IK service client
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        
        self.get_logger().info('Waiting for /compute_ik service...')
        if not self.ik_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('IK service not available!')
            return
        self.get_logger().info('IK service ready!')
        
    def test_ik(self, x, y, z):
        """Test if a position is reachable"""
        request = GetPositionIK.Request()
        
        # Set up IK request
        request.ik_request.group_name = 'manipulator'
        request.ik_request.pose_stamped.header.frame_id = 'base_link'
        request.ik_request.pose_stamped.header.stamp = self.get_clock().now().to_msg()
        request.ik_request.pose_stamped.pose.position.x = x
        request.ik_request.pose_stamped.pose.position.y = y
        request.ik_request.pose_stamped.pose.position.z = z
        request.ik_request.pose_stamped.pose.orientation.w = 1.0  # Identity
        
        request.ik_request.timeout.sec = 5
        
        self.get_logger().info(f'Testing IK: x={x:.3f}, y={y:.3f}, z={z:.3f}')
        
        future = self.ik_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        
        if future.result() is not None:
            result = future.result()
            if result.error_code.val == 1:  # SUCCESS
                self.get_logger().info('  -> IK FOUND!')
                return True
            else:
                self.get_logger().warn(f'  -> NO IK (code: {result.error_code.val})')
                return False
        else:
            self.get_logger().error('IK call failed')
            return False


def main():
    rclpy.init()
    node = IKTester()
    
    # Test positions
    test_positions = [
        (0.2, 0.0, 0.5, "Close front low"),
        (0.2, 0.0, 0.6, "Close front mid"),
        (0.2, 0.0, 0.7, "Close front high"),
        (0.3, 0.0, 0.5, "Medium front low"),
        (0.3, 0.0, 0.6, "Medium front mid"),
        (0.3, 0.0, 0.7, "Medium front high"),
        (0.35, 0.0, 0.6, "Far front"),
        (0.4, 0.0, 0.6, "Very far front"),
        (0.25, -0.15, 0.6, "Right side"),
        (0.288, -0.143, 0.64, "Your target"),
    ]
    
    print("\n" + "="*50)
    print("IK REACHABILITY TEST")
    print("="*50)
    
    results = []
    for x, y, z, desc in test_positions:
        success = node.test_ik(x, y, z)
        results.append((desc, x, y, z, success))
    
    print("\n" + "="*50)
    print("SUMMARY")
    print("="*50)
    reachable = 0
    for desc, x, y, z, success in results:
        status = "OK" if success else "NO"
        print(f"[{status}] ({x:.2f}, {y:.2f}, {z:.2f}) - {desc}")
        if success:
            reachable += 1
    print(f"\n{reachable}/{len(results)} positions reachable")
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
