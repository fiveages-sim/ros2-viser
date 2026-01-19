#!/usr/bin/env python3
"""Test script to check robot_description topic subscription."""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time


class RobotDescriptionSubscriber(Node):
    """Simple subscriber to test robot_description topic."""
    
    def __init__(self):
        super().__init__('test_robot_description_subscriber')
        
        # Use TRANSIENT_LOCAL durability to receive latched messages
        from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy, ReliabilityPolicy
        
        qos_profile = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,  # Receive latched messages
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST
        )
        
        self.subscription = self.create_subscription(
            String,
            '/robot_description',
            self.callback,
            qos_profile
        )
        self.received = False
        self.get_logger().info('Waiting for robot_description message...')
    
    def callback(self, msg):
        """Callback for robot description messages."""
        if self.received:
            return
        
        self.received = True
        data = msg.data
        
        self.get_logger().info(f'Received message!')
        self.get_logger().info(f'Message length: {len(data)} characters')
        self.get_logger().info(f'Message is empty: {len(data.strip()) == 0}')
        self.get_logger().info(f'First 200 characters: {data[:200]}')
        self.get_logger().info(f'Last 200 characters: {data[-200:]}')
        
        # Check if it looks like XML
        if data.strip().startswith('<?xml'):
            self.get_logger().info('✓ Message appears to be valid XML')
        else:
            self.get_logger().warn('✗ Message does not appear to be valid XML')
        
        # Check for URDF keywords
        urdf_keywords = ['robot', 'link', 'joint', 'urdf']
        found_keywords = [kw for kw in urdf_keywords if kw in data.lower()]
        self.get_logger().info(f'Found URDF keywords: {found_keywords}')


def main():
    """Main function."""
    rclpy.init()
    
    node = RobotDescriptionSubscriber()
    
    # Spin for up to 30 seconds
    start_time = time.time()
    timeout = 30.0
    
    while not node.received and (time.time() - start_time) < timeout:
        rclpy.spin_once(node, timeout_sec=0.1)
    
    if not node.received:
        node.get_logger().error('Timeout: Did not receive robot_description message')
        node.get_logger().error('Make sure robot_state_publisher or similar node is running')
    else:
        node.get_logger().info('Test completed successfully!')
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
