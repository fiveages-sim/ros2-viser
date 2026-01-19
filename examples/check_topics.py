#!/usr/bin/env python3
"""Check available ROS2 topics and find robot_description."""

import rclpy
from rclpy.node import Node
import time


def main():
    """Main function to check topics."""
    rclpy.init()
    
    node = Node('topic_checker')
    
    print("Checking available topics...")
    print("=" * 60)
    
    # Get all topic names and types
    topic_names_and_types = node.get_topic_names_and_types()
    
    # Find robot_description related topics
    robot_desc_topics = []
    all_topics = []
    
    for topic_name, topic_types in topic_names_and_types:
        all_topics.append((topic_name, topic_types))
        if 'robot_description' in topic_name.lower() or 'description' in topic_name.lower():
            robot_desc_topics.append((topic_name, topic_types))
    
    print(f"\nFound {len(all_topics)} total topics")
    print(f"Found {len(robot_desc_topics)} robot_description related topics\n")
    
    if robot_desc_topics:
        print("Robot description related topics:")
        for topic_name, topic_types in robot_desc_topics:
            print(f"  - {topic_name}")
            print(f"    Types: {topic_types}")
    else:
        print("No robot_description topics found!")
        print("\nLooking for similar topics:")
        for topic_name, topic_types in all_topics:
            if 'description' in topic_name.lower() or 'urdf' in topic_name.lower():
                print(f"  - {topic_name} ({topic_types})")
    
    print("\n" + "=" * 60)
    print("\nAll available topics:")
    for topic_name, topic_types in sorted(all_topics):
        print(f"  {topic_name}")
        print(f"    Types: {topic_types}")
    
    # Check for robot_state_publisher
    print("\n" + "=" * 60)
    print("\nChecking for robot_state_publisher node...")
    node_names = node.get_node_names()
    if any('robot_state_publisher' in name for name in node_names):
        print("✓ robot_state_publisher node found!")
        for name in node_names:
            if 'robot_state_publisher' in name:
                print(f"  - {name}")
    else:
        print("✗ robot_state_publisher node not found")
        print("\nAvailable nodes:")
        for name in sorted(node_names):
            print(f"  - {name}")
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
