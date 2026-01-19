#!/usr/bin/env python3
"""Example using ros2_viser with existing ROS2RobotInterface."""

import time
from ros2_robot_interface import ROS2RobotInterface, ROS2RobotInterfaceConfig
from ros2_viser import ROS2ViserVisualizer, ROS2ViserConfig


def main():
    """Main function using existing interface."""
    # Create ROS2 Robot Interface
    print("Creating ROS2 Robot Interface...")
    interface_config = ROS2RobotInterfaceConfig(
        joint_states_topic="/joint_states",
        end_effector_pose_topic="/left_current_pose",
        end_effector_target_topic="/left_target"
    )
    interface = ROS2RobotInterface(interface_config)
    interface.connect()
    print("ROS2 Robot Interface connected.")
    
    # Create visualizer with existing interface
    print("Creating ROS2 Viser visualizer...")
    viser_config = ROS2ViserConfig(
        ros2_interface=interface,  # Reuse existing interface
        robot_description_topic="/robot_description",
        root_node_name="/robot"
    )
    
    visualizer = ROS2ViserVisualizer(viser_config)
    
    try:
        visualizer.start()
        print("Visualizer started! Open the Viser web interface in your browser.")
        print("Press Ctrl+C to stop.")
        
        # Keep running
        while True:
            time.sleep(1)
            
    except KeyboardInterrupt:
        print("\nStopping visualizer...")
    finally:
        visualizer.stop()
        interface.disconnect()
        print("All resources cleaned up.")


if __name__ == "__main__":
    main()
