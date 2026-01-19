"""Configuration for ROS2 Viser visualizer."""

from dataclasses import dataclass
from typing import Optional


@dataclass
class ROS2ViserConfig:
    """Configuration for ROS2 Viser visualizer.
    
    Args:
        robot_description_topic: ROS2 topic for robot description (URDF).
                                  Default: "/robot_description"
        joint_states_topic: ROS2 topic for joint states. 
                           Default: "/joint_states"
        root_node_name: Root node name in Viser scene. Default: "/robot"
        update_rate: Update rate in Hz. Default: 30.0
        auto_connect: Whether to automatically connect to ROS2 interface.
                     Default: True
    """
    
    robot_description_topic: str = "/robot_description"
    """ROS2 topic for robot description (URDF)."""
    
    joint_states_topic: str = "/joint_states"
    """ROS2 topic for joint states."""
    
    root_node_name: str = "/robot"
    """Root node name in Viser scene."""
    
    update_rate: float = 30.0
    """Update rate in Hz."""
    
    auto_connect: bool = True
    """Whether to automatically connect to ROS2 interface."""
    
    # Optional: Use existing ROS2RobotInterface instance
    ros2_interface: Optional[object] = None
    """Optional: Use existing ROS2RobotInterface instance instead of creating new one."""
    
    enable_fsm_panel: bool = True
    """Whether to enable FSM (Finite State Machine) control panel. Default: True"""
    
    fsm_command_topic: str = "/fsm_command"
    """ROS2 topic for FSM commands. Default: "/fsm_command" """
    
    enable_gripper_panel: bool = True
    """Whether to enable Gripper control panel. Default: True"""
