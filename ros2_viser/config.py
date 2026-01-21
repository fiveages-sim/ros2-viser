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
    
    update_rate: float = 30
    """Update rate in Hz."""
    
    auto_connect: bool = True
    """Whether to automatically connect to ROS2 interface."""
    
    enable_fsm_panel: bool = True
    """Whether to enable FSM (Finite State Machine) control panel. Default: True"""
    
    fsm_command_topic: str = "/fsm_command"
    """ROS2 topic for FSM commands. Default: "/fsm_command" """
    
    enable_gripper_panel: bool = True
    """Whether to enable Gripper control panel. Default: True"""
    
    enable_joint_panel: bool = True
    """Whether to enable Joint control panel. Default: True"""
    
    enable_end_effector_marker: bool = True
    """Whether to enable draggable end-effector marker for sending pose commands. Default: True"""
    
    end_effector_link_name: Optional[str] = None
    """Name of the end-effector link. If None, will try to auto-detect from URDF. Default: None"""
    
    marker_scale: float = 0.15
    """Scale of the draggable marker. Default: 0.15"""
    
    marker_continuous_publish: bool = True
    """Whether to use continuous publish mode for end-effector markers.
    If True, marker position is published continuously while dragging.
    If False, marker position is only published when clicking the send button.
    Default: True"""
    
    language: str = "zh"
    """Language for GUI labels. Options: "zh" (Chinese) or "en" (English). Default: "zh" """
    
    robot_description_timeout: float = 30.0
    """Timeout in seconds for waiting for robot description from ROS2 topic.
    Set to 0.0 to wait indefinitely (no timeout).
    Default: 30.0 seconds"""