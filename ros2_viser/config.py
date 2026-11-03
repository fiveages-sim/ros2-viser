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

    prefer_namespaced_robots: bool = True
    """Ignore a root-namespace ``/robot_description`` when namespaced robots exist.

    Simulations often keep a global ``robot_state_publisher`` publishing
    ``/robot_description`` while the robot actually being driven lives under a
    namespace (``/g1d/robot_description``). Loading both puts an extra, unrelated
    robot in the shared scene, so the root entry is dropped from the picker (and
    any cached root session is removed) as soon as a namespaced robot appears.
    Disable this when you really do want the root robot alongside namespaced ones.
    """

    prune_stale_robots_after: float = 5.0
    """Seconds without any data before a robot is checked for a live publisher.

    When a simulation stops, its ``robot_description`` publisher disappears but
    the visualizer keeps the cached session (so the robot stays in the scene and
    its interface keeps spinning). After this grace period the interface is asked
    whether the topic still has a publisher; if not the session is destroyed and
    the picker options are refreshed, so the next simulation run starts from a
    clean scene. Set ``0`` to disable.
    """

    visualization_frame: str = ""
    """TF frame used as the Viser scene / world frame for placing the robot root.

    Empty string keeps the URDF root fixed at the scene origin. For mobile bases
    (e.g. G1D), prefer ``world`` / ``map`` / ``odom`` so the model and OCS2
    markers share the same ground plane via
    ``lookup_transform(visualization_frame, urdf_root_link)``. Leaving this empty
    while multiple namespaced robots are loaded stacks them at the origin and
    makes world-frame markers look height-shifted by the chassis/lift offset.
    """
    
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
    
    marker_continuous_publish: bool = False
    """Whether to use continuous publish mode for end-effector markers.
    If True, marker position is published continuously while dragging.
    If False, marker position is only published when clicking the send button.
    Default: False"""

    show_current_target_axes: bool = True
    """Show axes for the controller's last dual-arm ``*_current_target`` poses
    (RViz Pose display equivalent). Useful for debugging command vs feedback.
    Default: True"""
    
    language: str = "zh"
    """Language for GUI labels. Options: "zh" (Chinese) or "en" (English). Default: "zh" """
    
    robot_description_timeout: float = 30.0
    """Timeout in seconds for waiting for robot description from ROS2 topic.
    Set to 0.0 to wait indefinitely (no timeout).
    Default: 30.0 seconds"""