"""FSM Control Panel for ROS2 Viser."""

import logging
import threading
import time
from typing import Optional

import viser
from rclpy.subscription import Subscription
from std_msgs.msg import Int32

from ros2_robot_interface import ROS2RobotInterface

logger = logging.getLogger(__name__)


class FSMPanel:
    """FSM (Finite State Machine) control panel for robot state management.
    
    This panel provides a GUI interface for switching between robot operational
    states (HOME, HOLD, OCS2, MOVEJ) and performing special actions like pose switching.
    
    Example:
        ```python
        panel = FSMPanel(server, ros2_interface, "/fsm_command")
        panel.initialize()
        
        # In update loop:
        panel.update()
        ```
    """
    
    def __init__(
        self,
        server: viser.ViserServer,
        ros2_interface: ROS2RobotInterface,
        fsm_command_topic: str = "/fsm_command"
    ):
        """Initialize FSM panel.
        
        Args:
            server: Viser server instance for GUI creation.
            ros2_interface: ROS2RobotInterface for sending commands and subscribing to topics.
            fsm_command_topic: ROS2 topic name for FSM commands (default: "/fsm_command").
        """
        self.server = server
        self.ros2_interface = ros2_interface
        self.fsm_command_topic = fsm_command_topic
        
        # FSM state tracking
        self._current_fsm_state: str = "HOLD"
        self._fsm_state_lock = threading.Lock()
        
        # GUI elements
        self._fsm_state_label: Optional[viser.GuiTextHandle] = None
        self._fsm_button_handles: dict[str, viser.GuiButtonHandle] = {}
        self._fsm_switch_pose_button: Optional[viser.GuiButtonHandle] = None
        # Folder handle for cleanup
        self._folder_handle: Optional[viser.GuiFolderHandle] = None
        
        # ROS2 subscription (keep reference to prevent garbage collection)
        self._fsm_command_subscription: Optional[Subscription] = None
        
        # Initialization flag
        self._initialized = False
    
    def initialize(self):
        """Initialize the FSM panel GUI and ROS2 subscriptions."""
        # If already initialized, cleanup first to avoid duplicates
        if self._initialized:
            logger.warning("FSM panel already initialized, cleaning up first...")
            self.cleanup()
        
        try:
            # Initialize ROS2 subscription first
            self._init_fsm_subscription()
            
            # Initialize GUI
            self._init_gui()
            
            # Initial update to set button visibility
            self.update()
            
            self._initialized = True
            logger.info("✅ FSM control panel initialized")
            
        except Exception as e:
            logger.error(f"Failed to initialize FSM panel: {e}", exc_info=True)
            raise
    
    def _init_fsm_subscription(self):
        """Initialize ROS2 subscription to FSM command topic for state tracking."""
        if self.ros2_interface is None or not self.ros2_interface.is_connected:
            logger.warning("ROS2RobotInterface not connected, cannot subscribe to FSM commands")
            return
        
        ros2_node = self.ros2_interface.robot_node
        if ros2_node is None:
            logger.warning("ROS2RobotInterface node not available")
            return
        
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        
        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST
        )
        
        # Save subscription to prevent garbage collection
        self._fsm_command_subscription = ros2_node.create_subscription(
            Int32,
            self.fsm_command_topic,
            self._fsm_command_callback,
            qos_profile
        )
        
        logger.info(f"✅ Subscribed to {self.fsm_command_topic} for FSM state tracking")
    
    def _fsm_command_callback(self, msg: Int32):
        """Callback for FSM command messages to track current state.
        
        Note: We track commands to infer state, but the actual state might differ
        if commands are sent from multiple sources. This is a best-effort approach.
        
        IMPORTANT: This callback runs in the ROS2 executor thread. We should NOT
        call GUI operations directly here as they may block or cause deadlocks.
        Instead, we only update the state, and GUI updates happen in the update() method.
        """
        try:
            command = msg.data
            
            # Map command to target state name
            # Commands 0 and 100 are special (SWITCH/REST), don't change state directly
            state_map = {
                1: "HOME",
                2: "HOLD",
                3: "OCS2",
                4: "MOVEJ",
            }
            
            # Only update state for direct state commands (1-4)
            if command in state_map:
                new_state = state_map[command]
                
                with self._fsm_state_lock:
                    if new_state != self._current_fsm_state:
                        logger.debug(f"FSM state changed: {self._current_fsm_state} → {new_state}")
                        self._current_fsm_state = new_state
                        # Don't update GUI here - let update() handle it
                        # GUI operations in ROS2 callbacks can cause issues
        except Exception as e:
            # Catch all exceptions to prevent executor from stopping
            logger.error(f"Error in FSM command callback: {e}", exc_info=True)
    
    def _init_gui(self):
        """Initialize FSM control panel GUI elements."""
        if self.server is None:
            return
        
        try:
            # Create folder and save reference for cleanup
            self._folder_handle = self.server.gui.add_folder("FSM Control")
            with self._folder_handle:
                # Current state display (read-only)
                self._fsm_state_label = self.server.gui.add_text(
                    "Current State",
                    initial_value="HOLD",
                    disabled=True
                )
                
                # State transition buttons - show all possible target states as buttons
                # HOME button - Purple
                self._fsm_button_handles["to_home"] = self.server.gui.add_button("HOME", color=(128, 0, 128))
                self._fsm_button_handles["to_home"].on_click(
                    lambda _: self._send_fsm_command(1, "HOME")
                )
                
                # HOLD button - Yellow
                self._fsm_button_handles["to_hold"] = self.server.gui.add_button("HOLD", color="yellow")
                self._fsm_button_handles["to_hold"].on_click(
                    lambda _: self._send_fsm_command(2, "HOLD")
                )
                
                # OCS2 button - Blue
                self._fsm_button_handles["to_ocs2"] = self.server.gui.add_button("OCS2", color="blue")
                self._fsm_button_handles["to_ocs2"].on_click(
                    lambda _: self._send_fsm_command(3, "OCS2")
                )
                
                # MOVEJ button - Cyan
                self._fsm_button_handles["to_movej"] = self.server.gui.add_button("MOVEJ", color="cyan")
                self._fsm_button_handles["to_movej"].on_click(
                    lambda _: self._send_fsm_command(4, "MOVEJ")
                )
                
                # Special action: Switch pose (only available in HOME state)
                # This sends command 100, then automatically sends 0 after 0.1s
                # Deep orange/red-orange for special action to distinguish from yellow
                self._fsm_switch_pose_button = self.server.gui.add_button("切换姿态 (Home ↔ Rest)", color=(255, 100, 0))
                self._fsm_switch_pose_button.on_click(self._on_switch_pose_clicked)
                
        except Exception as e:
            logger.error(f"Failed to initialize FSM panel GUI: {e}", exc_info=True)
            raise
    
    def _on_switch_pose_clicked(self, _):
        """Handle switch pose button click.
        
        Sends command 100, then waits 0.1s and sends command 0.
        This is similar to the RViz panel implementation.
        """
        # Send command 100
        self._send_fsm_command(100, None)
        
        # Wait 0.1s then send command 0
        time.sleep(0.1)
        self._send_fsm_command(0, None)
    
    def _send_fsm_command(self, command: int, expected_state: Optional[str] = None):
        """Send FSM command via ROS2RobotInterface."""
        if self.ros2_interface is None or not self.ros2_interface.is_connected:
            logger.warning("ROS2RobotInterface not connected, cannot send FSM command")
            return
        
        try:
            self.ros2_interface.send_fsm_command(command)
            logger.info(f"Sent FSM command: {command}")
            
            # Update state if expected
            if expected_state:
                with self._fsm_state_lock:
                    self._current_fsm_state = expected_state
                # Note: update() will be called from the main update loop
        except Exception as e:
            logger.error(f"Failed to send FSM command: {e}", exc_info=True)
    
    def update(self):
        """Update FSM panel button visibility based on current state.
        
        This method should be called periodically from the main update loop
        (not from ROS2 callbacks) to avoid blocking the executor.
        """
        if not self._initialized or self.server is None or not self._fsm_button_handles:
            return
        
        try:
            with self._fsm_state_lock:
                current_state = self._current_fsm_state
            
            # Update state label
            if self._fsm_state_label is not None:
                self._fsm_state_label.value = current_state
            
            # Update button visibility based on current state
            # Show buttons for states that can be reached from current state
            if current_state == "HOME":
                # HOME state: can go to HOLD, can switch pose
                if "to_home" in self._fsm_button_handles:
                    self._fsm_button_handles["to_home"].visible = False
                if "to_hold" in self._fsm_button_handles:
                    self._fsm_button_handles["to_hold"].visible = True
                if "to_ocs2" in self._fsm_button_handles:
                    self._fsm_button_handles["to_ocs2"].visible = False
                if "to_movej" in self._fsm_button_handles:
                    self._fsm_button_handles["to_movej"].visible = False
                if self._fsm_switch_pose_button is not None:
                    self._fsm_switch_pose_button.visible = True
            elif current_state == "HOLD":
                # HOLD state: can go to HOME, OCS2, or MOVEJ
                if "to_home" in self._fsm_button_handles:
                    self._fsm_button_handles["to_home"].visible = True
                if "to_hold" in self._fsm_button_handles:
                    self._fsm_button_handles["to_hold"].visible = False
                if "to_ocs2" in self._fsm_button_handles:
                    self._fsm_button_handles["to_ocs2"].visible = True
                if "to_movej" in self._fsm_button_handles:
                    self._fsm_button_handles["to_movej"].visible = True
                if self._fsm_switch_pose_button is not None:
                    self._fsm_switch_pose_button.visible = False
            elif current_state == "OCS2":
                # OCS2 state: can only go to HOLD
                if "to_home" in self._fsm_button_handles:
                    self._fsm_button_handles["to_home"].visible = False
                if "to_hold" in self._fsm_button_handles:
                    self._fsm_button_handles["to_hold"].visible = True
                if "to_ocs2" in self._fsm_button_handles:
                    self._fsm_button_handles["to_ocs2"].visible = False
                if "to_movej" in self._fsm_button_handles:
                    self._fsm_button_handles["to_movej"].visible = False
                if self._fsm_switch_pose_button is not None:
                    self._fsm_switch_pose_button.visible = False
            elif current_state == "MOVEJ":
                # MOVEJ state: can only go to HOLD
                if "to_home" in self._fsm_button_handles:
                    self._fsm_button_handles["to_home"].visible = False
                if "to_hold" in self._fsm_button_handles:
                    self._fsm_button_handles["to_hold"].visible = True
                if "to_ocs2" in self._fsm_button_handles:
                    self._fsm_button_handles["to_ocs2"].visible = False
                if "to_movej" in self._fsm_button_handles:
                    self._fsm_button_handles["to_movej"].visible = False
                if self._fsm_switch_pose_button is not None:
                    self._fsm_switch_pose_button.visible = False
            else:
                # Unknown state: hide all buttons
                for handle in self._fsm_button_handles.values():
                    handle.visible = False
                if self._fsm_switch_pose_button is not None:
                    self._fsm_switch_pose_button.visible = False
                    
        except Exception as e:
            logger.warning(f"Failed to update FSM panel: {e}")
    
    def cleanup(self):
        """Cleanup resources (subscriptions, GUI elements, etc.)."""
        # Hide all GUI elements
        try:
            if self._fsm_state_label is not None:
                self._fsm_state_label.visible = False
            for handle in self._fsm_button_handles.values():
                if handle is not None:
                    handle.visible = False
            if self._fsm_switch_pose_button is not None:
                self._fsm_switch_pose_button.visible = False
            # Hide folder if it has a visible attribute
            if self._folder_handle is not None:
                try:
                    if hasattr(self._folder_handle, 'visible'):
                        self._folder_handle.visible = False
                    elif hasattr(self._folder_handle, 'remove'):
                        self._folder_handle.remove()
                except Exception as e:
                    logger.debug(f"Could not hide/remove folder: {e}")
        except Exception as e:
            logger.warning(f"Error hiding FSM panel GUI elements: {e}")
        
        # Cleanup ROS2 subscription
        if self._fsm_command_subscription is not None:
            try:
                self._fsm_command_subscription.destroy()
            except Exception as e:
                logger.warning(f"Error destroying FSM command subscription: {e}")
            self._fsm_command_subscription = None
        
        # Clear references
        self._fsm_state_label = None
        self._fsm_button_handles.clear()
        self._fsm_switch_pose_button = None
        self._folder_handle = None
        self._initialized = False
        logger.debug("FSM panel cleaned up")
