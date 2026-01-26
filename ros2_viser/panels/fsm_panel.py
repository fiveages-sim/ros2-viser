"""FSM Control Panel for ROS2 Viser."""

import logging
import time
from typing import Optional

import viser

from ros2_robot_interface import ROS2RobotInterface

from ..i18n import get_translator
from ..config import ROS2ViserConfig

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
        fsm_command_topic: str = "/fsm_command",
        config: Optional[ROS2ViserConfig] = None
    ):
        """Initialize FSM panel.
        
        Args:
            server: Viser server instance for GUI creation.
            ros2_interface: ROS2RobotInterface for sending commands and subscribing to topics.
            fsm_command_topic: ROS2 topic name for FSM commands (default: "/fsm_command").
            config: Optional ROS2ViserConfig for end-effector marker controls.
        """
        self.server = server
        self.ros2_interface = ros2_interface
        self.fsm_command_topic = fsm_command_topic
        self.config = config
        self.translator = get_translator()
        
        # FSM state tracking
        self._current_fsm_state: str = "INVALID"
        
        # GUI elements
        self._fsm_state_label: Optional[viser.GuiTextHandle] = None
        self._fsm_button_handles: dict[str, viser.GuiButtonHandle] = {}
        self._fsm_switch_pose_button: Optional[viser.GuiButtonHandle] = None
        # End-effector marker controls (only visible in OCS2 mode)
        self._marker_publish_mode_dropdown: Optional[viser.GuiDropdownHandle] = None
        self._send_marker_pose_button: Optional[viser.GuiButtonHandle] = None
        # Folder handle for cleanup
        self._folder_handle: Optional[viser.GuiFolderHandle] = None
        
        # Initialization flag
        self._initialized = False
        
        # Cleanup flag to prevent callbacks from running after cleanup
        self._cleaned_up = False
    
    def initialize(self):
        """Initialize the FSM panel GUI and ROS2 subscriptions."""
        # If already initialized, cleanup first to avoid duplicates
        if self._initialized:
            logger.warning("FSM panel already initialized, cleaning up first...")
            self.cleanup()
        
        try:
            # Reset cleanup flag
            self._cleaned_up = False
            
            # Initialize GUI
            self._init_gui()
            self._current_fsm_state = "INVALID"
            self.update()
            
            self._initialized = True
            logger.debug("✅ FSM control panel initialized")
            
        except Exception as e:
            logger.error(f"Failed to initialize FSM panel: {e}", exc_info=True)
            raise
    
    def _init_gui(self):
        """Initialize FSM control panel GUI elements."""
        if self.server is None:
            return
        
        try:
            # Create folder and save reference for cleanup
            self._folder_handle = self.server.gui.add_folder(self.translator("fsm_control"))
            with self._folder_handle:
                # Current state display (read-only)
                self._fsm_state_label = self.server.gui.add_text(
                    self.translator("current_state"),
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
                switch_pose_text = self.translator("switch_pose")
                self._fsm_switch_pose_button = self.server.gui.add_button(
                    f"{switch_pose_text} (Home ↔ Rest)", 
                    color=(255, 100, 0)
                )
                self._fsm_switch_pose_button.on_click(self._on_switch_pose_clicked)
                
                # End-effector marker controls (only if markers are enabled)
                # These controls are only visible in OCS2 mode
                if self.config is not None and self.config.enable_end_effector_marker:
                    self._marker_publish_mode_dropdown = self.server.gui.add_dropdown(
                        self.translator("marker_publish_mode"),
                        options=[self.translator("continuous_publish"), self.translator("single_publish")],
                        initial_value=self.translator("continuous_publish") if self.config.marker_continuous_publish else self.translator("single_publish")
                    )
             
                    self._marker_publish_mode_dropdown.visible = False
                    
                    # Send button for single-shot mode (initially hidden)
                    self._send_marker_pose_button = self.server.gui.add_button(
                        self.translator("send_marker_pose"),
                        color="green"
                    )
       
                    self._send_marker_pose_button.visible = False
                
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
        self.ros2_interface.send_fsm_command(command)
    
    def set_marker_callbacks(self, on_mode_changed, on_send_clicked):
        """Set callbacks for marker controls.
        
        Args:
            on_mode_changed: Callback function for publish mode dropdown change
            on_send_clicked: Callback function for send button click
        """
        if self._marker_publish_mode_dropdown is not None:
            self._marker_publish_mode_dropdown.on_update(on_mode_changed)
        if self._send_marker_pose_button is not None:
            self._send_marker_pose_button.on_click(on_send_clicked)
    
    def get_marker_controls(self):
        """Get marker control handles for connection to marker manager.
        
        Returns:
            Tuple of (publish_mode_dropdown, send_button) or (None, None) if not available.
        """
        return (self._marker_publish_mode_dropdown, self._send_marker_pose_button)
    
    def update(self):
        """Update FSM panel button visibility based on current state.
        
        This method should be called periodically from the main update loop
        (not from ROS2 callbacks) to avoid blocking the executor.
        Only updates button visibility when the state actually changes.
        """
        current_state = self.ros2_interface.get_fsm_state()
        # Update if state changed or if this is the first update (INVALID state)
        if current_state != self._current_fsm_state or self._current_fsm_state == "INVALID":
            self._current_fsm_state = current_state
            
            # Only update button visibility when state changes
            if current_state == "HOME":
                self._fsm_button_handles["to_home"].visible = False
                self._fsm_button_handles["to_hold"].visible = True
                self._fsm_button_handles["to_ocs2"].visible = False
                self._fsm_button_handles["to_movej"].visible = False
                self._fsm_switch_pose_button.visible = True
            elif current_state == "HOLD":
                self._fsm_button_handles["to_home"].visible = True
                self._fsm_button_handles["to_hold"].visible = False
                self._fsm_button_handles["to_ocs2"].visible = True
                self._fsm_button_handles["to_movej"].visible = True
                self._fsm_switch_pose_button.visible = False
            elif current_state == "OCS2":
                self._fsm_button_handles["to_home"].visible = False
                self._fsm_button_handles["to_hold"].visible = True
                self._fsm_button_handles["to_ocs2"].visible = False
                self._fsm_button_handles["to_movej"].visible = False
                self._fsm_switch_pose_button.visible = False
                # Show marker controls in OCS2 mode (visibility managed by marker manager)
                # Marker manager will handle showing/hiding based on publish mode
            elif current_state == "MOVEJ":
                # MOVEJ state: can only go to HOLD
                self._fsm_button_handles["to_home"].visible = False
                self._fsm_button_handles["to_hold"].visible = True
                self._fsm_button_handles["to_ocs2"].visible = False
                self._fsm_button_handles["to_movej"].visible = False
                self._fsm_switch_pose_button.visible = False
            else:
                logger.warning(f"Unknown FSM state: {current_state}")
    
    def update_gui_labels(self):
        """Update GUI labels with current language without recreating subscriptions.
        
        This method only updates the text/labels of GUI elements, preserving
        all ROS2 subscriptions and state.
        """
        if self._folder_handle is not None:
            old_folder = self._folder_handle
            self._folder_handle = self.server.gui.add_folder(self.translator("fsm_control"))
            with self._folder_handle:
                self._fsm_state_label = self.server.gui.add_text(
                    self.translator("current_state"),
                    initial_value=self._current_fsm_state,
                    disabled=True
                )
                
                # Recreate buttons
                self._fsm_button_handles["to_home"] = self.server.gui.add_button("HOME", color=(128, 0, 128))
                self._fsm_button_handles["to_home"].on_click(
                    lambda _: self._send_fsm_command(1, "HOME")
                )
                
                self._fsm_button_handles["to_hold"] = self.server.gui.add_button("HOLD", color="yellow")
                self._fsm_button_handles["to_hold"].on_click(
                    lambda _: self._send_fsm_command(2, "HOLD")
                )
                
                self._fsm_button_handles["to_ocs2"] = self.server.gui.add_button("OCS2", color="blue")
                self._fsm_button_handles["to_ocs2"].on_click(
                    lambda _: self._send_fsm_command(3, "OCS2")
                )
                
                self._fsm_button_handles["to_movej"] = self.server.gui.add_button("MOVEJ", color="cyan")
                self._fsm_button_handles["to_movej"].on_click(
                    lambda _: self._send_fsm_command(4, "MOVEJ")
                )
                
                # Recreate switch pose button
                switch_pose_text = self.translator("switch_pose")
                self._fsm_switch_pose_button = self.server.gui.add_button(
                    f"{switch_pose_text} (Home ↔ Rest)",
                    color=(255, 100, 0)
                )
                self._fsm_switch_pose_button.on_click(self._on_switch_pose_clicked)
            
            # Remove old folder
            try:
                old_folder.remove()
            except Exception as e:
                logger.debug(f"Could not remove old folder: {e}")
            
            self._current_fsm_state = "INVALID"
            self.update()
            
            logger.debug("FSM panel GUI labels updated")
    
    def cleanup(self):
        """Cleanup resources (subscriptions, GUI elements, etc.)."""
        # Set cleanup flag first to prevent callbacks from running
        self._cleaned_up = True
        
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
        
        # Note: No direct subscription to cleanup - ros2_interface handles it
        
        # Clear references
        self._fsm_state_label = None
        self._fsm_button_handles.clear()
        self._fsm_switch_pose_button = None
        self._marker_publish_mode_dropdown = None
        self._send_marker_pose_button = None
        self._folder_handle = None
        self._initialized = False
        logger.debug("FSM panel cleaned up")
