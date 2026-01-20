"""Gripper Control Panel for ROS2 Viser."""

import logging
import threading
from enum import Enum
from typing import Optional

import viser

from ros2_robot_interface import ROS2RobotInterface

from ..i18n import get_translator

logger = logging.getLogger(__name__)


class GripperType(Enum):
    """Gripper type enumeration."""
    LEFT = "left"
    RIGHT = "right"


class GripperPanel:
    """Gripper control panel for robot gripper management.
    
    This panel provides a GUI interface for controlling left and right grippers
    (for dual-arm robots) or a single gripper (for single-arm robots).
    
    Example:
        ```python
        panel = GripperPanel(server, ros2_interface)
        panel.initialize()
        
        # In update loop:
        panel.update()
        ```
    """
    
    def __init__(
        self,
        server: viser.ViserServer,
        ros2_interface: ROS2RobotInterface,
    ):
        """Initialize Gripper panel.
        
        Args:
            server: Viser server instance for GUI creation.
            ros2_interface: ROS2RobotInterface for sending commands and getting gripper state.
        """
        self.server = server
        self.ros2_interface = ros2_interface
        self.translator = get_translator()
        
        # Gripper state tracking
        self._left_gripper_open: bool = False
        self._right_gripper_open: bool = False
        self._gripper_state_lock = threading.Lock()
        
        # Detect dual-arm mode
        self._is_dual_arm: bool = False
        
        # GUI elements - separate buttons for Open and Close
        self._left_gripper_open_button: Optional[viser.GuiButtonHandle] = None
        self._left_gripper_close_button: Optional[viser.GuiButtonHandle] = None
        self._right_gripper_open_button: Optional[viser.GuiButtonHandle] = None
        self._right_gripper_close_button: Optional[viser.GuiButtonHandle] = None
        # Folder handle for cleanup
        self._folder_handle: Optional[viser.GuiFolderHandle] = None
        # Display names for buttons (e.g., "Left Hand", "Gripper")
        self._left_display_name: str = "Left Gripper"
        self._right_display_name: str = "Right Gripper"
        
        # Note: No direct subscriptions - use ros2_interface.gripper_handler.is_open instead
        
        # Initialization flag
        self._initialized = False
        
        # Cleanup flag to prevent callbacks from running after cleanup
        self._cleaned_up = False
    
    def initialize(self):
        """Initialize the Gripper panel GUI and ROS2 subscriptions."""
        # If already initialized, cleanup first to avoid duplicates
        if self._initialized:
            logger.warning("Gripper panel already initialized, cleaning up first...")
            self.cleanup()
        
        try:
            # Detect gripper availability and mode
            self._detect_grippers()
            
            # Check if any gripper controllers are available
            has_left = self.ros2_interface.left_gripper_handler is not None if self.ros2_interface else False
            has_right = self.ros2_interface.right_gripper_handler is not None if self.ros2_interface else False
            
            # Only initialize if at least one gripper controller is detected
            if not has_left and not has_right:
                logger.info("No gripper controllers detected, skipping Gripper panel initialization")
                self._initialized = False
                return
            
            # Reset cleanup flag
            self._cleaned_up = False
            
            # Initialize GUI
            self._init_gui()
            
            # Initial update to set button states
            self.update()
            
            self._initialized = True
            logger.info("✅ Gripper control panel initialized")
            
        except Exception as e:
            logger.error(f"Failed to initialize Gripper panel: {e}", exc_info=True)
            raise
    
    def _detect_grippers(self):
        """Detect available grippers and determine if dual-arm mode."""
        if self.ros2_interface is None or not self.ros2_interface.is_connected:
            logger.warning("ROS2RobotInterface not connected, cannot detect grippers")
            return
        
        # Check if left and right gripper handlers exist
        has_left = self.ros2_interface.left_gripper_handler is not None
        has_right = self.ros2_interface.right_gripper_handler is not None
        
        # Also check config to see if controllers were detected but handlers not created
        left_controller_detected = (
            self.ros2_interface.config.left_gripper_controller_name is not None or
            self.ros2_interface.config.gripper_command_topic is not None
        )
        right_controller_detected = (
            self.ros2_interface.config.right_gripper_controller_name is not None or
            self.ros2_interface.config.right_gripper_command_topic is not None
        )
        
        # Log detection details for debugging
        if left_controller_detected and not has_left:
            logger.warning(f"Left gripper controller detected ({self.ros2_interface.config.left_gripper_controller_name or self.ros2_interface.config.gripper_command_topic}) but handler not created (gripper_enabled={self.ros2_interface.config.gripper_enabled})")
        if right_controller_detected and not has_right:
            logger.warning(f"Right gripper controller detected ({self.ros2_interface.config.right_gripper_controller_name or self.ros2_interface.config.right_gripper_command_topic}) but handler not created (gripper_enabled={self.ros2_interface.config.gripper_enabled})")
        
        self._is_dual_arm = has_left and has_right
        
        if not has_left and not has_right:
            logger.info("No gripper controllers detected")
        elif self._is_dual_arm:
            logger.info("Detected mode: DUAL-ARM (left and right grippers)")
        else:
            if has_left and not has_right:
                logger.info("Detected mode: SINGLE-ARM (left gripper only)")
            elif has_right and not has_left:
                logger.info("Detected mode: SINGLE-ARM (right gripper only)")
            else:
                logger.info("Detected mode: SINGLE-ARM")
    
    def _init_gui(self):
        """Initialize Gripper control panel GUI elements."""
        if self.server is None:
            return
        
        try:
            # Check if any gripper controllers are available
            has_left = self.ros2_interface.left_gripper_handler is not None
            has_right = self.ros2_interface.right_gripper_handler is not None
            
            # Only create panel if at least one gripper controller is detected
            if not has_left and not has_right:
                logger.info("No gripper controllers detected, skipping EE Control panel")
                return
            
            # Get display names from controller names
            if has_left:
                left_controller_name = self.ros2_interface.config.left_gripper_controller_name
                self._left_display_name = self._get_display_name(left_controller_name)
            
            if has_right:
                right_controller_name = self.ros2_interface.config.right_gripper_controller_name
                self._right_display_name = self._get_display_name(right_controller_name)
            
            # Create folder and save reference for cleanup
            self._folder_handle = self.server.gui.add_folder(self.translator("ee_control"))
            with self._folder_handle:
                # Left gripper buttons (Open and Close)
                # Open button - Green
                open_text = self.translator("open")
                close_text = self.translator("close")
                self._left_gripper_open_button = self.server.gui.add_button(
                    f"{open_text} {self._left_display_name}", 
                    color="green"
                )
                self._left_gripper_open_button.on_click(lambda _: self._on_gripper_command(GripperType.LEFT, True))
                
                # Close button - Red
                self._left_gripper_close_button = self.server.gui.add_button(
                    f"{close_text} {self._left_display_name}", 
                    color="red"
                )
                self._left_gripper_close_button.on_click(lambda _: self._on_gripper_command(GripperType.LEFT, False))
                
                # Right gripper buttons (for dual-arm mode)
                # Open button - Green
                self._right_gripper_open_button = self.server.gui.add_button(
                    f"{open_text} {self._right_display_name}", 
                    color="green"
                )
                self._right_gripper_open_button.on_click(lambda _: self._on_gripper_command(GripperType.RIGHT, True))
                
                # Close button - Red
                self._right_gripper_close_button = self.server.gui.add_button(
                    f"{close_text} {self._right_display_name}", 
                    color="red"
                )
                self._right_gripper_close_button.on_click(lambda _: self._on_gripper_command(GripperType.RIGHT, False))
                
                # Update button visibility based on detected mode
                self._update_button_visibility()
                
        except Exception as e:
            logger.error(f"Failed to initialize Gripper panel GUI: {e}", exc_info=True)
            raise
    
    
    def _on_gripper_command(self, gripper_type: GripperType, should_open: bool):
        """Handle gripper button click - send open or close command.
        
        Args:
            gripper_type: LEFT or RIGHT gripper
            should_open: True to open, False to close
        """
        if self.ros2_interface is None or not self.ros2_interface.is_connected:
            logger.warning("ROS2RobotInterface not connected, cannot send gripper command")
            return
        
        try:
            if gripper_type == GripperType.LEFT:
                if self.ros2_interface.left_gripper_handler is None:
                    logger.warning("Left gripper handler not available")
                    return
                
                # Send command using target_command (recommended method)
                target_value = 1 if should_open else 0
                self.ros2_interface.left_gripper_handler.send_target_command(target_value)
                
                logger.info(f"Sent left gripper command: {'Open' if should_open else 'Close'}")
                
            elif gripper_type == GripperType.RIGHT:
                if self.ros2_interface.right_gripper_handler is None:
                    logger.warning("Right gripper handler not available")
                    return
                
                # Send command using target_command (recommended method)
                target_value = 1 if should_open else 0
                self.ros2_interface.right_gripper_handler.send_target_command(target_value)
                
                logger.info(f"Sent right gripper command: {'Open' if should_open else 'Close'}")
                
        except Exception as e:
            logger.error(f"Failed to send gripper command: {e}", exc_info=True)
    
    def _update_button_visibility(self):
        """Update button visibility based on detected gripper mode and current state."""
        has_left = self.ros2_interface.left_gripper_handler is not None
        has_right = self.ros2_interface.right_gripper_handler is not None
        
        with self._gripper_state_lock:
            left_open = self._left_gripper_open
            right_open = self._right_gripper_open
        
        if not has_left and not has_right:
            # No grippers: hide all buttons
            if self._left_gripper_open_button is not None:
                self._left_gripper_open_button.visible = False
            if self._left_gripper_close_button is not None:
                self._left_gripper_close_button.visible = False
            if self._right_gripper_open_button is not None:
                self._right_gripper_open_button.visible = False
            if self._right_gripper_close_button is not None:
                self._right_gripper_close_button.visible = False
        elif self._is_dual_arm:
            # Dual arm mode: show buttons based on state
            # Left gripper
            if self._left_gripper_open_button is not None:
                self._left_gripper_open_button.visible = not left_open  # Show Open if closed
            if self._left_gripper_close_button is not None:
                self._left_gripper_close_button.visible = left_open  # Show Close if open
            # Right gripper
            if self._right_gripper_open_button is not None:
                self._right_gripper_open_button.visible = not right_open  # Show Open if closed
            if self._right_gripper_close_button is not None:
                self._right_gripper_close_button.visible = right_open  # Show Close if open
        else:
            # Single arm mode: show left buttons based on state, hide right buttons
            # Left gripper
            if self._left_gripper_open_button is not None:
                self._left_gripper_open_button.visible = not left_open  # Show Open if closed
            if self._left_gripper_close_button is not None:
                self._left_gripper_close_button.visible = left_open  # Show Close if open
            # Right gripper (hidden in single-arm mode)
            if self._right_gripper_open_button is not None:
                self._right_gripper_open_button.visible = False
            if self._right_gripper_close_button is not None:
                self._right_gripper_close_button.visible = False
    
    def _get_display_name(self, controller_name: Optional[str]) -> str:
        """Get display name from controller name.
        
        Args:
            controller_name: Controller name (e.g., "left_hand_controller", "gripper_controller")
            
        Returns:
            Formatted display name (e.g., "Left Hand", "Gripper")
        """
        if not controller_name:
            return "Gripper"
        
        name = controller_name
        
        # Remove common suffixes
        suffixes = ["_controller", "controller", "_gripper", "gripper", "_hand", "hand"]
        for suffix in suffixes:
            name_lower = name.lower()
            suffix_lower = suffix.lower()
            if name_lower.endswith(suffix_lower):
                name = name[:len(name) - len(suffix)]
                break
        
        # Remove leading/trailing underscores
        name = name.strip('_')
        
        # Replace underscores with spaces
        name = name.replace('_', ' ')
        
        # Capitalize first letter of each word
        name = ' '.join(word.capitalize() for word in name.split())
        
        return name if name else "Gripper"
    
    def update(self):
        """Update Gripper panel button text based on current state.
        
        This method should be called periodically from the main update loop
        (not from ROS2 callbacks) to avoid blocking the executor.
        """
        if not self._initialized or self.server is None:
            return
        
        try:
            # Get state from gripper handlers (they subscribe to target_command internally)
            if self.ros2_interface.left_gripper_handler is not None:
                try:
                    with self.ros2_interface.left_gripper_handler.data_lock:
                        left_open = self.ros2_interface.left_gripper_handler.is_open
                    with self._gripper_state_lock:
                        self._left_gripper_open = left_open
                except Exception as e:
                    logger.debug(f"Could not get left gripper state from handler: {e}")
                    with self._gripper_state_lock:
                        left_open = self._left_gripper_open
            else:
                with self._gripper_state_lock:
                    left_open = self._left_gripper_open
            
            if self.ros2_interface.right_gripper_handler is not None:
                try:
                    with self.ros2_interface.right_gripper_handler.data_lock:
                        right_open = self.ros2_interface.right_gripper_handler.is_open
                    with self._gripper_state_lock:
                        self._right_gripper_open = right_open
                except Exception as e:
                    logger.debug(f"Could not get right gripper state from handler: {e}")
                    with self._gripper_state_lock:
                        right_open = self._right_gripper_open
            else:
                with self._gripper_state_lock:
                    right_open = self._right_gripper_open
            
            # Update button visibility based on current state
            # Left gripper
            if self.ros2_interface.left_gripper_handler is not None:
                if self._left_gripper_open_button is not None:
                    self._left_gripper_open_button.visible = not left_open  # Show Open if closed
                if self._left_gripper_close_button is not None:
                    self._left_gripper_close_button.visible = left_open  # Show Close if open
            
            # Right gripper (only in dual-arm mode)
            if (self._is_dual_arm and 
                self.ros2_interface.right_gripper_handler is not None):
                if self._right_gripper_open_button is not None:
                    self._right_gripper_open_button.visible = not right_open  # Show Open if closed
                if self._right_gripper_close_button is not None:
                    self._right_gripper_close_button.visible = right_open  # Show Close if open
                    
        except Exception as e:
            logger.warning(f"Failed to update Gripper panel: {e}")
    
    def update_gui_labels(self):
        """Update GUI labels with current language without recreating subscriptions.
        
        This method only updates the text/labels of GUI elements, preserving
        all ROS2 subscriptions and state.
        """
        if not self._initialized or self.server is None:
            return
        
        try:
            # Update folder name and button labels
            if self._folder_handle is not None:
                # Viser folders don't support renaming, so we need to recreate the folder
                # But we'll keep the subscriptions intact
                old_folder = self._folder_handle
                
                # Create new folder with new language
                self._folder_handle = self.server.gui.add_folder(self.translator("ee_control"))
                
                # Move all GUI elements to new folder (recreate them)
                with self._folder_handle:
                    open_text = self.translator("open")
                    close_text = self.translator("close")
                    
                    # Recreate left gripper buttons
                    self._left_gripper_open_button = self.server.gui.add_button(
                        f"{open_text} {self._left_display_name}",
                        color="green"
                    )
                    self._left_gripper_open_button.on_click(
                        lambda _: self._on_gripper_command(GripperType.LEFT, True)
                    )
                    
                    self._left_gripper_close_button = self.server.gui.add_button(
                        f"{close_text} {self._left_display_name}",
                        color="red"
                    )
                    self._left_gripper_close_button.on_click(
                        lambda _: self._on_gripper_command(GripperType.LEFT, False)
                    )
                    
                    # Recreate right gripper buttons (if dual-arm)
                    if self._is_dual_arm:
                        self._right_gripper_open_button = self.server.gui.add_button(
                            f"{open_text} {self._right_display_name}",
                            color="green"
                        )
                        self._right_gripper_open_button.on_click(
                            lambda _: self._on_gripper_command(GripperType.RIGHT, True)
                        )
                        
                        self._right_gripper_close_button = self.server.gui.add_button(
                            f"{close_text} {self._right_display_name}",
                            color="red"
                        )
                        self._right_gripper_close_button.on_click(
                            lambda _: self._on_gripper_command(GripperType.RIGHT, False)
                        )
                
                # Remove old folder
                try:
                    old_folder.remove()
                except Exception as e:
                    logger.debug(f"Could not remove old folder: {e}")
                
                # Update UI to reflect current state
                self.update()
                
                logger.debug("Gripper panel GUI labels updated")
        except Exception as e:
            logger.warning(f"Failed to update Gripper panel GUI labels: {e}")
    
    def cleanup(self):
        """Cleanup resources (subscriptions, GUI elements, etc.)."""
        # Set cleanup flag first to prevent callbacks from running
        self._cleaned_up = True
        
        # Hide all GUI elements
        try:
            if self._left_gripper_open_button is not None:
                self._left_gripper_open_button.visible = False
            if self._left_gripper_close_button is not None:
                self._left_gripper_close_button.visible = False
            if self._right_gripper_open_button is not None:
                self._right_gripper_open_button.visible = False
            if self._right_gripper_close_button is not None:
                self._right_gripper_close_button.visible = False
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
            logger.warning(f"Error hiding Gripper panel GUI elements: {e}")
        
        # Note: No direct subscriptions to cleanup - ros2_interface handles it
        
        # Clear references
        self._left_gripper_open_button = None
        self._left_gripper_close_button = None
        self._right_gripper_open_button = None
        self._right_gripper_close_button = None
        self._folder_handle = None
        self._initialized = False
        logger.debug("Gripper panel cleaned up")
