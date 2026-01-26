"""Gripper Control Panel for ROS2 Viser."""

import logging
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
        # Track previous state to detect changes
        self._prev_left_gripper_open: Optional[bool] = None
        self._prev_right_gripper_open: Optional[bool] = None
        
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
    
    def _has_gripper(self, gripper_type: GripperType) -> bool:
        """Check if a gripper handler is available.
        
        Args:
            gripper_type: LEFT or RIGHT gripper
            
        Returns:
            True if gripper handler exists, False otherwise
        """
        if self.ros2_interface is None:
            return False
        if gripper_type == GripperType.LEFT:
            return self.ros2_interface.left_gripper_handler is not None
        else:
            return self.ros2_interface.right_gripper_handler is not None
    
    def _get_gripper_handlers(self) -> tuple[bool, bool]:
        """Get availability status of left and right grippers.
        
        Returns:
            Tuple of (has_left, has_right) boolean values
        """
        has_left = self._has_gripper(GripperType.LEFT)
        has_right = self._has_gripper(GripperType.RIGHT)
        return has_left, has_right
    
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
            has_left, has_right = self._get_gripper_handlers()
            
            # Only initialize if at least one gripper controller is detected
            if not has_left and not has_right:
                logger.info("No gripper controllers detected, skipping Gripper panel initialization")
                self._initialized = False
                return
            
            # Reset cleanup flag
            self._cleaned_up = False
            
            # Reset state tracking to ensure first update() call will update button visibility
            self._prev_left_gripper_open = None
            self._prev_right_gripper_open = None
            
            # Initialize GUI
            self._init_gui()
            
            # Initial update to set button states
            self.update()
            
            self._initialized = True
            logger.debug("✅ Gripper control panel initialized")
            
        except Exception as e:
            logger.error(f"Failed to initialize Gripper panel: {e}", exc_info=True)
            raise
    
    def _detect_grippers(self):
        """Detect available grippers and determine if dual-arm mode."""
        if self.ros2_interface is None or not self.ros2_interface.is_connected:
            logger.warning("ROS2RobotInterface not connected, cannot detect grippers")
            return
        
        # Check if left and right gripper handlers exist
        has_left, has_right = self._get_gripper_handlers()
        
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
    
    def _create_gripper_buttons(self, gripper_type: GripperType, display_name: str):
        """Create open and close buttons for a gripper.
        
        Args:
            gripper_type: LEFT or RIGHT gripper
            display_name: Display name for the gripper (e.g., "Left Hand", "Gripper")
        """
        open_text = self.translator("open")
        close_text = self.translator("close")
        
        # Create open button (green)
        open_button = self.server.gui.add_button(
            f"{open_text} {display_name}",
            color="green"
        )
        open_button.on_click(lambda _: self._on_gripper_command(gripper_type, True))
        
        # Create close button (red)
        close_button = self.server.gui.add_button(
            f"{close_text} {display_name}",
            color="red"
        )
        close_button.on_click(lambda _: self._on_gripper_command(gripper_type, False))
        
        # Store button references
        if gripper_type == GripperType.LEFT:
            self._left_gripper_open_button = open_button
            self._left_gripper_close_button = close_button
        else:
            self._right_gripper_open_button = open_button
            self._right_gripper_close_button = close_button
    
    def _init_gui(self):
        """Initialize Gripper control panel GUI elements."""
        has_left, has_right = self._get_gripper_handlers()
        
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
            # Create buttons for available grippers
            if has_left:
                self._create_gripper_buttons(GripperType.LEFT, self._left_display_name)
            
            if has_right:
                self._create_gripper_buttons(GripperType.RIGHT, self._right_display_name)
            
    
    
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
            # Get the appropriate handler
            if gripper_type == GripperType.LEFT:
                handler = self.ros2_interface.left_gripper_handler
                gripper_name = "left"
            else:
                handler = self.ros2_interface.right_gripper_handler
                gripper_name = "right"
            
            if handler is None:
                logger.warning(f"{gripper_name.capitalize()} gripper handler not available")
                return
            
            # Send command using target_command (recommended method)
            target_value = 1 if should_open else 0
            handler.send_target_command(target_value)
                
        except Exception as e:
            logger.error(f"Failed to send gripper command: {e}", exc_info=True)
    
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
    
    def _update_gripper_state(self, gripper_type: GripperType):
        """Update button visibility for a single gripper based on current state.
        
        Args:
            gripper_type: LEFT or RIGHT gripper
        """
        if gripper_type == GripperType.LEFT:
            handler = self.ros2_interface.left_gripper_handler
            prev_state = self._prev_left_gripper_open
            open_button = self._left_gripper_open_button
            close_button = self._left_gripper_close_button
            update_buttons = True  # Left gripper always updates
        else:
            handler = self.ros2_interface.right_gripper_handler
            prev_state = self._prev_right_gripper_open
            open_button = self._right_gripper_open_button
            close_button = self._right_gripper_close_button
            update_buttons = self._is_dual_arm  # Right gripper only updates in dual-arm mode
        
        if handler is None:
            return
        
        is_open = handler.is_open
        
        # Update state
        if gripper_type == GripperType.LEFT:
            self._left_gripper_open = is_open
        else:
            self._right_gripper_open = is_open
        
        changed = is_open != prev_state or prev_state is None
        
        if changed:
            # Update previous state
            if gripper_type == GripperType.LEFT:
                self._prev_left_gripper_open = is_open
            else:
                self._prev_right_gripper_open = is_open
            
            # Update button visibility
            if update_buttons:
                if open_button is not None:
                    open_button.visible = not is_open  # Show Open if closed
                if close_button is not None:
                    close_button.visible = is_open  # Show Close if open
    
    def update(self):
        """Update Gripper panel button visibility based on current state.
        
        This method should be called periodically from the main update loop
        (not from ROS2 callbacks) to avoid blocking the executor.
        Only updates button visibility when the state actually changes.
        """
        if not self._initialized or self.server is None:
            return
        
        try:
            # Update state for available grippers
            if self._has_gripper(GripperType.LEFT):
                self._update_gripper_state(GripperType.LEFT)
            
            if self._has_gripper(GripperType.RIGHT):
                self._update_gripper_state(GripperType.RIGHT)
                    
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
            if self._folder_handle is not None:
                old_folder = self._folder_handle
                self._folder_handle = self.server.gui.add_folder(self.translator("ee_control"))
                with self._folder_handle:
                    # Recreate buttons for available grippers
                    has_left, has_right = self._get_gripper_handlers()
                    
                    if has_left:
                        self._create_gripper_buttons(GripperType.LEFT, self._left_display_name)
                    
                    if has_right and self._is_dual_arm:
                        self._create_gripper_buttons(GripperType.RIGHT, self._right_display_name)
                
                # Remove old folder
                try:
                    old_folder.remove()
                except Exception as e:
                    logger.debug(f"Could not remove old folder: {e}")
                
                self._prev_left_gripper_open = None
                self._prev_right_gripper_open = None
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
        
        # Reset state tracking
        self._prev_left_gripper_open = None
        self._prev_right_gripper_open = None
        
        logger.debug("Gripper panel cleaned up")
