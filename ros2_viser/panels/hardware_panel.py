"""Hardware Control Panel for ROS2 Viser."""

import logging
from typing import Optional

import viser

from ros2_robot_interface import ROS2RobotInterface

from ..i18n import get_translator
from ..modal_dialog import M6CCSConfigDialog

logger = logging.getLogger(__name__)


class HardwarePanel:
    """Hardware control panel for system hardware management.
    
    This panel provides a GUI interface for hardware system configuration.
    It is automatically shown when a "system" node is detected in the ROS2 node list.
    If "m6_ccs_system" node is detected, a configuration button is displayed.
    
    Example:
        ```python
        panel = HardwarePanel(server, ros2_interface, has_m6_ccs_system=True)
        panel.initialize()
        ```
    """
    
    def __init__(
        self,
        server: viser.ViserServer,
        ros2_interface: ROS2RobotInterface,
        has_m6_ccs_system: bool = False
    ):
        """Initialize Hardware panel.
        
        Args:
            server: Viser server instance for GUI creation.
            ros2_interface: ROS2RobotInterface for system operations.
            has_m6_ccs_system: Whether m6_ccs_system node is detected. Default: False
        """
        self.server = server
        self.ros2_interface = ros2_interface
        self.has_m6_ccs_system = has_m6_ccs_system
        self.translator = get_translator()
        
        # GUI elements
        self._folder_handle: Optional[viser.GuiFolderHandle] = None
        self._status_text: Optional[viser.GuiTextHandle] = None
        self._m6_ccs_config_button: Optional[viser.GuiButtonHandle] = None
        
        # Modal dialog instance
        self._m6_ccs_dialog: Optional[M6CCSConfigDialog] = None
        
        # Initialization flag
        self._initialized = False
        
        # Cleanup flag to prevent callbacks from running after cleanup
        self._cleaned_up = False
    
    def initialize(self):
        """Initialize the Hardware panel GUI."""
        # If already initialized, cleanup first to avoid duplicates
        if self._initialized:
            logger.warning("Hardware panel already initialized, cleaning up first...")
            self.cleanup()
        
        try:
            # Reset cleanup flag
            self._cleaned_up = False
            
            # Initialize GUI
            self._init_gui()
            
            self._initialized = True
            logger.debug("✅ Hardware control panel initialized")
            
        except Exception as e:
            logger.error(f"Failed to initialize Hardware panel: {e}", exc_info=True)
            raise
    
    def _init_gui(self):
        """Initialize Hardware control panel GUI elements."""
        if self.server is None:
            return
        
        try:
            # Create folder and save reference for cleanup
            self._folder_handle = self.server.gui.add_folder(self.translator("hardware_control"))
            with self._folder_handle:
                # Status display
                status_text = self.translator("hardware_system_detected")
                if self.has_m6_ccs_system:
                    status_text += f" ({self.translator('m6_ccs_system_detected')})"
                
                self._status_text = self.server.gui.add_text(
                    self.translator("status"),
                    initial_value=status_text,
                    disabled=True
                )
                
                # M6 CCS System configuration button (only shown if m6_ccs_system is detected)
                if self.has_m6_ccs_system:
                    self._m6_ccs_config_button = self.server.gui.add_button(
                        self.translator("configure_m6_ccs_system"),
                        color="green"
                    )
                    self._m6_ccs_config_button.on_click(
                        lambda _: self._on_m6_ccs_config_clicked()
                    )
                    logger.info("M6 CCS System configuration button added")
                
        except Exception as e:
            logger.error(f"Failed to initialize Hardware panel GUI: {e}", exc_info=True)
            raise
    
    def _on_m6_ccs_config_clicked(self):
        """Handle M6 CCS System configuration button click - opens configuration dialog."""
        if self._cleaned_up:
            return
        
        try:
            logger.info("M6 CCS System configuration button clicked - opening dialog")
            
            # Close existing dialog if any
            if self._m6_ccs_dialog is not None and self._m6_ccs_dialog.is_open:
                self._m6_ccs_dialog.close()
            
            # Find m6_ccs_system node
            nodes = self.ros2_interface.list_nodes()
            m6_ccs_node = None
            for node in nodes:
                if 'm6_ccs_system' in node['name'].lower() or 'm6_ccs_system' in node['full_name'].lower():
                    m6_ccs_node = node
                    break
            
            if m6_ccs_node is None:
                logger.warning("m6_ccs_system node not found")
                return
            
            # Create and show modal dialog
            self._m6_ccs_dialog = M6CCSConfigDialog(
                server=self.server,
                ros2_interface=self.ros2_interface,
                full_node_name=m6_ccs_node['full_name'],
                on_confirm=self._on_dialog_confirm,
                on_cancel=self._on_dialog_cancel
            )
            self._m6_ccs_dialog.show()
            
        except Exception as e:
            logger.error(f"Error handling M6 CCS System configuration: {e}", exc_info=True)
    
    def _on_dialog_confirm(self, changed_params: dict):
        """Handle dialog confirm button click.
        
        Args:
            changed_params: Dictionary containing changed parameters: {param_name: new_value}
        """
        if self._cleaned_up:
            return
        
        try:
            logger.info("M6 CCS System configuration confirmed")
            
            if changed_params:
                logger.info(f"Changed {len(changed_params)} parameters: {list(changed_params.keys())}")
                
                # Update status text
                if self._status_text is not None:
                    param_names = ', '.join(list(changed_params.keys())[:3])
                    if len(changed_params) > 3:
                        param_names += f" (+{len(changed_params) - 3} more)"
                    self._status_text.value = f"{self.translator('m6_ccs_system_config_clicked')} - {param_names} {self.translator('configuration_saved')}"
            else:
                logger.info("No parameters were changed")
                if self._status_text is not None:
                    self._status_text.value = f"{self.translator('m6_ccs_system_config_clicked')} - {self.translator('no_changes')}"
            
        except Exception as e:
            logger.error(f"Error confirming configuration: {e}", exc_info=True)
    
    def _on_dialog_cancel(self):
        """Handle dialog cancel button click."""
        if self._cleaned_up:
            return
        
        try:
            logger.info("M6 CCS System configuration cancelled")
            
        except Exception as e:
            logger.error(f"Error cancelling configuration: {e}", exc_info=True)
    
    def update_gui_labels(self):
        """Update GUI labels with current language without recreating subscriptions.
        
        This method only updates the text/labels of GUI elements, preserving
        all state and subscriptions.
        """
        if not self._initialized or self._cleaned_up or self.server is None:
            return
        
        try:
            # Viser folders don't support renaming, so we need to recreate the folder
            # But we'll keep the state intact
            old_folder = self._folder_handle
            
            # Store current status value
            current_status_value = None
            if self._status_text is not None:
                current_status_value = self._status_text.value
            
            # Create new folder with new language
            self._folder_handle = self.server.gui.add_folder(self.translator("hardware_control"))
            
            # Recreate GUI elements in new folder
            with self._folder_handle:
                # Status display
                status_text = self.translator("hardware_system_detected")
                if self.has_m6_ccs_system:
                    status_text += f" ({self.translator('m6_ccs_system_detected')})"
                
                # Use current value if available, otherwise use default
                status_value = current_status_value if current_status_value else status_text
                
                self._status_text = self.server.gui.add_text(
                    self.translator("status"),
                    initial_value=status_value,
                    disabled=True
                )
                
                # M6 CCS System configuration button (only shown if m6_ccs_system is detected)
                if self.has_m6_ccs_system:
                    self._m6_ccs_config_button = self.server.gui.add_button(
                        self.translator("configure_m6_ccs_system"),
                        color="green"
                    )
                    self._m6_ccs_config_button.on_click(
                        lambda _: self._on_m6_ccs_config_clicked()
                    )
            
            # Remove old folder
            if old_folder is not None:
                old_folder.remove()
            
            logger.debug("Hardware panel GUI labels updated")
            
        except Exception as e:
            logger.warning(f"Failed to update Hardware panel GUI labels: {e}", exc_info=True)
    
    def cleanup(self):
        """Cleanup the Hardware panel and remove GUI elements."""
        if self._cleaned_up:
            return
        
        self._cleaned_up = True
        
        try:
            # Close dialog if open
            if self._m6_ccs_dialog is not None and self._m6_ccs_dialog.is_open:
                self._m6_ccs_dialog.close()
            self._m6_ccs_dialog = None
            
            if self._folder_handle is not None:
                self._folder_handle.remove()
                self._folder_handle = None
            
            self._status_text = None
            self._m6_ccs_config_button = None
            
            self._initialized = False
            logger.info("Hardware panel cleaned up")
            
        except Exception as e:
            logger.warning(f"Error cleaning up Hardware panel: {e}")
