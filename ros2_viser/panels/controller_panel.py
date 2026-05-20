"""Controller Control Panel for ROS2 Viser."""

import logging
from typing import Optional, Dict, List

import viser

from ros2_robot_interface import ROS2RobotInterface

from ..i18n import get_translator
from ..modal_dialog.controller_config_dialog import ControllerConfigDialog

logger = logging.getLogger(__name__)


def _list_controllers(ros2_interface: ROS2RobotInterface) -> List[Dict[str, str]]:
    """List all controllers by finding controller_manager node and querying controllers.
    
    This function attempts to find controller_manager node and list controllers.
    If controller_manager is not found, it falls back to finding nodes with 'controller' in their name.
    
    Args:
        ros2_interface: ROS2RobotInterface instance.
        
    Returns:
        List of controller info dicts, each containing:
            - 'name': Controller name (display name)
            - 'full_name': Full node name (with namespace)
            - 'state': Controller state (if available)
    """
    controllers = []
    
    try:
        # Get all nodes
        nodes = ros2_interface.list_nodes()
        
        # Try to find controller_manager node first
        controller_manager_node = None
        for node in nodes:
            if 'controller_manager' in node['name'].lower():
                controller_manager_node = node
                break
        
        # If controller_manager found, try to use its service to list controllers
        if controller_manager_node is not None:
            try:
                # Try to use controller_manager service
                # Note: This requires controller_manager_msgs package
                # For now, we'll use a fallback approach
                logger.debug(f"Found controller_manager node: {controller_manager_node['full_name']}")
            except Exception as e:
                logger.debug(f"Could not use controller_manager service: {e}")
        
        # Fallback: Find all nodes with 'controller' in their name
        # Filter out controller_manager itself
        for node in nodes:
            node_name_lower = node['name'].lower()
            # Skip controller_manager node itself
            if 'controller_manager' in node_name_lower:
                continue
            
            # Look for nodes with 'controller' in name
            if 'controller' in node_name_lower:
                controllers.append({
                    'name': node['name'],
                    'full_name': node['full_name'],
                    'state': 'unknown'  # State not available without controller_manager service
                })
        
        logger.debug(f"Found {len(controllers)} controllers")
        return controllers
        
    except Exception as e:
        logger.warning(f"Failed to list controllers: {e}")
        return []


class ControllerPanel:
    """Controller control panel for managing ROS2 controllers.
    
    This panel provides a GUI interface for listing controllers and configuring their parameters.
    
    Example:
        ```python
        panel = ControllerPanel(server, ros2_interface)
        panel.initialize()
        ```
    """
    
    def __init__(
        self,
        server: viser.ViserServer,
        ros2_interface: ROS2RobotInterface
    ):
        """Initialize Controller panel.
        
        Args:
            server: Viser server instance for GUI creation.
            ros2_interface: ROS2RobotInterface for controller operations.
        """
        self.server = server
        self.ros2_interface = ros2_interface
        self.translator = get_translator()
        
        # GUI elements
        self._folder_handle: Optional[viser.GuiFolderHandle] = None
        self._status_text: Optional[viser.GuiTextHandle] = None
        self._controller_buttons: Dict[str, viser.GuiButtonHandle] = {}
        
        # Modal dialog instances
        self._dialogs: Dict[str, ControllerConfigDialog] = {}
        
        # Initialization flag
        self._initialized = False
        
        # Cleanup flag to prevent callbacks from running after cleanup
        self._cleaned_up = False
    
    def initialize(self):
        """Initialize the Controller panel GUI."""
        # If already initialized, cleanup first to avoid duplicates
        if self._initialized:
            logger.warning("Controller panel already initialized, cleaning up first...")
            self.cleanup()
        
        try:
            # Reset cleanup flag
            self._cleaned_up = False
            
            # Initialize GUI
            self._init_gui()
            
            self._initialized = True
            logger.debug("✅ Controller control panel initialized")
            
        except Exception as e:
            logger.error(f"Failed to initialize Controller panel: {e}", exc_info=True)
            raise
    
    def _init_gui(self):
        """Initialize Controller control panel GUI elements."""
        if self.server is None:
            return
        
        try:
            # Create folder and save reference for cleanup
            self._folder_handle = self.server.gui.add_folder(self.translator("controller_control"))
            
            with self._folder_handle:
                # Status display
                controllers = _list_controllers(self.ros2_interface)
                if controllers:
                    status_text = f"{self.translator('controllers_detected')}: {len(controllers)}"
                else:
                    status_text = self.translator("no_controllers_found")
                
                self._status_text = self.server.gui.add_text(
                    self.translator("status"),
                    initial_value=status_text,
                    disabled=True
                )
                
                # Create buttons for each controller
                if controllers:
                    for controller in controllers:
                        controller_name = controller['name']
                        button_label = f"{self.translator('configure')} {controller_name}"
                        
                        button = self.server.gui.add_button(
                            button_label,
                            color="blue"
                        )
                        button.on_click(
                            lambda _, ctrl=controller: self._on_controller_config_clicked(ctrl)
                        )
                        self._controller_buttons[controller_name] = button
                        logger.debug(f"Added configuration button for controller: {controller_name}")
                else:
                    # Show refresh button if no controllers found
                    refresh_button = self.server.gui.add_button(
                        self.translator("refresh_controllers"),
                        color="gray"
                    )
                    refresh_button.on_click(lambda _: self._refresh_controllers())
                    self._controller_buttons['refresh'] = refresh_button
                
        except Exception as e:
            logger.error(f"Failed to initialize Controller panel GUI: {e}", exc_info=True)
            raise
    
    def _refresh_controllers(self):
        """Refresh the controller list."""
        if self._cleaned_up:
            return
        
        try:
            logger.info("Refreshing controller list...")
            
            # Close all open dialogs
            for dialog in list(self._dialogs.values()):
                if dialog.is_open:
                    dialog.close()
            self._dialogs.clear()
            
            # Reinitialize GUI to refresh controller list
            self._init_gui()
            
        except Exception as e:
            logger.error(f"Error refreshing controllers: {e}", exc_info=True)
    
    def _on_controller_config_clicked(self, controller: Dict[str, str]):
        """Handle controller configuration button click - opens configuration dialog.
        
        Args:
            controller: Controller info dict with 'name' and 'full_name' keys.
        """
        if self._cleaned_up:
            return
        
        try:
            controller_name = controller['name']
            controller_node_name = controller['full_name']
            
            # Close existing dialog if any
            if controller_name in self._dialogs and self._dialogs[controller_name].is_open:
                self._dialogs[controller_name].close()
            
            # Create and show modal dialog
            dialog = ControllerConfigDialog(
                server=self.server,
                ros2_interface=self.ros2_interface,
                controller_name=controller_name,
                controller_node_name=controller_node_name,
                on_confirm=lambda changed_params: self._on_dialog_confirm(controller_name, changed_params),
                on_cancel=self._on_dialog_cancel
            )
            dialog.show()
            self._dialogs[controller_name] = dialog
            
        except Exception as e:
            logger.error(f"Error handling controller configuration: {e}", exc_info=True)
    
    def _on_dialog_confirm(self, controller_name: str, changed_params: dict):
        """Handle dialog confirm button click.
        
        Args:
            controller_name: Name of the controller.
            changed_params: Dictionary containing changed parameters: {param_name: new_value}
        """
        if self._cleaned_up:
            return
        
        try:
            if changed_params:
                # Update status text
                if self._status_text is not None:
                    param_names = ', '.join(list(changed_params.keys())[:3])
                    if len(changed_params) > 3:
                        param_names += f" (+{len(changed_params) - 3} more)"
                    self._status_text.value = f"{controller_name}: {param_names} {self.translator('configuration_saved')}"
            else:
                logger.info("No parameters were changed")
                if self._status_text is not None:
                    self._status_text.value = f"{controller_name}: {self.translator('no_changes')}"
            
        except Exception as e:
            logger.error(f"Error confirming configuration: {e}", exc_info=True)
    
    def _on_dialog_cancel(self):
        """Handle dialog cancel button click."""
        if self._cleaned_up:
            return
        
        try:
            logger.info("Controller configuration cancelled")
            
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
            # Get current controllers
            controllers = _list_controllers(self.ros2_interface)
            
            # Remove old folder
            old_folder = self._folder_handle
            
            # Create new folder with new language
            self._folder_handle = self.server.gui.add_folder(self.translator("controller_control"))
            
            # Recreate GUI elements in new folder
            with self._folder_handle:
                # Status display
                if controllers:
                    status_text = f"{self.translator('controllers_detected')}: {len(controllers)}"
                else:
                    status_text = self.translator("no_controllers_found")
                
                # Always refresh status from live controller list (retranslates on language change)
                status_value = status_text
                
                self._status_text = self.server.gui.add_text(
                    self.translator("status"),
                    initial_value=status_value,
                    disabled=True
                )
                
                # Recreate buttons for each controller
                self._controller_buttons.clear()
                if controllers:
                    for controller in controllers:
                        controller_name = controller['name']
                        button_label = f"{self.translator('configure')} {controller_name}"
                        
                        button = self.server.gui.add_button(
                            button_label,
                            color="blue"
                        )
                        button.on_click(
                            lambda _, ctrl=controller: self._on_controller_config_clicked(ctrl)
                        )
                        self._controller_buttons[controller_name] = button
                else:
                    # Show refresh button if no controllers found
                    refresh_button = self.server.gui.add_button(
                        self.translator("refresh_controllers"),
                        color="gray"
                    )
                    refresh_button.on_click(lambda _: self._refresh_controllers())
                    self._controller_buttons['refresh'] = refresh_button
            
            # Remove old folder
            if old_folder is not None:
                old_folder.remove()
            
            logger.debug("Controller panel GUI labels updated")
            
        except Exception as e:
            logger.warning(f"Failed to update Controller panel GUI labels: {e}", exc_info=True)
    
    def cleanup(self):
        """Cleanup the Controller panel and remove GUI elements."""
        if self._cleaned_up:
            return
        
        self._cleaned_up = True
        
        try:
            # Close all dialogs if open
            for dialog in list(self._dialogs.values()):
                if dialog.is_open:
                    dialog.close()
            self._dialogs.clear()
            
            if self._folder_handle is not None:
                self._folder_handle.remove()
                self._folder_handle = None
            
            self._status_text = None
            self._controller_buttons.clear()
            
            self._initialized = False
            logger.info("Controller panel cleaned up")
            
        except Exception as e:
            logger.warning(f"Error cleaning up Controller panel: {e}")
