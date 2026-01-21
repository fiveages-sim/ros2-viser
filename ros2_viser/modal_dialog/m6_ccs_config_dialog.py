"""M6 CCS System Configuration Dialog."""

import logging
from typing import Optional, Callable, Dict, Any, List

import viser

from ros2_robot_interface import ROS2RobotInterface

from .base_modal_dialog import BaseModalDialog
from ..i18n import get_translator

logger = logging.getLogger(__name__)


class M6CCSConfigDialog(BaseModalDialog):
    """Modal dialog for M6 CCS System configuration.
    
    This dialog queries all parameters from the M6 CCS System node,
    displays them for editing, and allows setting changed parameters.
    
    Example:
        ```python
        def on_confirm(changed_params):
            print(f"Changed parameters: {changed_params}")
        
        dialog = M6CCSConfigDialog(
            server, 
            ros2_interface, 
            node_name="m6_ccs_system",
            on_confirm=on_confirm
        )
        dialog.show()
        ```
    """
    
    def __init__(
        self,
        server: viser.ViserServer,
        ros2_interface: ROS2RobotInterface,
        full_node_name: str,
        on_confirm: Optional[Callable[[Dict[str, Any]], None]] = None,
        on_cancel: Optional[Callable[[], None]] = None
    ):
        """Initialize M6 CCS configuration dialog.
        
        Args:
            server: Viser server instance.
            ros2_interface: ROS2RobotInterface instance for querying and setting parameters.
            full_node_name: Full node name (with namespace), e.g., "/m6_ccs_system" or "/my_namespace/m6_ccs_system"
            on_confirm: Callback called when user confirms configuration.
                       Receives dict of changed parameters: {param_name: new_value}
            on_cancel: Callback called when user cancels.
        """
        translator = get_translator()
        title = translator("configure_m6_ccs_system")
        
        super().__init__(
            server=server,
            title=title,
            on_confirm=on_confirm,
            on_cancel=on_cancel
        )
        
        self.ros2_interface = ros2_interface
        self.full_node_name = full_node_name
        
        # Allowed configurable parameters (only these will be shown in the dialog)
        self._allowed_params = {
            'ctrl_mode',
            'max_joint_speed',
            'max_joint_acceleration',
            'cart_d_gains',
            'cart_k_gains',
            'joint_d_gains',
            'joint_k_gains',
            'left_dyn_param',
            'left_kine_param',
            'right_dyn_param',
            'right_kine_param'
        }
        
        # Parameters that depend on ctrl_mode
        self._joint_impedance_params = {'joint_d_gains', 'joint_k_gains'}
        self._cart_impedance_params = {'cart_d_gains', 'cart_k_gains'}
        
        # Parameter name translation keys (will be translated using global translator)
        self._param_name_keys = {
            'ctrl_mode': 'param_ctrl_mode',
            'max_joint_speed': 'param_max_joint_speed',
            'max_joint_acceleration': 'param_max_joint_acceleration',
            'cart_d_gains': 'param_cart_d_gains',
            'cart_k_gains': 'param_cart_k_gains',
            'joint_d_gains': 'param_joint_d_gains',
            'joint_k_gains': 'param_joint_k_gains',
            'left_dyn_param': 'param_left_dyn_param',
            'left_kine_param': 'param_left_kine_param',
            'right_dyn_param': 'param_right_dyn_param',
            'right_kine_param': 'param_right_kine_param'
        }
        
        # Control mode option translation keys
        self._ctrl_mode_keys = {
            'position': 'ctrl_mode_position',
            'joint_impedance': 'ctrl_mode_joint_impedance',
            'cart_impedance': 'ctrl_mode_cart_impedance'
        }
        
        # Store original parameter values for change detection
        self._original_params: Dict[str, Any] = {}
        # Store parameter metadata (type, read_only, etc.)
        self._param_metadata: Dict[str, Dict[str, Any]] = {}
        # Store reference to ctrl_mode dropdown for dynamic updates
        self._ctrl_mode_dropdown: Optional[viser.GuiDropdownHandle] = None
        # Store all parameter inputs (including hidden ones) for dynamic visibility control
        self._all_param_inputs: Dict[str, Any] = {}
        # Store mapping for ctrl_mode value conversion (Chinese display -> English value)
        self._ctrl_mode_value_map: Dict[str, str] = {}
    
    def _build_content(self, gui):
        """Build M6 CCS configuration dialog content."""
        try:
            # Query all parameters from the node
            logger.info(f"Querying parameters from node: {self.full_node_name}")
            params = self.ros2_interface.list_node_parameters(self.full_node_name)
            
            if not params:
                gui.add_text(
                    self.translator("no_parameters_found"),
                    initial_value=f"No configurable parameters found for {self.full_node_name}",
                    disabled=True
                )
                logger.warning(f"No parameters found for node {self.full_node_name}")
                return
            
            logger.info(f"Found {len(params)} parameters for {self.full_node_name}")
            
            # Store original values and metadata
            for param in params:
                param_name = param['name']
                self._original_params[param_name] = param['value']
                self._param_metadata[param_name] = {
                    'type': param['type'],
                    'read_only': param.get('read_only', False),
                    'description': param.get('description', '')
                }
            
            # Get current ctrl_mode value to determine which parameters to show
            current_ctrl_mode = None
            for param in params:
                if param['name'] == 'ctrl_mode':
                    current_ctrl_mode = str(param['value']) if param['value'] is not None else 'position'
                    break
            if current_ctrl_mode is None:
                current_ctrl_mode = 'position'
            
            # Store all parameter data for later use
            all_params_data = {}
            for param in params:
                all_params_data[param['name']] = param
            
            # Create input controls for each parameter
            # First, create ctrl_mode dropdown
            ctrl_mode_param = all_params_data.get('ctrl_mode')
            if ctrl_mode_param and not ctrl_mode_param.get('read_only', False):
                param_name = 'ctrl_mode'
                param_value = ctrl_mode_param['value']
                description = ctrl_mode_param.get('description', '')
                # Get translated parameter name using global translator
                param_key = self._param_name_keys.get(param_name, param_name)
                display_name = self.translator(param_key, default=param_name)
                label = display_name
                if description:
                    label = f"{display_name}\n({description[:50]}{'...' if len(description) > 50 else ''})"
                
                # English values for ROS2
                options_en = ['position', 'joint_impedance', 'cart_impedance']
                # Translated display labels using global translator
                options_display = [self.translator(self._ctrl_mode_keys[opt], default=opt) for opt in options_en]
                
                current_value = str(param_value) if param_value is not None else 'position'
                if current_value not in options_en:
                    current_value = options_en[0]
                
                # Find the index of current value for initial selection
                current_index = options_en.index(current_value)
                
                ctrl_mode_dropdown = gui.add_dropdown(
                    label,
                    options=options_display,
                    initial_value=options_display[current_index]
                )
                self._inputs[param_name] = ctrl_mode_dropdown
                self._ctrl_mode_dropdown = ctrl_mode_dropdown
                self._all_param_inputs[param_name] = ctrl_mode_dropdown
                
                # Store mapping for value conversion (display value -> English value)
                self._ctrl_mode_value_map = {display: en for display, en in zip(options_display, options_en)}
                
                # Set up callback for ctrl_mode changes
                def on_ctrl_mode_changed(*args):
                    # Convert translated display value back to English value
                    display_value = ctrl_mode_dropdown.value
                    en_value = self._ctrl_mode_value_map.get(display_value, 'position')
                    self._update_impedance_params_visibility(en_value)
                
                ctrl_mode_dropdown.on_update(on_ctrl_mode_changed)
            
            # Create other parameter controls
            for param in params:
                param_name = param['name']
                param_type = param['type']
                param_value = param['value']
                is_read_only = param.get('read_only', False)
                description = param.get('description', '')
                
                # Skip ctrl_mode (already created) and read-only parameters
                if param_name == 'ctrl_mode' or is_read_only:
                    continue
                
                # Only allow configuration of specific parameters
                if param_name not in self._allowed_params:
                    continue
                
                # Determine visibility based on current ctrl_mode
                should_show = True
                if param_name in self._joint_impedance_params:
                    should_show = (current_ctrl_mode == 'joint_impedance')
                elif param_name in self._cart_impedance_params:
                    should_show = (current_ctrl_mode == 'cart_impedance')
                
                # Create label with Chinese translation if available
                # Get translated parameter name using global translator
                param_key = self._param_name_keys.get(param_name, param_name)
                display_name = self.translator(param_key, default=param_name)
                label = display_name
                if description:
                    label = f"{display_name}\n({description[:50]}{'...' if len(description) > 50 else ''})"
                
                # Create appropriate input control based on parameter type
                input_handle = None
                if param_type == 'bool':
                    input_handle = gui.add_checkbox(
                        label,
                        initial_value=bool(param_value) if param_value is not None else False
                    )
                elif param_type == 'int':
                    # For integers, use a number input (slider or text)
                    # Using text input for now as slider requires min/max
                    input_handle = gui.add_text(
                        label,
                        initial_value=str(param_value) if param_value is not None else "0"
                    )
                elif param_type == 'double':
                    input_handle = gui.add_text(
                        label,
                        initial_value=str(param_value) if param_value is not None else "0.0"
                    )
                elif param_type == 'string':
                    input_handle = gui.add_text(
                        label,
                        initial_value=str(param_value) if param_value is not None else ""
                    )
                elif param_type in ('bool_array', 'int_array', 'double_array', 'string_array'):
                    # For arrays, use text input with comma-separated values
                    if param_value is not None and isinstance(param_value, list):
                        array_str = ', '.join(str(v) for v in param_value)
                    else:
                        array_str = ""
                    input_handle = gui.add_text(
                        label,
                        initial_value=array_str
                    )
                else:
                    # Unknown type, use text input
                    input_handle = gui.add_text(
                        label,
                        initial_value=str(param_value) if param_value is not None else ""
                    )
                
                # Store the input handle
                if input_handle is not None:
                    self._inputs[param_name] = input_handle
                    self._all_param_inputs[param_name] = input_handle
                    
                    # Set initial visibility for impedance parameters
                    if param_name in self._joint_impedance_params or param_name in self._cart_impedance_params:
                        if hasattr(input_handle, 'visible'):
                            input_handle.visible = should_show
                        elif hasattr(input_handle, 'disabled'):
                            # If no visible attribute, use disabled as a workaround
                            input_handle.disabled = not should_show
            
            if not self._inputs:
                gui.add_text(
                    self.translator("no_writable_parameters"),
                    initial_value="No writable parameters found (all parameters are read-only)",
                    disabled=True
                )
                
        except Exception as e:
            logger.error(f"Failed to build dialog content: {e}", exc_info=True)
            gui.add_text(
                "Error",
                initial_value=f"Failed to load parameters: {e}",
                disabled=True
            )
    
    def _update_impedance_params_visibility(self, ctrl_mode: str):
        """Update visibility of impedance parameters based on ctrl_mode.
        
        Args:
            ctrl_mode: Current control mode value ('position', 'joint_impedance', or 'cart_impedance')
        """
        try:
            # Update joint impedance parameters
            for param_name in self._joint_impedance_params:
                if param_name in self._all_param_inputs:
                    input_handle = self._all_param_inputs[param_name]
                    should_show = (ctrl_mode == 'joint_impedance')
                    if hasattr(input_handle, 'visible'):
                        input_handle.visible = should_show
                    elif hasattr(input_handle, 'disabled'):
                        input_handle.disabled = not should_show
            
            # Update cart impedance parameters
            for param_name in self._cart_impedance_params:
                if param_name in self._all_param_inputs:
                    input_handle = self._all_param_inputs[param_name]
                    should_show = (ctrl_mode == 'cart_impedance')
                    if hasattr(input_handle, 'visible'):
                        input_handle.visible = should_show
                    elif hasattr(input_handle, 'disabled'):
                        input_handle.disabled = not should_show
            
            logger.debug(f"Updated impedance parameters visibility for ctrl_mode: {ctrl_mode}")
        except Exception as e:
            logger.warning(f"Failed to update impedance parameters visibility: {e}", exc_info=True)
    
    def _get_input_values(self) -> Dict[str, Any]:
        """Get values from all dialog inputs and convert to appropriate types."""
        values = {}
        for name, handle in self._inputs.items():
            try:
                if hasattr(handle, 'value'):
                    raw_value = handle.value
                elif hasattr(handle, 'checked'):
                    raw_value = handle.checked
                elif hasattr(handle, 'selected'):
                    # For dropdown
                    raw_value = handle.selected
                else:
                    raw_value = None
                
                # Convert to appropriate type based on parameter metadata
                if name in self._param_metadata:
                    param_type = self._param_metadata[name]['type']
                    
                    if name == 'ctrl_mode':
                        # Convert translated display value back to English value for ctrl_mode
                        if hasattr(handle, 'selected'):
                            display_value = handle.selected
                        elif hasattr(handle, 'value'):
                            display_value = handle.value
                        else:
                            display_value = raw_value
                        # Convert to English value
                        values[name] = self._ctrl_mode_value_map.get(display_value, str(display_value))
                    elif param_type == 'bool':
                        if isinstance(raw_value, bool):
                            values[name] = raw_value
                        else:
                            values[name] = str(raw_value).lower() in ('true', '1', 'yes', 'on')
                    elif param_type == 'int':
                        try:
                            values[name] = int(float(str(raw_value)))
                        except (ValueError, TypeError):
                            logger.warning(f"Invalid integer value for {name}: {raw_value}")
                            values[name] = self._original_params.get(name, 0)
                    elif param_type == 'double':
                        try:
                            values[name] = float(str(raw_value))
                        except (ValueError, TypeError):
                            logger.warning(f"Invalid double value for {name}: {raw_value}")
                            values[name] = self._original_params.get(name, 0.0)
                    elif param_type == 'string':
                        values[name] = str(raw_value) if raw_value is not None else ""
                    elif param_type in ('bool_array', 'int_array', 'double_array', 'string_array'):
                        # Parse comma-separated array string
                        if raw_value:
                            array_str = str(raw_value).strip()
                            if array_str:
                                array_values = [v.strip() for v in array_str.split(',')]
                                
                                if param_type == 'bool_array':
                                    values[name] = [v.lower() in ('true', '1', 'yes', 'on') for v in array_values]
                                elif param_type == 'int_array':
                                    try:
                                        values[name] = [int(float(v)) for v in array_values]
                                    except (ValueError, TypeError):
                                        logger.warning(f"Invalid integer array for {name}: {raw_value}")
                                        values[name] = self._original_params.get(name, [])
                                elif param_type == 'double_array':
                                    try:
                                        values[name] = [float(v) for v in array_values]
                                    except (ValueError, TypeError):
                                        logger.warning(f"Invalid double array for {name}: {raw_value}")
                                        values[name] = self._original_params.get(name, [])
                                else:  # string_array
                                    values[name] = array_values
                            else:
                                values[name] = []
                        else:
                            values[name] = []
                    else:
                        values[name] = raw_value
                else:
                    values[name] = raw_value
                    
            except Exception as e:
                logger.warning(f"Failed to get value for input '{name}': {e}")
                values[name] = self._original_params.get(name)
        
        return values
    
    def _handle_confirm(self):
        """Handle confirm button click - detect changes and set parameters."""
        try:
            # Get current values
            current_values = self._get_input_values()
            
            # Detect changed parameters
            changed_params = {}
            for param_name, new_value in current_values.items():
                original_value = self._original_params.get(param_name)
                
                # Compare values (handle different types)
                if original_value != new_value:
                    changed_params[param_name] = new_value
                    logger.debug(f"Parameter {param_name} changed: {original_value} -> {new_value}")
            
            if not changed_params:
                logger.info("No parameters changed")
                if self.on_confirm is not None:
                    self.on_confirm({})
                self.close()
                return
            
            # Set changed parameters via ROS2 interface
            logger.info(f"Setting {len(changed_params)} changed parameters")
            success = self.ros2_interface.set_node_parameters(
                self.full_node_name,
                changed_params
            )
            
            if success:
                logger.info(f"Successfully set {len(changed_params)} parameters")
            else:
                logger.warning(f"Failed to set some parameters")
            
            # Call callback with changed parameters
            if self.on_confirm is not None:
                self.on_confirm(changed_params)
            
            self.close()
            
        except Exception as e:
            logger.error(f"Error handling dialog confirm: {e}", exc_info=True)
