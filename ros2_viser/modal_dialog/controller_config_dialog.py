"""Controller Configuration Dialog."""

import logging
from typing import Optional, Callable, Dict, Any, List

import viser

from ros2_robot_interface import ROS2RobotInterface

from .base_modal_dialog import BaseModalDialog
from ..i18n import get_translator

logger = logging.getLogger(__name__)


class ControllerConfigDialog(BaseModalDialog):
    """Modal dialog for controller configuration.
    
    This dialog queries all parameters from a controller node,
    displays them for editing, and allows setting changed parameters.
    
    Example:
        ```python
        def on_confirm(changed_params):
            print(f"Changed parameters: {changed_params}")
        
        dialog = ControllerConfigDialog(
            server, 
            ros2_interface, 
            controller_name="left_arm_controller",
            controller_node_name="/left_arm_controller",
            on_confirm=on_confirm
        )
        dialog.show()
        ```
    """
    
    def __init__(
        self,
        server: viser.ViserServer,
        ros2_interface: ROS2RobotInterface,
        controller_name: str,
        controller_node_name: str,
        on_confirm: Optional[Callable[[Dict[str, Any]], None]] = None,
        on_cancel: Optional[Callable[[], None]] = None
    ):
        """Initialize controller configuration dialog.
        
        Args:
            server: Viser server instance.
            ros2_interface: ROS2RobotInterface instance for querying and setting parameters.
            controller_name: Display name of the controller (e.g., "left_arm_controller").
            controller_node_name: Full node name of the controller (with namespace), 
                                 e.g., "/left_arm_controller" or "/my_namespace/left_arm_controller"
            on_confirm: Callback called when user confirms configuration.
                       Receives dict of changed parameters: {param_name: new_value}
            on_cancel: Callback called when user cancels.
        """
        translator = get_translator()
        title = f"{translator('configure_controller')}: {controller_name}"
        
        super().__init__(
            server=server,
            title=title,
            on_confirm=on_confirm,
            on_cancel=on_cancel
        )
        
        self.ros2_interface = ros2_interface
        self.controller_name = controller_name
        self.controller_node_name = controller_node_name
        
        # Allowed configurable parameters (only these will be shown in the dialog)
        self._allowed_params = {
            'home_duration',
            'home_interpolation_type',
            'home_tanh_scale',
            'hold_position_threshold',
            'movej_duration',
            'movej_interpolation_type',
            'movej_tanh_scale',
            'movej_trajectory_duration',
            'movej_trajectory_blend_ratio',
            'movel_duration',
            'movel_trajectory_duration',
            'waist_lifting_duration'
        }
        
        # Store original parameter values for change detection
        self._original_params: Dict[str, Any] = {}
        # Store parameter metadata (type, read_only, etc.)
        self._param_metadata: Dict[str, Dict[str, Any]] = {}
        # Store mapping for interpolation_type value conversion (display value -> English value)
        self._interpolation_value_map: Dict[str, str] = {}
        # Store all parameter inputs (including hidden ones) for dynamic visibility control
        self._all_param_inputs: Dict[str, Any] = {}
        # Store interpolation_type dropdowns and their corresponding tanh_scale inputs
        self._interpolation_dropdowns: Dict[str, Any] = {}  # {prefix: dropdown_handle}
        self._tanh_scale_inputs: Dict[str, Any] = {}  # {param_name: input_handle}
    
    def _build_content(self, gui):
        """Build controller configuration dialog content."""
        try:
            # Query all parameters from the controller node
            params = self.ros2_interface.list_node_parameters(self.controller_node_name)
            
            if not params:
                gui.add_text(
                    self.translator("no_parameters_found"),
                    initial_value=f"{self.translator('no_parameters_found')} for {self.controller_node_name}",
                    disabled=True
                )
                logger.warning(f"No parameters found for controller node {self.controller_node_name}")
                return
            
            # Store original values and metadata
            for param in params:
                param_name = param['name']
                self._original_params[param_name] = param['value']
                self._param_metadata[param_name] = {
                    'type': param['type'],
                    'read_only': param.get('read_only', False),
                    'description': param.get('description', '')
                }
            
            # Group parameters by prefix (part before first underscore)
            param_groups: Dict[str, List[Dict[str, Any]]] = {}
            for param in params:
                param_name = param['name']
                param_type = param['type']
                param_value = param['value']
                is_read_only = param.get('read_only', False)
                description = param.get('description', '')
                
                # Skip read-only parameters
                if is_read_only:
                    continue
                
                # Only allow configuration of specific parameters
                if param_name not in self._allowed_params:
                    continue
                
                # Extract prefix (part before first underscore)
                if '_' in param_name:
                    prefix = param_name.split('_')[0]
                else:
                    # If no underscore, use the whole name as prefix
                    prefix = param_name
                
                if prefix not in param_groups:
                    param_groups[prefix] = []
                
                param_groups[prefix].append({
                    'name': param_name,
                    'type': param_type,
                    'value': param_value,
                    'description': description
                })
            
            # Create folders for each prefix group and add parameters
            for prefix, group_params in sorted(param_groups.items()):
                # Create folder for this prefix
                folder = gui.add_folder(prefix)
                with folder:
                    # First pass: create interpolation_type dropdowns and find current interpolation type
                    current_interpolation_type = None
                    for param in group_params:
                        param_name = param['name']
                        if 'interpolation_type' in param_name:
                            param_value = param['value']
                            current_interpolation_type = str(param_value) if param_value is not None else 'tanh'
                            break
                    
                    # Second pass: create all parameters
                    for param in group_params:
                        param_name = param['name']
                        param_type = param['type']
                        param_value = param['value']
                        description = param['description']
                        
                        # Create label (use the part after prefix for shorter display)
                        # Remove prefix and first underscore from label
                        if '_' in param_name:
                            label_key = param_name.split('_', 1)[1]  # Everything after first underscore
                        else:
                            label_key = param_name
                        
                        # Translate parameter name
                        param_translation_key = f"param_{label_key}"
                        translated_label = self.translator(param_translation_key, default=label_key)
                        
                        if description:
                            translated_label = f"{translated_label}\n({description[:50]}{'...' if len(description) > 50 else ''})"
                        
                        # Create appropriate input control based on parameter type
                        input_handle = None
                        
                        # Special handling for interpolation_type parameters
                        if 'interpolation_type' in param_name:
                            # Use dropdown for interpolation_type with translated options
                            options_en = ['tanh', 'linear', 'doubles', 'none']
                            options_display = [
                                self.translator('interpolation_tanh', default='tanh'),
                                self.translator('interpolation_linear', default='linear'),
                                self.translator('interpolation_doubles', default='doubles'),
                                self.translator('interpolation_none', default='none')
                            ]
                            current_value = str(param_value) if param_value is not None else options_en[0]
                            # Ensure current value is in options, default to first option if not
                            if current_value not in options_en:
                                current_value = options_en[0]
                            # Find the index for display
                            current_index = options_en.index(current_value)
                            input_handle = gui.add_dropdown(
                                translated_label,
                                options=options_display,
                                initial_value=options_display[current_index]
                            )
                            # Store mapping for value conversion (display -> English value)
                            self._interpolation_value_map = {display: en for display, en in zip(options_display, options_en)}
                            # Store dropdown reference for this prefix
                            self._interpolation_dropdowns[prefix] = input_handle
                            
                            # Set up callback for interpolation_type changes
                            # Use lambda with captured variables to avoid closure issues
                            def make_callback(prefix_key, dropdown_handle):
                                def on_interpolation_changed(*args):
                                    try:
                                        # Get the selected value from dropdown
                                        # For viser dropdown, use .value property
                                        display_value = dropdown_handle.value
                                        
                                        # Convert translated display value back to English value
                                        en_value = self._interpolation_value_map.get(str(display_value), 'tanh')
                                        logger.debug(f"Interpolation type changed for {prefix_key}: {display_value} -> {en_value}")
                                        self._update_tanh_scale_visibility(prefix_key, en_value)
                                    except Exception as e:
                                        logger.error(f"Error in interpolation changed callback for {prefix_key}: {e}", exc_info=True)
                                return on_interpolation_changed
                            
                            input_handle.on_update(make_callback(prefix, input_handle))
                        elif 'tanh_scale' in param_name:
                            # tanh_scale should only be visible when interpolation_type is 'tanh'
                            should_show = (current_interpolation_type == 'tanh')
                            input_handle = gui.add_text(
                                    translated_label,
                                    initial_value=str(param_value) if param_value is not None else "0.0"
                                )
                            # Set initial visibility
                            if hasattr(input_handle, 'visible'):
                                input_handle.visible = should_show
                            elif hasattr(input_handle, 'disabled'):
                                input_handle.disabled = not should_show
                            # Store reference for dynamic visibility control
                            self._tanh_scale_inputs[param_name] = input_handle
                        elif param_type == 'bool':
                            input_handle = gui.add_checkbox(
                                translated_label,
                                initial_value=bool(param_value) if param_value is not None else False
                            )
                        elif param_type == 'int':
                            # For integers, use text input
                            input_handle = gui.add_text(
                                translated_label,
                                initial_value=str(param_value) if param_value is not None else "0"
                            )
                        elif param_type == 'double':
                            input_handle = gui.add_text(
                                translated_label,
                                initial_value=str(param_value) if param_value is not None else "0.0"
                            )
                        elif param_type == 'string':
                            input_handle = gui.add_text(
                                translated_label,
                                initial_value=str(param_value) if param_value is not None else ""
                            )
                        elif param_type in ('bool_array', 'int_array', 'double_array', 'string_array'):
                            # For arrays, use text input with comma-separated values
                            if param_value is not None and isinstance(param_value, list):
                                array_str = ', '.join(str(v) for v in param_value)
                            else:
                                array_str = ""
                            input_handle = gui.add_text(
                                translated_label,
                                initial_value=array_str
                            )
                        else:
                            # Unknown type, use text input
                            input_handle = gui.add_text(
                                translated_label,
                                initial_value=str(param_value) if param_value is not None else ""
                            )
                        
                        # Store the input handle
                        if input_handle is not None:
                            self._inputs[param_name] = input_handle
                            self._all_param_inputs[param_name] = input_handle
            
            if not self._inputs:
                gui.add_text(
                    self.translator("no_writable_parameters"),
                    initial_value=f"{self.translator('no_writable_parameters')} (all parameters are read-only)",
                    disabled=True
                )
                
        except Exception as e:
            logger.error(f"Failed to build dialog content: {e}", exc_info=True)
            gui.add_text(
                "Error",
                initial_value=f"Failed to load parameters: {e}",
                disabled=True
            )
    
    def _update_tanh_scale_visibility(self, prefix: str, interpolation_type: str):
        """Update visibility of tanh_scale parameters based on interpolation_type.
        
        Args:
            prefix: Parameter prefix (e.g., 'home', 'movej')
            interpolation_type: Current interpolation type value ('tanh', 'linear', 'doubles', 'none')
        """
        try:
            should_show = (interpolation_type == 'tanh')
            logger.debug(f"Updating tanh_scale visibility for prefix '{prefix}': interpolation_type={interpolation_type}, should_show={should_show}")
            
            # Update all tanh_scale inputs for this prefix
            updated_count = 0
            for param_name, input_handle in self._tanh_scale_inputs.items():
                if param_name.startswith(f"{prefix}_"):
                    logger.debug(f"Updating visibility for {param_name}: should_show={should_show}")
                    if hasattr(input_handle, 'visible'):
                        input_handle.visible = should_show
                        updated_count += 1
                        logger.debug(f"Set {param_name}.visible = {should_show}")
                    elif hasattr(input_handle, 'disabled'):
                        input_handle.disabled = not should_show
                        updated_count += 1
                        logger.debug(f"Set {param_name}.disabled = {not should_show}")
                    else:
                        logger.warning(f"Input handle for {param_name} has no visible or disabled attribute")
            
        except Exception as e:
            logger.error(f"Failed to update tanh_scale visibility: {e}", exc_info=True)
    
    def _get_input_values(self) -> Dict[str, Any]:
        """Get values from all dialog inputs and convert to appropriate types."""
        values = {}
        for name, handle in self._inputs.items():
            try:
                # Handle different input types
                if hasattr(handle, 'value'):
                    raw_value = handle.value
                elif hasattr(handle, 'checked'):
                    raw_value = handle.checked
                elif hasattr(handle, 'selected'):
                    # For dropdown
                    raw_value = handle.selected
                else:
                    raw_value = None
                
                # Special handling for interpolation_type (dropdown)
                if 'interpolation_type' in name:
                    # Convert translated display value back to English value
                    if hasattr(handle, 'selected'):
                        display_value = handle.selected
                    elif hasattr(handle, 'value'):
                        display_value = handle.value
                    else:
                        display_value = raw_value
                    # Convert to English value using mapping
                    values[name] = self._interpolation_value_map.get(str(display_value), str(display_value) if display_value is not None else "linear")
                    continue
                
                # Convert to appropriate type based on parameter metadata
                if name in self._param_metadata:
                    param_type = self._param_metadata[name]['type']
                    
                    if param_type == 'bool':
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
            success = self.ros2_interface.set_node_parameters(
                self.controller_node_name,
                changed_params
            )
            
            if success:
                pass
            else:
                logger.warning(f"Failed to set some parameters")
            
            # Call callback with changed parameters
            if self.on_confirm is not None:
                self.on_confirm(changed_params)
            
            self.close()
            
        except Exception as e:
            logger.error(f"Error handling dialog confirm: {e}", exc_info=True)
