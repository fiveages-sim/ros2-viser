"""ROS2 Viser Visualizer - Visualize ROS2 robots using Viser."""

import hashlib
import logging
import threading
import time
import warnings
from io import StringIO
from typing import Optional, Any

import numpy as np
import viser
from viser.extras import ViserUrdf
import yourdfpy

from ros2_robot_interface import ROS2RobotInterface, ROS2RobotInterfaceConfig

from .config import ROS2ViserConfig
from .panels import FSMPanel, GripperPanel, JointPanel, HardwarePanel
from .i18n import Translator, get_translator, set_global_language
from .end_effector_marker import EndEffectorMarkerManager

logger = logging.getLogger(__name__)


class ROS2ViserVisualizer:
    """Visualize ROS2 robot using Viser.
    
    This class subscribes to ROS2 topics to get robot description (URDF) and
    joint states, then visualizes the robot model in Viser.
    
    Example:
        ```python
        from ros2_viser import ROS2ViserVisualizer, ROS2ViserConfig
        
        config = ROS2ViserConfig(
            robot_description_topic="/robot_description",
            joint_states_topic="/joint_states"
        )
        
        visualizer = ROS2ViserVisualizer(config)
        visualizer.start()
        
        # Keep running
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            visualizer.stop()
        ```
    """
    
    def __init__(self, config: ROS2ViserConfig):
        """Initialize the ROS2 Viser visualizer.
        
        Args:
            config: Configuration for the visualizer.
        """
        self.config = config
        
        # Initialize translator with config language
        set_global_language(config.language)
        self.translator = get_translator()
        self.server: Optional[viser.ViserServer] = None
        self.urdf_vis: Optional[ViserUrdf] = None
        self.urdf: Optional[yourdfpy.URDF] = None
        
        self.ros2_interface: Optional[ROS2RobotInterface] = None
        
        # State
        self._running = False
        self._urdf_received = False
        self._update_thread: Optional[threading.Thread] = None
        
        
        # URDF reloading
        self._last_urdf_hash: Optional[str] = None  # Hash of last URDF to detect changes
        
        # Joint name mapping (from joint_states to URDF joint order)
        self.joint_name_to_index: dict[str, int] = {}
        self.urdf_joint_names: list[str] = []
        
        # Precomputed mapping for performance optimization
        self._joint_name_to_urdf_index: dict[str, int] = {}  # Maps joint name to URDF index (computed once)
        
        # Cache last joint positions to avoid unnecessary updates
        self._last_joint_positions: Optional[np.ndarray] = None
        
        # GUI panels
        self._fsm_panel: Optional[FSMPanel] = None
        self._gripper_panel: Optional[GripperPanel] = None
        self._joint_panel: Optional[JointPanel] = None
        self._hardware_panel: Optional[HardwarePanel] = None
        
        # Display control panel
        self._display_folder_handle: Optional[viser.GuiFolderHandle] = None
        self._show_visual_checkbox: Optional[viser.GuiCheckboxHandle] = None
        self._show_collision_checkbox: Optional[viser.GuiCheckboxHandle] = None
        self._language_dropdown: Optional[viser.GuiDropdownHandle] = None
        self._marker_publish_mode_dropdown: Optional[viser.GuiDropdownHandle] = None
        self._send_marker_pose_button: Optional[viser.GuiButtonHandle] = None
        self._show_visual: bool = True
        self._show_collision: bool = False
        
        # End-effector marker manager
        self._marker_manager: Optional[EndEffectorMarkerManager] = None
        
        # Note: No direct subscription - use ros2_interface.get_robot_description() instead
    
    def _parse_urdf_string(self, urdf_string: str, urdf_hash: Optional[str] = None):
        """Parse URDF string and initialize/reload visualization.
        
        Args:
            urdf_string: URDF XML string
            urdf_hash: Optional hash of the URDF string for change detection
            """
        if not urdf_string or not urdf_string.strip():
            logger.warning("URDF string is empty, skipping parse")
            return
        
        try:
            # If URDF already loaded, clean up old visualization and reload interfaces
            if self._urdf_received and self.urdf_vis is not None:
                logger.info("URDF changed, reloading visualization and ROS2 interfaces...")
                
                needs_reconnect = self.ros2_interface is not None
                if self._marker_manager is not None:
                    try:
                        self._marker_manager.cleanup()
                    except Exception as e:
                        logger.warning(f"Error cleaning up marker manager: {e}")
                    self._marker_manager = None
                
                # Clean up old visualization
                try:
                    self.urdf_vis.remove()
                except Exception as e:
                    logger.warning(f"Error removing old URDF visualization: {e}")
                self.urdf_vis = None
                
                if needs_reconnect:
                    logger.info("URDF changed - ROS2 interface reconnection will be handled in update loop")
            
            logger.debug(f"Parsing URDF (length: {len(urdf_string)} characters)")
            urdf_io = StringIO(urdf_string)
            try:
                # Try to load with explicit parameters to enable visual and collision scenes
                # build_scene_graph=True enables visual scene
                # load_collision_meshes=True and build_collision_scene_graph=True enable collision scene
                self.urdf = yourdfpy.URDF.load(
                    urdf_io,
                    build_scene_graph=True,
                    load_collision_meshes=True,
                    build_collision_scene_graph=True
                )
                logger.debug("URDF loaded with build_scene_graph=True, load_collision_meshes=True, build_collision_scene_graph=True")
            except (AttributeError, TypeError) as e:
                # Fallback: try with fewer parameters (older versions)
                if "build_scene_graph" in str(e) or "unexpected keyword" in str(e) or "load_collision_meshes" in str(e):
                    urdf_io.seek(0)  # Reset StringIO
                    try:
                        # Try with just build_scene_graph
                        self.urdf = yourdfpy.URDF.load(urdf_io, build_scene_graph=True)
                        logger.debug("URDF loaded with build_scene_graph=True (collision meshes may not be loaded)")
                    except (AttributeError, TypeError):
                        # Final fallback: load normally
                        urdf_io.seek(0)
                        self.urdf = yourdfpy.URDF.load(urdf_io)
                        logger.debug("URDF loaded with default settings (visual scene should be enabled, collision may not be loaded)")
                else:
                    raise
            
            # Set flag BEFORE logging to ensure it's set immediately
            self._urdf_received = True
            if urdf_hash:
                self._last_urdf_hash = urdf_hash
            
            logger.info("✅ URDF parsed successfully")
            
            # Extract ALL joint names from URDF (in order)
            # ViserUrdf.update_cfg() needs ALL joints, not just actuated ones
            # This includes: actuated joints, mimic joints, and fixed joints
            self.urdf_all_joint_names = []
            self.urdf_actuated_joint_names = []
            self.mimic_joint_info = {}  # {mimic_joint_name: (mimicked_joint_name, multiplier, offset)}
            
            for joint_name, joint in self.urdf.joint_map.items():
                self.urdf_all_joint_names.append(joint_name)
                
                # Track actuated joints
                if joint.type in ['revolute', 'prismatic', 'continuous']:
                    self.urdf_actuated_joint_names.append(joint_name)
                
                # Track mimic joints
                if joint.mimic is not None:
                    mimicked_joint = joint.mimic.joint
                    multiplier = joint.mimic.multiplier if joint.mimic.multiplier is not None else 1.0
                    offset = joint.mimic.offset if joint.mimic.offset is not None else 0.0
                    
                    # Check if the mimicked joint exists in the URDF
                    if mimicked_joint in self.urdf.joint_map:
                        # Check if the mimicked joint is actuated
                        mimicked_joint_obj = self.urdf.joint_map[mimicked_joint]
                        if mimicked_joint_obj.type not in ['revolute', 'prismatic', 'continuous']:
                            logger.warning(f"Mimic joint '{joint_name}' references non-actuated joint '{mimicked_joint}' "
                                            f"(type: {mimicked_joint_obj.type}). "
                                            f"This may cause issues. Please fix the URDF.")
                        # Track mimic joints (simple, non-recursive)
                        self.mimic_joint_info[joint_name] = (mimicked_joint, multiplier, offset)
                    else:
                        logger.warning(f"Mimic joint '{joint_name}' references unknown joint '{mimicked_joint}'. Skipping.")
            
            logger.debug(f"Extracted {len(self.urdf_all_joint_names)} total joints from URDF "
                        f"({len(self.urdf_actuated_joint_names)} actuated, "
                        f"{len(self.mimic_joint_info)} mimic, "
                        f"{len(self.urdf_all_joint_names) - len(self.urdf_actuated_joint_names) - len(self.mimic_joint_info)} fixed)")
            
            # Precompute mapping from joint name to URDF index (only computed once)
            self._joint_name_to_urdf_index = {name: i for i, name in enumerate(self.urdf_all_joint_names)}
            
            # Initialize Viser visualization
            self._init_viser()
            
            # Update joint_panel with URDF if it was created before URDF was loaded
            if self._joint_panel is not None and self.urdf is not None:
                self._joint_panel.set_urdf(self.urdf)
            
            # Update marker manager with URDF
            if self._marker_manager is not None:
                self._marker_manager.set_urdf(self.urdf)
            
        except Exception as e:
            logger.error(f"Failed to parse URDF: {e}", exc_info=True)
            logger.error(f"URDF data length: {len(urdf_string) if urdf_string else 0}")
            if urdf_string:
                logger.error(f"URDF data preview (first 500 chars): {urdf_string[:500]}")
            return
        
        # Give server a moment to fully start
        # This is safe because the update thread was already started in _init_viser()
        time.sleep(0.5)
    
    def _init_ros2_interface(self):
        """Initialize ROS2 Robot Interface."""
        interface_config = ROS2RobotInterfaceConfig(
            joint_states_topic=self.config.joint_states_topic,
            node_name="ros2_viser_node",
            gripper_enabled=self.config.enable_gripper_panel,
            joint_state_timeout=1.0,  # 5 second timeout for joint state data
        )
        self.ros2_interface = ROS2RobotInterface(interface_config)
        
        if self.config.auto_connect:
            self.ros2_interface.connect()
            logger.info("Connected to ROS2 Robot Interface")

    def _check_hardware_nodes(self):
        """Check for hardware system nodes and initialize hardware panel if needed.
        
        This method is called after connecting to ROS2 interface to detect
        any node whose name contains "system" (case-insensitive) and show
        the hardware panel accordingly. If "m6_ccs_system" is detected,
        a configuration button will be shown in the panel.
        """
        if self.ros2_interface is None or not self.ros2_interface.is_connected:
            return
        
        if self.server is None:
            return
        
        try:
            # Query node list
            nodes = self.ros2_interface.list_nodes()
            
            # Check for any node whose name contains "system" (case-insensitive)
            has_system_node = False
            has_m6_ccs_system = False
            detected_system_nodes = []
            
            for node in nodes:
                node_name_lower = node['name'].lower()
                # Check if node name contains "system" (any node with "system" in its name)
                if 'system' in node_name_lower:
                    has_system_node = True
                    detected_system_nodes.append(node['full_name'])
                    logger.info(f"Hardware system node detected: {node['full_name']}")
                
                # Specifically check for m6_ccs_system
                if 'm6_ccs_system' in node_name_lower:
                    has_m6_ccs_system = True
                    logger.info(f"M6 CCS System node detected: {node['full_name']}")
            
            # Initialize hardware panel if any system node is detected
            if has_system_node:
                logger.info(f"Found {len(detected_system_nodes)} system node(s): {', '.join(detected_system_nodes)}")
                
                # Cleanup existing hardware panel if any
                if self._hardware_panel is not None:
                    try:
                        self._hardware_panel.cleanup()
                    except Exception as e:
                        logger.warning(f"Error cleaning up Hardware panel: {e}")
                    self._hardware_panel = None
                
                # Create and initialize hardware panel
                self._hardware_panel = HardwarePanel(
                    self.server,
                    self.ros2_interface,
                    has_m6_ccs_system=has_m6_ccs_system
                )
                self._hardware_panel.initialize()
                logger.info("✅ Hardware panel initialized")
            else:
                logger.debug("No system node detected (no node name contains 'system'), hardware panel not shown")
                
        except Exception as e:
            logger.warning(f"Failed to check hardware nodes: {e}")
            # Don't raise - this is not critical
    
    def _check_robot_description_from_interface(self):
        """Check robot description from ros2_interface (called from update loop, not callback).
        
        This method checks if robot description has been received or changed,
        and triggers URDF parsing if needed.
        """
        if self.ros2_interface is None or not self.ros2_interface.is_connected:
            return
        
        try:
            # Get robot description from interface
            robot_description = self.ros2_interface.get_robot_description()
            # Check if message data is None or empty (including whitespace-only strings)
            if not robot_description or not robot_description.strip():
                return
            
            # Compute hash to detect changes
            urdf_hash = hashlib.md5(robot_description.encode()).hexdigest()
            if self._last_urdf_hash is not None and urdf_hash == self._last_urdf_hash:
                return
            
            if not self._urdf_received:
                logger.info(f"✅ Received robot description (length: {len(robot_description)} characters)")
            else:
                logger.info("URDF changed, reloading visualization...")
            
            self._parse_urdf_string(robot_description, urdf_hash)
        except Exception as e:
            logger.error(f"Error checking robot description from interface: {e}", exc_info=True)
    
    def _reinitialize_panels(self):
        """Reinitialize panels after ROS2 interface reconnection.
        
        This is called when URDF changes and ROS2 interface is reconnected.
        Panels need to be reinitialized to pick up new controller/topic configurations.
        """
        try:
            # Cleanup existing panels
            if self._fsm_panel is not None:
                try:
                    self._fsm_panel.cleanup()
                except Exception as e:
                    logger.warning(f"Error cleaning up FSM panel during reinit: {e}")
                self._fsm_panel = None
            
            if self._gripper_panel is not None:
                try:
                    self._gripper_panel.cleanup()
                except Exception as e:
                    logger.warning(f"Error cleaning up Gripper panel during reinit: {e}")
                self._gripper_panel = None
            
            if self._joint_panel is not None:
                try:
                    self._joint_panel.cleanup()
                except Exception as e:
                    logger.warning(f"Error cleaning up Joint panel during reinit: {e}")
                self._joint_panel = None
            
            if self._hardware_panel is not None:
                try:
                    self._hardware_panel.cleanup()
                except Exception as e:
                    logger.warning(f"Error cleaning up Hardware panel during reinit: {e}")
                self._hardware_panel = None
            
            # Reinitialize panels if enabled
            if self.config.enable_fsm_panel:
                self._fsm_panel = FSMPanel(
                    self.server,
                    self.ros2_interface,
                    self.config.fsm_command_topic
                )
                self._fsm_panel.initialize()
                logger.info("✅ FSM panel reinitialized")
            
            if self.config.enable_gripper_panel:
                self._gripper_panel = GripperPanel(
                    self.server,
                    self.ros2_interface
                )
                self._gripper_panel.initialize()
                logger.info("✅ Gripper panel reinitialized")
            
            if self.config.enable_joint_panel:
                self._joint_panel = JointPanel(
                    self.server,
                    self.ros2_interface,
                    urdf=self.urdf  # Pass pre-parsed URDF to avoid re-parsing
                )
                self._joint_panel.initialize()
                logger.info("✅ Joint panel reinitialized")
            
            # Reinitialize marker manager if enabled
            if self.config.enable_end_effector_marker:
                if self._marker_manager is not None:
                    try:
                        self._marker_manager.cleanup()
                    except Exception as e:
                        logger.warning(f"Error cleaning up marker manager during reinit: {e}")
                    self._marker_manager = None
                
                # Recreate marker manager
                if self.server is not None and self.ros2_interface is not None:
                    self._marker_manager = EndEffectorMarkerManager(
                        self.server,
                        self.ros2_interface,
                        self.config,
                        self.translator,
                        self.urdf,
                        self.config.root_node_name
                    )
                    # Connect GUI controls if they exist
                    if self._marker_publish_mode_dropdown is not None:
                        self._marker_manager.set_gui_controls(
                            self._marker_publish_mode_dropdown,
                            self._send_marker_pose_button
                        )
                    # Initialize markers (will be done when interface is ready)
                    self._marker_manager.initialize()
                    logger.info("✅ Marker manager reinitialized")
            
            # Recheck hardware nodes and reinitialize hardware panel if needed
            if self.ros2_interface is not None and self.ros2_interface.is_connected:
                self._check_hardware_nodes()
                
        except Exception as e:
            logger.error(f"Failed to reinitialize panels: {e}", exc_info=True)
    
    def _reinitialize_panels_with_language(self):
        """Update panel GUI labels with new language without destroying subscriptions.
        
        This method only updates GUI labels, preserving all ROS2 subscriptions
        and state. This is much safer than recreating everything.
        """
        try:
            # Update display control panel
            if self._display_folder_handle is not None:
                try:
                    self._display_folder_handle.remove()
                except Exception as e:
                    logger.warning(f"Error removing display control panel during language change: {e}")
                self._display_folder_handle = None
                self._show_visual_checkbox = None
                self._show_collision_checkbox = None
                self._language_dropdown = None
            
            # Reinitialize display control panel with new language
            self._init_display_control_panel()
            
            # Update panel GUI labels (preserving subscriptions and state)
            if self.config.enable_fsm_panel and self._fsm_panel is not None:
                try:
                    self._fsm_panel.update_gui_labels()
                    logger.info("✅ FSM panel labels updated with new language")
                except Exception as e:
                    logger.warning(f"Failed to update FSM panel labels: {e}")
            
            if self.config.enable_gripper_panel and self._gripper_panel is not None:
                try:
                    self._gripper_panel.update_gui_labels()
                    logger.info("✅ Gripper panel labels updated with new language")
                except Exception as e:
                    logger.warning(f"Failed to update Gripper panel labels: {e}")
            
            if self.config.enable_joint_panel and self._joint_panel is not None:
                try:
                    self._joint_panel.update_gui_labels()
                    logger.info("✅ Joint panel labels updated with new language")
                except Exception as e:
                    logger.warning(f"Failed to update Joint panel labels: {e}")
            
            if self._hardware_panel is not None:
                try:
                    self._hardware_panel.update_gui_labels()
                    logger.info("✅ Hardware panel labels updated with new language")
                except Exception as e:
                    logger.warning(f"Failed to update Hardware panel labels: {e}")
                
        except Exception as e:
            logger.error(f"Failed to update panels with new language: {e}", exc_info=True)
    
    def _init_display_control_panel(self):
        """Initialize display control panel with checkboxes for visual and collision."""
        if self.server is None:
            return
        
        try:
            # Create folder for display controls
            self._display_folder_handle = self.server.gui.add_folder(self.translator("display_control"))
            with self._display_folder_handle:
                # Language selection dropdown
                self._language_dropdown = self.server.gui.add_dropdown(
                    self.translator("select_language"),
                    options=["中文", "English"],
                    initial_value="中文" if self.translator.language == "zh" else "English"
                )
                self._language_dropdown.on_update(
                    lambda _: self._on_language_changed(self._language_dropdown.value)
                )
                
                # Checkbox for showing visual
                self._show_visual_checkbox = self.server.gui.add_checkbox(
                    self.translator("show_visual"),
                    initial_value=self._show_visual
                )
                self._show_visual_checkbox.on_update(
                    lambda _: self._on_show_visual_changed(self._show_visual_checkbox.value)
                )
                
                # Checkbox for showing collision
                self._show_collision_checkbox = self.server.gui.add_checkbox(
                    self.translator("show_collision"),
                    initial_value=self._show_collision
                )
                self._show_collision_checkbox.on_update(
                    lambda _: self._on_show_collision_changed(self._show_collision_checkbox.value)
                )
                
                # Marker publish mode selection (only if markers are enabled)
                # These controls are only visible in OCS2 mode
                if self.config.enable_end_effector_marker:
                    self._marker_publish_mode_dropdown = self.server.gui.add_dropdown(
                        self.translator("marker_publish_mode"),
                        options=[self.translator("continuous_publish"), self.translator("single_publish")],
                        initial_value=self.translator("continuous_publish") if self.config.marker_continuous_publish else self.translator("single_publish")
                    )
                    self._marker_publish_mode_dropdown.on_update(
                        lambda _: self._on_marker_publish_mode_changed(self._marker_publish_mode_dropdown.value)
                    )
                    # Initially hide (will be shown when in OCS2 mode)
                    self._marker_publish_mode_dropdown.visible = False
                    
                    # Send button for single-shot mode (initially hidden)
                    self._send_marker_pose_button = self.server.gui.add_button(
                        self.translator("send_marker_pose"),
                        color="green"
                    )
                    self._send_marker_pose_button.on_click(
                        lambda _: self._on_send_marker_pose_clicked()
                    )
                    # Initially hide (will be shown when in OCS2 mode and single-shot mode)
                    self._send_marker_pose_button.visible = False
                    
                    # Connect GUI controls to marker manager (if it exists)
                    if self._marker_manager is not None:
                        self._marker_manager.set_gui_controls(
                            self._marker_publish_mode_dropdown,
                            self._send_marker_pose_button
                        )
            
            logger.debug("Display control panel initialized")
        except Exception as e:
            logger.error(f"Failed to initialize display control panel: {e}", exc_info=True)
    
    def _on_show_visual_changed(self, value: bool):
        """Callback when show visual checkbox is toggled."""
        self._show_visual = value
        self._update_urdf_display()
        logger.debug(f"Show visual changed to: {value}")
    
    def _on_show_collision_changed(self, value: bool):
        """Callback when show collision checkbox is toggled."""
        self._show_collision = value
        self._update_urdf_display()
        logger.debug(f"Show collision changed to: {value}")
    
    def _on_marker_publish_mode_changed(self, mode_display: str):
        """Callback when marker publish mode dropdown is changed.
        
        Args:
            mode_display: Display name of selected mode.
        """
        if self._marker_manager is not None:
            self._marker_manager.on_publish_mode_changed(mode_display)
    
    def _on_send_marker_pose_clicked(self):
        """Callback when send marker pose button is clicked (single-shot mode)."""
        if self._marker_manager  is not None:
            self._marker_manager.on_send_button_clicked()
    
    def _on_language_changed(self, language_display: str):
        """Callback when language dropdown is changed.
        
        Args:
            language_display: Display name of selected language ("中文" or "English").
        """
        language_map = {
            "中文": "zh",
            "English": "en"
        }
        
        new_language = language_map.get(language_display, "zh")
        if new_language == self.translator.language:
            return
        
        logger.info(f"Language changed to: {new_language}")
        
        set_global_language(new_language)
        self.config.language = new_language
        self._reinitialize_panels_with_language()
    
    def _update_urdf_display(self):
        """Update URDF visualization based on display settings.
        
        Uses ViserUrdf's show_visual and show_collision attributes to control display.
        Ensures frame nodes remain visible to preserve frame hierarchy.
        """
        if self.urdf_vis is None or self.server is None:
            return
        
        try:
            # Use ViserUrdf's built-in attributes (following official demo pattern)
            if hasattr(self.urdf_vis, 'show_visual'):
                try:
                    self.urdf_vis.show_visual = self._show_visual
                    logger.debug(f"Set ViserUrdf.show_visual = {self._show_visual}")
                except Exception as e:
                    logger.debug(f"Could not set show_visual: {e}")
            
            if hasattr(self.urdf_vis, 'show_collision'):
                with warnings.catch_warnings():
                    warnings.filterwarnings(
                        "ignore",
                        message=".*Cannot set.*show_collision.*",
                        category=UserWarning
                    )
                    warnings.filterwarnings(
                        "ignore",
                        message=".*no collision meshes.*",
                        category=UserWarning
                    )
                    try:
                        self.urdf_vis.show_collision = self._show_collision
                        logger.debug(f"Set ViserUrdf.show_collision = {self._show_collision}")
                    except Exception as e:
                        logger.debug(f"Could not set show_collision: {e}")
            
            # Ensure frame nodes remain visible even when meshes are hidden
            root_path = self.config.root_node_name
            try:
                if hasattr(self.server.scene, '_nodes') or hasattr(self.server.scene, 'nodes'):
                    nodes_dict = getattr(self.server.scene, '_nodes', None) or getattr(self.server.scene, 'nodes', None)
                    if nodes_dict:
                        for node_path, node_obj in nodes_dict.items():
                            if node_path.startswith(root_path):
                                node_type_name = type(node_obj).__name__
                                is_frame_node = 'Frame' in node_type_name and 'Handle' in node_type_name
                                
                                if is_frame_node:
                                    if hasattr(node_obj, 'visible'):
                                        if not node_obj.visible:
                                            node_obj.visible = True
                                    elif hasattr(node_obj, 'set_visible'):
                                        node_obj.set_visible(True)
                        
                        logger.debug(f"Updated display: visual={self._show_visual}, collision={self._show_collision} (frame hierarchy preserved)")
            except Exception as e:
                logger.debug(f"Could not ensure frame node visibility: {e}")
                    
        except Exception as e:
            logger.warning(f"Failed to update URDF display: {e}")
    
    def _init_viser(self):
        """Initialize Viser server and URDF visualization."""
        if self.urdf is None:
            logger.warning("URDF not available, cannot initialize Viser")
            return
        
        try:
            # Create Viser server if it doesn't exist
            if self.server is None:
                try:
                    self.server = viser.ViserServer()
                except Exception as e:
                    logger.error(f"Failed to create Viser server: {e}", exc_info=True)
                    raise
                
                # Add ground grid (only once)
                logger.debug("Adding ground grid...")
                self.server.scene.add_grid("/ground", width=2, height=2)
                
                # Initialize display control panel
                self._init_display_control_panel()
                
                # Create panels if they don't exist yet
                # (They will be reinitialized if URDF changes)
                if self.config.enable_fsm_panel and self._fsm_panel is None:
                    self._fsm_panel = FSMPanel(
                        self.server,
                        self.ros2_interface,
                        self.config.fsm_command_topic
                    )
                    self._fsm_panel.initialize()
                
                if self.config.enable_gripper_panel and self._gripper_panel is None:
                    self._gripper_panel = GripperPanel(
                        self.server,
                        self.ros2_interface
                    )
                    self._gripper_panel.initialize()
                
                if self.config.enable_joint_panel and self._joint_panel is None:
                    self._joint_panel = JointPanel(
                        self.server,
                        self.ros2_interface,
                        urdf=self.urdf  # Pass pre-parsed URDF to avoid re-parsing
                    )
                    self._joint_panel.initialize()
                
                # Check for hardware system nodes and initialize hardware panel if needed
                # This is done after server is created and other panels are initialized
                if self.ros2_interface is not None and self.ros2_interface.is_connected:
                    self._check_hardware_nodes()
            
            # Create URDF visualization
            logger.debug(f"Creating ViserUrdf with {len(self.urdf_all_joint_names)} joints...")
            try:
                # Check if collision scene is available in URDF
                has_collision_scene = (
                    hasattr(self.urdf, 'collision_scene') and 
                    self.urdf.collision_scene is not None
                )
                
                # Create ViserUrdf with collision meshes enabled
                self.urdf_vis = ViserUrdf(
                    self.server,
                    self.urdf,
                    root_node_name=self.config.root_node_name,
                    load_meshes=True,
                    load_collision_meshes=has_collision_scene, 
                )
                logger.info(f"✅ URDF visualization initialized at {self.config.root_node_name}")
                if has_collision_scene:
                    logger.info(f"✅ {self.translator('collision_meshes_loaded')}")
                else:
                    logger.info(f"ℹ️  {self.translator('no_collision_meshes')}")
                
                # Update checkbox visibility based on what's actually loaded
                if self._show_collision_checkbox is not None:
                    self._show_collision_checkbox.visible = has_collision_scene
                
                # Apply initial display settings
                self._update_urdf_display()
            except Exception as e:
                logger.error(f"Failed to create ViserUrdf: {e}", exc_info=True)
                raise
            
            # Initialize end-effector marker manager if enabled
            if self.config.enable_end_effector_marker and self._marker_manager is None:
                if self.server is not None and self.ros2_interface is not None:
                    self._marker_manager = EndEffectorMarkerManager(
                        self.server,
                        self.ros2_interface,
                        self.config,
                        self.translator,
                        self.urdf,
                        self.config.root_node_name
                    )
                    # Connect GUI controls if they exist
                    if self._marker_publish_mode_dropdown is not None:
                        self._marker_manager.set_gui_controls(
                            self._marker_publish_mode_dropdown,
                            self._send_marker_pose_button
                        )
                    # Initialize markers (will be done when interface is ready)
                    self._marker_manager.initialize()
            
            # Start update loop if not already running
            if self._update_thread is None or not self._update_thread.is_alive():
                self._update_thread = threading.Thread(
                    target=self._update_loop,
                    daemon=True
                )
                self._update_thread.start()
                logger.debug("Update loop started")
                
        except Exception as e:
            logger.error(f"Failed to initialize Viser: {e}", exc_info=True)
            raise
    
    def _update_loop(self):
        """Update loop for robot visualization."""
        update_period = 1.0 / self.config.update_rate
        
        # Initialize to True because interface is already connected in start() method
        last_is_connected = True
        
        while self._running:
            
            # Check current connection state
            is_connected = (self.ros2_interface is not None and self.ros2_interface.is_connected)
            
            if not is_connected:
                last_is_connected = False
                time.sleep(update_period)
                continue
            else:
                # Detect robot reconnection: transition from disconnected to connected
                if not last_is_connected and self._urdf_received:
                    logger.info("Robot reconnected, checking URDF...")
                    try:
                        # Check for URDF changes
                        self._check_robot_description_from_interface()
                    except Exception as e:
                        logger.debug(f"Error checking robot description after reconnection: {e}")
                    
                    # Always reconnect to detect new topics/controllers (even if URDF didn't change)
                    logger.info("Reconnecting ROS2RobotInterface to detect new controllers/topics...")
                    try:
                        self.ros2_interface.disconnect()
                        time.sleep(0.5)
                        self._init_ros2_interface()
                        logger.info("✅ ROS2RobotInterface reconnected successfully")
                        
                        # Reinitialize panels that depend on ros2_interface configuration
                        self._reinitialize_panels()
                    except Exception as e:
                        logger.error(f"Failed to reconnect ROS2RobotInterface: {e}", exc_info=True)
                        # Continue anyway - visualization might still work
                
                # Update connection state (interface is connected at this point)
                last_is_connected = True
            
            self._fsm_panel.update()
            self._gripper_panel.update()
            self._joint_panel.update()
            self._marker_manager.update()
            
            # Update URDF visualization
            if self.urdf_vis is not None:
                joint_state = self.ros2_interface.get_joint_state()
                if joint_state is None:
                    time.sleep(update_period)
                    continue
                
                joint_positions = self._map_joint_states(
                    joint_state['names'],
                    joint_state['positions']
                )

                self.urdf_vis.update_cfg(joint_positions)   # Update URDF visualization
                
            time.sleep(update_period)
    
    def _map_joint_states(self, joint_names: list[str], positions: list[float]) -> np.ndarray:
        """Map joint states from ROS2 topic to URDF joint order (ALL joints)."""
        if len(self.urdf_all_joint_names) == 0:
            raise ValueError("URDF all joint names are not initialized")
        
        result = np.zeros(len(self.urdf_all_joint_names), dtype=np.float64)
        for joint_name, position in zip(joint_names, positions):
            if joint_name in self._joint_name_to_urdf_index:
                urdf_index = self._joint_name_to_urdf_index[joint_name]
                result[urdf_index] = position
        
        return result
    
    def start(self):
        """Start the visualizer."""
        if self._running:
            logger.warning("Visualizer is already running")
            return
        
        self._running = True
        
        # Initialize ROS2 Robot Interface (this creates the node and starts executor thread)
        self._init_ros2_interface()
        

        logger.info("Waiting for robot description from topic...")
        max_wait_time = self.config.robot_description_timeout  # seconds
        # If timeout is 0, wait indefinitely (no timeout check)
        wait_indefinitely = (max_wait_time == 0.0)
        start_time = time.time()
        check_interval = 2.0  # Print status every 2 seconds
        
        last_status_time = start_time
        robot_description = None
        
        while robot_description is None:
            # Check timeout only if not waiting indefinitely
            if not wait_indefinitely:
                if (time.time() - start_time) >= max_wait_time:
                    break
            
            # Poll robot_description from interface (non-blocking)
            # The executor thread in ros2_interface is already running, so callbacks are being processed
            if self.ros2_interface is not None and self.ros2_interface.is_connected:
                try:
                    robot_description = self.ros2_interface.get_robot_description()
                    if robot_description and len(robot_description.strip()) > 0:
                        break
                except Exception as e:
                    logger.debug(f"Error getting robot description: {e}")
            
            elapsed = time.time() - start_time
            if time.time() - last_status_time >= check_interval:
                if wait_indefinitely:
                    logger.info(f"Still waiting for robot description... ({elapsed:.1f}s, waiting indefinitely)")
                else:
                    logger.info(f"Still waiting for robot description... ({elapsed:.1f}s / {max_wait_time:.1f}s)")
                last_status_time = time.time()
            time.sleep(0.1)
        
        if robot_description is None or len(robot_description.strip()) == 0:
            if wait_indefinitely:
                # This shouldn't happen if waiting indefinitely, but handle it just in case
                logger.error("Failed to receive robot description (waiting indefinitely)")
            else:
                logger.error(f"Timeout after {max_wait_time}s waiting for robot description")
            logger.error("Please check:")
            logger.error("  1. Is /robot_description topic being published?")
            logger.error("  2. Is robot_state_publisher node running?")
            logger.error("  3. Try running: ros2 topic echo /robot_description")
            raise TimeoutError("Failed to receive robot description from ROS2 topic")
        
        # Parse URDF (this will set _urdf_received and start _update_loop)
        logger.info(f"✅ Received robot description (length: {len(robot_description)} characters)")
        urdf_hash = hashlib.md5(robot_description.encode()).hexdigest()
        self._parse_urdf_string(robot_description, urdf_hash)
        
        logger.info("✅ Visualizer started successfully")
    
    def stop(self):
        """Stop the visualizer and cleanup resources."""
        self._running = False
        
        # First, cleanup panels to set cleanup flags and prevent callbacks
        # This should be done before disconnecting ROS2 interface
        if self._fsm_panel is not None:
            try:
                self._fsm_panel.cleanup()
            except Exception as e:
                logger.warning(f"Error cleaning up FSM panel: {e}")
        
        if self._gripper_panel is not None:
            try:
                self._gripper_panel.cleanup()
            except Exception as e:
                logger.warning(f"Error cleaning up Gripper panel: {e}")
        
        if self._hardware_panel is not None:
            try:
                self._hardware_panel.cleanup()
            except Exception as e:
                logger.warning(f"Error cleaning up Hardware panel: {e}")
        
        # Wait for update thread first (it uses ros2_interface)
        if self._update_thread is not None:
            self._update_thread.join(timeout=2.0)
        
        # Wait a bit to let any in-flight ROS2 callbacks complete
        # This gives executor time to finish current operations
        time.sleep(0.2)
        
        # Cleanup ROS2 Robot Interface (this will shutdown executor)
        # Do this after panels are cleaned up and update thread is stopped
        # Note: We always own the interface (created in _init_ros2_interface)
        if self.ros2_interface is not None:
            try:
                self.ros2_interface.disconnect()
            except Exception as e:
                # This error can occur if executor is still running when disconnect is called
                # It's safe to ignore as the disconnect will still complete
                logger.debug(f"Error during ROS2 interface disconnect (may be expected during shutdown): {e}")
        
        # Cleanup display control panel
        if self._display_folder_handle is not None:
            try:
                self._display_folder_handle.remove()
            except Exception as e:
                logger.warning(f"Error removing display control panel: {e}")
        
        # Cleanup end-effector marker manager
        if self._marker_manager is not None:
            try:
                self._marker_manager.cleanup()
            except Exception as e:
                logger.warning(f"Error cleaning up marker manager: {e}")
            self._marker_manager = None
        
        # Cleanup Viser
        if self.urdf_vis is not None:
            try:
                self.urdf_vis.remove()
            except Exception as e:
                logger.warning(f"Error removing URDF visualization: {e}")
        
        logger.info("Visualizer stopped")
    
    def __enter__(self):
        """Context manager entry."""
        self.start()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit."""
        self.stop()
