"""ROS2 Viser Visualizer - Visualize ROS2 robots using Viser."""

import hashlib
import logging
import threading
import time
import warnings
from io import StringIO
from typing import Optional

import numpy as np
import viser
from rclpy.node import Node
from viser.extras import ViserUrdf
import yourdfpy

from ros2_robot_interface import ROS2RobotInterface, ROS2RobotInterfaceConfig

from .config import ROS2ViserConfig
from .panels import FSMPanel, GripperPanel, JointPanel
from .i18n import Translator, get_translator, set_global_language

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
        
        # ROS2 Robot Interface (required - we use its node for all ROS2 operations)
        self.ros2_interface: Optional[ROS2RobotInterface] = None
        self._own_interface = False
        
        # State
        self._running = False
        self._urdf_received = False
        self._update_thread: Optional[threading.Thread] = None
        
        # Flag to trigger ROS2 interface reconnection (set in callback, handled in update loop)
        self._needs_reconnect = False
        self._reconnect_lock = threading.Lock()
        
        # URDF reloading
        self._urdf_lock = threading.Lock()  # Lock for URDF reloading
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
        
        # Display control panel
        self._display_folder_handle: Optional[viser.GuiFolderHandle] = None
        self._show_visual_checkbox: Optional[viser.GuiCheckboxHandle] = None
        self._show_collision_checkbox: Optional[viser.GuiCheckboxHandle] = None
        self._language_dropdown: Optional[viser.GuiDropdownHandle] = None
        self._show_visual: bool = True
        self._show_collision: bool = False
        
        # Note: No direct subscription - use ros2_interface.get_robot_description() instead
    
    def _parse_urdf_string(self, urdf_string: str, urdf_hash: Optional[str] = None):
        """Parse URDF string and initialize/reload visualization.
        
        Args:
            urdf_string: URDF XML string
            urdf_hash: Optional hash of the URDF string for change detection
        """
        with self._urdf_lock:
            if not urdf_string or len(urdf_string.strip()) == 0:
                logger.warning("URDF string is empty, skipping parse")
                return
            
            try:
                # If URDF already loaded, clean up old visualization and reload interfaces
                if self._urdf_received and self.urdf_vis is not None:
                    logger.info("URDF changed, reloading visualization and ROS2 interfaces...")
                    
                    # Clean up old visualization
                    try:
                        self.urdf_vis.remove()
                    except Exception as e:
                        logger.warning(f"Error removing old URDF visualization: {e}")
                    self.urdf_vis = None
                    
                    # Mark that ROS2 interface needs reconnection (don't do it here - we're in a callback thread)
                    # The reconnection will be handled in the update loop to avoid threading issues
                    if self._own_interface and self.ros2_interface is not None:
                        with self._reconnect_lock:
                            self._needs_reconnect = True
                        logger.info("URDF changed - ROS2 interface reconnection will be handled in update loop")
                
                # Parse URDF from string using StringIO
                logger.debug(f"Parsing URDF (length: {len(urdf_string)} characters)")
                # yourdfpy.URDF.load() can load from a file-like object
                # Load with visual scene and collision scene enabled (needed for ViserUrdf visualization)
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
                # Note: We're already inside _urdf_lock, so call the unlocked version
                self._init_viser_unlocked()
                
                # Update joint_panel with URDF if it was created before URDF was loaded
                if self._joint_panel is not None and self.urdf is not None:
                    self._joint_panel.set_urdf(self.urdf)
                
            except Exception as e:
                logger.error(f"Failed to parse URDF: {e}", exc_info=True)
                logger.error(f"URDF data length: {len(urdf_string) if urdf_string else 0}")
                if urdf_string:
                    logger.error(f"URDF data preview (first 500 chars): {urdf_string[:500]}")
                return
        
        # Give server a moment to fully start (outside lock to avoid holding it too long)
        # This is safe because the update thread was already started in _init_viser_unlocked()
        # and it will wait for the lock if needed
        time.sleep(0.5)
    
    def _init_ros2_interface(self):
        """Initialize ROS2 Robot Interface."""
        interface_config = ROS2RobotInterfaceConfig(
            joint_states_topic=self.config.joint_states_topic,
            node_name="ros2_viser_node",
            gripper_enabled=self.config.enable_gripper_panel,
        )
        self.ros2_interface = ROS2RobotInterface(interface_config)
        self._own_interface = True
        
        if self.config.auto_connect:
            self.ros2_interface.connect()
            logger.info("Connected to ROS2 Robot Interface")
        
        # Note: Robot description is now tracked by ros2_interface
        # We'll poll it in the update loop or wait for it to be available
    
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
            if robot_description is None:
                return
            
            # Check if message data is empty
            if not robot_description or len(robot_description.strip()) == 0:
                return
            
            # Compute hash to detect changes
            urdf_hash = hashlib.md5(robot_description.encode()).hexdigest()
            
            # Check if URDF has changed
            if self._last_urdf_hash is not None and urdf_hash == self._last_urdf_hash:
                # URDF hasn't changed, skip
                return
            
            if not self._urdf_received:
                logger.info(f"✅ Received robot description (length: {len(robot_description)} characters)")
            else:
                logger.info("URDF changed, reloading visualization...")
            
            # Parse and reload URDF (this is thread-safe and handles locks internally)
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
                
        except Exception as e:
            logger.error(f"Failed to update panels with new language: {e}", exc_info=True)
    
    def _init_viser(self):
        """Initialize Viser server and URDF visualization (with lock)."""
        with self._urdf_lock:
            self._init_viser_unlocked()
    
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
    
    def _on_language_changed(self, language_display: str):
        """Callback when language dropdown is changed.
        
        Args:
            language_display: Display name of selected language ("中文" or "English").
        """
        # Map display name to language code
        language_map = {
            "中文": "zh",
            "English": "en"
        }
        
        new_language = language_map.get(language_display, "zh")
        if new_language == self.translator.language:
            # Language hasn't changed, skip
            return
        
        logger.info(f"Language changed to: {new_language}")
        
        # Update global language
        set_global_language(new_language)
        
        # Update config to persist language setting
        self.config.language = new_language
        
        # Reinitialize all panels to update labels
        # Note: This will recreate GUI elements with new language
        self._reinitialize_panels_with_language()
    
    def _update_urdf_display(self):
        """Update URDF visualization based on display settings."""
        if self.urdf_vis is None or self.server is None:
            return
        
        try:
            # Method 1: Try to access ViserUrdf's internal attributes to control display
            # ViserUrdf has show_visual and show_collision attributes
            # Suppress warnings when setting show_collision if no collision meshes exist
            if hasattr(self.urdf_vis, 'show_visual'):
                try:
                    self.urdf_vis.show_visual = self._show_visual
                    logger.debug(f"Set ViserUrdf.show_visual = {self._show_visual}")
                except Exception as e:
                    logger.debug(f"Could not set show_visual: {e}")
            
            if hasattr(self.urdf_vis, 'show_collision'):
                # Suppress the warning about no collision meshes - this is expected
                # if the URDF doesn't have collision geometry defined
                with warnings.catch_warnings():
                    # Filter out warnings about collision meshes not being loaded
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
            
            # Method 2: Try to access scene nodes and set visibility
            # ViserUrdf creates scene nodes for visual and collision geometries
            if hasattr(self.urdf_vis, 'scene_nodes'):
                # If ViserUrdf has a scene_nodes attribute, update visibility
                for node_name, node in self.urdf_vis.scene_nodes.items():
                    if 'visual' in node_name.lower():
                        # Set visual node visibility
                        if hasattr(node, 'visible'):
                            node.visible = self._show_visual
                    elif 'collision' in node_name.lower():
                        # Set collision node visibility
                        if hasattr(node, 'visible'):
                            node.visible = self._show_collision
            
            # Method 3: Access the server's scene and update node visibility directly
            # Traverse scene nodes under the root node to find visual/collision meshes
            root_path = self.config.root_node_name
            try:
                # Get all nodes in the scene that start with the root path
                # ViserUrdf typically creates nodes like /robot/link_name/visual or /robot/link_name/collision
                # We need to iterate through all scene nodes and update their visibility
                
                # Try to access scene's internal node structure
                if hasattr(self.server.scene, '_nodes') or hasattr(self.server.scene, 'nodes'):
                    nodes_dict = getattr(self.server.scene, '_nodes', None) or getattr(self.server.scene, 'nodes', None)
                    if nodes_dict:
                        for node_path, node_obj in nodes_dict.items():
                            if node_path.startswith(root_path):
                                # Check if this is a visual or collision node
                                path_lower = node_path.lower()
                                if '/visual' in path_lower or path_lower.endswith('/visual'):
                                    # This is a visual node
                                    if hasattr(node_obj, 'visible'):
                                        node_obj.visible = self._show_visual
                                    elif hasattr(node_obj, 'set_visible'):
                                        node_obj.set_visible(self._show_visual)
                                elif '/collision' in path_lower or path_lower.endswith('/collision'):
                                    # This is a collision node
                                    if hasattr(node_obj, 'visible'):
                                        node_obj.visible = self._show_collision
                                    elif hasattr(node_obj, 'set_visible'):
                                        node_obj.set_visible(self._show_collision)
                
                logger.debug(f"Updated display: visual={self._show_visual}, collision={self._show_collision}")
            except Exception as e:
                logger.debug(f"Could not update scene node visibility directly: {e}")
                    
        except Exception as e:
            logger.warning(f"Failed to update URDF display: {e}")
            # Log the error but don't raise - the visualization will still work
            # even if we can't control visual/collision separately
    
    def _init_viser_unlocked(self):
        """Initialize Viser server and URDF visualization (without lock, assumes lock is already held)."""
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
            
            # Create URDF visualization
            logger.debug(f"Creating ViserUrdf with {len(self.urdf_all_joint_names)} joints...")
            try:
                # Check if collision scene is available in URDF
                has_collision_scene = (
                    hasattr(self.urdf, 'collision_scene') and 
                    self.urdf.collision_scene is not None
                )
                
                # Create ViserUrdf with collision meshes enabled
                # According to Viser documentation, we need to pass load_collision_meshes=True
                self.urdf_vis = ViserUrdf(
                    self.server,
                    self.urdf,
                    root_node_name=self.config.root_node_name,
                    load_meshes=True,  # Load visual meshes
                    load_collision_meshes=has_collision_scene,  # Load collision meshes if available
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
            
            # Start update loop if not already running
            # Note: The sleep after starting the thread is done outside this method
            # to avoid holding the lock for too long
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
        last_joint_state_log = 0.0
        last_fsm_panel_update = 0.0
        last_gripper_panel_update = 0.0
        last_joint_panel_update = 0.0
        log_interval = 5.0  # Log joint state status every 5 seconds
        fsm_panel_update_interval = 0.1  # Update FSM panel every 100ms
        gripper_panel_update_interval = 0.1  # Update Gripper panel every 100ms
        joint_panel_update_interval = 0.1  # Update Joint panel every 100ms
        
        while self._running:
            current_time = time.time()
            
            # Check robot description from interface (non-blocking, called from main loop)
            if self.ros2_interface is not None and self.ros2_interface.is_connected:
                try:
                    self._check_robot_description_from_interface()
                except Exception as e:
                    logger.debug(f"Error checking robot description: {e}")
            
            # Check if ROS2 interface needs reconnection (triggered by URDF change)
            with self._reconnect_lock:
                needs_reconnect = self._needs_reconnect
                if needs_reconnect:
                    self._needs_reconnect = False  # Clear flag
            
            if needs_reconnect and self._own_interface and self.ros2_interface is not None:
                logger.info("Reconnecting ROS2RobotInterface to detect new controllers/topics...")
                try:
                    # Disconnect old interface
                    self.ros2_interface.disconnect()
                    
                    # Wait a bit for ROS2 to clean up old topic registrations
                    # This prevents detecting stale topics from the previous connection
                    time.sleep(0.5)
                    
                    # Recreate interface with current configuration to ensure latest settings
                    # This ensures gripper_enabled and other settings are up to date
                    self._init_ros2_interface()
                    
                    logger.info("✅ ROS2RobotInterface reconnected successfully")
                    
                    # Reinitialize panels that depend on ros2_interface configuration
                    self._reinitialize_panels()
                except Exception as e:
                    logger.error(f"Failed to reconnect ROS2RobotInterface: {e}", exc_info=True)
                    # Continue anyway - visualization might still work
            
            # Update FSM panel periodically (not in ROS2 callback to avoid blocking)
            if self._fsm_panel is not None and (current_time - last_fsm_panel_update) >= fsm_panel_update_interval:
                try:
                    self._fsm_panel.update()
                    last_fsm_panel_update = current_time
                except Exception as e:
                    logger.warning(f"Failed to update FSM panel in update loop: {e}")
            
            # Update Gripper panel periodically (not in ROS2 callback to avoid blocking)
            if self._gripper_panel is not None and (current_time - last_gripper_panel_update) >= gripper_panel_update_interval:
                try:
                    self._gripper_panel.update()
                    last_gripper_panel_update = current_time
                except Exception as e:
                    logger.warning(f"Failed to update Gripper panel in update loop: {e}")
            
            # Update Joint panel periodically (not in ROS2 callback to avoid blocking)
            if self._joint_panel is not None and (current_time - last_joint_panel_update) >= joint_panel_update_interval:
                try:
                    self._joint_panel.update()
                    last_joint_panel_update = current_time
                except Exception as e:
                    logger.warning(f"Failed to update Joint panel in update loop: {e}")
            
            # Use lock to ensure URDF is not being reloaded during update
            with self._urdf_lock:
                if self.urdf_vis is not None and self.ros2_interface is not None:
                    # Check if interface is connected before using it
                    if not self.ros2_interface.is_connected:
                        # Log periodically if interface is not connected
                        if current_time - last_joint_state_log >= log_interval:
                            logger.debug("ROS2RobotInterface is not connected, waiting for reconnection...")
                            last_joint_state_log = current_time
                        time.sleep(update_period)
                        continue
                    
                    # Get joint state from ROS2 interface
                    joint_state = self.ros2_interface.get_joint_state()
                    
                    if joint_state is not None:
                        # Map joint states to URDF joint order
                        joint_positions = self._map_joint_states(
                            joint_state['names'],
                            joint_state['positions']
                        )
                        
                        if joint_positions is not None:
                            # Update visualization
                            # _map_joint_states already returns a properly formatted numpy array
                            try:
                                # Ensure array is contiguous (for performance)
                                if not joint_positions.flags['C_CONTIGUOUS']:
                                    joint_positions = np.ascontiguousarray(joint_positions, dtype=np.float64)
                                
                                # Skip update if values haven't changed (especially important for complex visual models)
                                # This reduces unnecessary rendering for visual models which are more complex
                                if self._last_joint_positions is not None:
                                    if np.array_equal(joint_positions, self._last_joint_positions):
                                        # Values unchanged, skip update to avoid unnecessary rendering
                                        # This is especially beneficial for complex visual models
                                        pass  # Skip update, but continue with sleep
                                    else:
                                        # Values changed, update visualization
                                        self.urdf_vis.update_cfg(joint_positions)
                                        # Cache the updated positions
                                        self._last_joint_positions = joint_positions.copy()
                                else:
                                    # First update, always perform it
                                    self.urdf_vis.update_cfg(joint_positions)
                                    # Cache the updated positions
                                    self._last_joint_positions = joint_positions.copy()
                            except Exception as e:
                                logger.warning(f"Failed to update visualization: {e}")
                                if logger.isEnabledFor(logging.DEBUG):
                                    logger.debug(f"Joint positions type: {type(joint_positions)}, "
                                               f"shape: {getattr(joint_positions, 'shape', 'N/A')}, "
                                               f"dtype: {getattr(joint_positions, 'dtype', 'N/A')}")
                                    if hasattr(joint_positions, '__len__') and len(joint_positions) > 0:
                                        logger.debug(f"First element type: {type(joint_positions[0]) if hasattr(joint_positions, '__getitem__') else 'N/A'}")
                    else:
                        # Log periodically if joint state is None
                        if current_time - last_joint_state_log >= log_interval:
                            logger.debug("Joint state is None - waiting for joint state messages...")
                            last_joint_state_log = current_time
                elif self.urdf_vis is None:
                    # Log periodically if URDF visualization is not initialized
                    if current_time - last_joint_state_log >= log_interval:
                        logger.debug("URDF visualization not initialized yet...")
                        last_joint_state_log = current_time
                elif self.ros2_interface is None:
                    # Log periodically if ROS2 interface is not available
                    if current_time - last_joint_state_log >= log_interval:
                        logger.debug("ROS2 interface not available...")
                        last_joint_state_log = current_time
            
            time.sleep(update_period)
    
    def _map_joint_states(self, joint_names: list[str], positions: list[float]) -> Optional[np.ndarray]:
        """Map joint states from ROS2 topic to URDF joint order (ALL joints).
        
        Optimized version that minimizes dictionary lookups and type conversions.
        
        This function handles:
        - Actuated joints: get value from joint state
        - Mimic joints: compute from mimicked joint using multiplier and offset
        - Fixed joints: use 0.0
        
        Args:
            joint_names: Joint names from ROS2 topic
            positions: Joint positions from ROS2 topic
            
        Returns:
            Array of joint positions for ALL joints in URDF order, or None if mapping fails.
        """
        if not self.urdf_all_joint_names:
            return None
        
        num_joints = len(self.urdf_all_joint_names)
        
        # Pre-allocate result array with zeros (default for all joints)
        result = np.zeros(num_joints, dtype=np.float64)
        
        # Use precomputed mapping (computed once during URDF parsing)
        for joint_name, position in zip(joint_names, positions):
            if joint_name in self._joint_name_to_urdf_index:
                # Joint exists in URDF, set its value directly
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
        
        # Wait for URDF by polling ros2_interface directly (avoiding circular dependency)
        # We can't wait for _urdf_received because it's only set in _parse_urdf_string,
        # which is called from _check_robot_description_from_interface in _update_loop,
        # but _update_loop only starts after URDF is parsed in _init_viser_unlocked.
        # The ros2_interface already has an executor thread running, so callbacks will be called.
        logger.info("Waiting for robot description from topic...")
        max_wait_time = 30.0  # seconds
        start_time = time.time()
        check_interval = 2.0  # Print status every 2 seconds
        
        last_status_time = start_time
        robot_description = None
        
        while robot_description is None and (time.time() - start_time) < max_wait_time:
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
                logger.info(f"Still waiting for robot description... ({elapsed:.1f}s / {max_wait_time:.1f}s)")
                last_status_time = time.time()
            time.sleep(0.1)
        
        if robot_description is None or len(robot_description.strip()) == 0:
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
        
        # Wait for update thread first (it uses ros2_interface)
        if self._update_thread is not None:
            self._update_thread.join(timeout=2.0)
        
        # Wait a bit to let any in-flight ROS2 callbacks complete
        # This gives executor time to finish current operations
        time.sleep(0.2)
        
        # Cleanup ROS2 Robot Interface (this will shutdown executor)
        # Do this after panels are cleaned up and update thread is stopped
        if self.ros2_interface is not None and self._own_interface:
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
