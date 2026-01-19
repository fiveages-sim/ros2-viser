"""ROS2 Viser Visualizer - Visualize ROS2 robots using Viser."""

import hashlib
import logging
import threading
import time
from io import StringIO
from typing import Optional

import numpy as np
import viser
from rclpy.node import Node
from rclpy.subscription import Subscription
from std_msgs.msg import String
from viser.extras import ViserUrdf
import yourdfpy

from ros2_robot_interface import ROS2RobotInterface, ROS2RobotInterfaceConfig

from .config import ROS2ViserConfig
from .panels import FSMPanel, GripperPanel

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
        
        # GUI panels
        self._fsm_panel: Optional[FSMPanel] = None
        self._gripper_panel: Optional[GripperPanel] = None
        
        # Keep references to subscriptions to prevent garbage collection
        self._robot_description_subscription: Optional[Subscription] = None
    
    def _init_robot_description_subscription(self):
        """Initialize robot description subscription using ros2_interface's node."""
        if self.ros2_interface is None or not self.ros2_interface.is_connected:
            logger.warning("ROS2RobotInterface not connected, cannot subscribe to robot_description")
            return
        
        # Get the node from ros2_interface
        ros2_node = self.ros2_interface.robot_node
        if ros2_node is None:
            logger.warning("ROS2RobotInterface node not available")
            return
        
        logger.info(f"Setting up subscription to {self.config.robot_description_topic}...")
        
        # Subscribe to robot description topic
        # Use TRANSIENT_LOCAL durability to receive latched messages
        from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy, ReliabilityPolicy
        
        qos_profile = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,  # Receive latched messages
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST
        )
        
        # Save subscription to prevent garbage collection
        self._robot_description_subscription = ros2_node.create_subscription(
            String,
            self.config.robot_description_topic,
            self._robot_description_callback,
            qos_profile
        )
        
        logger.info(f"✅ Subscribed to {self.config.robot_description_topic}")
        
        # Check if executor is running (needed to receive messages)
        if not (hasattr(self.ros2_interface, 'executor') and self.ros2_interface.executor is not None):
            logger.warning(f"⚠️  ROS2RobotInterface executor may not be running - messages won't be received!")
        
        # Check if topic exists
        try:
            topic_names = ros2_node.get_topic_names_and_types()
            topic_list = [name for name, _ in topic_names]
            if self.config.robot_description_topic not in topic_list:
                logger.warning(f"⚠️  Topic {self.config.robot_description_topic} not found in available topics")
        except Exception as e:
            logger.debug(f"Could not check topic availability: {e}")
        
        # Give executor a moment to process the subscription
        import time
        time.sleep(0.5)
    
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
                # Load with visual scene enabled (needed for ViserUrdf visualization)
                urdf_io = StringIO(urdf_string)
                try:
                    # Try to load with explicit parameters to enable visual scene
                    # build_scene_graph=True enables visual scene (default in newer versions)
                    self.urdf = yourdfpy.URDF.load(urdf_io, build_scene_graph=True)
                    logger.debug("URDF loaded with build_scene_graph=True")
                except (AttributeError, TypeError) as e:
                    # Fallback: try without build_scene_graph parameter (older versions)
                    if "build_scene_graph" in str(e) or "unexpected keyword" in str(e):
                        urdf_io.seek(0)  # Reset StringIO
                        # Older versions: load normally (should enable visual scene by default)
                        self.urdf = yourdfpy.URDF.load(urdf_io)
                        logger.debug("URDF loaded with default settings (visual scene should be enabled)")
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
                
                # Initialize Viser visualization
                # Note: We're already inside _urdf_lock, so call the unlocked version
                self._init_viser_unlocked()
                
            except Exception as e:
                logger.error(f"Failed to parse URDF: {e}", exc_info=True)
                logger.error(f"URDF data length: {len(urdf_string) if urdf_string else 0}")
                if urdf_string:
                    logger.error(f"URDF data preview (first 500 chars): {urdf_string[:500]}")
    
    def _init_ros2_interface(self):
        """Initialize ROS2 Robot Interface if needed."""
        if self.config.ros2_interface is not None:
            # Use existing interface
            self.ros2_interface = self.config.ros2_interface
            self._own_interface = False
            logger.info("Using provided ROS2RobotInterface instance")
            
            # Ensure it's connected
            if not self.ros2_interface.is_connected:
                logger.warning("Provided ROS2RobotInterface is not connected, connecting now...")
                self.ros2_interface.connect()
        else:
            # Create new interface
            interface_config = ROS2RobotInterfaceConfig(
                joint_states_topic=self.config.joint_states_topic,
                # These are required but not used for visualization
                end_effector_pose_topic="/dummy_pose",
                end_effector_target_topic="/dummy_target",
            )
            self.ros2_interface = ROS2RobotInterface(interface_config)
            self._own_interface = True
            
            if self.config.auto_connect:
                self.ros2_interface.connect()
                logger.info("Connected to ROS2 Robot Interface")
        
        # Initialize robot description subscription using the interface's node
        self._init_robot_description_subscription()
    
    def _robot_description_callback(self, msg: String):
        """Callback for robot description (URDF) messages.
        
        This callback handles both initial URDF loading and automatic reloading
        when the robot description changes.
        
        IMPORTANT: This callback runs in the ROS2 executor thread. We should
        avoid long-running operations here. URDF parsing is done in a separate
        thread-safe manner.
        """
        try:
            logger.debug(f"Robot description callback triggered (message length: {len(msg.data) if msg.data else 0})")
            
            # Check if message data is empty or None
            if not msg.data or len(msg.data.strip()) == 0:
                logger.warning("Received empty robot description message")
                return
            
            # Compute hash to detect changes
            urdf_hash = hashlib.md5(msg.data.encode()).hexdigest()
            
            # Check if URDF has changed
            if self._last_urdf_hash is not None and urdf_hash == self._last_urdf_hash:
                # URDF hasn't changed, skip
                logger.debug("URDF hasn't changed, skipping")
                return
            
            if not self._urdf_received:
                logger.info(f"✅ Received robot description (length: {len(msg.data)} characters)")
            else:
                logger.info("URDF changed, reloading visualization...")
            
            # Parse and reload URDF (this is thread-safe and handles locks internally)
            self._parse_urdf_string(msg.data, urdf_hash)
        except Exception as e:
            # Catch all exceptions to prevent executor from stopping
            logger.error(f"Error in robot description callback: {e}", exc_info=True)
    
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
                
        except Exception as e:
            logger.error(f"Failed to reinitialize panels: {e}", exc_info=True)
    
    def _init_viser(self):
        """Initialize Viser server and URDF visualization (with lock)."""
        with self._urdf_lock:
            self._init_viser_unlocked()
    
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
                    
                    # Try to get server URL/port information
                    server_url = None
                    try:
                        # Check various possible attributes
                        if hasattr(self.server, 'url'):
                            server_url = self.server.url
                        elif hasattr(self.server, '_url'):
                            server_url = self.server._url
                        elif hasattr(self.server, 'port'):
                            port = self.server.port
                            server_url = f"http://localhost:{port}"
                        elif hasattr(self.server, '_port'):
                            port = self.server._port
                            server_url = f"http://localhost:{port}"
                    except Exception as e:
                        logger.debug(f"Could not get server URL from attributes: {e}")
                    
                    # Print server information
                    if server_url:
                        print(f"\n{'='*60}")
                        print(f"✅ Viser visualization server is running!")
                        print(f"🌐 Open your browser and navigate to: {server_url}")
                        print(f"{'='*60}\n")
                    else:
                        # Default ports to try
                        default_urls = [
                            "http://localhost:8080",
                            "http://localhost:8010",
                            "http://localhost:8000"
                        ]
                        print(f"\n{'='*60}")
                        print(f"✅ Viser visualization server is running!")
                        print(f"🌐 Try opening one of these URLs in your browser:")
                        for url in default_urls:
                            print(f"   - {url}")
                        print(f"{'='*60}\n")
                        
                except Exception as e:
                    logger.error(f"Failed to create Viser server: {e}", exc_info=True)
                    raise
                
                # Add ground grid (only once)
                logger.debug("Adding ground grid...")
                self.server.scene.add_grid("/ground", width=2, height=2)
                
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
            
            # Create URDF visualization
            logger.debug(f"Creating ViserUrdf with {len(self.urdf_all_joint_names)} joints...")
            try:
                self.urdf_vis = ViserUrdf(
                    self.server,
                    self.urdf,
                    root_node_name=self.config.root_node_name
                )
                logger.info(f"✅ URDF visualization initialized at {self.config.root_node_name}")
            except Exception as e:
                logger.error(f"Failed to create ViserUrdf: {e}", exc_info=True)
                raise
            
            # Start update loop if not already running
            if self._update_thread is None or not self._update_thread.is_alive():
                self._update_thread = threading.Thread(
                    target=self._update_loop,
                    daemon=True
                )
                self._update_thread.start()
                logger.debug("Update loop started")
            
            # Give server a moment to fully start
            time.sleep(0.5)
                
        except Exception as e:
            logger.error(f"Failed to initialize Viser: {e}", exc_info=True)
            raise
    
    def _update_loop(self):
        """Update loop for robot visualization."""
        update_period = 1.0 / self.config.update_rate
        last_joint_state_log = 0.0
        last_fsm_panel_update = 0.0
        last_gripper_panel_update = 0.0
        log_interval = 5.0  # Log joint state status every 5 seconds
        fsm_panel_update_interval = 0.1  # Update FSM panel every 100ms
        gripper_panel_update_interval = 0.1  # Update Gripper panel every 100ms
        
        while self._running:
            current_time = time.time()
            
            # Check if ROS2 interface needs reconnection (triggered by URDF change)
            with self._reconnect_lock:
                needs_reconnect = self._needs_reconnect
                if needs_reconnect:
                    self._needs_reconnect = False  # Clear flag
            
            if needs_reconnect and self._own_interface and self.ros2_interface is not None:
                logger.info("Reconnecting ROS2RobotInterface to detect new controllers/topics...")
                try:
                    # Disconnect and reconnect to re-detect configuration
                    # This is safe to do in the update loop thread (not in ROS2 callback thread)
                    self.ros2_interface.disconnect()
                    self.ros2_interface.connect()
                    logger.info("✅ ROS2RobotInterface reconnected successfully")
                    
                    # Reinitialize robot description subscription (old subscription was destroyed)
                    self._init_robot_description_subscription()
                    
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
                            try:
                                # Ensure joint_positions is a proper numpy array with correct dtype
                                # Convert to numpy array and ensure all elements are numpy scalars
                                joint_positions_array = np.array(joint_positions, dtype=np.float64, copy=True)
                                
                                # Ensure array is 1D and contiguous
                                if joint_positions_array.ndim != 1:
                                    joint_positions_array = joint_positions_array.flatten()
                                
                                # Double-check: ensure all elements are numpy float64 scalars
                                # This is critical - ViserUrdf.update_cfg() may call .item() on elements
                                joint_positions_array = np.array([np.float64(x) for x in joint_positions_array], dtype=np.float64)
                                
                                self.urdf_vis.update_cfg(joint_positions_array)
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
        
        This function handles:
        - Actuated joints: get value from joint state
        - Mimic joints: compute from mimicked joint using multiplier and offset (recursively)
        - Fixed joints: use 0.0
        
        Args:
            joint_names: Joint names from ROS2 topic
            positions: Joint positions from ROS2 topic
            
        Returns:
            Array of joint positions for ALL joints in URDF order, or None if mapping fails.
        """
        if not self.urdf_all_joint_names:
            return None
        
        # Create mapping from ROS2 joint state (convert to numpy float64)
        joint_dict = {}
        for name, pos in zip(joint_names, positions):
            # Ensure positions are numpy float64
            joint_dict[name] = np.float64(pos)
        
        # First pass: collect actuated joint values
        actuated_values = {}
        for joint_name in self.urdf_actuated_joint_names:
            if joint_name in joint_dict:
                actuated_values[joint_name] = joint_dict[joint_name]
            else:
                # Joint not found, use 0.0 as default
                logger.debug(f"Actuated joint {joint_name} not found in joint state, using 0.0")
                actuated_values[joint_name] = np.float64(0.0)
        
        # Second pass: compute all joint values (actuated, mimic, fixed)
        all_joint_values = {}
        for joint_name in self.urdf_all_joint_names:
            if joint_name in self.mimic_joint_info:
                # Mimic joint: compute from directly mimicked joint
                mimicked_joint, multiplier, offset = self.mimic_joint_info[joint_name]
                
                if mimicked_joint in actuated_values:
                    # Compute value from mimicked actuated joint
                    # Ensure all operations use numpy types
                    base_value = np.float64(actuated_values[mimicked_joint])
                    mult = np.float64(multiplier)
                    off = np.float64(offset)
                    value = base_value * mult + off
                    all_joint_values[joint_name] = np.float64(value)
                else:
                    logger.debug(f"Mimic joint {joint_name} references non-actuated joint {mimicked_joint}, using 0.0")
                    all_joint_values[joint_name] = np.float64(0.0)
            elif joint_name in actuated_values:
                # Actuated joint: use value from joint state
                all_joint_values[joint_name] = actuated_values[joint_name]
            else:
                # Fixed joint or unknown: use 0.0
                all_joint_values[joint_name] = np.float64(0.0)
        
        # Map to URDF joint order (ALL joints)
        # Create list and convert to numpy array in one step
        # This ensures all values are properly converted to numpy float64
        mapped_positions = [np.float64(all_joint_values[joint_name]) for joint_name in self.urdf_all_joint_names]
        
        # Convert to numpy array - this should preserve numpy float64 types
        # Use np.fromiter for better type preservation, or np.array with explicit dtype
        result = np.fromiter(mapped_positions, dtype=np.float64, count=len(mapped_positions))
        
        return result
    
    def start(self):
        """Start the visualizer."""
        if self._running:
            logger.warning("Visualizer is already running")
            return
        
        self._running = True
        
        # Initialize ROS2 Robot Interface (this creates the node)
        self._init_ros2_interface()
        
        # Wait for URDF
        logger.info("Waiting for robot description from topic...")
        max_wait_time = 30.0  # seconds
        start_time = time.time()
        check_interval = 2.0  # Print status every 2 seconds
        
        last_status_time = start_time
        while not self._urdf_received and (time.time() - start_time) < max_wait_time:
            # Check with lock to ensure we see the latest value
            with self._urdf_lock:
                urdf_received = self._urdf_received
            
            if urdf_received:
                break
                
            elapsed = time.time() - start_time
            if time.time() - last_status_time >= check_interval:
                logger.info(f"Still waiting for robot description... ({elapsed:.1f}s / {max_wait_time:.1f}s)")
                last_status_time = time.time()
            time.sleep(0.1)
        
        # Final check with lock
        with self._urdf_lock:
            urdf_received = self._urdf_received
        
        if not urdf_received:
            logger.error(f"Timeout after {max_wait_time}s waiting for robot description")
            logger.error("Please check:")
            logger.error("  1. Is /robot_description topic being published?")
            logger.error("  2. Is robot_state_publisher node running?")
            logger.error("  3. Try running: ros2 topic echo /robot_description")
            raise TimeoutError("Failed to receive robot description from ROS2 topic")
        
        logger.info("✅ Visualizer started successfully")
    
    def stop(self):
        """Stop the visualizer and cleanup resources."""
        self._running = False
        
        # Wait for update thread
        if self._update_thread is not None:
            self._update_thread.join(timeout=2.0)
        
        # Cleanup ROS2 Robot Interface
        if self.ros2_interface is not None and self._own_interface:
            try:
                self.ros2_interface.disconnect()
            except Exception as e:
                logger.warning(f"Error disconnecting ROS2 interface: {e}")
        
        # Cleanup panels
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
