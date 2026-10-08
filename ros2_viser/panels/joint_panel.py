"""Joint Control Panel for ROS2 Viser."""

import logging
import math
import time
from typing import Optional, Dict, List, Any
from io import StringIO

import viser
import yourdfpy

from ros2_robot_interface import ROS2RobotInterface

from ..i18n import get_translator

logger = logging.getLogger(__name__)


class JointPanel:
    """Joint control panel for robot joint management.
    
    This panel provides a GUI interface for controlling robot joints by category
    (body, head, left, right, left_hand, right_hand). It supports both OCS2 mode
    (end-effector pose control for left/right arms) and MOVEJ mode (joint position control).
    
    Example:
        ```python
        panel = JointPanel(server, ros2_interface)
        panel.initialize()
        
        # In update loop:
        panel.update()
        ```
    """
    
    def __init__(
        self,
        server: viser.ViserServer,
        ros2_interface: ROS2RobotInterface,
        urdf: Optional[Any] = None,
    ):
        """Initialize Joint panel.
        
        Args:
            server: Viser server instance for GUI creation.
            ros2_interface: ROS2RobotInterface for sending commands and getting joint state.
            urdf: Optional pre-parsed URDF object (yourdfpy.URDF) to avoid re-parsing.
        """
        self.server = server
        self.ros2_interface = ros2_interface
        self.urdf = urdf  # Pre-parsed URDF from visualizer
        self.translator = get_translator()
        
        # FSM state tracking
        self._current_fsm_state: int = 2
        self._is_joint_control_enabled: bool = False  # Enabled when state is OCS2 or MOVEJ
        
        # Joint state tracking
        self._joint_names: List[str] = []
        self._joint_positions: Dict[str, float] = {}
        self._joints_initialized: bool = False
        
        # Category mapping
        self._category_to_joints: Dict[str, List[str]] = {}
        self._joint_to_category: Dict[str, str] = {}
        
        # GUI elements
        self._folder_handle: Optional[viser.GuiFolderHandle] = None
        self._category_dropdown: Optional[viser.GuiDropdownHandle] = None
        self._status_text: Optional[viser.GuiTextHandle] = None
        self._send_button: Optional[viser.GuiButtonHandle] = None
        self._body_link3_pose_html: Optional[viser.GuiHtmlHandle] = None

        # Waist control UI handles
        self._waist_enabled: bool = False
        self._waist_command_enabled: bool = False
        self._waist_turning_command_enabled: bool = False
        self._waist_phi_command_enabled: bool = False
        self._waist_pose_relative_enabled: bool = False
        self._waist_pose_absolute_enabled: bool = False
        self._waist_folder = None
        self._waist_lifting_slider = None
        self._waist_speed_slider = None
        self._waist_turn_speed_slider = None
        self._waist_phi_speed_slider = None
        self._waist_pose_x_slider = None
        self._waist_pose_z_slider = None
        self._waist_pose_phi_slider = None
        self._waist_pose_relative_button = None
        self._waist_pose_absolute_button = None
        self._waist_action_group = None
        self._waist_hold_up_button = None
        self._waist_hold_down_button = None
        self._waist_hold_turn_left_button = None
        self._waist_hold_phi_forward_button = None
        self._waist_hold_turn_right_button = None
        self._waist_hold_phi_backward_button = None
        self._waist_hold_active: bool = False
        self._waist_turn_hold_active: bool = False
        self._waist_phi_hold_active: bool = False
        self._waist_last_hold_callback_time: float = 0.0
        self._waist_turn_last_hold_callback_time: float = 0.0
        self._waist_phi_last_hold_callback_time: float = 0.0
        self._waist_hold_release_timeout_s: float = 0.15
        # Cached button labels (for button_group / hold dispatch)
        self._waist_label_step_up: str = ""
        self._waist_label_step_down: str = ""
        self._waist_label_hold_up: str = ""
        self._waist_label_hold_down: str = ""
        
        # Joint control GUI elements
        self._joint_controls: Dict[str, Dict] = {}  # joint_name -> {slider, label, etc}
        
        # Left/Right arm pose controls (for OCS2 mode)
        self._left_arm_controls: Dict[str, viser.GuiSliderHandle] = {}
        self._right_arm_controls: Dict[str, viser.GuiSliderHandle] = {}
        
        # Track previous target poses to detect changes
        self._last_left_target_pose: Optional[Dict[str, float]] = None
        self._last_right_target_pose: Optional[Dict[str, float]] = None
        self._last_body_current_target: Optional[List[float]] = None
        
        # Joint limits cache (from URDF)
        self._joint_limits: Dict[str, Dict[str, float]] = {}  # joint_name -> {'lower': float, 'upper': float}
        self._joint_limits_initialized: bool = False
        

        
        # Initialization flag
        self._initialized = False
        
        # Cleanup flag
        self._cleaned_up = False
        
        # Current category (will be set to first available category during initialization)
        self._current_category: str = ""
    
    def initialize(self):
        """Initialize the Joint panel GUI and ROS2 subscriptions."""
        if self._initialized:
            logger.warning("Joint panel already initialized, cleaning up first...")
            self.cleanup()
        
        try:
            self._cleaned_up = False
            
            # Initialize ROS2 subscriptions
            self._init_subscriptions()
            
            # Initialize GUI
            self._init_gui()
            
            # If ros2_interface is already connected, get actual FSM state instead of using default
            if self.ros2_interface is not None and self.ros2_interface.is_connected:
                try:
                    actual_state = self.ros2_interface.get_fsm_state()
                    self._current_fsm_state = actual_state
                    self._is_joint_control_enabled = (actual_state in (3, 4))
                    logger.debug(f"Initialized with actual FSM state: {actual_state}")
                except Exception as e:
                    logger.debug(f"Could not get initial FSM state from ros2_interface: {e}")
            
            # Initial update
            self.update()
            
            self._initialized = True
            logger.debug("✅ Joint control panel initialized")
            
        except Exception as e:
            logger.error(f"Failed to initialize Joint panel: {e}", exc_info=True)
            raise
    
    def _init_subscriptions(self):
        """Initialize ROS2 subscriptions."""
        if self.ros2_interface is None or not self.ros2_interface.is_connected:
            logger.warning("ROS2RobotInterface not connected, cannot subscribe")
            return
        
        ros2_node = self.ros2_interface.robot_node
        if ros2_node is None:
            logger.warning("ROS2RobotInterface node not available")
            return
        
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        
        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST
        )
        
        # Note: We don't subscribe to topics directly here
        # Instead, we use ros2_interface methods to avoid duplicate subscriptions:
        # - FSM state: will be tracked via fsm_panel or through interface (if available)
        # - Joint state: use ros2_interface.get_joint_state() in update() method
        # - Left/Right target: use ros2_interface.left_arm_handler.get_target_pose() and right_arm_handler.get_target_pose()
    
    def _update_fsm_state_from_interface(self) -> bool:
        """Update FSM state from ros2_interface (called from update loop, not callback).
        
        Gets FSM state from ros2_robot_interface to avoid duplicate subscription.
        
        Returns:
            True if FSM state changed, False otherwise.
        """
        if self.ros2_interface is None:
            return False
        
        try:
            # Get FSM state from interface
            state = self.ros2_interface.get_fsm_state()
            
            if state != self._current_fsm_state:
                self._current_fsm_state = state
                # Enable joint control when state is OCS2 or MOVEJ
                self._is_joint_control_enabled = (state in (3, 4))
                return True
            return False
        except Exception as e:
            logger.debug(f"Could not get FSM state from ros2_interface: {e}")
            return False
    
    def _update_joint_state_from_interface(self):
        """Update joint state from ros2_interface (called from update loop, not callback)."""
        if self._cleaned_up or self.ros2_interface is None:
            return
        
        try:
            # Get categorized joint state from interface (non-blocking)
            # This uses ros2_interface's built-in categorization logic
            categorized_joint_state = self.ros2_interface.get_joint_state(categorized=True)
            if categorized_joint_state is None:
                return
            
            # Check if we need to initialize joints (quick check)
            needs_init = False
            if not self._joints_initialized:
                needs_init = True
            
            # Initialize joints from categorized state (this will acquire lock internally for data modification)
            # But GUI operations are done outside lock to avoid blocking
            if needs_init:
                self._initialize_joints_from_categorized(categorized_joint_state)
            
            # Get control mode status
            is_control_mode = self._is_joint_control_enabled
            
            # Only update from joint state when NOT in control mode
            if not is_control_mode:
                # Update joint positions from categorized state (quick operation, minimal lock time)
                updates = {}  # Store updates
                if self._joints_initialized:
                    # Update positions from categorized joint state
                    for category, joint_data in categorized_joint_state.items():
                            if category == 'timestamp':
                                continue
                            names = joint_data.get('names', [])
                            positions = joint_data.get('positions', [])
                            for i, name in enumerate(names):
                                if name in self._joint_positions and i < len(positions):
                                    self._joint_positions[name] = positions[i]
                                    updates[name] = positions[i]
                
                # Update GUI sliders OUTSIDE lock to avoid blocking
                for name, position in updates.items():
                    if name in self._joint_controls:
                        control = self._joint_controls[name]
                        if 'slider' in control:
                            try:
                                control['slider'].value = position
                            except Exception:
                                pass  # Ignore errors during update
        except Exception as e:
            logger.error(f"Error updating joint state from interface: {e}", exc_info=True)

    def _refresh_current_category_joint_positions_once(self):
        """Refresh cached positions for the selected category before rebuilding controls."""
        if (
            self._cleaned_up
            or self.ros2_interface is None
            or not self._joints_initialized
            or not self._current_category
        ):
            return

        try:
            categorized_joint_state = self.ros2_interface.get_joint_state(categorized=True)
            if categorized_joint_state is None:
                return

            category_mapping = {
                'head': 'head',
                'body': 'body',
                'left_arm': 'left',
                'right_arm': 'right',
                'left_gripper': 'left_hand',
                'right_gripper': 'right_hand',
                'arm': 'left',
                'gripper': 'left_hand',
            }

            for ros2_category, joint_data in categorized_joint_state.items():
                if ros2_category == 'timestamp':
                    continue
                if category_mapping.get(ros2_category) != self._current_category:
                    continue

                names = joint_data.get('names', [])
                positions = joint_data.get('positions', [])
                for i, name in enumerate(names):
                    if name in self._joint_positions and i < len(positions):
                        self._joint_positions[name] = positions[i]
        except Exception as e:
            logger.debug(f"Could not refresh current category joint positions: {e}")
    
    def _update_target_poses_from_interface(self):
        """Update left/right target poses from ros2_interface (called from update loop, not callback).
        
        Only updates when FSM state is OCS2, as target poses are only relevant in OCS2 mode.
        """
        if self._cleaned_up or self.ros2_interface is None:
            return
        
        # Only update target poses in OCS2 mode
        current_state = self._current_fsm_state
        
        if current_state != 3:
            return
        
        try:
            # Update left arm target pose from interface
            if self.ros2_interface.left_arm_handler:
                target_pose = self.ros2_interface.left_arm_handler.get_target_pose()
                if target_pose and self._left_arm_controls:
                    # Extract pose values
                    current_pose = {
                        'x': target_pose.position.x,
                        'y': target_pose.position.y,
                        'z': target_pose.position.z,
                        'qx': target_pose.orientation.x,
                        'qy': target_pose.orientation.y,
                        'qz': target_pose.orientation.z,
                        'qw': target_pose.orientation.w
                    }
                    
                    # Only update if pose has changed
                    if self._last_left_target_pose != current_pose:
                        try:
                            if 'x' in self._left_arm_controls:
                                self._left_arm_controls['x'].value = current_pose['x']
                            if 'y' in self._left_arm_controls:
                                self._left_arm_controls['y'].value = current_pose['y']
                            if 'z' in self._left_arm_controls:
                                self._left_arm_controls['z'].value = current_pose['z']
                            if 'qx' in self._left_arm_controls:
                                self._left_arm_controls['qx'].value = current_pose['qx']
                            if 'qy' in self._left_arm_controls:
                                self._left_arm_controls['qy'].value = current_pose['qy']
                            if 'qz' in self._left_arm_controls:
                                self._left_arm_controls['qz'].value = current_pose['qz']
                            if 'qw' in self._left_arm_controls:
                                self._left_arm_controls['qw'].value = current_pose['qw']
                            
                            # Update cached pose
                            self._last_left_target_pose = current_pose.copy()
                        except Exception:
                            pass  # Ignore errors during update
            
            # Update right arm target pose from interface
            if self.ros2_interface.right_arm_handler:
                target_pose = self.ros2_interface.right_arm_handler.get_target_pose()
                if target_pose and self._right_arm_controls:
                    # Extract pose values
                    current_pose = {
                        'x': target_pose.position.x,
                        'y': target_pose.position.y,
                        'z': target_pose.position.z,
                        'qx': target_pose.orientation.x,
                        'qy': target_pose.orientation.y,
                        'qz': target_pose.orientation.z,
                        'qw': target_pose.orientation.w
                    }
                    
                    # Only update if pose has changed
                    if self._last_right_target_pose != current_pose:
                        try:
                            if 'x' in self._right_arm_controls:
                                self._right_arm_controls['x'].value = current_pose['x']
                            if 'y' in self._right_arm_controls:
                                self._right_arm_controls['y'].value = current_pose['y']
                            if 'z' in self._right_arm_controls:
                                self._right_arm_controls['z'].value = current_pose['z']
                            if 'qx' in self._right_arm_controls:
                                self._right_arm_controls['qx'].value = current_pose['qx']
                            if 'qy' in self._right_arm_controls:
                                self._right_arm_controls['qy'].value = current_pose['qy']
                            if 'qz' in self._right_arm_controls:
                                self._right_arm_controls['qz'].value = current_pose['qz']
                            if 'qw' in self._right_arm_controls:
                                self._right_arm_controls['qw'].value = current_pose['qw']
                            
                            # Update cached pose
                            self._last_right_target_pose = current_pose.copy()
                        except Exception:
                            pass  # Ignore errors during update
        except Exception as e:
            logger.error(f"Error updating target poses from interface: {e}", exc_info=True)
    
    def _update_body_current_target_from_interface(self):
        """Update body current target from ros2_interface.

        This avoids duplicate subscriptions in JointPanel. The actual subscription
        is handled inside ROS2RobotInterface.
        """
        if self._cleaned_up or self.ros2_interface is None:
            return
        
        try:
            # Get latest body current target from interface
            body_current_target = self.ros2_interface.get_body_current_target()
            if body_current_target is None:
                return
            
            body_joint_names = self._category_to_joints.get("body", [])
            if not body_joint_names:
                return
            
            # Only update if target positions actually changed
            current_positions = list(body_current_target)
            if self._last_body_current_target == current_positions:
                return
            
            update_count = min(len(body_joint_names), len(current_positions))
            
            for i in range(update_count):
                joint_name = body_joint_names[i]
                target_value = current_positions[i]
                
                # Update cached joint position
                self._joint_positions[joint_name] = target_value
                
                # Update GUI slider if it exists
                if joint_name in self._joint_controls:
                    control = self._joint_controls[joint_name]
                    if 'slider' in control:
                        try:
                            control['slider'].value = target_value
                        except Exception:
                            pass  # Ignore GUI update errors
            
            self._last_body_current_target = current_positions.copy()
        except Exception as e:
            logger.error(f"Error updating body current target from interface: {e}", exc_info=True)
    
    def _get_available_categories(self) -> List[str]:
        """Get available joint categories based on ros2_interface configuration.
        
        Returns list of categories that have controllers available.
        """
        categories = []
        
        # Check which controllers are available
        if self.ros2_interface.head_joint_controller_pub is not None:
            categories.append('head')
        
        if self.ros2_interface.body_joint_controller_pub is not None:
            categories.append('body')
        
        if self.ros2_interface.left_arm_handler is not None:
            categories.append('left')
        
        if self.ros2_interface.right_arm_handler is not None:
            categories.append('right')
        
        if self.ros2_interface.left_hand_joint_controller_pub is not None:
            categories.append('left_hand')
        
        if self.ros2_interface.right_hand_joint_controller_pub is not None:
            categories.append('right_hand')
        
        return categories

    _CATEGORY_I18N_KEYS = {
        "body": "category_body",
        "head": "category_head",
        "left": "category_left",
        "right": "category_right",
        "left_hand": "category_left_hand",
        "right_hand": "category_right_hand",
    }

    def _categories_with_joints(self) -> List[str]:
        """Categories that have at least one joint in the current model."""
        available = self._get_available_categories()
        return [
            cat
            for cat in available
            if cat in self._category_to_joints and self._category_to_joints[cat]
        ]

    def _translated_category_label(self, category: str) -> str:
        key = self._CATEGORY_I18N_KEYS.get(category, category)
        return self.translator(key, category)

    def _set_current_category(self, category: str) -> None:
        """Switch joint category and refresh controls."""
        if not category or category == self._current_category:
            return

        self._current_category = category
        if self._category_dropdown is not None:
            label = self._translated_category_label(category)
            if label in self._category_dropdown.options:
                self._category_dropdown.value = label

        self._refresh_current_category_joint_positions_once()
        self._rebuild_joint_controls()

    def set_urdf(self, urdf: Any) -> None:
        """Set URDF object from visualizer (to avoid re-parsing).
        
        Args:
            urdf: Pre-parsed URDF object (yourdfpy.URDF) from visualizer.
        """
        self.urdf = urdf
        # Reload joint limits if URDF is set after initialization
        if self._joints_initialized and not self._joint_limits_initialized:
            self._load_joint_limits_from_urdf()
    
    def _load_joint_limits_from_urdf(self):
        """Load joint limits from URDF.
        
        Uses pre-parsed URDF from visualizer if available, otherwise parses from robot_description.
        """
        if self._joint_limits_initialized:
            return
        
        try:
            urdf_to_use = None
            
            # Prefer pre-parsed URDF from visualizer
            if self.urdf is not None:
                urdf_to_use = self.urdf
            elif self.ros2_interface is not None:
                # Fallback: parse from robot_description if URDF not provided
                robot_description = self.ros2_interface.get_robot_description()
                if robot_description:
                    urdf_io = StringIO(robot_description)
                    urdf_to_use = yourdfpy.URDF.load(urdf_io)
            
            if urdf_to_use is None:
                logger.debug("URDF not available yet, skipping joint limits loading")
                return
            
            # Extract joint limits
            for joint_name, joint in urdf_to_use.joint_map.items():
                if joint.limit is not None:
                    lower = joint.limit.lower if joint.limit.lower is not None else -6.28
                    upper = joint.limit.upper if joint.limit.upper is not None else 6.28
                    self._joint_limits[joint_name] = {
                        'lower': lower,
                        'upper': upper
                    }
                elif joint.type in ['revolute', 'prismatic']:
                    # For joints without explicit limits, use default values
                    # Continuous joints don't have limits
                    self._joint_limits[joint_name] = {
                        'lower': -6.28,
                        'upper': 6.28
                    }
            
            self._joint_limits_initialized = True
            logger.debug(f"Loaded joint limits for {len(self._joint_limits)} joints from URDF")
        except Exception as e:
            logger.warning(f"Failed to load joint limits from URDF: {e}")
            # Use default limits if URDF parsing fails
            self._joint_limits_initialized = True
    
    def _initialize_joints_from_categorized(self, categorized_joint_state: Dict[str, Any]):
        """Initialize joint data structures from categorized joint state.
        
        Uses ros2_interface's built-in categorization to determine which joints
        belong to which category (head, body, left_arm, right_arm, left_hand, right_hand).
        Only includes joints that have corresponding controllers available.
        
        This method modifies shared data structures, so it should be called
        with proper synchronization. GUI operations are done outside the lock
        to avoid blocking.
        """
        # Double-check pattern to avoid unnecessary work
        if self._joints_initialized:
            return
        
        # Prepare data structures (work done inside lock for thread safety)
        joint_names_list = []
        category_to_joints_dict = {}
        joint_to_category_dict = {}
        joint_positions_dict = {}
        
        # Map ros2_interface categories to joint_panel categories
        # ros2_interface uses: 'head', 'body', 'left_arm', 'right_arm', 'left_gripper', 'right_gripper', 'arm', 'gripper', 'other'
        # joint_panel uses: 'head', 'body', 'left', 'right', 'left_hand', 'right_hand'
        category_mapping = {
            'head': 'head',
            'body': 'body',
            'left_arm': 'left',
            'right_arm': 'right',
            'left_gripper': 'left_hand',  # Gripper joints are treated as hand joints
            'right_gripper': 'right_hand',
            'arm': 'left',  # Single-arm mode
            'gripper': 'left_hand',  # Single-arm mode gripper
        }
        
        # Get available categories based on controllers
        available_categories = set(self._get_available_categories())
        
        # Process categorized joints from ros2_interface
        for ros2_category, joint_data in categorized_joint_state.items():
            if ros2_category == 'timestamp':
                continue
            
            # Map to joint_panel category
            panel_category = category_mapping.get(ros2_category)
            if panel_category is None:
                continue  # Skip 'other' category
            
            # Only include categories that have controllers available
            if panel_category not in available_categories:
                continue
            
            joint_names = joint_data.get('names', [])
            positions = joint_data.get('positions', [])
            
            # Add joints to data structures
            for i, joint_name in enumerate(joint_names):
                joint_names_list.append(joint_name)
                joint_to_category_dict[joint_name] = panel_category
                if panel_category not in category_to_joints_dict:
                    category_to_joints_dict[panel_category] = []
                category_to_joints_dict[panel_category].append(joint_name)
                # Initialize position from categorized state if available
                if i < len(positions):
                    joint_positions_dict[joint_name] = positions[i]
                else:
                    joint_positions_dict[joint_name] = 0.0
        
        # Update shared data structures (quick operation)
        self._joint_names = joint_names_list
        self._category_to_joints = category_to_joints_dict
        self._joint_to_category = joint_to_category_dict
        self._joint_positions = joint_positions_dict
        self._joints_initialized = True
        
        # Log and rebuild GUI OUTSIDE lock to avoid blocking
        logger.debug(f"Initialized {len(self._joint_names)} joints for control")
        for cat, joints in self._category_to_joints.items():
            logger.debug(f"  {cat}: {len(joints)} joints")
        
        # Load joint limits from URDF (outside lock to avoid blocking)
        if not self._joint_limits_initialized:
            self._load_joint_limits_from_urdf()
        
        # Rebuild GUI after joints initialized (outside lock to avoid blocking)
        if self._initialized:
            self._rebuild_joint_controls()
    
    def _init_gui(self):
        """Initialize Joint control panel GUI elements."""
        if self.server is None:
            return
        
        try:
            self._folder_handle = self.server.gui.add_folder(self.translator("joint_control"))
            
            with self._folder_handle:
                # Category selection dropdown
                # Get available categories and set initial category
                available_categories = self._get_available_categories()
                if not available_categories:
                    logger.warning("No joint categories available")
                    return
                
                # Set initial category to first available
                self._current_category = available_categories[0]
                
                # Build category options with translations
                category_map = {
                    'body': 'category_body',
                    'head': 'category_head',
                    'left': 'category_left',
                    'right': 'category_right',
                    'left_hand': 'category_left_hand',
                    'right_hand': 'category_right_hand'
                }
                category_options = []
                for cat in available_categories:
                    trans_key = category_map.get(cat, cat)
                    category_options.append(self.translator(trans_key, cat))
                
                self._category_dropdown = self.server.gui.add_dropdown(
                    self.translator("joint_category"),
                    options=category_options,
                    initial_value=category_options[0]
                )
                self._category_dropdown.on_update(lambda _: self._on_category_changed())
                
                # Status text
                self._status_text = self.server.gui.add_text(
                    self.translator("status"),
                    initial_value=self.translator("waiting_for_joints"),
                    disabled=True
                )
                
                # Send button will be created in _rebuild_joint_controls() to appear at the bottom
                
                # Waist control folder (only visible in body category)
                self._create_waist_controls()
        except Exception as e:
            logger.error(f"Failed to initialize Joint panel GUI: {e}", exc_info=True)
            raise
    
    def _refresh_waist_enabled_from_interface(self):
        """Query waist_lifting_enabled and velocity command availability."""
        if self._cleaned_up or self.ros2_interface is None:
            return

        self._waist_command_enabled = (
            getattr(self.ros2_interface, "waist_lifting_command_pub", None) is not None
        )
        self._waist_turning_command_enabled = (
            getattr(self.ros2_interface, "waist_turning_command_pub", None) is not None
        )
        self._waist_phi_command_enabled = (
            getattr(self.ros2_interface, "waist_phi_command_pub", None) is not None
        )
        self._waist_pose_relative_enabled = (
            getattr(self.ros2_interface, "waist_lifting_pose_relative_pub", None) is not None
        )
        self._waist_pose_absolute_enabled = (
            getattr(self.ros2_interface, "waist_lifting_pose_absolute_pub", None) is not None
        )

        try:
            # Prefer discovered body controller node (split body_joint_controller or WBC).
            body_node = getattr(self.ros2_interface, "body_controller", "") or ""
            candidate_nodes = []
            if body_node:
                candidate_nodes.append(body_node if body_node.startswith("/") else f"/{body_node}")
            # Fallbacks for older stacks / incomplete discovery
            for fallback in ("/body_joint_controller", "/ocs2_wbc_controller"):
                if fallback not in candidate_nodes:
                    candidate_nodes.append(fallback)

            enabled = False
            queried_node = None
            for node_name in candidate_nodes:
                try:
                    params = self.ros2_interface.list_node_parameters(node_name)
                except Exception:
                    continue
                for param in params:
                    if param.get("name") == "waist_lifting_enabled":
                        enabled = bool(param.get("value"))
                        queried_node = node_name
                        break
                if queried_node is not None:
                    break

            self._waist_enabled = enabled
            logger.info(
                "Waist control enabled=%s (node=%s), lift command=%s, turn command=%s, "
                "pose relative=%s, pose absolute=%s",
                self._waist_enabled,
                queried_node,
                self._waist_command_enabled,
                self._waist_turning_command_enabled,
                self._waist_pose_relative_enabled,
                self._waist_pose_absolute_enabled,
            )
        except Exception as e:
            logger.warning(f"Failed to query waist_lifting_enabled: {e}")
            self._waist_enabled = False

    def _create_waist_controls(self):
        """Create waist control UI (step distance + hold velocity)."""
        self._refresh_waist_enabled_from_interface()
        if not self._waist_enabled:
            return
        if self._folder_handle is None:
            return

        if self._waist_phi_hold_active:
            self._stop_waist_phi_velocity()

        # Remove old waist UI first to avoid duplicates when rebuilding labels
        if self._waist_folder is not None:
            try:
                self._waist_folder.remove()
            except Exception:
                pass

        # Reset handles
        self._waist_folder = None
        self._body_link3_pose_html = None
        self._waist_lifting_slider = None
        self._waist_speed_slider = None
        self._waist_turn_speed_slider = None
        self._waist_phi_speed_slider = None
        self._waist_pose_x_slider = None
        self._waist_pose_z_slider = None
        self._waist_pose_phi_slider = None
        self._waist_pose_relative_button = None
        self._waist_pose_absolute_button = None
        self._waist_action_group = None
        self._waist_hold_up_button = None
        self._waist_hold_down_button = None
        self._waist_hold_turn_left_button = None
        self._waist_hold_phi_forward_button = None
        self._waist_hold_turn_right_button = None
        self._waist_hold_phi_backward_button = None
        self._waist_hold_active = False
        self._waist_turn_hold_active = False
        self._waist_phi_hold_active = False

        self._waist_label_step_up = self.translator("waist_step_up")
        self._waist_label_step_down = self.translator("waist_step_down")
        self._waist_label_hold_up = self.translator("waist_hold_up")
        self._waist_label_hold_down = self.translator("waist_hold_down")

        # Step distance in button group; velocity hold on on_hold buttons (same folder).
        group_options = [self._waist_label_step_up, self._waist_label_step_down]

        self._waist_folder = self.server.gui.add_folder(self.translator("waist_control"))

        with self._waist_folder:
            if self._waist_pose_relative_enabled or self._waist_pose_absolute_enabled:
                self._waist_pose_x_slider = self.server.gui.add_slider(
                    self.translator("waist_pose_x"),
                    min=-0.5,
                    max=0.5,
                    step=0.01,
                    initial_value=0.0,
                )
                self._waist_pose_z_slider = self.server.gui.add_slider(
                    self.translator("waist_pose_z"),
                    min=-0.5,
                    max=1.2,
                    step=0.01,
                    initial_value=0.0,
                )
                self._waist_pose_phi_slider = self.server.gui.add_slider(
                    self.translator("waist_pose_phi"),
                    min=-1.3,
                    max=1.3,
                    step=0.01,
                    initial_value=0.0,
                )

            self._create_body_link3_pose_display(
                parent=self._waist_folder,
                enter_context=False,
            )

            if self._waist_pose_relative_enabled:
                self._waist_pose_relative_button = self.server.gui.add_button(
                    self.translator("waist_send_pose_relative"),
                    color="green",
                )
                self._waist_pose_relative_button.on_click(
                    self._on_waist_pose_relative_clicked
                )

            if self._waist_pose_absolute_enabled:
                self._waist_pose_absolute_button = self.server.gui.add_button(
                    self.translator("waist_send_pose_absolute"),
                    color="blue",
                )
                self._waist_pose_absolute_button.on_click(
                    self._on_waist_pose_absolute_clicked
                )

            self._waist_lifting_slider = self.server.gui.add_slider(
                self.translator("waist_lifting_distance"),
                min=0.0,
                max=0.5,
                step=0.01,
                initial_value=0.1,
            )

            self._waist_action_group = self.server.gui.add_button_group(
                self.translator("waist_actions"),
                options=group_options,
            )
            self._waist_action_group.on_click(self._on_waist_action_group_click)

            # Viser button_group only supports on_click (step). Hold uses on_hold on
            # companion buttons with matching labels (rendered below the group).
            if self._waist_command_enabled:
                self._waist_speed_slider = self.server.gui.add_slider(
                    self.translator("waist_lifting_ratio"),
                    min=0.05,
                    max=1.0,
                    step=0.05,
                    initial_value=0.3,
                )

                self._waist_hold_up_button = self.server.gui.add_button(
                    self._waist_label_hold_up,
                    color="green",
                )
                self._waist_hold_up_button.on_hold(callback_hz=10.0)(
                    lambda _: self._on_waist_hold_tick(1.0)
                )

                self._waist_hold_down_button = self.server.gui.add_button(
                    self._waist_label_hold_down,
                    color="green",
                )
                self._waist_hold_down_button.on_hold(callback_hz=10.0)(
                    lambda _: self._on_waist_hold_tick(-1.0)
                )

            if self._waist_turning_command_enabled:
                self._waist_turn_speed_slider = self.server.gui.add_slider(
                    self.translator("waist_turning_ratio"),
                    min=0.05,
                    max=1.0,
                    step=0.05,
                    initial_value=0.3,
                )

                self._waist_hold_turn_left_button = self.server.gui.add_button(
                    self.translator("waist_hold_turn_left"),
                    color="blue",
                )
                self._waist_hold_turn_left_button.on_hold(callback_hz=10.0)(
                    lambda _: self._on_waist_turn_hold_tick(1.0)
                )

                self._waist_hold_turn_right_button = self.server.gui.add_button(
                    self.translator("waist_hold_turn_right"),
                    color="blue",
                )
                self._waist_hold_turn_right_button.on_hold(callback_hz=10.0)(
                    lambda _: self._on_waist_turn_hold_tick(-1.0)
                )

            if self._waist_phi_command_enabled:
                self._waist_phi_speed_slider = self.server.gui.add_slider(
                    self.translator("waist_phi_ratio"),
                    min=0.0,
                    max=1.0,
                    step=0.01,
                    initial_value=0.5,
                )

                self._waist_hold_phi_forward_button = self.server.gui.add_button(
                    self.translator("waist_hold_phi_forward"),
                    color="orange",
                )
                self._waist_hold_phi_forward_button.on_hold(callback_hz=10.0)(
                    lambda _: self._on_waist_phi_hold_tick(1.0)
                )

                self._waist_hold_phi_backward_button = self.server.gui.add_button(
                    self.translator("waist_hold_phi_backward"),
                    color="orange",
                )
                self._waist_hold_phi_backward_button.on_hold(callback_hz=10.0)(
                    lambda _: self._on_waist_phi_hold_tick(-1.0)
                )

        self._update_waist_visibility()

    def _rebuild_joint_controls(self):
        """Rebuild joint control GUI elements based on current category."""
        if not self._joints_initialized or self._folder_handle is None:
            return
        
        # Clear existing controls
        for control in self._joint_controls.values():
            if 'slider' in control:
                try:
                    control['slider'].remove()
                except Exception:
                    pass
        
        self._joint_controls.clear()
        
        # Clear left/right arm controls
        for control in self._left_arm_controls.values():
            try:
                control.remove()
            except Exception:
                pass
        self._left_arm_controls.clear()
        
        for control in self._right_arm_controls.values():
            try:
                control.remove()
            except Exception:
                pass
        self._right_arm_controls.clear()
        
        # Clear send button (will be recreated at the bottom)
        if self._send_button is not None:
            try:
                self._send_button.remove()
            except Exception:
                pass
            self._send_button = None

        # Get joints for current category
        current_state = self._current_fsm_state
        
        # Get joints to show
        joints_to_show = list(self._category_to_joints.get(self._current_category, []))  # Copy
        
        # For left/right category:
        # - OCS2 mode: show pose controls
        # - MOVEJ mode: show joint controls
        if current_state == 3 and self._current_category == "left":
            self._create_left_arm_pose_controls()
        elif current_state == 3 and self._current_category == "right":
            self._create_right_arm_pose_controls()
        else:
            # Create joint sliders (for all categories including left/right in MOVEJ mode)
            for joint_name in joints_to_show:
                self._create_joint_control(joint_name)

        if self._current_category == "body" and self._body_link3_pose_html is None:
            self._create_body_link3_pose_display()

        # Create send button at the bottom (after all controls)
        if self._folder_handle is not None:
            with self._folder_handle:
                self._send_button = self.server.gui.add_button(
                    self.translator("send_joint_positions"),
                    color="green"
                )
                self._send_button.on_click(lambda _: self._on_send_button_clicked())
        
        # Update category dropdown options
        self._update_category_options()

    def _create_body_link3_pose_display(
        self,
        parent: Optional[Any] = None,
        enter_context: bool = True,
    ):
        """Create a read-only display for body_link3 pose in base_footprint."""
        container = parent or self._folder_handle
        if container is None:
            return

        if enter_context:
            with container:
                self._body_link3_pose_html = self.server.gui.add_html(
                    self._format_body_link3_message(
                        self.translator("body_link3_pose"),
                        self.translator("body_link3_pose_waiting")
                    )
                )
        else:
            self._body_link3_pose_html = self.server.gui.add_html(
                self._format_body_link3_message(
                    self.translator("body_link3_pose"),
                    self.translator("body_link3_pose_waiting")
                )
            )

    def _get_pose_control_initial_values(self, side: str) -> List[float]:
        """Get current target pose values for OCS2 pose controls."""
        initial_values = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
        if self.ros2_interface is None:
            return initial_values

        handler = (
            self.ros2_interface.left_arm_handler
            if side == "left"
            else self.ros2_interface.right_arm_handler
        )
        if handler is None:
            return initial_values

        try:
            target_pose = handler.get_target_pose()
        except Exception as e:
            logger.debug(f"Could not get {side} arm target pose for control init: {e}")
            target_pose = None

        # When Viser starts while the robot is already in OCS2 mode, the
        # current_target topic may not have arrived yet. The marker is initialized
        # from the current end-effector pose in that case, so use the same fallback
        # here to keep the sliders synchronized with the visible marker.
        if target_pose is None:
            try:
                target_pose = handler.get_pose()
            except Exception as e:
                logger.debug(f"Could not get {side} arm current pose for control init: {e}")
                target_pose = None

        if target_pose is None:
            return initial_values

        current_pose = {
            'x': target_pose.position.x,
            'y': target_pose.position.y,
            'z': target_pose.position.z,
            'qx': target_pose.orientation.x,
            'qy': target_pose.orientation.y,
            'qz': target_pose.orientation.z,
            'qw': target_pose.orientation.w,
        }

        if side == "left":
            self._last_left_target_pose = current_pose.copy()
        else:
            self._last_right_target_pose = current_pose.copy()

        return [
            current_pose['x'],
            current_pose['y'],
            current_pose['z'],
            current_pose['qx'],
            current_pose['qy'],
            current_pose['qz'],
            current_pose['qw'],
        ]

    def _create_left_arm_pose_controls(self):
        """Create pose controls for left arm (OCS2 mode)."""
        if self._folder_handle is None:
            return
        
        param_names = ['x', 'y', 'z', 'qx', 'qy', 'qz', 'qw']
        labels = ['X (m)', 'Y (m)', 'Z (m)', 'QX', 'QY', 'QZ', 'QW']
        initial_values = self._get_pose_control_initial_values("left")
        ranges = [(-2.0, 2.0), (-2.0, 2.0), (-2.0, 2.0),
                  (-1.0, 1.0), (-1.0, 1.0), (-1.0, 1.0), (-1.0, 1.0)]
        
        with self._folder_handle:
            for i, (param, label, init_val, (min_val, max_val)) in enumerate(
                zip(param_names, labels, initial_values, ranges)
            ):
                slider = self.server.gui.add_slider(
                    f"Left {label}",
                    min=min_val,
                    max=max_val,
                    step=0.01,
                    initial_value=init_val
                )
                self._left_arm_controls[param] = slider
    
    def _create_right_arm_pose_controls(self):
        """Create pose controls for right arm (OCS2 mode)."""
        if self._folder_handle is None:
            return
        
        param_names = ['x', 'y', 'z', 'qx', 'qy', 'qz', 'qw']
        labels = ['X (m)', 'Y (m)', 'Z (m)', 'QX', 'QY', 'QZ', 'QW']
        initial_values = self._get_pose_control_initial_values("right")
        ranges = [(-2.0, 2.0), (-2.0, 2.0), (-2.0, 2.0),
                  (-1.0, 1.0), (-1.0, 1.0), (-1.0, 1.0), (-1.0, 1.0)]
        
        with self._folder_handle:
            for i, (param, label, init_val, (min_val, max_val)) in enumerate(
                zip(param_names, labels, initial_values, ranges)
            ):
                slider = self.server.gui.add_slider(
                    f"Right {label}",
                    min=min_val,
                    max=max_val,
                    step=0.01,
                    initial_value=init_val
                )
                self._right_arm_controls[param] = slider
    
    def _create_joint_control(self, joint_name: str):
        """Create a slider control for a joint with limits from URDF."""
        if self._folder_handle is None:
            return
        
        # Get current position
        current_pos = self._joint_positions.get(joint_name, 0.0)
        
        # Get joint limits from URDF, fallback to default if not available
        if joint_name in self._joint_limits:
            min_val = self._joint_limits[joint_name]['lower']
            max_val = self._joint_limits[joint_name]['upper']
        else:
            # Default range: -2π to 2π (fallback if limits not loaded)
            min_val = -6.28
            max_val = 6.28
            logger.debug(f"Using default limits for joint {joint_name} (limits not found in URDF)")
        
        # Ensure current position is within limits
        current_pos = max(min_val, min(max_val, current_pos))
        
        with self._folder_handle:
            slider = self.server.gui.add_slider(
                joint_name,
                min=min_val,
                max=max_val,
                step=0.01,
                initial_value=current_pos
            )
            
            self._joint_controls[joint_name] = {
                'slider': slider,
                'joint_name': joint_name
            }
    
    def _update_category_options(self):
        """Update category dropdown options based on available controllers and joints."""
        if self._category_dropdown is None:
            return
        
        # Get available categories based on controllers
        available_categories = self._get_available_categories()
        
        # Build category options with translations
        category_map = {
            'body': 'category_body',
            'head': 'category_head',
            'left': 'category_left',
            'right': 'category_right',
            'left_hand': 'category_left_hand',
            'right_hand': 'category_right_hand'
        }
        options = []
        for cat in available_categories:
            # Only add if there are joints in this category
            if cat in self._category_to_joints and len(self._category_to_joints[cat]) > 0:
                trans_key = category_map.get(cat, cat)
                options.append(self.translator(trans_key, cat))
        
        # Update dropdown (if options changed)
        if options != self._category_dropdown.options:
            try:
                self._category_dropdown.options = options
            except Exception:
                # If can't update, recreate dropdown
                old_value = self._current_category
                old_dropdown = self._category_dropdown
                
                with self._folder_handle:
                    # Find the index of old_value in options
                    initial_index = 0
                    if old_value and old_value != "all":
                        category_map = {
                            'body': 'category_body',
                            'head': 'category_head',
                            'left': 'category_left',
                            'right': 'category_right',
                            'left_hand': 'category_left_hand',
                            'right_hand': 'category_right_hand'
                        }
                        trans_key = category_map.get(old_value, old_value)
                        old_translated = self.translator(trans_key, old_value)
                        if old_translated in options:
                            initial_index = options.index(old_translated)
                    
                    self._category_dropdown = self.server.gui.add_dropdown(
                        self.translator("joint_category"),
                        options=options,
                        initial_value=options[initial_index] if options else options[0]
                    )
                    self._category_dropdown.on_update(lambda _: self._on_category_changed())
                
                try:
                    old_dropdown.remove()
                except Exception:
                    pass
    
    def _on_category_changed(self):
        """Handle category dropdown change."""
        if self._category_dropdown is None:
            return
        
        selected = self._category_dropdown.value
        
        for cat in self._categories_with_joints():
            if self._translated_category_label(cat) == selected:
                self._set_current_category(cat)
                break

        self.update()
    
    def _on_send_button_clicked(self):
        """Handle send button click."""
        if not self._is_joint_control_enabled or not self._joints_initialized:
            return
        
        try:
            self._publish_joint_positions()
        except Exception as e:
            logger.error(f"Failed to publish joint positions: {e}", exc_info=True)
    
    def _publish_joint_positions(self):
        """Publish joint positions based on current category and mode."""
        if not self._joints_initialized:
            return
        
        current_state = self._current_fsm_state
        
        # Handle left arm in OCS2 mode
        if current_state == 3 and self._current_category == "left":
            if self.ros2_interface.left_arm_handler and self._left_arm_controls:
                from geometry_msgs.msg import Pose
                pose = Pose()
                pose.position.x = self._left_arm_controls.get('x').value if 'x' in self._left_arm_controls else 0.0
                pose.position.y = self._left_arm_controls.get('y').value if 'y' in self._left_arm_controls else 0.0
                pose.position.z = self._left_arm_controls.get('z').value if 'z' in self._left_arm_controls else 0.0
                pose.orientation.x = self._left_arm_controls.get('qx').value if 'qx' in self._left_arm_controls else 0.0
                pose.orientation.y = self._left_arm_controls.get('qy').value if 'qy' in self._left_arm_controls else 0.0
                pose.orientation.z = self._left_arm_controls.get('qz').value if 'qz' in self._left_arm_controls else 0.0
                pose.orientation.w = self._left_arm_controls.get('qw').value if 'qw' in self._left_arm_controls else 1.0
                
                # Get frame_id from current_target to keep synchronization
                frame_id = self.ros2_interface.left_arm_handler.get_frame_id()
                if frame_id:
                    self.ros2_interface.left_arm_handler.send_target_stamped(frame_id, pose)
                    logger.info(f"Published left arm end-effector target (OCS2 mode) in frame '{frame_id}'")
                else:
                    # Fallback to non-stamped if frame_id is not available
                    self.ros2_interface.send_end_effector_target(pose)
                    logger.info("Published left arm end-effector target (OCS2 mode, no frame_id available)")
            return
        
        # Handle right arm in OCS2 mode
        if current_state == 3 and self._current_category == "right":
            if self.ros2_interface.right_arm_handler and self._right_arm_controls:
                from geometry_msgs.msg import Pose
                pose = Pose()
                pose.position.x = self._right_arm_controls.get('x').value if 'x' in self._right_arm_controls else 0.0
                pose.position.y = self._right_arm_controls.get('y').value if 'y' in self._right_arm_controls else 0.0
                pose.position.z = self._right_arm_controls.get('z').value if 'z' in self._right_arm_controls else 0.0
                pose.orientation.x = self._right_arm_controls.get('qx').value if 'qx' in self._right_arm_controls else 0.0
                pose.orientation.y = self._right_arm_controls.get('qy').value if 'qy' in self._right_arm_controls else 0.0
                pose.orientation.z = self._right_arm_controls.get('qz').value if 'qz' in self._right_arm_controls else 0.0
                pose.orientation.w = self._right_arm_controls.get('qw').value if 'qw' in self._right_arm_controls else 1.0
                
                # Get frame_id from current_target to keep synchronization
                frame_id = self.ros2_interface.right_arm_handler.get_frame_id()
                if frame_id:
                    self.ros2_interface.right_arm_handler.send_target_stamped(frame_id, pose)
                    logger.info(f"Published right arm end-effector target (OCS2 mode) in frame '{frame_id}'")
                else:
                    # Fallback to non-stamped if frame_id is not available
                    self.ros2_interface.send_right_end_effector_target(pose)
                    logger.info("Published right arm end-effector target (OCS2 mode, no frame_id available)")
            return
        
        # Handle joint position control for other categories
        joint_names = list(self._category_to_joints.get(self._current_category, []))  # Copy
        
        if not joint_names:
            return
        
        # Collect positions from sliders
        positions = []
        for joint_name in joint_names:
            if joint_name in self._joint_controls:
                slider = self._joint_controls[joint_name]['slider']
                positions.append(slider.value)
            else:
                positions.append(self._joint_positions.get(joint_name, 0.0))
        
        # Publish based on category
        if self._current_category == "head":
            self.ros2_interface.send_head_joint_positions(positions)
        elif self._current_category == "body":
            self.ros2_interface.send_body_joint_positions(positions)
        elif self._current_category == "left_hand":
            self.ros2_interface.send_left_hand_joint_positions(positions)
        elif self._current_category == "right_hand":
            self.ros2_interface.send_right_hand_joint_positions(positions)
        elif self._current_category == "left":
            # For left arm in MOVEJ mode, use arm handler
            if self.ros2_interface.left_arm_handler:
                self.ros2_interface.left_arm_handler.send_joint_positions(positions)
                logger.info(f"Published {len(positions)} left arm joint positions (MOVEJ mode)")
            else:
                logger.warning("Left arm handler not available")
        elif self._current_category == "right":
            # For right arm in MOVEJ mode, use arm handler
            if self.ros2_interface.right_arm_handler:
                self.ros2_interface.right_arm_handler.send_joint_positions(positions)
                logger.info(f"Published {len(positions)} right arm joint positions (MOVEJ mode)")
            else:
                logger.warning("Right arm handler not available")
        
        if self._current_category not in ["left", "right"]:
            logger.info(f"Published {len(positions)} joint positions for category: {self._current_category}")
    
    def _on_waist_action_group_click(self, event: viser.GuiEvent):
        """Handle waist button group clicks (step distance commands)."""
        if self._cleaned_up or self.ros2_interface is None:
            return

        label = event.target.value
        distance = self._waist_lifting_slider.value if self._waist_lifting_slider is not None else 0.0

        if label == self._waist_label_step_up:
            self.ros2_interface.send_waist_lifting_pose_relative(0.0, distance, 0.0)
        elif label == self._waist_label_step_down:
            self.ros2_interface.send_waist_lifting_pose_relative(0.0, -distance, 0.0)

    def _get_waist_pose_values(self) -> tuple[float, float, float]:
        x = self._waist_pose_x_slider.value if self._waist_pose_x_slider is not None else 0.0
        z = self._waist_pose_z_slider.value if self._waist_pose_z_slider is not None else 0.0
        phi = self._waist_pose_phi_slider.value if self._waist_pose_phi_slider is not None else 0.0
        return x, z, phi

    def _on_waist_pose_relative_clicked(self, _):
        """Handle waist x/z/phi relative pose command."""
        if (
            self._cleaned_up
            or not self._waist_pose_relative_enabled
            or self.ros2_interface is None
        ):
            return

        try:
            x, z, phi = self._get_waist_pose_values()
            self.ros2_interface.send_waist_lifting_pose_relative(x, z, phi)
        except Exception as e:
            logger.warning(f"Failed to send waist relative pose: {e}")

    def _on_waist_pose_absolute_clicked(self, _):
        """Handle waist x/z/phi absolute pose command."""
        if (
            self._cleaned_up
            or not self._waist_pose_absolute_enabled
            or self.ros2_interface is None
        ):
            return

        try:
            x, z, phi = self._get_waist_pose_values()
            self.ros2_interface.send_waist_lifting_pose_absolute(x, z, phi)
        except Exception as e:
            logger.warning(f"Failed to send waist absolute pose: {e}")

    def _get_waist_velocity_scale(self, direction: float) -> float:
        speed = self._waist_speed_slider.value if self._waist_speed_slider is not None else 0.3
        return max(-1.0, min(1.0, direction * speed))

    def _get_waist_turn_velocity_scale(self, direction: float) -> float:
        speed = (
            self._waist_turn_speed_slider.value
            if self._waist_turn_speed_slider is not None
            else 0.3
        )
        return max(-1.0, min(1.0, direction * speed))

    def _get_waist_phi_velocity_scale(self, direction: float) -> float:
        speed = (
            self._waist_phi_speed_slider.value
            if self._waist_phi_speed_slider is not None
            else 0.5
        )
        return max(-1.0, min(1.0, direction * speed))

    def _on_waist_hold_tick(self, direction: float):
        """Called repeatedly while a lift hold button is pressed (on_hold)."""
        if self._cleaned_up or not self._waist_command_enabled or self.ros2_interface is None:
            return

        self._stop_waist_phi_velocity()
        self._stop_waist_turning_velocity()
        self._waist_last_hold_callback_time = time.time()
        self._waist_hold_active = True
        try:
            self.ros2_interface.send_waist_lifting_velocity_scale(
                self._get_waist_velocity_scale(direction)
            )
        except Exception as e:
            logger.warning(f"Failed to send waist lifting velocity: {e}")

    def _on_waist_turn_hold_tick(self, direction: float):
        """Called repeatedly while a turn hold button is pressed (on_hold)."""
        if (
            self._cleaned_up
            or not self._waist_turning_command_enabled
            or self.ros2_interface is None
        ):
            return

        self._stop_waist_phi_velocity()
        self._stop_waist_velocity()
        self._waist_turn_last_hold_callback_time = time.time()
        self._waist_turn_hold_active = True
        try:
            self.ros2_interface.send_waist_turning_velocity_scale(
                self._get_waist_turn_velocity_scale(direction)
            )
        except Exception as e:
            logger.warning(f"Failed to send waist turning velocity: {e}")

    def _on_waist_phi_hold_tick(self, direction: float):
        """Called repeatedly while a turn hold button is pressed (on_hold)."""
        if (
            self._cleaned_up
            or not self._waist_phi_command_enabled
            or self.ros2_interface is None
        ):
            return

        self._stop_waist_velocity()
        self._stop_waist_turning_velocity()
        self._waist_phi_last_hold_callback_time = time.time()
        self._waist_phi_hold_active = True
        try:
            self.ros2_interface.send_waist_phi_velocity_scale(
                self._get_waist_phi_velocity_scale(direction)
            )
        except Exception as e:
            logger.warning(f"Failed to send waist phi velocity: {e}")

    def _stop_waist_velocity(self):
        """Stop continuous waist lifting (velocity command)."""
        if not self._waist_command_enabled or self.ros2_interface is None:
            self._waist_hold_active = False
            return
        try:
            self.ros2_interface.send_waist_lifting_velocity_scale(0.0)
        except Exception as e:
            logger.debug(f"Failed to stop waist lifting velocity: {e}")
        self._waist_hold_active = False

    def _stop_waist_turning_velocity(self):
        """Stop continuous waist turning (velocity command)."""
        if not self._waist_turning_command_enabled or self.ros2_interface is None:
            self._waist_turn_hold_active = False
            return
        try:
            self.ros2_interface.send_waist_turning_velocity_scale(0.0)
        except Exception as e:
            logger.debug(f"Failed to stop waist turning velocity: {e}")
        self._waist_turn_hold_active = False

    def _stop_waist_phi_velocity(self):
        """Stop continuous waist phi (velocity command)."""
        if not self._waist_phi_command_enabled or self.ros2_interface is None:
            self._waist_phi_hold_active = False
            return
        try:
            self.ros2_interface.send_waist_phi_velocity_scale(0.0)
        except Exception as e:
            logger.debug(f"Failed to stop waist phi velocity: {e}")
        self._waist_phi_hold_active = False

    def _update_waist_hold_release(self):
        """Detect button release after on_hold callbacks stop."""
        now = time.time()
        if self._waist_hold_active:
            if now - self._waist_last_hold_callback_time > self._waist_hold_release_timeout_s:
                self._stop_waist_velocity()
        if self._waist_turn_hold_active:
            if (
                now - self._waist_turn_last_hold_callback_time
                > self._waist_hold_release_timeout_s
            ):
                self._stop_waist_turning_velocity()

        if self._waist_phi_hold_active:
            if (
                now - self._waist_phi_last_hold_callback_time
                > self._waist_hold_release_timeout_s
            ):
                self._stop_waist_phi_velocity()

    def _update_waist_visibility(self):
        if self._waist_folder is None:
            return

        controller = (
            getattr(self.ros2_interface, "body_controller", "") or ""
        ).lower()
        # Match RViz: split body permits OCS2/MOVEJ; WBC and other controllers
        # expose waist commands only in MOVEJ.
        allowed_states = (3, 4) if "body_joint_controller" in controller else (4,)
        visible = (
            self._current_category == "body"
            and self._is_joint_control_enabled
            and self._joints_initialized
            and self._waist_enabled
            and self._current_fsm_state in allowed_states
        )

        if not visible:
            if self._waist_hold_active:
                self._stop_waist_velocity()
            if self._waist_turn_hold_active:
                self._stop_waist_turning_velocity()
            if self._waist_phi_hold_active:
                self._stop_waist_phi_velocity()
        self._waist_folder.visible = visible

    @staticmethod
    def _format_body_link3_pose(
        transform,
        title: str = "body_link3 @ base_footprint",
    ) -> str:
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        sinp = 2.0 * (rotation.w * rotation.y - rotation.z * rotation.x)
        phi = math.asin(max(-1.0, min(1.0, sinp)))
        return (
            '<div style="font-family:SimHei,\'Microsoft YaHei\',Arial,sans-serif;'
            'font-weight:700;font-size:12px;line-height:1.2;color:#111;'
            'margin-top:6px;">'
            '<div style="margin-bottom:4px;text-align:center;color:#6b7280;'
            f'font-weight:600;">{title}</div>'
            '<div style="display:grid;grid-template-columns:repeat(3,1fr);'
            'gap:4px;text-align:center;">'
            f"{JointPanel._format_body_link3_cell('x', translation.x)}"
            f"{JointPanel._format_body_link3_cell('z', translation.z)}"
            f"{JointPanel._format_body_link3_cell('phi', phi)}"
            "</div>"
            "</div>"
        )

    @staticmethod
    def _format_body_link3_cell(label: str, value: float) -> str:
        return (
            '<div style="padding:2px 3px;">'
            f'<span style="display:block;font-size:11px;">{label}</span>'
            f'<span style="display:block;">{value:+.3f}</span>'
            "</div>"
        )

    @staticmethod
    def _format_body_link3_message(title: str, message: str) -> str:
        return (
            '<div style="font-family:SimHei,\'Microsoft YaHei\',Arial,sans-serif;'
            'font-weight:700;font-size:12px;color:#111;margin-top:6px;">'
            '<div style="margin-bottom:4px;text-align:center;color:#6b7280;'
            f'font-weight:600;">{title}</div>'
            f"{message}"
            "</div>"
        )

    def _update_body_link3_pose_display(self):
        """Update body_link3 pose text from TF."""
        if self._body_link3_pose_html is None:
            return
        if self._cleaned_up or self.ros2_interface is None:
            return

        visible = (
            self._current_category == "body"
            and self._is_joint_control_enabled
            and self._joints_initialized
        )
        self._body_link3_pose_html.visible = visible
        if not visible:
            return

        try:
            transform = self.ros2_interface.lookup_transform(
                "base_footprint",
                "body_link3",
            )
            if transform is None:
                self._body_link3_pose_html.content = self._format_body_link3_message(
                    self.translator("body_link3_pose"),
                    self.translator("body_link3_pose_unavailable")
                )
                return

            self._body_link3_pose_html.content = self._format_body_link3_pose(
                transform,
                self.translator("body_link3_pose"),
            )
        except Exception as e:
            logger.debug(f"Failed to update body_link3 pose display: {e}")
            self._body_link3_pose_html.content = self._format_body_link3_message(
                self.translator("body_link3_pose"),
                self.translator("body_link3_pose_unavailable")
            )

    def update(self):
        """Update Joint panel visibility and state.
        
        This method should be called periodically from the main update loop.
        """
        if not self._initialized or self.server is None:
            return
        
        try:
            # Update FSM state from interface (non-blocking, called from main loop)
            fsm_state_changed = self._update_fsm_state_from_interface()
            
            # Update joint state from interface when FSM state changes OR when joints are not initialized
            # This ensures joints are initialized even if FSM state doesn't change
            if fsm_state_changed or not self._joints_initialized:
                self._update_joint_state_from_interface()

            # Left/right arm controls depend on the FSM mode:
            # OCS2 shows end-effector pose controls, MOVEJ shows joint sliders.
            # When the mode changes after joints are already initialized, rebuild
            # the active category so the UI switches between those control types.
            if fsm_state_changed and self._joints_initialized:
                self._refresh_current_category_joint_positions_once()
                self._rebuild_joint_controls()

            # Update target poses from interface (for OCS2 mode)
            self._update_target_poses_from_interface()
            # Update body current target from interface
            self._update_body_current_target_from_interface()
            
            is_enabled = self._is_joint_control_enabled
            current_state = self._current_fsm_state
            
            # Update status text (only visible when control is not enabled)
            if self._status_text is not None:
                # Only show status text when control is not enabled
                self._status_text.visible = not is_enabled
                
                if not self._joints_initialized:
                    self._status_text.value = self.translator("waiting_for_joints")
                elif not is_enabled:
                    self._status_text.value = self.translator("switch_to_joint_control_state")
                else:
                    self._status_text.value = self.translator("ready")
            
            # Update send button visibility
            if self._send_button is not None:
                self._send_button.visible = is_enabled and self._joints_initialized
            
            # Update category dropdown visibility
            if self._category_dropdown is not None:
                self._category_dropdown.visible = is_enabled and self._joints_initialized
            
            # Update joint controls visibility
            for control in self._joint_controls.values():
                if 'slider' in control:
                    try:
                        control['slider'].visible = is_enabled and self._joints_initialized
                    except Exception:
                        pass
            
            # Update left/right arm controls visibility
            show_left = (is_enabled and self._joints_initialized and 
                        current_state == 3 and self._current_category == "left")
            for control in self._left_arm_controls.values():
                try:
                    control.visible = show_left
                except Exception:
                    pass
            
            show_right = (is_enabled and self._joints_initialized and 
                         current_state == 3 and self._current_category == "right")
            for control in self._right_arm_controls.values():
                try:
                    control.visible = show_right
                except Exception:
                    pass
            
            self._update_waist_visibility()
            self._update_body_link3_pose_display()
            self._update_waist_hold_release()

        except Exception as e:
            logger.warning(f"Failed to update Joint panel: {e}")
    
    def update_gui_labels(self):
        """Update GUI labels with current language."""
        if not self._initialized or self.server is None:
            return
        
        try:
            # Recreate folder with new language
            old_folder = self._folder_handle
            
            self._folder_handle = self.server.gui.add_folder(self.translator("joint_control"))
            
            # Recreate all GUI elements
            with self._folder_handle:
                # Get available categories
                available_categories = self._get_available_categories()
                if not available_categories:
                    return
                
                # Build category options with translations
                category_map = {
                    'body': 'category_body',
                    'head': 'category_head',
                    'left': 'category_left',
                    'right': 'category_right',
                    'left_hand': 'category_left_hand',
                    'right_hand': 'category_right_hand'
                }
                category_options = []
                for cat in available_categories:
                    trans_key = category_map.get(cat, cat)
                    category_options.append(self.translator(trans_key, cat))
                
                # Set initial category if not set
                if not self._current_category:
                    self._current_category = available_categories[0]
                
                # Find the index of current category in options
                current_trans_key = category_map.get(self._current_category, self._current_category)
                current_translated = self.translator(current_trans_key, self._current_category)
                initial_index = 0
                if current_translated in category_options:
                    initial_index = category_options.index(current_translated)
                
                self._category_dropdown = self.server.gui.add_dropdown(
                    self.translator("joint_category"),
                    options=category_options,
                    initial_value=category_options[initial_index] if category_options else category_options[0]
                )
                self._category_dropdown.on_update(lambda _: self._on_category_changed())
                
                self._status_text = self.server.gui.add_text(
                    self.translator("status"),
                    initial_value=self.translator("waiting_for_joints"),
                    disabled=True
                )
                
                # Send button will be created in _rebuild_joint_controls() to appear at the bottom
            
                # Recreate waist controls
                self._create_waist_controls()

            # Rebuild joint controls
            self._rebuild_joint_controls()
            
            # Remove old folder
            try:
                old_folder.remove()
            except Exception:
                pass
            
            logger.debug("Joint panel GUI labels updated")
        except Exception as e:
            logger.warning(f"Failed to update Joint panel GUI labels: {e}")
    
    def cleanup(self):
        """Cleanup resources."""
        self._stop_waist_phi_velocity()
        self._cleaned_up = True
        
        # Hide GUI elements
        try:
            if self._folder_handle is not None:
                try:
                    if hasattr(self._folder_handle, 'visible'):
                        self._folder_handle.visible = False
                    elif hasattr(self._folder_handle, 'remove'):
                        self._folder_handle.remove()
                except Exception:
                    pass
        except Exception as e:
            logger.warning(f"Error hiding Joint panel GUI elements: {e}")
        
        # Clear references
        self._folder_handle = None
        self._category_dropdown = None
        self._status_text = None
        self._send_button = None
        self._body_link3_pose_html = None
        # Clear waist UI references
        self._waist_enabled = False
        self._waist_command_enabled = False
        self._waist_turning_command_enabled = False
        self._waist_phi_command_enabled = False
        self._waist_pose_relative_enabled = False
        self._waist_pose_absolute_enabled = False
        self._waist_pose_x_slider = None
        self._waist_pose_z_slider = None
        self._waist_pose_phi_slider = None
        self._waist_pose_relative_button = None
        self._waist_pose_absolute_button = None
        self._waist_folder = None
        self._waist_lifting_slider = None
        self._waist_speed_slider = None
        self._waist_turn_speed_slider = None
        self._waist_phi_speed_slider = None
        self._waist_action_group = None
        self._waist_hold_up_button = None
        self._waist_hold_down_button = None
        self._waist_hold_turn_left_button = None
        self._waist_hold_phi_forward_button = None
        self._waist_hold_turn_right_button = None
        self._waist_hold_phi_backward_button = None
        self._waist_hold_active = False
        self._waist_turn_hold_active = False
        self._waist_phi_hold_active = False
        self._joint_controls.clear()
        self._left_arm_controls.clear()
        self._right_arm_controls.clear()
        
        # Reset cached target poses
        self._last_left_target_pose = None
        self._last_right_target_pose = None
        
        # Note: We don't have direct subscriptions anymore
        # All data is obtained through ros2_interface methods
        
        self._initialized = False
        logger.debug("Joint panel cleaned up")
