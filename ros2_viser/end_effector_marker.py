"""End-effector marker manager for draggable pose control."""

import logging
import time
from typing import Optional, Any, Tuple

import viser
import yourdfpy

from ros2_robot_interface import ROS2RobotInterface
from .config import ROS2ViserConfig
from .i18n import Translator

logger = logging.getLogger(__name__)


class EndEffectorMarkerManager:
    """Manages draggable end-effector markers for pose control.
    
    This class handles the creation, update, and interaction of draggable markers
    that allow users to control robot end-effector poses in OCS2 mode.
    
    Example:
        ```python
        manager = EndEffectorMarkerManager(
            server, ros2_interface, config, translator, urdf, root_node_name
        )
        manager.initialize()
        
        # In update loop:
        manager.update()
        ```
    """
    
    def __init__(
        self,
        server: viser.ViserServer,
        ros2_interface: ROS2RobotInterface,
        config: ROS2ViserConfig,
        translator: Translator,
        urdf: Optional[yourdfpy.URDF],
        root_node_name: str
    ):
        """Initialize the end-effector marker manager.
        
        Args:
            server: Viser server instance
            ros2_interface: ROS2 robot interface
            config: Visualizer configuration
            translator: Translator for i18n
            urdf: URDF model (optional, for link path resolution)
            root_node_name: Root node name for scene graph
        """
        self.server = server
        self.ros2_interface = ros2_interface
        self.config = config
        self.translator = translator
        self.urdf = urdf
        self.root_node_name = root_node_name
        
        self._left_ee_marker: Optional[viser.TransformControlsHandle] = None
        self._right_ee_marker: Optional[viser.TransformControlsHandle] = None
        self._ee_frame_node: Optional[viser.FrameHandle] = None
        
        self._last_left_ee_pose: Optional[Tuple] = None
        self._last_right_ee_pose: Optional[Tuple] = None
        self._left_frame_id: Optional[str] = None
        self._right_frame_id: Optional[str] = None
        self._left_is_urdf_link: bool = False
        self._right_is_urdf_link: bool = False
        self._left_urdf_frame_path: Optional[str] = None
        self._right_urdf_frame_path: Optional[str] = None
        self._marker_base_frame: Optional[str] = None
        
        self._last_fsm_mode: Optional[bool] = None
        self._marker_continuous_publish: bool = config.marker_continuous_publish
        self._pending_left_pose: Optional[Tuple] = None
        self._pending_right_pose: Optional[Tuple] = None
        self._last_left_target_pose: Optional[Any] = None
        self._last_right_target_pose: Optional[Any] = None
        self._last_marker_update_time: float = 0.0
        self._marker_update_cooldown: float = 0.5
        self._is_sending: bool = False  # Flag to prevent concurrent sends
        
        self._marker_publish_mode_dropdown: Optional[viser.GuiDropdownHandle] = None
        self._send_marker_pose_button: Optional[viser.GuiButtonHandle] = None
        
        self._initialized = False
    
    def initialize(self):
        """Initialize markers if ROS2 interface is ready."""
        if self.server is None or self.ros2_interface is None:
            logger.debug("Cannot initialize markers: server or interface is None")
            return
        
        if not self.ros2_interface.is_connected:
            logger.debug("ROS2 interface not connected, will initialize markers later")
            return
        
        try:
            if self.ros2_interface.left_arm_handler is not None:
                logger.debug("Initializing left arm marker...")
                self._init_single_ee_marker("left")
            else:
                logger.debug("Left arm handler is None, skipping left marker")
            
            if self.ros2_interface.right_arm_handler is not None:
                logger.debug("Initializing right arm marker...")
                self._init_single_ee_marker("right")
            else:
                logger.debug("Right arm handler is None, skipping right marker")
            
            self._initialized = True
        except Exception as e:
            logger.warning(f"Failed to initialize end-effector markers: {e}", exc_info=True)
    
    def set_gui_controls(
        self,
        publish_mode_dropdown: Optional[viser.GuiDropdownHandle] = None,
        send_button: Optional[viser.GuiButtonHandle] = None
    ):
        """Set GUI control handles (called from visualizer).
        
        Args:
            publish_mode_dropdown: Dropdown for publish mode selection
            send_button: Button for sending marker pose in single-shot mode
        """
        self._marker_publish_mode_dropdown = publish_mode_dropdown
        self._send_marker_pose_button = send_button
    
    def cleanup(self):
        """Cleanup markers and frame nodes."""
        if self._left_ee_marker is not None:
            try:
                self._left_ee_marker.remove()
            except Exception as e:
                logger.debug(f"Error removing left EE marker: {e}")
            self._left_ee_marker = None
            self._last_left_ee_pose = None
        
        if self._ee_frame_node is not None:
            try:
                self._ee_frame_node.remove()
            except Exception as e:
                logger.debug(f"Error removing shared frame node: {e}")
            self._ee_frame_node = None
        
        if self._right_ee_marker is not None:
            try:
                self._right_ee_marker.remove()
            except Exception as e:
                logger.debug(f"Error removing right EE marker: {e}")
            self._right_ee_marker = None
            self._last_right_ee_pose = None
        
        self._initialized = False
    
    def update(self):
        """Update marker positions and handle user interactions.
        
        This should be called periodically from the update loop.
        """
        if not self.config.enable_end_effector_marker:
            return
        
        ros2_interface = self.ros2_interface
        if ros2_interface is None or not ros2_interface.is_connected:
            return
        
        if not self._initialized:
            left_handler = ros2_interface.left_arm_handler
            right_handler = ros2_interface.right_arm_handler
            left_needs_init = self._left_ee_marker is None and left_handler is not None
            right_needs_init = self._right_ee_marker is None and right_handler is not None
            
            if left_needs_init or right_needs_init:
                try:
                    if left_needs_init:
                        logger.debug("Initializing left arm marker...")
                        self._init_single_ee_marker("left")
                    if right_needs_init:
                        logger.debug("Initializing right arm marker...")
                        self._init_single_ee_marker("right")
                    if left_needs_init or right_needs_init:
                        self._initialized = True
                except Exception as e:
                    logger.warning(f"Error initializing end-effector markers: {e}", exc_info=True)
        
        is_ocs2 = self._is_ocs2_mode()
        
        if is_ocs2 and self._last_fsm_mode is False:
            logger.info("Entered OCS2 mode, resetting marker positions to current end-effector poses")
            self._reset_marker_positions()
        
        self._last_fsm_mode = is_ocs2
        self._update_marker_visibility(is_ocs2)
        self._update_gui_visibility(is_ocs2)
        
        if not is_ocs2:
            return
        
        # Cache references to avoid repeated attribute access
        left_marker = self._left_ee_marker
        right_marker = self._right_ee_marker
        left_handler = ros2_interface.left_arm_handler
        right_handler = ros2_interface.right_arm_handler
        
        target_pose_updated = self._update_markers_from_target_poses()
        
        # Process left arm marker
        if left_marker is not None and left_handler is not None:
            self._process_marker_update(
                "left", left_marker, target_pose_updated, self._last_left_ee_pose
            )
        
        # Process right arm marker
        if right_marker is not None and right_handler is not None:
            self._process_marker_update(
                "right", right_marker, target_pose_updated, self._last_right_ee_pose
            )
    
    def _process_marker_update(
        self,
        arm: str,
        marker: viser.TransformControlsHandle,
        target_pose_updated: bool,
        last_pose: Optional[Tuple]
    ):
        """Process marker position update for a single arm.
        
        Args:
            arm: "left" or "right"
            marker: Marker handle
            target_pose_updated: Whether marker was updated from target pose
            last_pose: Last known pose (will be updated)
        """
        # Get marker position and orientation (avoid creating tuple if not needed)
        marker_pos = marker.position
        marker_wxyz = marker.wxyz
        
        # Quick check: compare individual values before creating tuple
        if last_pose is not None:
            last_pos, last_wxyz = last_pose
            # Compare position and orientation with small epsilon
            if (abs(marker_pos[0] - last_pos[0]) < 1e-6 and
                abs(marker_pos[1] - last_pos[1]) < 1e-6 and
                abs(marker_pos[2] - last_pos[2]) < 1e-6 and
                abs(marker_wxyz[0] - last_wxyz[0]) < 1e-6 and
                abs(marker_wxyz[1] - last_wxyz[1]) < 1e-6 and
                abs(marker_wxyz[2] - last_wxyz[2]) < 1e-6 and
                abs(marker_wxyz[3] - last_wxyz[3]) < 1e-6):
                # No change, skip processing
                if target_pose_updated:
                    # Still update last pose if target was updated
                    current_pose = (tuple(marker_pos), tuple(marker_wxyz))
                    if arm == "left":
                        self._last_left_ee_pose = current_pose
                    else:
                        self._last_right_ee_pose = current_pose
                return
        
        # Position changed, create tuple and process
        current_position = tuple(marker_pos)
        current_wxyz = tuple(marker_wxyz)
        current_pose = (current_position, current_wxyz)
        
        if not target_pose_updated:
            self._last_marker_update_time = time.time()
            
            if self._marker_continuous_publish:
                self._send_pose_command(arm, current_pose)
                if arm == "left":
                    self._last_left_ee_pose = current_pose
                else:
                    self._last_right_ee_pose = current_pose
            else:
                # Only update pending pose if not currently sending
                if not self._is_sending:
                    if arm == "left":
                        self._pending_left_pose = current_pose
                        self._last_left_ee_pose = current_pose
                    else:
                        self._pending_right_pose = current_pose
                        self._last_right_ee_pose = current_pose
                    logger.debug(f"{arm.capitalize()} arm marker moved, saved to pending (single-shot mode)")
        else:
            # Target pose was updated, just sync last_pose
            if arm == "left":
                self._last_left_ee_pose = current_pose
            else:
                self._last_right_ee_pose = current_pose
    
    def on_publish_mode_changed(self, mode_display: str):
        """Callback when marker publish mode dropdown is changed.
        
        Args:
            mode_display: Display name of selected mode.
        """
        continuous_text = self.translator("continuous_publish")
        is_continuous = (mode_display == continuous_text)
        
        self._marker_continuous_publish = is_continuous
        self.config.marker_continuous_publish = is_continuous
        
        if self._send_marker_pose_button is not None:
            is_ocs2 = self._is_ocs2_mode()
            self._send_marker_pose_button.visible = is_ocs2 and not is_continuous
        
        logger.info(f"Marker publish mode changed to: {'continuous' if is_continuous else 'single'}")
    
    def on_send_button_clicked(self):
        """Callback when send marker pose button is clicked (single-shot mode)."""
        if not self._is_ocs2_mode():
            logger.warning("Not in OCS2 mode, cannot send marker pose")
            return
        
        if self.ros2_interface is None or not self.ros2_interface.is_connected:
            logger.warning("ROS2 interface not connected, cannot send marker pose")
            return
        
        # Prevent concurrent sends
        if self._is_sending:
            logger.debug("Send already in progress, ignoring duplicate click")
            return
        
        # Capture pending poses and clear them immediately to prevent race conditions
        # This ensures that update() won't interfere during sending
        pending_left = self._pending_left_pose
        pending_right = self._pending_right_pose
        
        # Clear immediately to prevent update() from modifying them
        self._pending_left_pose = None
        self._pending_right_pose = None
        
        has_left = pending_left is not None
        has_right = pending_right is not None
        
        # Set sending flag to prevent concurrent sends
        self._is_sending = True
        try:
            if has_left and has_right:
                try:
                    from geometry_msgs.msg import Pose
                    
                    left_position, left_wxyz = pending_left
                    left_pose = Pose()
                    left_pose.position.x = left_position[0]
                    left_pose.position.y = left_position[1]
                    left_pose.position.z = left_position[2]
                    left_pose.orientation.w = left_wxyz[0]
                    left_pose.orientation.x = left_wxyz[1]
                    left_pose.orientation.y = left_wxyz[2]
                    left_pose.orientation.z = left_wxyz[3]
                    
                    right_position, right_wxyz = pending_right
                    right_pose = Pose()
                    right_pose.position.x = right_position[0]
                    right_pose.position.y = right_position[1]
                    right_pose.position.z = right_position[2]
                    right_pose.orientation.w = right_wxyz[0]
                    right_pose.orientation.x = right_wxyz[1]
                    right_pose.orientation.y = right_wxyz[2]
                    right_pose.orientation.z = right_wxyz[3]
                    
                    left_frame_id = self._left_frame_id or self.ros2_interface.left_arm_handler.get_frame_id()
                    right_frame_id = self._right_frame_id or self.ros2_interface.right_arm_handler.get_frame_id()
                    common_frame_id = left_frame_id or (self._marker_base_frame if self._marker_base_frame is not None else "base_link")
                    
                    if left_frame_id != right_frame_id:
                        logger.debug(f"Left and right arms have different frame_ids ({left_frame_id} vs {right_frame_id}), using left arm's frame_id: {common_frame_id}")
                    
                    self.ros2_interface.send_dual_arm_target_stamped(left_pose, right_pose, frame_id=common_frame_id)
                    logger.info(f"Sent dual arm target poses (single-shot mode, frame: {common_frame_id}, no transformation)")
                except Exception as e:
                    logger.error(f"Failed to send dual arm target poses: {e}", exc_info=True)
            elif has_left:
                self._send_pose_command_stamped("left", pending_left)
            elif has_right:
                self._send_pose_command_stamped("right", pending_right)
            else:
                logger.warning("No pending poses to send")
        finally:
            # Always clear the sending flag, even if an exception occurred
            self._is_sending = False
    
    def set_urdf(self, urdf: Optional[yourdfpy.URDF]):
        """Update URDF reference (called when URDF changes).
        
        Args:
            urdf: New URDF model
        """
        self.urdf = urdf
    
    # Private methods
    
    def _get_link_frame_path(self, link_name: str, prefer_collision: bool = False) -> Optional[str]:
        """Get the viser scene path for a URDF link.
        
        Args:
            link_name: Name of the link in the URDF
            prefer_collision: If True, prefer collision scene path (default: False, prefer visual)
            
        Returns:
            Viser scene path for the link, or None if not found
        """
        if self.urdf is None:
            return None
        
        # Prefer visual scene first (usually always visible)
        # Try visual scene first
        if hasattr(self.urdf, 'scene'):
            try:
                scene = self.urdf.scene
                base = scene.graph.base_frame
                parents = scene.graph.transforms.parents
                
                if link_name == base:
                    return f"{self.root_node_name}/visual"
                
                if link_name in parents:
                    frames = []
                    current = link_name
                    while current != base and current in parents:
                        frames.append(current)
                        current = parents[current]
                    
                    if current == base:
                        frames.append(base)
                        path_parts = frames[::-1]
                        viser_path = f"{self.root_node_name}/visual/" + "/".join(path_parts[1:])
                        logger.debug(f"Using visual scene path for link '{link_name}': {viser_path}")
                        return viser_path
            except Exception as e:
                logger.debug(f"Error computing visual scene path for {link_name}: {e}")
        
        # Fallback to collision scene (if preferred or visual not available)
        if prefer_collision and hasattr(self.urdf, 'collision_scene') and self.urdf.collision_scene is not None:
            try:
                scene = self.urdf.collision_scene
                base = scene.graph.base_frame
                parents = scene.graph.transforms.parents
                
                if link_name == base:
                    return f"{self.root_node_name}/collision"
                
                if link_name in parents:
                    frames = []
                    current = link_name
                    while current != base and current in parents:
                        frames.append(current)
                        current = parents[current]
                    
                    if current == base:
                        frames.append(base)
                        path_parts = frames[::-1]
                        viser_path = f"{self.root_node_name}/collision/" + "/".join(path_parts[1:])
                        logger.debug(f"Using collision scene path for link '{link_name}': {viser_path}")
                        return viser_path
            except Exception as e:
                logger.debug(f"Error computing collision scene path for {link_name}: {e}")
        
        return None
    
    def _get_urdf_link_transform(self, link_name: str, base_frame: str = None) -> Optional[Tuple]:
        """Get the transform from base_frame (or URDF root) to a URDF link.
        
        Args:
            link_name: Name of the link in the URDF
            base_frame: Base frame name (if None, uses URDF root)
            
        Returns:
            Tuple of (position, wxyz) or None if not found
        """
        if self.urdf is None:
            return None
        
        try:
            scene = None
            if hasattr(self.urdf, 'scene') and self.urdf.scene is not None:
                scene = self.urdf.scene
            elif hasattr(self.urdf, 'collision_scene') and self.urdf.collision_scene is not None:
                scene = self.urdf.collision_scene
            else:
                return None
            
            graph = scene.graph
            base = graph.base_frame
            
            if base_frame is not None and base_frame != base:
                base_to_root = self._get_urdf_link_transform(base_frame, None)
                if base_to_root is None:
                    return None
                
                root_to_link = self._get_urdf_link_transform(link_name, None)
                if root_to_link is None:
                    return None
                
                return None
            
            if link_name == base:
                return ((0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0))
            
            if link_name not in graph.transforms.parents:
                return None
            
            try:
                transform = graph.get_transform(base, link_name)
                if transform is not None:
                    import numpy as np
                    trans = transform[:3, 3]
                    rot_matrix = transform[:3, :3]
                    
                    try:
                        from scipy.spatial.transform import Rotation
                        rot = Rotation.from_matrix(rot_matrix)
                        quat = rot.as_quat()
                        wxyz = (float(quat[3]), float(quat[0]), float(quat[1]), float(quat[2]))
                    except ImportError:
                        trace = rot_matrix[0, 0] + rot_matrix[1, 1] + rot_matrix[2, 2]
                        if trace > 0:
                            s = np.sqrt(trace + 1.0) * 2
                            w = 0.25 * s
                            x = (rot_matrix[2, 1] - rot_matrix[1, 2]) / s
                            y = (rot_matrix[0, 2] - rot_matrix[2, 0]) / s
                            z = (rot_matrix[1, 0] - rot_matrix[0, 1]) / s
                            wxyz = (float(w), float(x), float(y), float(z))
                        else:
                            wxyz = (1.0, 0.0, 0.0, 0.0)
                    
                    position = (float(trans[0]), float(trans[1]), float(trans[2]))
                    return (position, wxyz)
            except Exception as e:
                logger.debug(f"Error getting URDF transform for {link_name}: {e}")
                return None
        except Exception as e:
            logger.debug(f"Error accessing URDF scene for {link_name}: {e}")
            return None
        
        return None
    
    def _init_single_ee_marker(self, arm: str):
        """Initialize a single end-effector marker for left or right arm.
        
        Args:
            arm: "left" or "right"
        """
        if self.server is None or self.ros2_interface is None:
            return
        
        handler = None
        if arm == "left":
            handler = self.ros2_interface.left_arm_handler
        elif arm == "right":
            handler = self.ros2_interface.right_arm_handler
        else:
            logger.warning(f"Unknown arm: {arm}")
            return
        
        if handler is None:
            logger.warning(f"{arm} arm handler is None, cannot create marker")
            return
        
        frame_id = handler.get_frame_id()
        if frame_id is None:
            logger.warning(f"Could not get frame_id for {arm} arm, will use default base_link")
            frame_id = "base_link"
        
        if arm == "left" and self._marker_base_frame is None:
            self._marker_base_frame = frame_id
            logger.info(f"Set marker base_frame to '{frame_id}' from left arm handler")
        
        link_frame_path = self._get_link_frame_path(frame_id)
        is_urdf_link = link_frame_path is not None
        if arm == "left":
            self._left_frame_id = frame_id
            self._left_is_urdf_link = is_urdf_link
            if is_urdf_link and link_frame_path is not None:
                self._left_urdf_frame_path = link_frame_path
        else:
            self._right_frame_id = frame_id
            self._right_is_urdf_link = is_urdf_link
            if is_urdf_link and link_frame_path is not None:
                self._right_urdf_frame_path = link_frame_path
        
        if is_urdf_link and link_frame_path is not None:
            marker_name = f"{link_frame_path}/ee_target_{arm}"
            logger.info(f"Frame '{frame_id}' is a URDF link, binding marker to shared URDF frame '{link_frame_path}' for {arm} arm")
        else:
            frame_node_name = f"/frames/{arm}_arm_frame"
            marker_name = f"{frame_node_name}/ee_target"
            
            logger.info(f"Frame '{frame_id}' is not a URDF link, using independent frame node in world coordinates for {arm} arm")
            
            if self._ee_frame_node is None:
                try:
                    self._ee_frame_node = self.server.scene.add_frame(
                        frame_node_name,
                        show_axes=False
                    )
                    self._ee_frame_node.position = (0.0, 0.0, 0.0)
                    self._ee_frame_node.wxyz = (1.0, 0.0, 0.0, 0.0)
                    logger.info(f"Created independent frame node '{frame_node_name}' in world coordinates (frame_id: {frame_id})")
                except Exception as e:
                    logger.warning(f"Failed to create independent frame node: {e}")
        current_pose = handler.get_pose()
        if current_pose is None:
            current_pose = handler.get_target_pose()
        
        if current_pose is None:
            logger.warning(f"Could not get current pose for {arm} arm, using default position")
            position = (0.5, 0.0, 0.5)
            wxyz = (1.0, 0.0, 0.0, 0.0)
        else:
            position = (
                float(current_pose.position.x),
                float(current_pose.position.y),
                float(current_pose.position.z)
            )
            wxyz = (
                float(current_pose.orientation.w),
                float(current_pose.orientation.x),
                float(current_pose.orientation.y),
                float(current_pose.orientation.z)
            )
        
        try:
            logger.debug(f"Creating {arm} arm marker at path '{marker_name}' with position {position}, wxyz {wxyz}")
            if arm == "left":
                self._left_ee_marker = self.server.scene.add_transform_controls(
                    marker_name,
                    scale=self.config.marker_scale,
                    position=position,
                    wxyz=wxyz
                )
                self._left_ee_marker.visible = False
                self._last_left_ee_pose = (position, wxyz)
                logger.info(f"✅ Created left arm end-effector marker at {position} (in frame: {frame_id}, path: {marker_name}, initially hidden)")
            else:
                self._right_ee_marker = self.server.scene.add_transform_controls(
                    marker_name,
                    scale=self.config.marker_scale,
                    position=position,
                    wxyz=wxyz
                )
                self._right_ee_marker.visible = False
                self._last_right_ee_pose = (position, wxyz)
                logger.info(f"✅ Created right arm end-effector marker at {position} (in frame: {frame_id}, path: {marker_name}, initially hidden)")
        except Exception as e:
            logger.error(f"Failed to create {arm} arm marker at path '{marker_name}': {e}", exc_info=True)
            if arm == "left":
                self._left_ee_marker = None
            else:
                self._right_ee_marker = None
    
    def _is_ocs2_mode(self) -> bool:
        """Check if robot is in OCS2 mode (FSM command == 3)."""
        if self.ros2_interface is None or not self.ros2_interface.is_connected:
            return False
        
        try:
            fsm_command = self.ros2_interface.get_fsm_command()
            return fsm_command == 3
        except Exception as e:
            logger.debug(f"Could not get FSM command: {e}")
            try:
                fsm_state = self.ros2_interface.get_fsm_state()
                return fsm_state == "OCS2"
            except Exception:
                return False
    
    def _update_marker_visibility(self, is_ocs2: bool):
        """Update marker visibility based on FSM state."""
        if self._left_ee_marker is not None:
            try:
                self._left_ee_marker.visible = is_ocs2
            except Exception as e:
                logger.debug(f"Could not set left marker visibility: {e}")
        
        if self._right_ee_marker is not None:
            try:
                self._right_ee_marker.visible = is_ocs2
            except Exception as e:
                logger.debug(f"Could not set right marker visibility: {e}")
    
    def _update_gui_visibility(self, is_ocs2: bool):
        """Update GUI control visibility based on FSM state and publish mode."""
        if self._marker_publish_mode_dropdown is not None:
            try:
                self._marker_publish_mode_dropdown.visible = is_ocs2
            except Exception as e:
                logger.debug(f"Could not set marker publish mode dropdown visibility: {e}")
        
        if self._send_marker_pose_button is not None:
            try:
                self._send_marker_pose_button.visible = is_ocs2 and not self._marker_continuous_publish
            except Exception as e:
                logger.debug(f"Could not set send marker pose button visibility: {e}")
    
    def _update_markers_from_target_poses(self) -> bool:
        """Update marker positions from current target poses if they have changed.
        
        Returns:
            True if any marker was updated from target pose in this call, False otherwise.
        """
        if self.ros2_interface is None or not self.ros2_interface.is_connected:
            return False
        
        marker_updated = False
        
        current_time = time.time()
        time_since_last_update = current_time - self._last_marker_update_time
        if time_since_last_update < self._marker_update_cooldown:
            return False
        if self._left_ee_marker is not None and self.ros2_interface.left_arm_handler is not None:
            try:
                target_pose = self.ros2_interface.left_arm_handler.get_target_pose()
                if target_pose is not None:
                    target_changed = (
                        self._last_left_target_pose is None or
                        self._last_left_target_pose.position.x != target_pose.position.x or
                        self._last_left_target_pose.position.y != target_pose.position.y or
                        self._last_left_target_pose.position.z != target_pose.position.z or
                        self._last_left_target_pose.orientation.x != target_pose.orientation.x or
                        self._last_left_target_pose.orientation.y != target_pose.orientation.y or
                        self._last_left_target_pose.orientation.z != target_pose.orientation.z or
                        self._last_left_target_pose.orientation.w != target_pose.orientation.w
                    )
                    
                    if target_changed:
                        if self._left_frame_id is not None:
                            position = (
                                float(target_pose.position.x),
                                float(target_pose.position.y),
                                float(target_pose.position.z)
                            )
                            wxyz = (
                                float(target_pose.orientation.w),
                                float(target_pose.orientation.x),
                                float(target_pose.orientation.y),
                                float(target_pose.orientation.z)
                            )
                            
                            self._left_ee_marker.position = position
                            self._left_ee_marker.wxyz = wxyz
                            marker_updated = True
                            from geometry_msgs.msg import Pose
                            self._last_left_target_pose = Pose()
                            self._last_left_target_pose.position.x = target_pose.position.x
                            self._last_left_target_pose.position.y = target_pose.position.y
                            self._last_left_target_pose.position.z = target_pose.position.z
                            self._last_left_target_pose.orientation.x = target_pose.orientation.x
                            self._last_left_target_pose.orientation.y = target_pose.orientation.y
                            self._last_left_target_pose.orientation.z = target_pose.orientation.z
                            self._last_left_target_pose.orientation.w = target_pose.orientation.w
                            logger.debug(f"Updated left marker position from target pose: {position}")
                elif self._last_left_target_pose is not None:
                    self._last_left_target_pose = None
            except Exception as e:
                logger.debug(f"Error updating left marker from target pose: {e}")
        
        # Update right arm marker from target pose
        if self._right_ee_marker is not None and self.ros2_interface.right_arm_handler is not None:
            try:
                target_pose = self.ros2_interface.right_arm_handler.get_target_pose()
                if target_pose is not None:
                    target_changed = (
                        self._last_right_target_pose is None or
                        self._last_right_target_pose.position.x != target_pose.position.x or
                        self._last_right_target_pose.position.y != target_pose.position.y or
                        self._last_right_target_pose.position.z != target_pose.position.z or
                        self._last_right_target_pose.orientation.x != target_pose.orientation.x or
                        self._last_right_target_pose.orientation.y != target_pose.orientation.y or
                        self._last_right_target_pose.orientation.z != target_pose.orientation.z or
                        self._last_right_target_pose.orientation.w != target_pose.orientation.w
                    )
                    
                    if target_changed:
                        if self._right_frame_id is not None:
                            position = (
                                float(target_pose.position.x),
                                float(target_pose.position.y),
                                float(target_pose.position.z)
                            )
                            wxyz = (
                                float(target_pose.orientation.w),
                                float(target_pose.orientation.x),
                                float(target_pose.orientation.y),
                                float(target_pose.orientation.z)
                            )
                            
                            self._right_ee_marker.position = position
                            self._right_ee_marker.wxyz = wxyz
                            marker_updated = True
                            from geometry_msgs.msg import Pose
                            self._last_right_target_pose = Pose()
                            self._last_right_target_pose.position.x = target_pose.position.x
                            self._last_right_target_pose.position.y = target_pose.position.y
                            self._last_right_target_pose.position.z = target_pose.position.z
                            self._last_right_target_pose.orientation.x = target_pose.orientation.x
                            self._last_right_target_pose.orientation.y = target_pose.orientation.y
                            self._last_right_target_pose.orientation.z = target_pose.orientation.z
                            self._last_right_target_pose.orientation.w = target_pose.orientation.w
                            logger.debug(f"Updated right marker position from target pose: {position}")
                elif self._last_right_target_pose is not None:
                    self._last_right_target_pose = None
            except Exception as e:
                logger.debug(f"Error updating right marker from target pose: {e}")
        
        return marker_updated
    
    def _reset_marker_positions(self):
        """Reset marker positions to current end-effector poses when entering OCS2 mode."""
        if self.ros2_interface is None or not self.ros2_interface.is_connected:
            return
        
        if self._left_ee_marker is not None and self.ros2_interface.left_arm_handler is not None:
            try:
                current_pose = self.ros2_interface.left_arm_handler.get_pose()
                if current_pose is None:
                    current_pose = self.ros2_interface.left_arm_handler.get_target_pose()
                
                if current_pose is not None and self._left_frame_id is not None:
                    position = (
                        float(current_pose.position.x),
                        float(current_pose.position.y),
                        float(current_pose.position.z)
                    )
                    wxyz = (
                        float(current_pose.orientation.w),
                        float(current_pose.orientation.x),
                        float(current_pose.orientation.y),
                        float(current_pose.orientation.z)
                    )
                    
                    self._left_ee_marker.position = position
                    self._left_ee_marker.wxyz = wxyz
                    self._last_left_ee_pose = (position, wxyz)
                    logger.info(f"Reset left arm marker position to {position} (in frame: {self._left_frame_id})")
                else:
                    logger.warning("Could not get current pose for left arm, cannot reset marker")
            except Exception as e:
                logger.warning(f"Failed to reset left arm marker position: {e}")
        
        if self._right_ee_marker is not None and self.ros2_interface.right_arm_handler is not None:
            try:
                current_pose = self.ros2_interface.right_arm_handler.get_pose()
                if current_pose is None:
                    current_pose = self.ros2_interface.right_arm_handler.get_target_pose()
                
                if current_pose is not None and self._right_frame_id is not None:
                    position = (
                        float(current_pose.position.x),
                        float(current_pose.position.y),
                        float(current_pose.position.z)
                    )
                    wxyz = (
                        float(current_pose.orientation.w),
                        float(current_pose.orientation.x),
                        float(current_pose.orientation.y),
                        float(current_pose.orientation.z)
                    )
                    
                    self._right_ee_marker.position = position
                    self._right_ee_marker.wxyz = wxyz
                    self._last_right_ee_pose = (position, wxyz)
                    logger.info(f"Reset right arm marker position to {position} (in frame: {self._right_frame_id})")
                else:
                    logger.warning("Could not get current pose for right arm, cannot reset marker")
            except Exception as e:
                logger.warning(f"Failed to reset right arm marker position: {e}")
    
    def _send_pose_command(self, arm: str, pose_tuple: Tuple):
        """Send pose command for the specified arm (continuous mode).
        
        Args:
            arm: "left" or "right"
            pose_tuple: (position, wxyz) tuple
        """
        if self.ros2_interface is None or not self.ros2_interface.is_connected:
            return
        
        try:
            from geometry_msgs.msg import Pose
            pose = Pose()
            position, wxyz = pose_tuple
            pose.position.x = position[0]
            pose.position.y = position[1]
            pose.position.z = position[2]
            pose.orientation.w = wxyz[0]
            pose.orientation.x = wxyz[1]
            pose.orientation.y = wxyz[2]
            pose.orientation.z = wxyz[3]
            
            if arm == "left":
                handler = self.ros2_interface.left_arm_handler
            else:
                handler = self.ros2_interface.right_arm_handler
            
            frame_id = self._marker_base_frame or self._left_frame_id or "base_link"
            
            handler.send_target_stamped(frame_id, pose)
            logger.debug(f"Sent {arm} arm target pose (continuous mode, frame: {frame_id}, no transformation)")
        except Exception as e:
            logger.warning(f"Failed to send {arm} arm pose command: {e}")
    
    def _send_pose_command_stamped(self, arm: str, pose_tuple: Tuple):
        """Send pose command using send_target_stamped (single-shot mode).
        
        Args:
            arm: "left" or "right"
            pose_tuple: (position, wxyz) tuple
        """
        if self.ros2_interface is None or not self.ros2_interface.is_connected:
            return
        
        try:
            from geometry_msgs.msg import Pose
            pose = Pose()
            position, wxyz = pose_tuple
            pose.position.x = position[0]
            pose.position.y = position[1]
            pose.position.z = position[2]
            pose.orientation.w = wxyz[0]
            pose.orientation.x = wxyz[1]
            pose.orientation.y = wxyz[2]
            pose.orientation.z = wxyz[3]
            
            if arm == "left":
                handler = self.ros2_interface.left_arm_handler
            else:
                handler = self.ros2_interface.right_arm_handler
            
            frame_id = self._marker_base_frame or self._left_frame_id or "base_link"
            
            handler.send_target_stamped(frame_id, pose)
            logger.info(f"Sent {arm} arm target pose (single-shot mode, frame: {frame_id}, no transformation)")
        except Exception as e:
            logger.warning(f"Failed to send {arm} arm pose command (stamped): {e}")
