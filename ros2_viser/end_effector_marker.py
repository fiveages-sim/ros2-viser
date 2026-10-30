"""End-effector marker manager for draggable pose control."""

import logging
import time
from typing import Optional, Any, Tuple

import viser
import yourdfpy

from arms_ros2_control_msgs.msg import WbcCurrentState

from ros2_robot_interface import ROS2RobotInterface
from .config import ROS2ViserConfig
from .i18n import Translator

logger = logging.getLogger(__name__)


def _quat_multiply_wxyz(a: Tuple[float, float, float, float], b: Tuple[float, float, float, float]):
    """Hamilton product of wxyz quaternions (a ⊗ b)."""
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


def _quat_conjugate_wxyz(q: Tuple[float, float, float, float]):
    return (q[0], -q[1], -q[2], -q[3])


def _rotate_vec_wxyz(q: Tuple[float, float, float, float], v: Tuple[float, float, float]):
    qv = (0.0, v[0], v[1], v[2])
    return _quat_multiply_wxyz(_quat_multiply_wxyz(q, qv), _quat_conjugate_wxyz(q))[1:]


def _compose_pose(
    a_pos: Tuple[float, float, float],
    a_wxyz: Tuple[float, float, float, float],
    b_pos: Tuple[float, float, float],
    b_wxyz: Tuple[float, float, float, float],
):
    """Compose SE3 poses: result = a ⊗ b."""
    rotated = _rotate_vec_wxyz(a_wxyz, b_pos)
    pos = (a_pos[0] + rotated[0], a_pos[1] + rotated[1], a_pos[2] + rotated[2])
    return pos, _quat_multiply_wxyz(a_wxyz, b_wxyz)


def _invert_pose(
    pos: Tuple[float, float, float],
    wxyz: Tuple[float, float, float, float],
):
    inv_q = _quat_conjugate_wxyz(wxyz)
    inv_t = _rotate_vec_wxyz(inv_q, (-pos[0], -pos[1], -pos[2]))
    return inv_t, inv_q


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
        root_node_name: str,
        *,
        attach_to_urdf_frames: bool = True,
        marker_path_prefix: str = "",
    ):
        """Initialize the end-effector marker manager.
        
        Args:
            server: Viser server or client handle (needs ``.scene``)
            ros2_interface: ROS2 robot interface
            config: Visualizer configuration
            translator: Translator for i18n
            urdf: URDF model (optional, for link path resolution)
            root_node_name: Root node name for scene graph
            attach_to_urdf_frames: If False, always use independent world frames
                (needed for per-client ``client.scene`` markers).
            marker_path_prefix: Optional path prefix to keep per-client markers unique.
        """
        self.server = server
        self.ros2_interface = ros2_interface
        self.config = config
        self.translator = translator
        self.urdf = urdf
        self.root_node_name = root_node_name
        self.attach_to_urdf_frames = attach_to_urdf_frames
        self.marker_path_prefix = marker_path_prefix.rstrip("/")
        
        self._left_ee_marker: Optional[viser.TransformControlsHandle] = None
        self._right_ee_marker: Optional[viser.TransformControlsHandle] = None
        self._body_marker: Optional[viser.TransformControlsHandle] = None
        self._head_marker: Optional[viser.TransformControlsHandle] = None
        # Read-only axes for controller ``*_current_target`` (RViz Pose Axes).
        self._left_current_target_axes: Optional[viser.FrameHandle] = None
        self._right_current_target_axes: Optional[viser.FrameHandle] = None
        self._last_left_current_target_display: Optional[Any] = None
        self._last_right_current_target_display: Optional[Any] = None
        self._show_current_target_axes: bool = bool(
            getattr(config, "show_current_target_axes", True)
        )
        self._ee_frame_node: Optional[viser.FrameHandle] = None
        self._arm_frame_nodes: dict[str, viser.FrameHandle] = {}
        
        self._last_left_ee_pose: Optional[Tuple] = None
        self._last_right_ee_pose: Optional[Tuple] = None
        self._last_body_marker_pose: Optional[Tuple] = None
        self._last_head_marker_pose: Optional[Tuple] = None
        self._left_frame_id: Optional[str] = None
        self._right_frame_id: Optional[str] = None
        self._body_frame_id: Optional[str] = None
        self._head_frame_id: Optional[str] = None
        self._left_is_urdf_link: bool = False
        self._right_is_urdf_link: bool = False
        self._left_urdf_frame_path: Optional[str] = None
        self._right_urdf_frame_path: Optional[str] = None
        self._marker_base_frame: Optional[str] = None
        # Scene / viz frame: OCS2 poses (often ``world``) are converted into this
        # frame before display when markers are not parented under a URDF link.
        self._visualization_frame: str = (
            (config.visualization_frame or "").strip()
        )
        self._urdf_root_link: Optional[str] = None
        
        self._last_fsm_mode: Optional[bool] = None
        self._last_body_tracking: bool = False
        self._last_head_tracking: bool = False
        self._marker_continuous_publish: bool = config.marker_continuous_publish
        self._pending_left_pose: Optional[Tuple] = None
        self._pending_right_pose: Optional[Tuple] = None
        self._pending_body_pose: Optional[Tuple] = None
        self._pending_head_pose: Optional[Tuple] = None
        self._last_left_target_pose: Optional[Any] = None
        self._last_right_target_pose: Optional[Any] = None
        self._last_body_target_pose: Optional[Any] = None
        self._last_head_target_pose: Optional[Any] = None
        self._last_marker_update_time: float = 0.0
        self._marker_update_cooldown: float = 0.5
        self._is_sending: bool = False  # Flag to prevent concurrent sends
        self._last_tf_warn_time: float = 0.0
        self._suppress_marker_send_until: float = 0.0
        
        self._marker_publish_mode_dropdown: Optional[viser.GuiDropdownHandle] = None
        self._send_marker_pose_button: Optional[viser.GuiButtonHandle] = None
        
        self._initialized = False

    def _warn_tf_limited(self, message: str, *args) -> None:
        now = time.time()
        if now - self._last_tf_warn_time < 5.0:
            return
        self._last_tf_warn_time = now
        logger.warning(message, *args)

    def _default_pose_frame(self) -> str:
        """Best-effort pose frame before the first PoseStamped arrives."""
        if self._urdf_root_link:
            return self._urdf_root_link
        # G1D / wheeled stacks: footprint on ground; never default to base_link
        # (base_link is +0.1114 m above base_footprint on G1D).
        return "base_footprint"

    def _refresh_arm_frame_ids(self) -> None:
        """Keep cached pose frames in sync with the latest PoseStamped headers."""
        for arm in ("left", "right"):
            handler = self._get_arm_handler(arm)
            if handler is None:
                continue
            latest = handler.get_frame_id()
            if not latest:
                continue
            if arm == "left":
                if self._left_frame_id != latest:
                    if self._left_frame_id and self._left_frame_id != latest:
                        logger.info(
                            "Left arm pose frame updated: %s → %s",
                            self._left_frame_id,
                            latest,
                        )
                    self._left_frame_id = latest
                    if self._marker_base_frame is None:
                        self._marker_base_frame = latest
            else:
                if self._right_frame_id != latest:
                    if self._right_frame_id and self._right_frame_id != latest:
                        logger.info(
                            "Right arm pose frame updated: %s → %s",
                            self._right_frame_id,
                            latest,
                        )
                    self._right_frame_id = latest

    def _refresh_body_head_frame_ids(self) -> None:
        """Keep body/head pose frames in sync with PoseStamped headers."""
        iface = self.ros2_interface
        if iface is None:
            return
        body_frame = (
            getattr(iface, "body_current_target_pose_frame_id", None)
            or getattr(iface, "body_current_pose_frame_id", None)
        )
        if body_frame:
            self._body_frame_id = body_frame
            if self._marker_base_frame is None:
                self._marker_base_frame = body_frame
        head_frame = getattr(iface, "head_current_target_pose_frame_id", None)
        if head_frame:
            self._head_frame_id = head_frame
        elif self._body_frame_id:
            self._head_frame_id = self._body_frame_id
        elif self._marker_base_frame:
            self._head_frame_id = self._marker_base_frame
        else:
            self._head_frame_id = self._default_pose_frame()
        if self._body_frame_id is None:
            self._body_frame_id = self._marker_base_frame or self._default_pose_frame()

    def set_scene_context(
        self,
        *,
        visualization_frame: str = "",
        urdf_root_link: Optional[str] = None,
    ) -> None:
        """Update the scene frame used to place independent (non-URDF) markers."""
        self._visualization_frame = (visualization_frame or "").strip()
        if urdf_root_link is not None:
            self._urdf_root_link = urdf_root_link
        # Refresh visible markers so they jump to the new scene frame immediately.
        if self._initialized:
            self._reset_marker_positions()
    
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

            if self._body_marker_supported():
                self._init_body_or_head_marker("body")
            if self._head_marker_supported():
                self._init_body_or_head_marker("head")
            
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
            self._left_ee_marker.remove()
            self._left_ee_marker = None
            self._last_left_ee_pose = None

        if self._right_ee_marker is not None:
            self._right_ee_marker.remove()
            self._right_ee_marker = None
            self._last_right_ee_pose = None

        if self._body_marker is not None:
            self._body_marker.remove()
            self._body_marker = None
            self._last_body_marker_pose = None

        if self._head_marker is not None:
            self._head_marker.remove()
            self._head_marker = None
            self._last_head_marker_pose = None

        for axes in (self._left_current_target_axes, self._right_current_target_axes):
            if axes is not None:
                try:
                    axes.remove()
                except Exception:
                    pass
        self._left_current_target_axes = None
        self._right_current_target_axes = None
        self._last_left_current_target_display = None
        self._last_right_current_target_display = None

        for node in self._arm_frame_nodes.values():
            try:
                node.remove()
            except Exception:
                pass
        self._arm_frame_nodes.clear()

        if self._ee_frame_node is not None:
            try:
                self._ee_frame_node.remove()
            except Exception:
                pass
            self._ee_frame_node = None

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
            body_needs_init = self._body_marker is None and self._body_marker_supported()
            head_needs_init = self._head_marker is None and self._head_marker_supported()
            
            if left_needs_init or right_needs_init or body_needs_init or head_needs_init:
                try:
                    if left_needs_init:
                        logger.debug("Initializing left arm marker...")
                        self._init_single_ee_marker("left")
                    if right_needs_init:
                        logger.debug("Initializing right arm marker...")
                        self._init_single_ee_marker("right")
                    if body_needs_init:
                        self._init_body_or_head_marker("body")
                    if head_needs_init:
                        self._init_body_or_head_marker("head")
                    self._initialized = True
                except Exception as e:
                    logger.warning(f"Error initializing end-effector markers: {e}", exc_info=True)
        
        is_ocs2 = self._is_ocs2_mode()
        show_body = self._should_show_body_marker()
        show_head = self._should_show_head_marker()
        # Refresh pose frames every tick — init may have run before the first
        # PoseStamped and wrongly frozen frame_id to "base_link" (G1D uses
        # base_footprint; that 0.11m offset alone makes the hand jump).
        self._refresh_arm_frame_ids()
        self._refresh_body_head_frame_ids()

        entering_ocs2 = is_ocs2 and self._last_fsm_mode is not True
        entering_body = show_body and not self._last_body_tracking
        entering_head = show_head and not self._last_head_tracking
        if entering_ocs2:
            logger.info(
                "Entered OCS2 mode, resetting markers (pose_frames L=%s R=%s, scene=%s)",
                self._left_frame_id,
                self._right_frame_id,
                self._get_scene_frame(),
            )
            self._pending_left_pose = None
            self._pending_right_pose = None
            self._pending_body_pose = None
            self._pending_head_pose = None
            self._reset_marker_positions()
            # Ignore drag/send for a short window so enter-reset cannot publish.
            self._suppress_marker_send_until = time.time() + 0.4
        else:
            if entering_body:
                self._pending_body_pose = None
                self._reset_body_or_head_marker_position("body")
                self._suppress_marker_send_until = time.time() + 0.3
            if entering_head:
                self._pending_head_pose = None
                self._reset_body_or_head_marker_position("head")
                self._suppress_marker_send_until = time.time() + 0.3

        self._last_fsm_mode = is_ocs2
        self._last_body_tracking = show_body
        self._last_head_tracking = show_head
        self._update_marker_visibility(is_ocs2, show_body=show_body, show_head=show_head)
        self._update_gui_visibility(is_ocs2)

        # Always refresh read-only current-target axes (even outside OCS2) so
        # debugging still works in HOLD after a command — same idea as RViz Pose.
        self._update_current_target_axes()

        if not is_ocs2:
            return

        # Cache references to avoid repeated attribute access
        left_marker = self._left_ee_marker
        right_marker = self._right_ee_marker
        left_handler = ros2_interface.left_arm_handler
        right_handler = ros2_interface.right_arm_handler

        target_pose_updated = self._update_markers_from_target_poses()
        suppress_send = time.time() < getattr(self, "_suppress_marker_send_until", 0.0)

        # Process left arm marker
        if left_marker is not None and left_handler is not None:
            self._process_marker_update(
                "left",
                left_marker,
                target_pose_updated,
                self._last_left_ee_pose,
                allow_send=not suppress_send,
            )

        # Process right arm marker
        if right_marker is not None and right_handler is not None:
            self._process_marker_update(
                "right",
                right_marker,
                target_pose_updated,
                self._last_right_ee_pose,
                allow_send=not suppress_send,
            )

        if show_body and self._body_marker is not None:
            self._process_marker_update(
                "body",
                self._body_marker,
                target_pose_updated,
                self._last_body_marker_pose,
                allow_send=not suppress_send,
            )
        if show_head and self._head_marker is not None:
            self._process_marker_update(
                "head",
                self._head_marker,
                target_pose_updated,
                self._last_head_marker_pose,
                allow_send=not suppress_send,
            )

    def set_show_current_target_axes(self, visible: bool) -> None:
        """Toggle RViz-style dual-arm ``*_current_target`` axes visibility."""
        self._show_current_target_axes = bool(visible)
        self.config.show_current_target_axes = self._show_current_target_axes
        if not self._show_current_target_axes:
            for axes in (self._left_current_target_axes, self._right_current_target_axes):
                if axes is not None:
                    axes.visible = False
        else:
            self._update_current_target_axes(force=True)

    def _ensure_current_target_axes(self, arm: str) -> Optional[viser.FrameHandle]:
        """Create (if needed) the read-only axes frame for one arm's current target."""
        existing = (
            self._left_current_target_axes if arm == "left" else self._right_current_target_axes
        )
        if existing is not None:
            return existing
        if self.server is None:
            return None
        prefix = f"{self.marker_path_prefix}/" if self.marker_path_prefix else ""
        path = f"{prefix}{arm}_current_target_axes"
        try:
            # Axes scale roughly matches RViz Pose Axes Length ~0.1–0.3.
            axes_scale = max(0.08, float(self.config.marker_scale) * 1.2)
            axes = self.server.scene.add_frame(
                path,
                show_axes=True,
                axes_length=axes_scale,
                axes_radius=max(0.003, axes_scale * 0.04),
            )
            axes.visible = False
            if arm == "left":
                self._left_current_target_axes = axes
            else:
                self._right_current_target_axes = axes
            return axes
        except Exception as e:
            logger.warning("Failed to create %s current-target axes: %s", arm, e)
            return None

    def _update_current_target_axes(self, *, force: bool = False) -> None:
        """Place axes at the controller's last ``left/right_current_target`` poses."""
        if self.ros2_interface is None or not self.ros2_interface.is_connected:
            return
        if not self._show_current_target_axes and not force:
            return

        for arm in ("left", "right"):
            handler = self._get_arm_handler(arm)
            axes = (
                self._left_current_target_axes if arm == "left" else self._right_current_target_axes
            )
            last_display = (
                self._last_left_current_target_display
                if arm == "left"
                else self._last_right_current_target_display
            )

            if handler is None:
                if axes is not None:
                    axes.visible = False
                continue

            target_pose = handler.get_target_pose()
            if target_pose is None or not self._show_current_target_axes:
                if axes is not None:
                    axes.visible = False
                if arm == "left":
                    self._last_left_current_target_display = None
                else:
                    self._last_right_current_target_display = None
                continue

            frame_id = (
                handler.get_target_frame_id()
                if hasattr(handler, "get_target_frame_id")
                else handler.get_frame_id()
            ) or self._default_pose_frame()

            unchanged = (
                not force
                and last_display is not None
                and last_display.position.x == target_pose.position.x
                and last_display.position.y == target_pose.position.y
                and last_display.position.z == target_pose.position.z
                and last_display.orientation.x == target_pose.orientation.x
                and last_display.orientation.y == target_pose.orientation.y
                and last_display.orientation.z == target_pose.orientation.z
                and last_display.orientation.w == target_pose.orientation.w
            )
            if unchanged and axes is not None and axes.visible:
                continue

            position, wxyz = self._extract_pose_data(target_pose)
            converted = self._pose_frame_to_scene(
                frame_id,
                position,
                wxyz,
                attached_to_urdf_link=False,
            )
            if converted is None:
                if axes is not None:
                    axes.visible = False
                continue
            position, wxyz = converted

            axes = self._ensure_current_target_axes(arm)
            if axes is None:
                continue
            axes.position = position
            axes.wxyz = wxyz
            axes.visible = True
            copied = self._copy_pose(target_pose)
            if arm == "left":
                self._last_left_current_target_display = copied
            else:
                self._last_right_current_target_display = copied
    
    def _store_last_marker_pose(self, kind: str, pose: Tuple) -> None:
        if kind == "left":
            self._last_left_ee_pose = pose
        elif kind == "right":
            self._last_right_ee_pose = pose
        elif kind == "body":
            self._last_body_marker_pose = pose
        elif kind == "head":
            self._last_head_marker_pose = pose

    def _store_pending_marker_pose(self, kind: str, pose: Tuple) -> None:
        if kind == "left":
            self._pending_left_pose = pose
        elif kind == "right":
            self._pending_right_pose = pose
        elif kind == "body":
            self._pending_body_pose = pose
        elif kind == "head":
            self._pending_head_pose = pose

    def _process_marker_update(
        self,
        arm: str,
        marker: viser.TransformControlsHandle,
        target_pose_updated: bool,
        last_pose: Optional[Tuple],
        *,
        allow_send: bool = True,
    ):
        """Process marker position update for left/right/body/head.
        
        Args:
            arm: "left", "right", "body", or "head"
            marker: Marker handle
            target_pose_updated: Whether marker was updated from target pose
            last_pose: Last known pose (will be updated)
            allow_send: If False, only sync last_pose (used right after OCS2 enter).
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
                    self._store_last_marker_pose(arm, current_pose)
                return
        
        # Position changed, create tuple and process
        current_position = tuple(marker_pos)
        current_wxyz = tuple(marker_wxyz)
        current_pose = (current_position, current_wxyz)
        
        if not target_pose_updated:
            self._last_marker_update_time = time.time()

            if not allow_send:
                self._store_last_marker_pose(arm, current_pose)
                return

            if self._marker_continuous_publish:
                # Contract with PoseBasedReferenceManager / arms_target_manager:
                # continuous / VR → unstamped left_target (Pose, already in base_frame);
                # single-shot → left_target/stamped (PoseStamped, TF to base if needed).
                self._send_pose_command(arm, current_pose, use_stamped=False)
                self._store_last_marker_pose(arm, current_pose)
            else:
                # Only update pending pose if not currently sending
                if not self._is_sending:
                    self._store_pending_marker_pose(arm, current_pose)
                    self._store_last_marker_pose(arm, current_pose)
                    logger.debug(f"{arm} marker moved, saved to pending (single-shot mode)")
        else:
            # Target pose was updated, just sync last_pose
            self._store_last_marker_pose(arm, current_pose)
    
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
        pending_body = self._pending_body_pose
        pending_head = self._pending_head_pose
        
        # Clear immediately to prevent update() from modifying them
        self._pending_left_pose = None
        self._pending_right_pose = None
        self._pending_body_pose = None
        self._pending_head_pose = None
        
        has_left = pending_left is not None
        has_right = pending_right is not None
        
        # Set sending flag to prevent concurrent sends
        self._is_sending = True
        try:
            if has_left and has_right:
                left_cmd = self._scene_tuple_to_command_tuple("left", pending_left)
                right_cmd = self._scene_tuple_to_command_tuple("right", pending_right)
                if left_cmd is None or right_cmd is None:
                    logger.warning(
                        "Not sending dual-arm poses: scene→pose_frame TF unavailable"
                    )
                else:
                    left_pose = self._create_pose_from_tuple(left_cmd)
                    right_pose = self._create_pose_from_tuple(right_cmd)
                    self.ros2_interface.send_dual_arm_target_stamped(
                        left_pose,
                        right_pose,
                        frame_id=self._left_frame_id or self._default_pose_frame(),
                    )
            elif has_left:
                self._send_pose_command("left", pending_left)
            elif has_right:
                self._send_pose_command("right", pending_right)

            if pending_body is not None and self._should_show_body_marker():
                self._send_pose_command("body", pending_body)
            if pending_head is not None and self._should_show_head_marker():
                self._send_pose_command("head", pending_head)

            if not (has_left or has_right or pending_body or pending_head):
                logger.debug("No pending poses to send")
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

    def _get_scene_frame(self) -> Optional[str]:
        """Frame that coincides with the Viser scene origin for independent markers."""
        if self._visualization_frame:
            return self._visualization_frame
        return self._urdf_root_link

    def _lookup_tf_pose(
        self, target_frame: str, source_frame: str
    ) -> Optional[Tuple[Tuple[float, float, float], Tuple[float, float, float, float]]]:
        if not target_frame or not source_frame or target_frame == source_frame:
            return ((0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0))
        if self.ros2_interface is None or not self.ros2_interface.is_connected:
            return None
        tf_buffer = getattr(self.ros2_interface, "tf_buffer", None)
        if tf_buffer is None:
            return None
        try:
            from rclpy.time import Time

            transform = tf_buffer.lookup_transform(target_frame, source_frame, Time())
            t = transform.transform.translation
            r = transform.transform.rotation
            return (
                (float(t.x), float(t.y), float(t.z)),
                (float(r.w), float(r.x), float(r.y), float(r.z)),
            )
        except Exception as e:
            logger.debug(
                "TF %s <- %s unavailable for marker: %s", target_frame, source_frame, e
            )
            return None

    def _pose_frame_to_scene(
        self,
        pose_frame: Optional[str],
        position: Tuple[float, float, float],
        wxyz: Tuple[float, float, float, float],
        *,
        attached_to_urdf_link: bool,
    ) -> Optional[Tuple[Tuple[float, float, float], Tuple[float, float, float, float]]]:
        """Convert a pose in ``pose_frame`` into scene coordinates for the marker.

        Returns ``None`` when a required TF is missing. Callers must not fall back
        to the raw pose — that is what produced the world/AGV height mismatch
        (marker floating by the same Δh as the robot-to-ground gap).
        """
        if attached_to_urdf_link:
            # Parent is already the URDF link; keep pose relative to that link.
            return position, wxyz
        scene_frame = self._get_scene_frame()
        if not scene_frame or not pose_frame or scene_frame == pose_frame:
            return position, wxyz
        parent = self._lookup_tf_pose(scene_frame, pose_frame)
        if parent is None:
            self._warn_tf_limited(
                "Skip marker pose: no TF %s <- %s (raw pose would be height-wrong)",
                scene_frame,
                pose_frame,
            )
            return None
        return _compose_pose(parent[0], parent[1], position, wxyz)

    def _scene_to_pose_frame(
        self,
        pose_frame: Optional[str],
        position: Tuple[float, float, float],
        wxyz: Tuple[float, float, float, float],
        *,
        attached_to_urdf_link: bool,
    ) -> Optional[Tuple[Tuple[float, float, float], Tuple[float, float, float, float]]]:
        """Convert a scene marker pose back into ``pose_frame`` for publishing.

        Returns ``None`` when a required TF is missing (do not publish raw scene pose).
        """
        if attached_to_urdf_link:
            return position, wxyz
        scene_frame = self._get_scene_frame()
        if not scene_frame or not pose_frame or scene_frame == pose_frame:
            return position, wxyz
        parent = self._lookup_tf_pose(pose_frame, scene_frame)
        if parent is None:
            self._warn_tf_limited(
                "Skip marker command: no TF %s <- %s",
                pose_frame,
                scene_frame,
            )
            return None
        return _compose_pose(parent[0], parent[1], position, wxyz)
    
    def _get_arm_handler(self, arm: str):
        """Get the arm handler for the specified arm."""
        if self.ros2_interface is None:
            return None
        return (self.ros2_interface.left_arm_handler if arm == "left" 
                else self.ros2_interface.right_arm_handler if arm == "right" else None)

    def _arm_is_urdf_attached(self, arm: str) -> bool:
        return self._left_is_urdf_link if arm == "left" else self._right_is_urdf_link
    
    def _extract_pose_data(self, pose) -> Tuple:
        """Extract position and wxyz from a Pose object.
        
        Args:
            pose: geometry_msgs.msg.Pose object
            
        Returns:
            Tuple of (position, wxyz) where position is (x, y, z) and wxyz is (w, x, y, z)
        """
        position = (
            float(pose.position.x),
            float(pose.position.y),
            float(pose.position.z)
        )
        wxyz = (
            float(pose.orientation.w),
            float(pose.orientation.x),
            float(pose.orientation.y),
            float(pose.orientation.z)
        )
        return (position, wxyz)
    
    def _create_pose_from_tuple(self, pose_tuple: Tuple):
        """Create a geometry_msgs.msg.Pose from a (position, wxyz) tuple.
        
        Args:
            pose_tuple: (position, wxyz) tuple where position is (x, y, z) and wxyz is (w, x, y, z)
            
        Returns:
            geometry_msgs.msg.Pose object
        """
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
        return pose
    
    def _copy_pose(self, source_pose):
        """Create a copy of a Pose object.
        
        Args:
            source_pose: geometry_msgs.msg.Pose object
            
        Returns:
            New Pose object with copied values
        """
        from geometry_msgs.msg import Pose
        pose = Pose()
        pose.position.x = source_pose.position.x
        pose.position.y = source_pose.position.y
        pose.position.z = source_pose.position.z
        pose.orientation.x = source_pose.orientation.x
        pose.orientation.y = source_pose.orientation.y
        pose.orientation.z = source_pose.orientation.z
        pose.orientation.w = source_pose.orientation.w
        return pose
    
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
        
        handler = self._get_arm_handler(arm)
        if handler is None:
            logger.warning(f"{arm} arm handler is None, cannot create marker")
            return
        
        frame_id = handler.get_frame_id()
        if not frame_id:
            frame_id = self._default_pose_frame()
            logger.warning(
                "No pose frame_id yet for %s arm; using %r until PoseStamped arrives",
                arm,
                frame_id,
            )
        
        if arm == "left" and self._marker_base_frame is None:
            self._marker_base_frame = frame_id
            logger.debug(f"Set marker base_frame to '{frame_id}' from left arm handler")
        
        link_frame_path = (
            self._get_link_frame_path(frame_id) if self.attach_to_urdf_frames else None
        )
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
            logger.debug(
                "Frame '%s' is a URDF link, binding marker to '%s' for %s arm",
                frame_id,
                link_frame_path,
                arm,
            )
        else:
            prefix = self.marker_path_prefix or "/frames"
            frame_node_name = f"{prefix}/{arm}_arm_frame"
            marker_name = f"{frame_node_name}/ee_target"
            scene_frame = self._get_scene_frame() or "scene"
            logger.info(
                "Frame '%s' is not a URDF link; independent marker in scene frame '%s' for %s arm",
                frame_id,
                scene_frame,
                arm,
            )
            if arm not in self._arm_frame_nodes:
                try:
                    frame_node = self.server.scene.add_frame(
                        frame_node_name,
                        show_axes=False,
                    )
                    frame_node.position = (0.0, 0.0, 0.0)
                    frame_node.wxyz = (1.0, 0.0, 0.0, 0.0)
                    self._arm_frame_nodes[arm] = frame_node
                    # Keep legacy attribute for first arm (back-compat).
                    if self._ee_frame_node is None:
                        self._ee_frame_node = frame_node
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
            position, wxyz = self._extract_pose_data(current_pose)
            converted = self._pose_frame_to_scene(
                frame_id,
                position,
                wxyz,
                attached_to_urdf_link=is_urdf_link,
            )
            if converted is None:
                logger.warning(
                    "Creating %s arm marker at fallback pose; TF %s→scene unavailable",
                    arm,
                    frame_id,
                )
                position, wxyz = (0.5, 0.0, 0.5), (1.0, 0.0, 0.0, 0.0)
            else:
                position, wxyz = converted

        try:
            logger.debug(
                "Creating %s arm marker at '%s' scene_pos=%s (pose_frame=%s)",
                arm,
                marker_name,
                position,
                frame_id,
            )
            marker = self.server.scene.add_transform_controls(
                marker_name,
                scale=self.config.marker_scale,
                position=position,
                wxyz=wxyz,
            )
            marker.visible = False
            pose = (position, wxyz)
            if arm == "left":
                self._left_ee_marker = marker
                self._last_left_ee_pose = pose
            else:
                self._right_ee_marker = marker
                self._last_right_ee_pose = pose
        except Exception as e:
            logger.error(f"Failed to create {arm} arm marker at path '{marker_name}': {e}", exc_info=True)
            if arm == "left":
                self._left_ee_marker = None
                self._last_left_ee_pose = None
            else:
                self._right_ee_marker = None
                self._last_right_ee_pose = None

    def _head_marker_link_name(self) -> str:
        cfg = getattr(self.ros2_interface, "config", None) if self.ros2_interface else None
        name = getattr(cfg, "head_marker_link_name", None) if cfg is not None else None
        return (name or "head_link2").strip() or "head_link2"

    def _get_body_or_head_source_pose(self, kind: str, *, allow_tf_seed: bool = True):
        """Return (pose, frame_id) used to seed/sync body or head markers.

        ``allow_tf_seed`` enables ATM-style TF bootstrap for the head marker when
        ``head_current_target`` has not arrived yet. Continuous target sync should
        pass ``allow_tf_seed=False`` so live TF does not fight user drag.
        """
        iface = self.ros2_interface
        if iface is None:
            return None, None
        if kind == "body":
            pose = getattr(iface, "body_current_target_pose", None) or getattr(
                iface, "body_current_pose", None
            )
            frame_id = (
                getattr(iface, "body_current_target_pose_frame_id", None)
                or getattr(iface, "body_current_pose_frame_id", None)
                or self._body_frame_id
                or self._default_pose_frame()
            )
            return pose, frame_id

        pose = getattr(iface, "head_current_target_pose", None)
        frame_id = (
            getattr(iface, "head_current_target_pose_frame_id", None)
            or self._head_frame_id
            or self._body_frame_id
            or self._default_pose_frame()
        )
        if pose is not None or not allow_tf_seed:
            return pose, frame_id

        # Seed from TF: pose_frame <- head_link (same as ATM HeadMarker.syncPoseFromTf).
        link = self._head_marker_link_name()
        tf_pose = self._lookup_tf_pose(frame_id, link)
        if tf_pose is None:
            return None, frame_id
        from geometry_msgs.msg import Pose

        pose_msg = Pose()
        pose_msg.position.x, pose_msg.position.y, pose_msg.position.z = tf_pose[0]
        (
            pose_msg.orientation.w,
            pose_msg.orientation.x,
            pose_msg.orientation.y,
            pose_msg.orientation.z,
        ) = tf_pose[1]
        return pose_msg, frame_id

    def _init_body_or_head_marker(self, kind: str) -> None:
        """Create an independent transform-controls marker for body or head tracking."""
        if self.server is None:
            return
        self._refresh_body_head_frame_ids()
        pose, frame_id = self._get_body_or_head_source_pose(kind)
        if kind == "body":
            self._body_frame_id = frame_id
        else:
            self._head_frame_id = frame_id

        if pose is None:
            position = (0.0, 0.0, 0.9) if kind == "body" else (0.0, 0.0, 1.5)
            wxyz = (1.0, 0.0, 0.0, 0.0)
        else:
            position, wxyz = self._extract_pose_data(pose)
            converted = self._pose_frame_to_scene(
                frame_id, position, wxyz, attached_to_urdf_link=False
            )
            if converted is None:
                position = (0.0, 0.0, 0.9) if kind == "body" else (0.0, 0.0, 1.5)
                wxyz = (1.0, 0.0, 0.0, 0.0)
            else:
                position, wxyz = converted

        prefix = f"{self.marker_path_prefix}/" if self.marker_path_prefix else ""
        marker_name = f"{prefix}{kind}_target"
        try:
            marker = self.server.scene.add_transform_controls(
                marker_name,
                scale=self.config.marker_scale,
                position=position,
                wxyz=wxyz,
            )
            marker.visible = False
            pose_tuple = (position, wxyz)
            if kind == "body":
                self._body_marker = marker
                self._last_body_marker_pose = pose_tuple
            else:
                self._head_marker = marker
                self._last_head_marker_pose = pose_tuple
            logger.debug("Created %s marker at %s (frame=%s)", kind, position, frame_id)
        except Exception as e:
            logger.error("Failed to create %s marker: %s", kind, e, exc_info=True)
            if kind == "body":
                self._body_marker = None
                self._last_body_marker_pose = None
            else:
                self._head_marker = None
                self._last_head_marker_pose = None

    def _reset_body_or_head_marker_position(self, kind: str) -> None:
        marker = self._body_marker if kind == "body" else self._head_marker
        if marker is None:
            return
        pose, frame_id = self._get_body_or_head_source_pose(kind)
        if pose is None:
            logger.debug("No source pose to reset %s marker", kind)
            return
        if kind == "body":
            self._body_frame_id = frame_id
        else:
            self._head_frame_id = frame_id
        position, wxyz = self._extract_pose_data(pose)
        converted = self._pose_frame_to_scene(
            frame_id, position, wxyz, attached_to_urdf_link=False
        )
        if converted is None:
            return
        position, wxyz = converted
        marker.position = position
        marker.wxyz = wxyz
        self._store_last_marker_pose(kind, (position, wxyz))

    def _update_body_or_head_marker_from_target(self, kind: str) -> bool:
        marker = self._body_marker if kind == "body" else self._head_marker
        if marker is None:
            return False
        pose, frame_id = self._get_body_or_head_source_pose(kind, allow_tf_seed=False)
        last_target = (
            self._last_body_target_pose if kind == "body" else self._last_head_target_pose
        )
        if pose is None:
            if kind == "body":
                self._last_body_target_pose = None
            else:
                self._last_head_target_pose = None
            return False

        target_changed = (
            last_target is None
            or last_target.position.x != pose.position.x
            or last_target.position.y != pose.position.y
            or last_target.position.z != pose.position.z
            or last_target.orientation.x != pose.orientation.x
            or last_target.orientation.y != pose.orientation.y
            or last_target.orientation.z != pose.orientation.z
            or last_target.orientation.w != pose.orientation.w
        )
        if not target_changed:
            return False

        if kind == "body":
            self._body_frame_id = frame_id
        else:
            self._head_frame_id = frame_id
        position, wxyz = self._extract_pose_data(pose)
        converted = self._pose_frame_to_scene(
            frame_id, position, wxyz, attached_to_urdf_link=False
        )
        if converted is None:
            return False
        position, wxyz = converted
        marker.position = position
        marker.wxyz = wxyz
        copied = self._copy_pose(pose)
        if kind == "body":
            self._last_body_target_pose = copied
            self._last_body_marker_pose = (position, wxyz)
        else:
            self._last_head_target_pose = copied
            self._last_head_marker_pose = (position, wxyz)
        return True
    
    def _is_ocs2_mode(self) -> bool:
        """Check if robot is in OCS2 mode (FSM state == OCS2)."""
        if self.ros2_interface is None or not self.ros2_interface.is_connected:
            return False
        
        fsm_state = self.ros2_interface.get_fsm_state()
        return fsm_state == 3

    def _body_marker_supported(self) -> bool:
        if self.ros2_interface is None:
            return False
        return bool(
            getattr(self.ros2_interface, "is_wbc", False)
            or getattr(self.ros2_interface, "body_target_pub", None) is not None
        )

    def _head_marker_supported(self) -> bool:
        if self.ros2_interface is None:
            return False
        cap = getattr(self.ros2_interface, "wbc_capability", None)
        if cap is not None and not cap.head_tracking_ee_enabled:
            return False
        return bool(
            getattr(self.ros2_interface, "is_wbc", False)
            or getattr(self.ros2_interface, "head_target_pub", None) is not None
        )

    def _should_show_body_marker(self) -> bool:
        """Match ATM: OCS2 + BODY_TRACKING and not HEAD_FORWARD."""
        if not self._is_ocs2_mode() or not self._body_marker_supported():
            return False
        state = getattr(self.ros2_interface, "wbc_state", None)
        if state is None:
            return False
        return (
            state.body_state == WbcCurrentState.BODY_TRACKING
            and state.head_state != WbcCurrentState.HEAD_FORWARD
        )

    def _should_show_head_marker(self) -> bool:
        """Match ATM: OCS2 + HEAD_TRACKING."""
        if not self._is_ocs2_mode() or not self._head_marker_supported():
            return False
        state = getattr(self.ros2_interface, "wbc_state", None)
        if state is None:
            return False
        return state.head_state == WbcCurrentState.HEAD_TRACKING
    
    def _update_marker_visibility(
        self,
        is_ocs2: bool,
        *,
        show_body: Optional[bool] = None,
        show_head: Optional[bool] = None,
    ):
        """Update marker visibility based on FSM / WBC tracking state."""
        if self._left_ee_marker is not None:
            self._left_ee_marker.visible = is_ocs2
        if self._right_ee_marker is not None:
            self._right_ee_marker.visible = is_ocs2
        if show_body is None:
            show_body = self._should_show_body_marker()
        if show_head is None:
            show_head = self._should_show_head_marker()
        if self._body_marker is not None:
            self._body_marker.visible = bool(show_body)
        if self._head_marker is not None:
            self._head_marker.visible = bool(show_head)
    
    def _update_gui_visibility(self, is_ocs2: bool):
        """Update GUI control visibility based on FSM state and publish mode."""
        if self._marker_publish_mode_dropdown is not None:
            self._marker_publish_mode_dropdown.visible = is_ocs2
        
        if self._send_marker_pose_button is not None:
            self._send_marker_pose_button.visible = is_ocs2 and not self._marker_continuous_publish
    
    def _update_single_marker_from_target_pose(self, arm: str) -> bool:
        """Update a single marker position from current target pose if it has changed.
        
        Args:
            arm: "left" or "right"
            
        Returns:
            True if marker was updated, False otherwise.
        """
        marker = self._left_ee_marker if arm == "left" else self._right_ee_marker
        handler = self._get_arm_handler(arm)
        frame_id = self._left_frame_id if arm == "left" else self._right_frame_id
        
        if marker is None or handler is None or frame_id is None:
            return False
        
        target_pose = handler.get_target_pose()
        last_target_pose = self._last_left_target_pose if arm == "left" else self._last_right_target_pose
        
        if target_pose is None:
            if last_target_pose is not None:
                if arm == "left":
                    self._last_left_target_pose = None
                else:
                    self._last_right_target_pose = None
            return False
        
        target_changed = (
            last_target_pose is None or
            last_target_pose.position.x != target_pose.position.x or
            last_target_pose.position.y != target_pose.position.y or
            last_target_pose.position.z != target_pose.position.z or
            last_target_pose.orientation.x != target_pose.orientation.x or
            last_target_pose.orientation.y != target_pose.orientation.y or
            last_target_pose.orientation.z != target_pose.orientation.z or
            last_target_pose.orientation.w != target_pose.orientation.w
        )
        
        if target_changed:
            position, wxyz = self._extract_pose_data(target_pose)
            converted = self._pose_frame_to_scene(
                frame_id,
                position,
                wxyz,
                attached_to_urdf_link=self._arm_is_urdf_attached(arm),
            )
            if converted is None:
                return False
            position, wxyz = converted
            marker.position = position
            marker.wxyz = wxyz
            if arm == "left":
                self._last_left_target_pose = self._copy_pose(target_pose)
            else:
                self._last_right_target_pose = self._copy_pose(target_pose)
            logger.debug(f"Updated {arm} marker position from target pose: {position}")
            return True
        
        return False
    
    def _update_markers_from_target_poses(self) -> bool:
        """Update marker positions from current target poses if they have changed.
        
        Returns:
            True if any marker was updated from target pose in this call, False otherwise.
        """
        if self.ros2_interface is None or not self.ros2_interface.is_connected:
            return False
        
        current_time = time.time()
        time_since_last_update = current_time - self._last_marker_update_time
        if time_since_last_update < self._marker_update_cooldown:
            return False
        
        left_updated = self._update_single_marker_from_target_pose("left")
        right_updated = self._update_single_marker_from_target_pose("right")
        body_updated = False
        head_updated = False
        if self._should_show_body_marker():
            body_updated = self._update_body_or_head_marker_from_target("body")
        if self._should_show_head_marker():
            head_updated = self._update_body_or_head_marker_from_target("head")
        
        return left_updated or right_updated or body_updated or head_updated
    
    def _reset_single_marker_position(self, arm: str):
        """Reset a single marker position to current end-effector pose.
        
        Args:
            arm: "left" or "right"
        """
        marker = self._left_ee_marker if arm == "left" else self._right_ee_marker
        handler = self._get_arm_handler(arm)
        frame_id = self._left_frame_id if arm == "left" else self._right_frame_id
        
        if marker is None or handler is None or frame_id is None:
            return
        
        current_pose = handler.get_pose()
        if current_pose is None:
            current_pose = handler.get_target_pose()
        
        if current_pose is not None:
            position, wxyz = self._extract_pose_data(current_pose)
            converted = self._pose_frame_to_scene(
                frame_id,
                position,
                wxyz,
                attached_to_urdf_link=self._arm_is_urdf_attached(arm),
            )
            if converted is None:
                logger.warning(
                    "Could not reset %s arm marker (TF %s → %s missing)",
                    arm,
                    frame_id,
                    self._get_scene_frame(),
                )
                return
            position, wxyz = converted
            marker.position = position
            marker.wxyz = wxyz
            pose = (position, wxyz)
            if arm == "left":
                self._last_left_ee_pose = pose
            else:
                self._last_right_ee_pose = pose
            logger.debug(
                "Reset %s arm marker to %s (pose_frame=%s, scene=%s)",
                arm,
                position,
                frame_id,
                self._get_scene_frame(),
            )
        else:
            logger.warning(f"Could not get current pose for {arm} arm, cannot reset marker")
    
    def _reset_marker_positions(self):
        """Reset marker positions to current end-effector poses when entering OCS2 mode."""
        if self.ros2_interface is None or not self.ros2_interface.is_connected:
            return
        
        self._reset_single_marker_position("left")
        self._reset_single_marker_position("right")
        if self._should_show_body_marker():
            self._reset_body_or_head_marker_position("body")
        if self._should_show_head_marker():
            self._reset_body_or_head_marker_position("head")
    
    def _scene_tuple_to_command_tuple(self, arm: str, pose_tuple: Tuple) -> Tuple:
        """Convert a scene-frame marker pose tuple into the command frame."""
        position, wxyz = pose_tuple
        if arm == "body":
            frame_id = self._body_frame_id
            attached = False
        elif arm == "head":
            frame_id = self._head_frame_id
            attached = False
        else:
            frame_id = self._left_frame_id if arm == "left" else self._right_frame_id
            attached = self._arm_is_urdf_attached(arm)
        return self._scene_to_pose_frame(
            frame_id,
            position,
            wxyz,
            attached_to_urdf_link=attached,
        )

    def _send_pose_command(self, arm: str, pose_tuple: Tuple, use_stamped: bool = True):
        """Send pose command for the specified marker.
        
        Args:
            arm: "left", "right", "body", or "head"
            pose_tuple: (position, wxyz) tuple in scene / marker coordinates
            use_stamped: If True, publish PoseStamped to ``*/stamped``;
                if False, publish Pose to ``/left_target`` / ``/body_target`` / etc.
        """
        if self.ros2_interface is None or not self.ros2_interface.is_connected:
            return
        
        try:
            cmd_tuple = self._scene_tuple_to_command_tuple(arm, pose_tuple)
            if cmd_tuple is None:
                logger.warning(
                    "Not sending %s pose: scene→pose_frame TF unavailable", arm
                )
                return
            pose = self._create_pose_from_tuple(cmd_tuple)

            if arm in ("body", "head"):
                frame_id = (
                    (self._body_frame_id if arm == "body" else self._head_frame_id)
                    or self._marker_base_frame
                    or self._default_pose_frame()
                )
                if arm == "body":
                    if use_stamped:
                        self.ros2_interface.send_body_target_stamped(frame_id, pose)
                    else:
                        self.ros2_interface.send_body_target(pose)
                else:
                    if use_stamped:
                        self.ros2_interface.send_head_target_stamped(frame_id, pose)
                    else:
                        self.ros2_interface.send_head_target(pose)
                return

            handler = self._get_arm_handler(arm)
            if handler is None:
                logger.warning(f"{arm} arm handler not available")
                return
            
            frame_id = (
                (self._left_frame_id if arm == "left" else self._right_frame_id)
                or self._marker_base_frame
                or self._default_pose_frame()
            )
            if use_stamped:
                handler.send_target_stamped(frame_id, pose)
            else:
                # Unstamped topics are interpreted as controller base_frame.
                # Only safe when command numbers are already in that frame.
                handler.send_target(pose)
        except Exception as e:
            logger.warning(f"Failed to send {arm} arm pose command: {e}")
    
