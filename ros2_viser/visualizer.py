"""ROS2 Viser Visualizer - Visualize ROS2 robots using Viser."""

import hashlib
import logging
import threading
import time
import warnings
from dataclasses import dataclass, field
from io import StringIO
from typing import Dict, Optional

import numpy as np
import viser
import yourdfpy
from viser.extras import ViserUrdf

from ros2_robot_interface import ROS2RobotInterface, ROS2RobotInterfaceConfig
from ros2_robot_interface.utils.discovery import discover_robot_description_topics
from ros2_robot_interface.utils.topic_namespace import topic_prefix_from_robot_description
from .config import ROS2ViserConfig
from .end_effector_marker import EndEffectorMarkerManager
from .i18n import get_translator, set_global_language
from .network_utils import get_local_ip, get_viser_server_port, display_server_info
from .panels import FSMPanel, GripperPanel, JointPanel, HardwarePanel, ControllerPanel

logger = logging.getLogger(__name__)

# Dropdown sentinel; display text comes from i18n ("empty_workspace").
_EMPTY_WORKSPACE_VALUE = ""
# Visualization-frame sentinel: keep URDF root fixed at the scene origin.
_VIZ_FRAME_FIXED = "__fixed__"
_PREFERRED_VIZ_FRAMES = ("world", "map", "odom", "base_footprint", "base_link")


@dataclass
class RobotSession:
    """Cached per-robot ROS interface + Viser scene state."""

    robot_description_topic: str
    topic_prefix: str
    root_node_name: str
    interface: ROS2RobotInterface
    urdf: yourdfpy.URDF
    urdf_vis: ViserUrdf
    robot_base_frame: viser.FrameHandle
    urdf_root_link: Optional[str]
    urdf_hash: str
    urdf_all_joint_names: list[str] = field(default_factory=list)
    urdf_actuated_joint_names: list[str] = field(default_factory=list)
    mimic_joint_info: dict = field(default_factory=dict)
    joint_name_to_urdf_index: dict[str, int] = field(default_factory=dict)
    visible: bool = True


@dataclass
class ClientView:
    """Per-browser GUI / control binding (shared scene, private panels)."""

    client_id: int
    client: object  # viser.ClientHandle
    selected_robot_topic: Optional[str] = None
    display_folder: Optional[object] = None
    robot_dropdown: Optional[object] = None
    refresh_robots_button: Optional[object] = None
    viz_frame_dropdown: Optional[object] = None
    refresh_viz_frames_button: Optional[object] = None
    language_dropdown: Optional[object] = None
    show_visual_checkbox: Optional[object] = None
    show_collision_checkbox: Optional[object] = None
    show_current_target_checkbox: Optional[object] = None
    fsm_panel: Optional[FSMPanel] = None
    gripper_panel: Optional[GripperPanel] = None
    joint_panel: Optional[JointPanel] = None
    hardware_panel: Optional[HardwarePanel] = None
    controller_panel: Optional[ControllerPanel] = None
    marker_manager: Optional[EndEffectorMarkerManager] = None
    switching: bool = False
    controls_bound: bool = False


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

        # Per-robot cache: robot_description topic -> session (interfaces stay connected).
        self._robot_sessions: Dict[str, RobotSession] = {}
        self._active_session_key: Optional[str] = None
        self._active_root_node_name: str = config.root_node_name
        # Per-browser views (client.gui panels); scene robots are shared and always shown.
        self._client_views: Dict[int, ClientView] = {}
        
        # State
        self._running = False
        self._urdf_received = False
        self._update_thread: Optional[threading.Thread] = None
        
        
        # URDF reloading
        self._last_urdf_hash: Optional[str] = None  # Hash of last URDF to detect changes
        
        # Joint name mapping (from joint_states to URDF joint order)
        self.joint_name_to_index: dict[str, int] = {}
        self.urdf_joint_names: list[str] = []
        self.urdf_all_joint_names: list[str] = []
        self.urdf_actuated_joint_names: list[str] = []
        self.mimic_joint_info: dict = {}
        
        # Precomputed mapping for performance optimization
        self._joint_name_to_urdf_index: dict[str, int] = {}  # Maps joint name to URDF index (computed once)
        
        # Cache last joint positions to avoid unnecessary updates
        self._last_joint_positions: Optional[np.ndarray] = None
        
        # GUI panels
        self._fsm_panel: Optional[FSMPanel] = None
        self._gripper_panel: Optional[GripperPanel] = None
        self._joint_panel: Optional[JointPanel] = None
        self._hardware_panel: Optional[HardwarePanel] = None
        self._controller_panel: Optional[ControllerPanel] = None
        
        # Display control panel
        self._display_folder_handle: Optional[viser.GuiFolderHandle] = None
        self._show_visual_checkbox: Optional[viser.GuiCheckboxHandle] = None
        self._show_collision_checkbox: Optional[viser.GuiCheckboxHandle] = None
        self._show_current_target_checkbox: Optional[viser.GuiCheckboxHandle] = None
        self._language_dropdown: Optional[viser.GuiDropdownHandle] = None
        self._show_visual: bool = True
        self._show_collision: bool = False
        self._show_current_target: bool = bool(
            getattr(config, "show_current_target_axes", True)
        )

        # Robot namespace picker
        self._discovered_robot_topics: list[str] = []
        self._selected_robot_topic: Optional[str] = None
        self._robot_dropdown: Optional[viser.GuiDropdownHandle] = None
        self._refresh_robots_button: Optional[viser.GuiButtonHandle] = None
        self._switching_robot = False
        self._robot_switch_lock = threading.Lock()
        # Serialize scene graph mutations (switch thread vs update loop).
        self._scene_lock = threading.Lock()
        # After an intentional connect/switch, skip the update-loop "reconnected" path once.
        self._skip_reconnect_detection = False

        # Visualization TF frame (scene world). Empty / fixed => URDF root at origin.
        self._viz_frame_dropdown: Optional[viser.GuiDropdownHandle] = None
        self._refresh_viz_frames_button: Optional[viser.GuiButtonHandle] = None
        self._selected_viz_frame: str = (
            config.visualization_frame.strip()
            if config.visualization_frame and config.visualization_frame.strip()
            else _VIZ_FRAME_FIXED
        )
        self._discovered_viz_frames: list[str] = []
        self._urdf_root_link: Optional[str] = None
        self._robot_base_frame: Optional[viser.FrameHandle] = None
        self._last_viz_tf_warn_time: float = 0.0
        
        # End-effector marker manager
        self._marker_manager: Optional[EndEffectorMarkerManager] = None
        
        # Note: No direct subscription - use ros2_interface.get_robot_description() instead

    @staticmethod
    def _session_ros_node_name(topic_prefix: str) -> str:
        if not topic_prefix:
            return "ros2_viser_node"
        suffix = topic_prefix.strip("/").replace("/", "_")
        return f"ros2_viser_{suffix}"

    def _session_root_node_name(self, robot_description_topic: str) -> str:
        prefix = topic_prefix_from_robot_description(robot_description_topic)
        base = self.config.root_node_name.rstrip("/") or "/robot"
        if not prefix:
            return base
        suffix = prefix.strip("/").replace("/", "_")
        return f"{base}_{suffix}"

    def _bind_session(self, session: Optional[RobotSession]) -> None:
        """Point convenience attributes at the active session (or clear them)."""
        if session is None:
            self._active_session_key = None
            self.ros2_interface = None
            self.urdf = None
            self.urdf_vis = None
            self._robot_base_frame = None
            self._urdf_root_link = None
            self._urdf_received = False
            self._last_urdf_hash = None
            self.urdf_all_joint_names = []
            self.urdf_actuated_joint_names = []
            self.mimic_joint_info = {}
            self._joint_name_to_urdf_index = {}
            self._active_root_node_name = self.config.root_node_name
            return

        self._active_session_key = session.robot_description_topic
        self.ros2_interface = session.interface
        self.urdf = session.urdf
        self.urdf_vis = session.urdf_vis
        self._robot_base_frame = session.robot_base_frame
        self._urdf_root_link = session.urdf_root_link
        self._urdf_received = True
        self._last_urdf_hash = session.urdf_hash
        self.urdf_all_joint_names = session.urdf_all_joint_names
        self.urdf_actuated_joint_names = session.urdf_actuated_joint_names
        self.mimic_joint_info = session.mimic_joint_info
        self._joint_name_to_urdf_index = session.joint_name_to_urdf_index
        self._active_root_node_name = session.root_node_name

    def _set_session_scene_visible(self, session: RobotSession, visible: bool) -> None:
        session.visible = visible
        try:
            session.robot_base_frame.visible = visible
        except Exception as e:
            logger.debug(f"Failed to set session visibility: {e}")

    def _cleanup_control_ui(self) -> None:
        """Tear down panels/markers without touching cached ROS interfaces or scenes."""
        if self._marker_manager is not None:
            try:
                self._marker_manager.cleanup()
            except Exception as e:
                logger.warning(f"Error cleaning up marker manager: {e}")
            self._marker_manager = None

        for attr in (
            "_fsm_panel",
            "_gripper_panel",
            "_joint_panel",
            "_hardware_panel",
            "_controller_panel",
        ):
            panel = getattr(self, attr)
            if panel is not None:
                try:
                    panel.cleanup()
                except Exception as e:
                    logger.warning(f"Error cleaning up {attr}: {e}")
                setattr(self, attr, None)

    def _deactivate_current_session(self) -> None:
        """Unbind global convenience pointers; keep all robots visible in the scene."""
        # Shared-scene multi-client mode: never hide other robots on switch.
        self._cleanup_control_ui()
        self._bind_session(None)

    def _ensure_scene_environment(self) -> None:
        """Re-apply ground + default lighting after heavy scene mutations.

        Switching robots rebuilds large ViserUrdf meshes from a worker thread;
        that message flood can leave the browser without default lights / HDRI.
        """
        if self.server is None:
            return
        try:
            # Keep a mild studio HDRI so meshes stay lit even if default lights glitch.
            self.server.scene.configure_environment_map(
                "warehouse",
                background=False,
                environment_intensity=0.85,
            )
            self.server.scene.configure_default_lights(enabled=True, cast_shadow=False)
            self.server.scene.add_grid("/ground", width=2, height=2)
        except Exception as e:
            logger.debug(f"Failed to restore scene environment: {e}")

    def _destroy_session(self, topic: str) -> None:
        """Fully remove one cached session (scene + disconnect)."""
        session = self._robot_sessions.pop(topic, None)
        if session is None:
            return
        if self._active_session_key == topic:
            self._cleanup_control_ui()
            self._bind_session(None)
        with self._scene_lock:
            try:
                session.urdf_vis.remove()
            except Exception as e:
                logger.debug(f"Error removing session URDF: {e}")
            try:
                session.robot_base_frame.remove()
            except Exception as e:
                logger.debug(f"Error removing session base frame: {e}")
            self._ensure_scene_environment()
        try:
            if session.interface.is_connected:
                session.interface.disconnect()
        except Exception as e:
            logger.debug(f"Error disconnecting session interface: {e}")

    def _destroy_all_sessions(self) -> None:
        for topic in list(self._robot_sessions.keys()):
            self._destroy_session(topic)
    
    def _parse_urdf_string(self, urdf_string: str, urdf_hash: Optional[str] = None):
        """Reload the active session's URDF visualization after a description change."""
        if not urdf_string or not urdf_string.strip():
            logger.warning("URDF string is empty, skipping parse")
            return
        if not self._active_session_key or self._active_session_key not in self._robot_sessions:
            logger.warning("No active robot session; cannot reload URDF visualization")
            return

        session = self._robot_sessions[self._active_session_key]
        try:
            logger.info("URDF changed for active session, rebuilding visualization...")
            self._cleanup_control_ui()
            with self._scene_lock:
                try:
                    session.urdf_vis.remove()
                except Exception as e:
                    logger.warning(f"Error removing old URDF visualization: {e}")
                try:
                    session.robot_base_frame.remove()
                except Exception as e:
                    logger.warning(f"Error removing old robot base frame: {e}")

            (
                urdf,
                urdf_all_joint_names,
                urdf_actuated_joint_names,
                mimic_joint_info,
                joint_name_to_urdf_index,
            ) = self._load_urdf_model(urdf_string)
            robot_base_frame, urdf_vis, urdf_root_link = self._create_urdf_scene(
                urdf, session.root_node_name
            )

            session.urdf = urdf
            session.urdf_vis = urdf_vis
            session.robot_base_frame = robot_base_frame
            session.urdf_root_link = urdf_root_link
            session.urdf_hash = urdf_hash or session.urdf_hash
            session.urdf_all_joint_names = urdf_all_joint_names
            session.urdf_actuated_joint_names = urdf_actuated_joint_names
            session.mimic_joint_info = mimic_joint_info
            session.joint_name_to_urdf_index = joint_name_to_urdf_index
            session.visible = True

            self._bind_session(session)
            logger.info("✅ URDF parsed successfully")
            self._update_urdf_display()
            self._refresh_viz_frame_dropdown(keep_selection=True)
            for view in self._client_views.values():
                if view.selected_robot_topic == session.robot_description_topic:
                    self._reinitialize_client_panels(view)
            time.sleep(0.2)
        except Exception as e:
            logger.error(f"Failed to parse URDF: {e}", exc_info=True)
            logger.error(f"URDF data length: {len(urdf_string) if urdf_string else 0}")
            if urdf_string:
                logger.error(f"URDF data preview (first 500 chars): {urdf_string[:500]}")
            return
    
    def _create_ros2_interface(
        self,
        *,
        topic_prefix: str = "",
        robot_description_topic: str = "/robot_description",
        joint_states_topic: Optional[str] = None,
    ) -> ROS2RobotInterface:
        """Create and connect a ROS2RobotInterface for one robot namespace."""
        # Put the node in the robot namespace so TransformListener can remap
        # /tf -> /{ns}/tf (see ROS2RobotInterface.connect).
        node_namespace = topic_prefix.strip("/") if topic_prefix else ""
        interface_config = ROS2RobotInterfaceConfig(
            joint_states_topic=joint_states_topic or self.config.joint_states_topic,
            node_name=self._session_ros_node_name(topic_prefix),
            namespace=node_namespace,
            gripper_enabled=self.config.enable_gripper_panel,
            # Visualization tolerates brief gaps better than control clients.
            joint_state_timeout=3.0,
            topic_prefix=topic_prefix,
            robot_description_topic=robot_description_topic,
            nav_enabled=False,
        )
        interface = ROS2RobotInterface(interface_config)
        if self.config.auto_connect:
            interface.connect()
            logger.info(
                "Connected to ROS2 Robot Interface "
                f"(node={interface_config.node_name!r}, topic_prefix={topic_prefix!r}, "
                f"description={robot_description_topic})"
            )
        return interface

    def _init_ros2_interface(
        self,
        *,
        topic_prefix: str = "",
        robot_description_topic: str = "/robot_description",
        joint_states_topic: Optional[str] = None,
    ):
        """Initialize ROS2 Robot Interface and bind it as the active interface."""
        self.ros2_interface = self._create_ros2_interface(
            topic_prefix=topic_prefix,
            robot_description_topic=robot_description_topic,
            joint_states_topic=joint_states_topic,
        )

    def _empty_workspace_label(self) -> str:
        return self.translator("empty_workspace")

    def _default_workspace_label(self) -> str:
        """Label for a robot published at the root, with no namespace."""
        return self.translator("default_workspace")

    def _namespace_label_for_topic(self, topic: str) -> str:
        """Dropdown label: namespace only (``/g1d``), not ``.../robot_description``."""
        try:
            prefix = topic_prefix_from_robot_description(topic)
        except ValueError:
            prefix = topic.rsplit("/", 1)[0] if topic.endswith("/robot_description") else topic
        if not prefix:
            return self._default_workspace_label()
        return prefix if prefix.startswith("/") else f"/{prefix}"

    def _robot_dropdown_options(self) -> list[str]:
        labels = [self._empty_workspace_label()]
        for topic in self._discovered_robot_topics:
            label = self._namespace_label_for_topic(topic)
            if label not in labels:
                labels.append(label)
        return labels

    def _dropdown_value_for_topic(self, topic: Optional[str]) -> str:
        if not topic:
            return self._empty_workspace_label()
        return self._namespace_label_for_topic(topic)

    def _topic_from_dropdown_value(self, value: str) -> Optional[str]:
        if value == self._empty_workspace_label() or value == _EMPTY_WORKSPACE_VALUE:
            return None
        for topic in self._discovered_robot_topics:
            if self._namespace_label_for_topic(topic) == value:
                return topic
        # Back-compat if an old full topic string is still selected.
        if value in self._discovered_robot_topics:
            return value
        if value in ("/", self._default_workspace_label()):
            return "/robot_description"
        ns = value if value.startswith("/") else f"/{value}"
        return f"{ns}/robot_description"

    def _scan_robot_description_topics(self) -> list[str]:
        """Scan the ROS graph for ``*/robot_description`` String topics."""
        try:
            topics = discover_robot_description_topics()
            logger.info(f"Discovered {len(topics)} robot_description topic(s): {topics}")
            return topics
        except Exception as e:
            logger.warning(f"Failed to discover robot_description topics: {e}")
            return []

    def _set_robot_picker_enabled(self, enabled: bool) -> None:
        for handle in (
            self._robot_dropdown,
            self._refresh_robots_button,
            self._viz_frame_dropdown,
            self._refresh_viz_frames_button,
        ):
            if handle is None:
                continue
            try:
                handle.disabled = not enabled
            except Exception:
                pass

    def _fixed_viz_frame_label(self) -> str:
        return self.translator("viz_frame_fixed")

    def _viz_frame_dropdown_options(self) -> list[str]:
        """Build dropdown options: fixed label + preferred + discovered TF frames."""
        options: list[str] = [self._fixed_viz_frame_label()]
        seen = {_VIZ_FRAME_FIXED, self._fixed_viz_frame_label()}
        for name in list(_PREFERRED_VIZ_FRAMES) + list(self._discovered_viz_frames):
            if not name or name in seen:
                continue
            options.append(name)
            seen.add(name)
        if (
            self._urdf_root_link
            and self._urdf_root_link not in seen
            and self._urdf_root_link != self._fixed_viz_frame_label()
        ):
            options.append(self._urdf_root_link)
        return options

    def _dropdown_value_for_viz_frame(self, frame: str) -> str:
        if not frame or frame == _VIZ_FRAME_FIXED:
            return self._fixed_viz_frame_label()
        return frame

    def _viz_frame_from_dropdown_value(self, value: str) -> str:
        if value == self._fixed_viz_frame_label() or value == _VIZ_FRAME_FIXED:
            return _VIZ_FRAME_FIXED
        return value

    def _list_tf_frame_names(self) -> list[str]:
        """Union of TF frame ids across every cached robot's namespaced TF tree."""
        frames: set[str] = set()

        def _collect(iface: Optional[ROS2RobotInterface]) -> None:
            if iface is None or not iface.is_connected:
                return
            tf_buffer = getattr(iface, "tf_buffer", None)
            if tf_buffer is None:
                return
            try:
                import yaml

                raw = tf_buffer.all_frames_as_yaml()
                data = yaml.safe_load(raw) if raw else None
                if isinstance(data, dict):
                    frames.update(str(k) for k in data.keys())
            except Exception as e:
                logger.debug(f"Failed to list TF frames: {e}")

        for session in self._robot_sessions.values():
            _collect(session.interface)
        if not self._robot_sessions:
            _collect(self.ros2_interface)
        return sorted(frames)

    def _sync_client_viz_frame_dropdowns(self, options: list[str], selected_display: str) -> None:
        for view in self._client_views.values():
            if view.viz_frame_dropdown is None:
                continue
            try:
                view.viz_frame_dropdown.options = options
                view.viz_frame_dropdown.value = selected_display
            except Exception as e:
                logger.debug(f"Failed to sync client viz-frame dropdown: {e}")

    def _prefer_mobile_viz_frame(self) -> Optional[str]:
        """Pick world/map/odom when present so mobile bases share the marker frame."""
        frames = set(self._discovered_viz_frames) or set(self._list_tf_frame_names())
        for name in ("world", "map", "odom"):
            if name in frames:
                return name
        return None

    def _refresh_viz_frame_dropdown(self, *, keep_selection: bool = True) -> None:
        previous = self._selected_viz_frame if keep_selection else _VIZ_FRAME_FIXED
        self._discovered_viz_frames = self._list_tf_frame_names()
        # If still on FIXED with no explicit config, auto-promote to world/map/odom
        # so multi-robot meshes and OCS2 markers share the same ground plane.
        if (
            previous == _VIZ_FRAME_FIXED
            and not (self.config.visualization_frame or "").strip()
        ):
            preferred = self._prefer_mobile_viz_frame()
            if preferred:
                previous = preferred
                self._selected_viz_frame = preferred
                self.config.visualization_frame = preferred
                logger.info(
                    "Auto-selected visualization frame %r (mobile base / OCS2 height align)",
                    preferred,
                )
                self._update_robot_base_from_tf()
                self._sync_marker_scene_context()
        options = self._viz_frame_dropdown_options()
        selected_display = self._dropdown_value_for_viz_frame(previous)
        if selected_display not in options:
            # Keep an explicitly configured / previously chosen TF frame even if
            # it is not in the graph yet (e.g. world before localization starts).
            if previous and previous != _VIZ_FRAME_FIXED and previous not in options:
                options = options + [previous]
                selected_display = previous
            else:
                selected_display = self._fixed_viz_frame_label()
                previous = _VIZ_FRAME_FIXED
        self._selected_viz_frame = (
            _VIZ_FRAME_FIXED
            if selected_display == self._fixed_viz_frame_label()
            else selected_display
        )
        self.config.visualization_frame = (
            "" if self._selected_viz_frame == _VIZ_FRAME_FIXED else self._selected_viz_frame
        )
        if self._viz_frame_dropdown is not None:
            try:
                self._viz_frame_dropdown.options = options
                self._viz_frame_dropdown.value = selected_display
            except Exception as e:
                logger.warning(f"Failed to update visualization frame dropdown: {e}")
        self._sync_client_viz_frame_dropdowns(options, selected_display)

    def _on_refresh_viz_frames_clicked(self) -> None:
        self._refresh_viz_frame_dropdown(keep_selection=True)
        logger.info(
            "Visualization frames refreshed (%d TF frame(s))",
            len(self._discovered_viz_frames),
        )

    def _sync_marker_scene_context(self) -> None:
        """Push current viz-frame / URDF-root into every marker manager."""
        viz = (
            ""
            if self._selected_viz_frame == _VIZ_FRAME_FIXED
            else self._selected_viz_frame
        )
        for view in self._client_views.values():
            if view.marker_manager is None:
                continue
            root = None
            if view.selected_robot_topic and view.selected_robot_topic in self._robot_sessions:
                root = self._robot_sessions[view.selected_robot_topic].urdf_root_link
            else:
                root = self._urdf_root_link
            view.marker_manager.set_scene_context(
                visualization_frame=viz,
                urdf_root_link=root,
            )
        if self._marker_manager is not None:
            self._marker_manager.set_scene_context(
                visualization_frame=viz,
                urdf_root_link=self._urdf_root_link,
            )

    def _on_viz_frame_dropdown_changed(self, value: str) -> None:
        frame = self._viz_frame_from_dropdown_value(value)
        if frame == self._selected_viz_frame:
            return
        self._selected_viz_frame = frame
        self.config.visualization_frame = (
            "" if frame == _VIZ_FRAME_FIXED else frame
        )
        logger.info(
            "Visualization frame set to %s",
            "URDF root (fixed)" if frame == _VIZ_FRAME_FIXED else frame,
        )
        if frame == _VIZ_FRAME_FIXED:
            self._reset_robot_base_pose()
        else:
            self._update_robot_base_from_tf()
        self._sync_marker_scene_context()
        # Keep every browser's viz-frame picker in sync (shared scene pose).
        selected_display = self._dropdown_value_for_viz_frame(frame)
        options = self._viz_frame_dropdown_options()
        if selected_display not in options:
            options = options + [selected_display]
        self._sync_client_viz_frame_dropdowns(options, selected_display)

    def _resolve_urdf_root_link(self) -> Optional[str]:
        if self.urdf is None:
            return None
        try:
            if hasattr(self.urdf, "base_link") and self.urdf.base_link:
                return str(self.urdf.base_link)
        except Exception:
            pass
        try:
            scene = getattr(self.urdf, "scene", None)
            if scene is not None and hasattr(scene, "graph"):
                base = scene.graph.base_frame
                if base:
                    return str(base)
        except Exception:
            pass
        return None

    def _reset_robot_base_pose(self) -> None:
        """Reset every session URDF root to the scene origin."""
        for session in self._robot_sessions.values():
            try:
                session.robot_base_frame.position = (0.0, 0.0, 0.0)
                session.robot_base_frame.wxyz = (1.0, 0.0, 0.0, 0.0)
            except Exception as e:
                logger.debug(f"Failed to reset robot base pose: {e}")
        if self._robot_base_frame is not None and not self._robot_sessions:
            try:
                self._robot_base_frame.position = (0.0, 0.0, 0.0)
                self._robot_base_frame.wxyz = (1.0, 0.0, 0.0, 0.0)
            except Exception as e:
                logger.debug(f"Failed to reset robot base pose: {e}")

    def _lookup_transform_quiet(
        self,
        target_frame: str,
        source_frame: str,
        interface: Optional[ROS2RobotInterface] = None,
    ):
        """TF lookup without spamming warnings on missing transforms."""
        iface = interface if interface is not None else self.ros2_interface
        if iface is None or not iface.is_connected:
            return None
        tf_buffer = getattr(iface, "tf_buffer", None)
        if tf_buffer is None:
            return None
        try:
            from rclpy.time import Time

            return tf_buffer.lookup_transform(target_frame, source_frame, Time())
        except Exception:
            return None

    def _apply_session_base_from_tf(self, session: RobotSession) -> None:
        """Place one session's URDF root in the selected visualization frame via TF."""
        frame = self._selected_viz_frame
        if not frame or frame == _VIZ_FRAME_FIXED:
            return
        root_link = session.urdf_root_link
        if not root_link:
            return
        if frame == root_link:
            try:
                session.robot_base_frame.position = (0.0, 0.0, 0.0)
                session.robot_base_frame.wxyz = (1.0, 0.0, 0.0, 0.0)
            except Exception:
                pass
            return

        transform = self._lookup_transform_quiet(
            frame, root_link, interface=session.interface
        )
        if transform is None:
            now = time.time()
            if now - self._last_viz_tf_warn_time > 5.0:
                logger.warning(
                    "No TF %s <- %s for %s; robot root stays at last pose",
                    frame,
                    root_link,
                    session.robot_description_topic,
                )
                self._last_viz_tf_warn_time = now
            return

        t = transform.transform.translation
        r = transform.transform.rotation
        try:
            session.robot_base_frame.position = (float(t.x), float(t.y), float(t.z))
            session.robot_base_frame.wxyz = (
                float(r.w),
                float(r.x),
                float(r.y),
                float(r.z),
            )
        except Exception as e:
            logger.debug(f"Failed to apply robot base TF pose: {e}")

    def _update_robot_base_from_tf(self) -> None:
        """Update every cached robot root from TF (shared scene)."""
        for session in self._robot_sessions.values():
            self._apply_session_base_from_tf(session)

    def _refresh_robot_dropdown(self, *, select_topic: Optional[str] = None) -> None:
        """Refresh discovered robots and update the dropdown options."""
        self._discovered_robot_topics = self._scan_robot_description_topics()
        if select_topic is not None:
            self._selected_robot_topic = select_topic
        options = self._robot_dropdown_options()
        initial = self._dropdown_value_for_topic(self._selected_robot_topic)
        if initial not in options:
            initial = self._empty_workspace_label()
            self._selected_robot_topic = None
        if self._robot_dropdown is not None:
            try:
                self._robot_dropdown.options = options
                self._robot_dropdown.value = initial
            except Exception as e:
                logger.warning(f"Failed to update robot dropdown: {e}")

    def _on_refresh_robots_clicked(self) -> None:
        if self._switching_robot:
            return
        previous = self._selected_robot_topic
        self._refresh_robot_dropdown(select_topic=previous)
        self._refresh_viz_frame_dropdown(keep_selection=True)
        logger.info("Robot list refreshed")

    def _on_robot_dropdown_changed(self, value: str) -> None:
        if self._switching_robot:
            return
        topic = self._topic_from_dropdown_value(value)
        if topic == self._selected_robot_topic:
            return
        self._request_robot_switch(topic)

    def _on_client_robot_dropdown_changed(self, view: ClientView, value: str) -> None:
        if view.switching or self._switching_robot:
            return
        topic = self._topic_from_dropdown_value(value)
        if topic == view.selected_robot_topic:
            return
        self._request_robot_switch(topic, client_view=view)

    def _request_robot_switch(
        self,
        robot_description_topic: Optional[str],
        *,
        client_view: Optional[ClientView] = None,
    ) -> None:
        """Switch robot on a background thread (connect may take several seconds)."""
        if self._switching_robot:
            return
        if client_view is not None and client_view.switching:
            return
        thread = threading.Thread(
            target=self._switch_robot,
            kwargs={
                "robot_description_topic": robot_description_topic,
                "client_view": client_view,
            },
            daemon=True,
            name="ros2-viser-robot-switch",
        )
        thread.start()

    def _unload_current_robot(self) -> None:
        """Deprecated path: deactivate UI only and keep sessions cached."""
        self._deactivate_current_session()

    def _wait_for_robot_description(
        self,
        timeout: Optional[float] = None,
        interface: Optional[ROS2RobotInterface] = None,
    ) -> Optional[str]:
        """Poll interface for a non-empty robot description string."""
        iface = interface if interface is not None else self.ros2_interface
        max_wait_time = self.config.robot_description_timeout if timeout is None else timeout
        wait_indefinitely = max_wait_time == 0.0
        start_time = time.time()
        last_status_time = start_time
        check_interval = 2.0

        while self._running:
            if not wait_indefinitely and (time.time() - start_time) >= max_wait_time:
                break
            if iface is not None and iface.is_connected:
                try:
                    description = iface.get_robot_description()
                    if description and description.strip():
                        return description
                except Exception as e:
                    logger.debug(f"Error getting robot description: {e}")
            elapsed = time.time() - start_time
            if time.time() - last_status_time >= check_interval:
                if wait_indefinitely:
                    logger.info(
                        f"Still waiting for robot description... ({elapsed:.1f}s, waiting indefinitely)"
                    )
                else:
                    logger.info(
                        f"Still waiting for robot description... ({elapsed:.1f}s / {max_wait_time:.1f}s)"
                    )
                last_status_time = time.time()
            time.sleep(0.1)
        return None

    def _load_urdf_model(self, urdf_string: str) -> tuple[
        yourdfpy.URDF,
        list[str],
        list[str],
        dict,
        dict[str, int],
    ]:
        """Parse a URDF string into a model and joint metadata (no scene side effects)."""
        urdf_io = StringIO(urdf_string)
        try:
            urdf = yourdfpy.URDF.load(
                urdf_io,
                build_scene_graph=True,
                load_collision_meshes=True,
                build_collision_scene_graph=True,
            )
        except (AttributeError, TypeError) as e:
            if "build_scene_graph" in str(e) or "unexpected keyword" in str(e) or "load_collision_meshes" in str(e):
                urdf_io.seek(0)
                try:
                    urdf = yourdfpy.URDF.load(urdf_io, build_scene_graph=True)
                except (AttributeError, TypeError):
                    urdf_io.seek(0)
                    urdf = yourdfpy.URDF.load(urdf_io)
            else:
                raise

        # Prefer yourdfpy's actuated order: ViserUrdf.update_cfg() expects either
        # that sequence or a name->value dict (NOT joint_map iteration order).
        urdf_actuated_joint_names = list(getattr(urdf, "actuated_joint_names", []) or [])
        if not urdf_actuated_joint_names:
            urdf_actuated_joint_names = [
                name
                for name, joint in urdf.joint_map.items()
                if joint.type in ("revolute", "prismatic", "continuous")
                and joint.mimic is None
            ]
        urdf_all_joint_names = [j.name for j in urdf.robot.joints]
        mimic_joint_info: dict = {}
        for joint_name, joint in urdf.joint_map.items():
            if joint.mimic is not None:
                mimicked_joint = joint.mimic.joint
                multiplier = joint.mimic.multiplier if joint.mimic.multiplier is not None else 1.0
                offset = joint.mimic.offset if joint.mimic.offset is not None else 0.0
                if mimicked_joint in urdf.joint_map:
                    mimic_joint_info[joint_name] = (mimicked_joint, multiplier, offset)
        # Index into actuated configuration vector (what update_cfg consumes).
        joint_name_to_urdf_index = {
            name: i for i, name in enumerate(urdf_actuated_joint_names)
        }
        return (
            urdf,
            urdf_all_joint_names,
            urdf_actuated_joint_names,
            mimic_joint_info,
            joint_name_to_urdf_index,
        )

    def _create_urdf_scene(
        self,
        urdf: yourdfpy.URDF,
        root_node_name: str,
    ) -> tuple[viser.FrameHandle, ViserUrdf, Optional[str]]:
        """Create parent frame + ViserUrdf under ``root_node_name``."""
        self._ensure_viser_server()
        assert self.server is not None

        has_collision_scene = (
            hasattr(urdf, "collision_scene") and urdf.collision_scene is not None
        )
        # Resolve root link from a temporary bind of urdf for helper reuse.
        previous_urdf = self.urdf
        self.urdf = urdf
        urdf_root_link = self._resolve_urdf_root_link()
        self.urdf = previous_urdf

        # Only load collision meshes when the user asked to show them — loading
        # both visual+collision GLBs on every namespace switch floods the client
        # and can drop default lights.
        load_collision = bool(has_collision_scene and self._show_collision)

        with self._scene_lock:
            robot_base_frame = self.server.scene.add_frame(root_node_name, show_axes=False)
            robot_base_frame.position = (0.0, 0.0, 0.0)
            robot_base_frame.wxyz = (1.0, 0.0, 0.0, 0.0)

            urdf_vis = ViserUrdf(
                self.server,
                urdf,
                root_node_name=root_node_name,
                load_meshes=True,
                load_collision_meshes=load_collision,
            )
            # Joint frames start at identity until the first update_cfg(); push a
            # zero configuration so the skeleton is assembled before joint_states.
            try:
                n_actuated = len(getattr(urdf, "actuated_joint_names", []) or [])
                if n_actuated:
                    urdf_vis.update_cfg(np.zeros(n_actuated, dtype=np.float64))
            except Exception as e:
                logger.warning(f"Initial URDF configuration failed for {root_node_name}: {e}")

            if load_collision:
                with warnings.catch_warnings():
                    warnings.filterwarnings("ignore", category=UserWarning)
                    try:
                        urdf_vis.show_collision = True
                    except Exception:
                        pass
            try:
                urdf_vis.show_visual = self._show_visual
            except Exception:
                pass

            self._ensure_scene_environment()

        if self._show_collision_checkbox is not None:
            self._show_collision_checkbox.visible = has_collision_scene
        return robot_base_frame, urdf_vis, urdf_root_link

    def _activate_session(
        self,
        robot_description_topic: str,
        *,
        client_view: Optional[ClientView] = None,
    ) -> None:
        """Ensure session is visible; bind panels only for the requesting browser."""
        session = self._robot_sessions.get(robot_description_topic)
        if session is None:
            raise KeyError(f"No cached session for {robot_description_topic}")

        self._set_session_scene_visible(session, True)
        self._bind_session(session)
        self._selected_robot_topic = robot_description_topic
        self._update_urdf_display()
        self._update_robot_base_from_tf()
        self._refresh_viz_frame_dropdown(keep_selection=True)

        if client_view is not None:
            client_view.selected_robot_topic = robot_description_topic
            self._reinitialize_client_panels(client_view)
            client_view.controls_bound = True
        # No shared server.gui panels in multi-client mode (option B).
        logger.info("Activated robot session: %s", robot_description_topic)

    def _create_session(self, robot_description_topic: str) -> RobotSession:
        """Connect, parse URDF, build scene, and cache a new robot session."""
        topic_prefix = topic_prefix_from_robot_description(robot_description_topic)
        root_node_name = self._session_root_node_name(robot_description_topic)
        # joint_states resolved in connect(): prefer /{prefix}/joint_states;
        # with node namespace set, no global /joint_states fallback.
        interface = self._create_ros2_interface(
            topic_prefix=topic_prefix,
            robot_description_topic=robot_description_topic,
            joint_states_topic=None,
        )
        robot_base_frame = None
        urdf_vis = None
        try:
            description = self._wait_for_robot_description(interface=interface)
            if not description:
                raise TimeoutError(
                    f"Failed to receive robot description from {robot_description_topic}"
                )

            urdf_hash = hashlib.md5(description.encode()).hexdigest()
            logger.info(
                f"✅ Received robot description (length: {len(description)} characters)"
            )
            (
                urdf,
                urdf_all_joint_names,
                urdf_actuated_joint_names,
                mimic_joint_info,
                joint_name_to_urdf_index,
            ) = self._load_urdf_model(description)
            logger.info("✅ URDF parsed successfully")

            robot_base_frame, urdf_vis, urdf_root_link = self._create_urdf_scene(
                urdf, root_node_name
            )
            session = RobotSession(
                robot_description_topic=robot_description_topic,
                topic_prefix=topic_prefix,
                root_node_name=root_node_name,
                interface=interface,
                urdf=urdf,
                urdf_vis=urdf_vis,
                robot_base_frame=robot_base_frame,
                urdf_root_link=urdf_root_link,
                urdf_hash=urdf_hash,
                urdf_all_joint_names=urdf_all_joint_names,
                urdf_actuated_joint_names=urdf_actuated_joint_names,
                mimic_joint_info=mimic_joint_info,
                joint_name_to_urdf_index=joint_name_to_urdf_index,
                visible=True,
            )
            self._robot_sessions[robot_description_topic] = session
            time.sleep(0.2)
            return session
        except Exception:
            if urdf_vis is not None:
                try:
                    urdf_vis.remove()
                except Exception:
                    pass
            if robot_base_frame is not None:
                try:
                    robot_base_frame.remove()
                except Exception:
                    pass
            try:
                if interface.is_connected:
                    interface.disconnect()
            except Exception:
                pass
            raise

    def _cleanup_client_control_ui(self, view: ClientView) -> None:
        """Tear down one browser's panels/markers."""
        if view.marker_manager is not None:
            try:
                view.marker_manager.cleanup()
            except Exception as e:
                logger.warning(f"Error cleaning up client marker manager: {e}")
            view.marker_manager = None
        for attr in (
            "fsm_panel",
            "gripper_panel",
            "joint_panel",
            "hardware_panel",
            "controller_panel",
        ):
            panel = getattr(view, attr)
            if panel is not None:
                try:
                    panel.cleanup()
                except Exception as e:
                    logger.warning(f"Error cleaning up client {attr}: {e}")
                setattr(view, attr, None)
        view.controls_bound = False

    def _reinitialize_client_panels(self, view: ClientView) -> None:
        """Bind control panels on this client to its selected robot session."""
        self._cleanup_client_control_ui(view)
        topic = view.selected_robot_topic
        if not topic or topic not in self._robot_sessions:
            return
        session = self._robot_sessions[topic]
        client = view.client
        # Temporarily expose session via convenience attrs for helpers that read self.*
        self._bind_session(session)

        if self.config.enable_fsm_panel:
            view.fsm_panel = FSMPanel(
                client,
                session.interface,
                self.config.fsm_command_topic,
                self.config,
            )
            view.fsm_panel.initialize()

        if self.config.enable_gripper_panel:
            view.gripper_panel = GripperPanel(client, session.interface)
            view.gripper_panel.initialize()

        if self.config.enable_joint_panel:
            view.joint_panel = JointPanel(client, session.interface, urdf=session.urdf)
            view.joint_panel.initialize()

        if self.config.enable_end_effector_marker:
            view.marker_manager = EndEffectorMarkerManager(
                client,
                session.interface,
                self.config,
                self.translator,
                session.urdf,
                session.root_node_name,
                attach_to_urdf_frames=False,
                marker_path_prefix=f"/client_{view.client_id}/ee",
            )
            view.marker_manager.set_scene_context(
                visualization_frame=(
                    ""
                    if self._selected_viz_frame == _VIZ_FRAME_FIXED
                    else self._selected_viz_frame
                ),
                urdf_root_link=session.urdf_root_link,
            )
            view.marker_manager.set_show_current_target_axes(self._show_current_target)
            if view.fsm_panel is not None:
                marker_dropdown, marker_button = view.fsm_panel.get_marker_controls()
                if marker_dropdown is not None and marker_button is not None:
                    view.marker_manager.set_gui_controls(marker_dropdown, marker_button)
                    view.fsm_panel.set_marker_callbacks(
                        lambda _: view.marker_manager.on_publish_mode_changed(
                            marker_dropdown.value
                        ),
                        lambda _: view.marker_manager.on_send_button_clicked(),
                    )
            view.marker_manager.initialize()

        enable_controller = (
            self.config.enable_controller_panel
            if hasattr(self.config, "enable_controller_panel")
            else True
        )
        if enable_controller:
            view.controller_panel = ControllerPanel(client, session.interface)
            view.controller_panel.initialize()

        # Hardware panel still uses shared node discovery via the session interface.
        try:
            nodes = session.interface.list_nodes()
            has_system = any("system" in n["name"].lower() for n in nodes)
            has_m6 = any("m6_ccs_system" in n["name"].lower() for n in nodes)
            if has_system:
                view.hardware_panel = HardwarePanel(
                    client, session.interface, has_m6_ccs_system=has_m6
                )
                view.hardware_panel.initialize()
        except Exception as e:
            logger.debug(f"Client hardware panel skipped: {e}")

    def _set_client_picker_enabled(self, view: ClientView, enabled: bool) -> None:
        for handle in (view.robot_dropdown, view.refresh_robots_button):
            if handle is None:
                continue
            try:
                handle.disabled = not enabled
            except Exception:
                pass

    def _refresh_client_robot_dropdown(self, view: ClientView) -> None:
        if view.robot_dropdown is None:
            return
        options = self._robot_dropdown_options()
        initial = self._dropdown_value_for_topic(view.selected_robot_topic)
        if initial not in options:
            initial = self._empty_workspace_label()
            view.selected_robot_topic = None
        try:
            view.robot_dropdown.options = options
            view.robot_dropdown.value = initial
        except Exception as e:
            logger.warning(f"Failed to refresh client robot dropdown: {e}")

    def _switch_robot(
        self,
        robot_description_topic: Optional[str],
        *,
        client_view: Optional[ClientView] = None,
    ) -> None:
        """Load/activate a robot session; bind panels for one browser when provided."""
        with self._robot_switch_lock:
            if self._switching_robot:
                return
            self._switching_robot = True
        if client_view is not None:
            client_view.switching = True
            self._set_client_picker_enabled(client_view, False)
        else:
            self._set_robot_picker_enabled(False)
        try:
            logger.info(
                "Switching robot to %s (client=%s)",
                robot_description_topic or "empty workspace",
                None if client_view is None else client_view.client_id,
            )

            if robot_description_topic is None:
                if client_view is not None:
                    self._cleanup_client_control_ui(client_view)
                    client_view.selected_robot_topic = None
                else:
                    self._deactivate_current_session()
                    self._selected_robot_topic = None
                logger.info("Empty workspace selected (sessions kept visible/cached)")
                return

            if (
                client_view is not None
                and robot_description_topic == client_view.selected_robot_topic
                and robot_description_topic in self._robot_sessions
                and client_view.controls_bound
            ):
                logger.info("Client already controlling %s", robot_description_topic)
                return

            if robot_description_topic not in self._robot_sessions:
                self._create_session(robot_description_topic)
                # New robot appears for every browser; refresh their pickers.
                for other in self._client_views.values():
                    if other is not client_view:
                        self._refresh_client_robot_dropdown(other)

            self._activate_session(robot_description_topic, client_view=client_view)
            # Namespace switch rebuilds meshes off the GUI thread; re-assert lights.
            self._ensure_scene_environment()
        except Exception as e:
            logger.error(f"Failed to switch robot: {e}", exc_info=True)
            self._ensure_scene_environment()
            try:
                if (
                    robot_description_topic
                    and robot_description_topic in self._robot_sessions
                    and not any(
                        v.selected_robot_topic == robot_description_topic
                        for v in self._client_views.values()
                        if v is not client_view
                    )
                ):
                    # Only destroy if no other client is using it and create failed mid-way.
                    pass
            except Exception:
                pass
            if client_view is not None:
                self._cleanup_client_control_ui(client_view)
                client_view.selected_robot_topic = None
                if client_view.robot_dropdown is not None:
                    try:
                        client_view.robot_dropdown.value = self._empty_workspace_label()
                    except Exception:
                        pass
            else:
                self._selected_robot_topic = None
                if self._robot_dropdown is not None:
                    try:
                        self._robot_dropdown.value = self._empty_workspace_label()
                    except Exception:
                        pass
        finally:
            self._skip_reconnect_detection = True
            self._switching_robot = False
            if client_view is not None:
                client_view.switching = False
                self._set_client_picker_enabled(client_view, True)
            else:
                self._set_robot_picker_enabled(True)

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
                    logger.debug(f"Hardware system node detected: {node['full_name']}")
                
                # Specifically check for m6_ccs_system
                if 'm6_ccs_system' in node_name_lower:
                    has_m6_ccs_system = True
                    logger.info(f"M6 CCS System node detected: {node['full_name']}")
            
            # Initialize hardware panel if any system node is detected
            if has_system_node:
                logger.debug(f"Found {len(detected_system_nodes)} system node(s): {', '.join(detected_system_nodes)}")
                
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
                logger.debug("✅ Hardware panel initialized")
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
            
            if self._controller_panel is not None:
                try:
                    self._controller_panel.cleanup()
                except Exception as e:
                    logger.warning(f"Error cleaning up Controller panel during reinit: {e}")
                self._controller_panel = None
            
            # Reinitialize panels if enabled
            if self.config.enable_fsm_panel:
                self._fsm_panel = FSMPanel(
                    self.server,
                    self.ros2_interface,
                    self.config.fsm_command_topic,
                    self.config
                )
                self._fsm_panel.initialize()
                # Connect marker controls to marker manager if available
                if self._marker_manager is not None and self.config.enable_end_effector_marker:
                    marker_dropdown, marker_button = self._fsm_panel.get_marker_controls()
                    if marker_dropdown is not None and marker_button is not None:
                        self._marker_manager.set_gui_controls(marker_dropdown, marker_button)
                        # Set up callbacks
                        marker_dropdown.on_update(
                            lambda _: self._on_marker_publish_mode_changed(marker_dropdown.value)
                        )
                        marker_button.on_click(
                            lambda _: self._on_send_marker_pose_clicked()
                        )
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
                        self._active_root_node_name,
                    )
                    self._marker_manager.set_show_current_target_axes(self._show_current_target)
                    # Connect GUI controls from FSM panel if available
                    if self._fsm_panel is not None and self.config.enable_end_effector_marker:
                        marker_dropdown, marker_button = self._fsm_panel.get_marker_controls()
                        if marker_dropdown is not None and marker_button is not None:
                            self._marker_manager.set_gui_controls(marker_dropdown, marker_button)
                            # Set up callbacks via FSM panel
                            self._fsm_panel.set_marker_callbacks(
                                lambda _: self._on_marker_publish_mode_changed(marker_dropdown.value),
                                lambda _: self._on_send_marker_pose_clicked()
                            )
                    # Initialize markers (will be done when interface is ready)
                    self._marker_manager.initialize()
                    logger.info("✅ Marker manager reinitialized")
            
            # Recheck hardware nodes and reinitialize hardware panel if needed
            if self.ros2_interface is not None and self.ros2_interface.is_connected:
                self._check_hardware_nodes()
            
            # Initialize controller panel if enabled
            if self.config.enable_controller_panel if hasattr(self.config, 'enable_controller_panel') else True:
                self._controller_panel = ControllerPanel(
                    self.server,
                    self.ros2_interface
                )
                self._controller_panel.initialize()
                logger.info("✅ Controller panel reinitialized")
                
        except Exception as e:
            logger.error(f"Failed to reinitialize panels: {e}", exc_info=True)
    
    def _reconnect_marker_gui_controls(self):
        """Reconnect marker manager to FSM panel GUI after controls were recreated."""
        if (
            not self.config.enable_end_effector_marker
            or self._marker_manager is None
            or self._fsm_panel is None
        ):
            return

        marker_dropdown, marker_button = self._fsm_panel.get_marker_controls()
        if marker_dropdown is None or marker_button is None:
            return

        self._marker_manager.set_gui_controls(marker_dropdown, marker_button)
        self._fsm_panel.set_marker_callbacks(
            lambda _: self._on_marker_publish_mode_changed(marker_dropdown.value),
            lambda _: self._on_send_marker_pose_clicked(),
        )

    def _reinitialize_panels_with_language(self):
        """Refresh per-client panel labels after a language change."""
        try:
            for view in list(self._client_views.values()):
                for panel in (
                    view.fsm_panel,
                    view.gripper_panel,
                    view.joint_panel,
                    view.hardware_panel,
                    view.controller_panel,
                ):
                    if panel is None:
                        continue
                    try:
                        panel.update_gui_labels()
                    except Exception as e:
                        logger.debug(f"Client panel label update failed: {e}")
            # Shared server.gui path (legacy); keep if still present.
            if self._fsm_panel is not None:
                self._fsm_panel.update_gui_labels()
                self._reconnect_marker_gui_controls()
            if self._gripper_panel is not None:
                self._gripper_panel.update_gui_labels()
            if self._joint_panel is not None:
                self._joint_panel.update_gui_labels()
            if self._hardware_panel is not None:
                self._hardware_panel.update_gui_labels()
            if self._controller_panel is not None:
                self._controller_panel.update_gui_labels()
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
                options = self._robot_dropdown_options()
                initial = self._dropdown_value_for_topic(self._selected_robot_topic)
                if initial not in options:
                    initial = self._empty_workspace_label()
                self._robot_dropdown = self.server.gui.add_dropdown(
                    self.translator("select_robot"),
                    options=options,
                    initial_value=initial,
                )
                self._robot_dropdown.on_update(
                    lambda _: self._on_robot_dropdown_changed(self._robot_dropdown.value)
                )
                self._refresh_robots_button = self.server.gui.add_button(
                    self.translator("refresh_robots")
                )
                self._refresh_robots_button.on_click(
                    lambda _: self._on_refresh_robots_clicked()
                )

                viz_options = self._viz_frame_dropdown_options()
                viz_initial = self._dropdown_value_for_viz_frame(self._selected_viz_frame)
                if viz_initial not in viz_options:
                    viz_options = viz_options + [viz_initial]
                self._viz_frame_dropdown = self.server.gui.add_dropdown(
                    self.translator("select_viz_frame"),
                    options=viz_options,
                    initial_value=viz_initial,
                )
                self._viz_frame_dropdown.on_update(
                    lambda _: self._on_viz_frame_dropdown_changed(
                        self._viz_frame_dropdown.value
                    )
                )
                self._refresh_viz_frames_button = self.server.gui.add_button(
                    self.translator("refresh_viz_frames")
                )
                self._refresh_viz_frames_button.on_click(
                    lambda _: self._on_refresh_viz_frames_clicked()
                )

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

                self._show_current_target_checkbox = self.server.gui.add_checkbox(
                    self.translator("show_current_target"),
                    initial_value=self._show_current_target,
                )
                self._show_current_target_checkbox.on_update(
                    lambda _: self._on_show_current_target_changed(
                        self._show_current_target_checkbox.value
                    )
                )
            
            logger.debug("Display control panel initialized")
        except Exception as e:
            logger.error(f"Failed to initialize display control panel: {e}", exc_info=True)
    
    def _on_show_visual_changed(self, value: bool):
        """Callback when show visual checkbox is toggled."""
        self._show_visual = value
        self._update_urdf_display()
        logger.debug(f"Show visual changed to: {value}")
    
    def _reload_session_scene(self, session: RobotSession) -> None:
        """Rebuild one session's ViserUrdf (e.g. to load collision meshes on demand)."""
        with self._scene_lock:
            try:
                session.urdf_vis.remove()
            except Exception as e:
                logger.debug(f"Error removing URDF before reload: {e}")
            try:
                session.robot_base_frame.remove()
            except Exception as e:
                logger.debug(f"Error removing base frame before reload: {e}")
        robot_base_frame, urdf_vis, urdf_root_link = self._create_urdf_scene(
            session.urdf, session.root_node_name
        )
        session.robot_base_frame = robot_base_frame
        session.urdf_vis = urdf_vis
        session.urdf_root_link = urdf_root_link
        if self._active_session_key == session.robot_description_topic:
            self._bind_session(session)

    def _on_show_current_target_changed(self, value: bool):
        """Toggle RViz-style dual-arm current-target axes (``*_current_target``)."""
        self._show_current_target = bool(value)
        self.config.show_current_target_axes = self._show_current_target
        if self._marker_manager is not None:
            self._marker_manager.set_show_current_target_axes(self._show_current_target)
        for view in self._client_views.values():
            if view.marker_manager is not None:
                view.marker_manager.set_show_current_target_axes(self._show_current_target)
            if view.show_current_target_checkbox is not None:
                try:
                    if view.show_current_target_checkbox.value != self._show_current_target:
                        view.show_current_target_checkbox.value = self._show_current_target
                except Exception:
                    pass
        if self._show_current_target_checkbox is not None:
            try:
                if self._show_current_target_checkbox.value != self._show_current_target:
                    self._show_current_target_checkbox.value = self._show_current_target
            except Exception:
                pass
        logger.debug(f"Show current target axes changed to: {value}")

    def _on_show_collision_changed(self, value: bool):
        """Callback when show collision checkbox is toggled."""
        self._show_collision = value
        if value:
            for session in list(self._robot_sessions.values()):
                has_collision = (
                    hasattr(session.urdf, "collision_scene")
                    and session.urdf.collision_scene is not None
                )
                loaded = getattr(session.urdf_vis, "_collision_root_frame", None) is not None
                if has_collision and not loaded:
                    logger.info(
                        "Loading collision meshes on demand for %s",
                        session.robot_description_topic,
                    )
                    try:
                        self._reload_session_scene(session)
                    except Exception as e:
                        logger.warning(f"Failed to load collision meshes: {e}")
        self._update_urdf_display()
        self._ensure_scene_environment()
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
        # Display panel recreate keeps _selected_robot_topic and translates empty label.
        self._reinitialize_panels_with_language()
    
    def _update_urdf_display(self):
        """Update URDF visualization based on display settings for all sessions."""
        if self.server is None:
            return
        for session in self._robot_sessions.values():
            urdf_vis = session.urdf_vis
            try:
                if hasattr(urdf_vis, "show_visual"):
                    try:
                        urdf_vis.show_visual = self._show_visual
                    except Exception:
                        pass
                if hasattr(urdf_vis, "show_collision"):
                    with warnings.catch_warnings():
                        warnings.filterwarnings("ignore", category=UserWarning)
                        try:
                            urdf_vis.show_collision = self._show_collision
                        except Exception:
                            pass
                self._set_session_scene_visible(session, True)
            except Exception as e:
                logger.debug(f"Failed to update URDF display for {session.robot_description_topic}: {e}")

    def _setup_client_view(self, client) -> None:
        """Create per-browser display + robot picker GUI on ``client.gui``."""
        client_id = int(getattr(client, "client_id", id(client)))
        if client_id in self._client_views:
            return

        view = ClientView(client_id=client_id, client=client)
        preload_topic: Optional[str] = None
        try:
            view.display_folder = client.gui.add_folder(self.translator("display_control"))
            with view.display_folder:
                options = self._robot_dropdown_options()
                initial = self._empty_workspace_label()
                # Prefer already-loaded single robot for new browsers.
                # Dropdown options are namespace labels ("默认工作空间" or "/g1d"), not topic paths.
                if len(self._robot_sessions) == 1:
                    preload_topic = next(iter(self._robot_sessions.keys()))
                    initial = self._dropdown_value_for_topic(preload_topic)
                view.robot_dropdown = client.gui.add_dropdown(
                    self.translator("select_robot"),
                    options=options,
                    initial_value=initial if initial in options else self._empty_workspace_label(),
                )
                view.robot_dropdown.on_update(
                    lambda _: self._on_client_robot_dropdown_changed(
                        view, view.robot_dropdown.value
                    )
                )
                view.refresh_robots_button = client.gui.add_button(
                    self.translator("refresh_robots")
                )
                view.refresh_robots_button.on_click(
                    lambda _: self._on_client_refresh_robots(view)
                )

                viz_options = self._viz_frame_dropdown_options()
                viz_initial = self._dropdown_value_for_viz_frame(self._selected_viz_frame)
                if viz_initial not in viz_options:
                    viz_options = viz_options + [viz_initial]
                view.viz_frame_dropdown = client.gui.add_dropdown(
                    self.translator("select_viz_frame"),
                    options=viz_options,
                    initial_value=viz_initial,
                )
                view.viz_frame_dropdown.on_update(
                    lambda _: self._on_viz_frame_dropdown_changed(
                        view.viz_frame_dropdown.value
                    )
                )
                view.refresh_viz_frames_button = client.gui.add_button(
                    self.translator("refresh_viz_frames")
                )
                view.refresh_viz_frames_button.on_click(
                    lambda _: self._on_refresh_viz_frames_clicked()
                )

                view.language_dropdown = client.gui.add_dropdown(
                    self.translator("select_language"),
                    options=["中文", "English"],
                    initial_value="中文" if self.translator.language == "zh" else "English",
                )
                view.language_dropdown.on_update(
                    lambda _: self._on_language_changed(view.language_dropdown.value)
                )
                view.show_visual_checkbox = client.gui.add_checkbox(
                    self.translator("show_visual"),
                    initial_value=self._show_visual,
                )
                view.show_visual_checkbox.on_update(
                    lambda _: self._on_show_visual_changed(view.show_visual_checkbox.value)
                )
                view.show_collision_checkbox = client.gui.add_checkbox(
                    self.translator("show_collision"),
                    initial_value=self._show_collision,
                )
                view.show_collision_checkbox.on_update(
                    lambda _: self._on_show_collision_changed(
                        view.show_collision_checkbox.value
                    )
                )
                view.show_current_target_checkbox = client.gui.add_checkbox(
                    self.translator("show_current_target"),
                    initial_value=self._show_current_target,
                )
                view.show_current_target_checkbox.on_update(
                    lambda _: self._on_show_current_target_changed(
                        view.show_current_target_checkbox.value
                    )
                )
        except Exception as e:
            logger.error(f"Failed to create client GUI: {e}", exc_info=True)
            return

        self._client_views[client_id] = view
        logger.info("Client %s connected (per-client robot picker ready)", client_id)
        if preload_topic:
            # Bind panels now. selected_robot_topic stays unset until activation,
            # so this is not treated as "already controlling".
            self._request_robot_switch(preload_topic, client_view=view)

    def _on_client_refresh_robots(self, view: ClientView) -> None:
        self._discovered_robot_topics = self._scan_robot_description_topics()
        self._refresh_client_robot_dropdown(view)
        for other in self._client_views.values():
            if other is not view:
                self._refresh_client_robot_dropdown(other)

    def _teardown_client_view(self, client) -> None:
        client_id = int(getattr(client, "client_id", id(client)))
        view = self._client_views.pop(client_id, None)
        if view is None:
            return
        self._cleanup_client_control_ui(view)
        logger.info("Client %s disconnected", client_id)

    def _ensure_viser_server(self):
        """Create Viser server, ground grid, and per-client GUI hooks."""
        if self.server is not None:
            return

        try:
            self.server = viser.ViserServer()
        except Exception as e:
            logger.error(f"Failed to create Viser server: {e}", exc_info=True)
            raise

        try:
            server_port = get_viser_server_port(self.server)
            local_ip, interface_name, is_wifi = get_local_ip()
            server_url = f"http://{local_ip}:{server_port}"
            display_server_info(server_url, local_ip, interface_name, is_wifi, server_port)
        except Exception as e:
            logger.warning(f"⚠️ 无法显示服务器信息: {e}")

        logger.debug("Adding ground grid and default lighting...")
        self._ensure_scene_environment()

        @self.server.on_client_connect
        def _on_connect(client) -> None:
            # New browsers sometimes miss the initial light/env messages.
            self._ensure_scene_environment()
            self._setup_client_view(client)

        @self.server.on_client_disconnect
        def _on_disconnect(client) -> None:
            self._teardown_client_view(client)

        if self._update_thread is None or not self._update_thread.is_alive():
            self._update_thread = threading.Thread(
                target=self._update_loop,
                daemon=True,
            )
            self._update_thread.start()
            logger.debug("Update loop started")

    def _init_control_panels(self):
        """Create FSM / gripper / joint / hardware / controller panels after ROS connect."""
        if self.server is None or self.ros2_interface is None:
            return

        if self.config.enable_fsm_panel and self._fsm_panel is None:
            self._fsm_panel = FSMPanel(
                self.server,
                self.ros2_interface,
                self.config.fsm_command_topic,
                self.config,
            )
            self._fsm_panel.initialize()

        if self.config.enable_gripper_panel and self._gripper_panel is None:
            self._gripper_panel = GripperPanel(
                self.server,
                self.ros2_interface,
            )
            self._gripper_panel.initialize()

        if self.config.enable_joint_panel and self._joint_panel is None:
            self._joint_panel = JointPanel(
                self.server,
                self.ros2_interface,
                urdf=self.urdf,
            )
            self._joint_panel.initialize()

        if self.ros2_interface.is_connected:
            self._check_hardware_nodes()
            enable_controller = (
                self.config.enable_controller_panel
                if hasattr(self.config, "enable_controller_panel")
                else True
            )
            if enable_controller and self._controller_panel is None:
                self._controller_panel = ControllerPanel(
                    self.server,
                    self.ros2_interface,
                )
                self._controller_panel.initialize()
                logger.debug("✅ Controller panel initialized")

    def _init_viser(self):
        """Initialize Viser server and URDF visualization."""
        self._ensure_viser_server()

        if self.urdf is None:
            logger.debug("URDF not available yet; empty Viser scene is ready")
            return
        
        try:
            # Control panels need a connected interface + URDF
            if self.ros2_interface is not None:
                self._init_control_panels()
            
            # Create URDF visualization
            logger.debug(f"Creating ViserUrdf with {len(self.urdf_all_joint_names)} joints...")
            try:
                # Check if collision scene is available in URDF
                has_collision_scene = (
                    hasattr(self.urdf, 'collision_scene') and 
                    self.urdf.collision_scene is not None
                )

                self._urdf_root_link = self._resolve_urdf_root_link()
                # Parent frame owns the whole robot pose in the selected viz TF frame.
                if self._robot_base_frame is not None:
                    try:
                        self._robot_base_frame.remove()
                    except Exception:
                        pass
                    self._robot_base_frame = None
                self._robot_base_frame = self.server.scene.add_frame(
                    self._active_root_node_name,
                    show_axes=False,
                )
                self._reset_robot_base_pose()
                
                # Create ViserUrdf with collision meshes enabled
                self.urdf_vis = ViserUrdf(
                    self.server,
                    self.urdf,
                    root_node_name=self._active_root_node_name,
                    load_meshes=True,
                    load_collision_meshes=has_collision_scene, 
                )
                logger.debug(f"✅ URDF visualization initialized at {self._active_root_node_name}")
                if has_collision_scene:
                    logger.debug(f"✅ {self.translator('collision_meshes_loaded')}")
                else:
                    logger.info(f"ℹ️  {self.translator('no_collision_meshes')}")
                
                # Update checkbox visibility based on what's actually loaded
                if self._show_collision_checkbox is not None:
                    self._show_collision_checkbox.visible = has_collision_scene
                
                # Apply initial display settings
                self._update_urdf_display()
                self._refresh_viz_frame_dropdown(keep_selection=True)
                self._update_robot_base_from_tf()
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
                        self._active_root_node_name,
                    )
                    self._marker_manager.set_show_current_target_axes(self._show_current_target)
                    # Connect GUI controls from FSM panel if available
                    if self._fsm_panel is not None:
                        marker_dropdown, marker_button = self._fsm_panel.get_marker_controls()
                        if marker_dropdown is not None and marker_button is not None:
                            self._marker_manager.set_gui_controls(marker_dropdown, marker_button)
                            # Set up callbacks via FSM panel
                            self._fsm_panel.set_marker_callbacks(
                                lambda _: self._on_marker_publish_mode_changed(marker_dropdown.value),
                                lambda _: self._on_send_marker_pose_clicked()
                            )
                    # Initialize markers (will be done when interface is ready)
                    self._marker_manager.initialize()
                
        except Exception as e:
            logger.error(f"Failed to initialize Viser: {e}", exc_info=True)
            raise
    
    def _map_joint_states_for_session(
        self,
        session: RobotSession,
        joint_names: list[str],
        positions: list[float],
    ) -> dict[str, float]:
        """Build name→position map for yourdfpy ``update_cfg`` (order-safe)."""
        actuated = set(session.urdf_actuated_joint_names)
        cfg: dict[str, float] = {}
        for joint_name, position in zip(joint_names, positions):
            if joint_name in actuated:
                cfg[joint_name] = float(position)
        if not cfg and joint_names:
            now = time.time()
            last = getattr(session, "_last_joint_map_warn_time", 0.0)
            if now - last > 5.0:
                sample = list(joint_names)[:5]
                logger.warning(
                    "No joint_states names matched URDF actuated joints for %s "
                    "(got %d names, e.g. %s; urdf has %d actuated)",
                    session.robot_description_topic,
                    len(joint_names),
                    sample,
                    len(session.urdf_actuated_joint_names),
                )
                session._last_joint_map_warn_time = now  # type: ignore[attr-defined]
        return cfg

    def _update_loop(self):
        """Update all shared robot scenes and each browser's control panels."""
        update_period = 1.0 / self.config.update_rate
        last_joint_warn: dict[str, float] = {}

        while self._running:
            # Keep driving shared robot meshes even while a client is switching
            # panels; only panel rebind needs to wait on _switching_robot.
            for session in list(self._robot_sessions.values()):
                try:
                    # Still apply TF even if joint_states briefly time out.
                    if session.interface.robot_node is not None:
                        self._apply_session_base_from_tf(session)
                    joint_state = session.interface.get_joint_state()
                    if joint_state is None:
                        key = session.robot_description_topic
                        now = time.time()
                        if now - last_joint_warn.get(key, 0.0) > 5.0:
                            topic = getattr(
                                session.interface.config, "joint_states_topic", "?"
                            )
                            ns = (
                                getattr(session.interface.config, "namespace", "") or ""
                            ).strip("/")
                            effective = (
                                f"/{ns}/{str(topic).lstrip('/')}"
                                if ns and isinstance(topic, str) and not topic.startswith("/")
                                else topic
                            )
                            logger.warning(
                                "No joint_states yet for %s (subscribed to %s → %s)",
                                key,
                                topic,
                                effective,
                            )
                            last_joint_warn[key] = now
                        continue
                    joint_cfg = self._map_joint_states_for_session(
                        session,
                        joint_state["names"],
                        joint_state["positions"],
                    )
                    if joint_cfg:
                        session.urdf_vis.update_cfg(joint_cfg)
                except Exception as e:
                    key = session.robot_description_topic
                    now = time.time()
                    if now - last_joint_warn.get(key + ":err", 0.0) > 5.0:
                        logger.warning(
                            "Session update failed for %s: %s",
                            key,
                            e,
                        )
                        last_joint_warn[key + ":err"] = now

            if self._switching_robot:
                time.sleep(update_period)
                continue

            # Per-client panels / markers (each browser controls its selected robot).
            for view in list(self._client_views.values()):
                if view.switching:
                    continue
                try:
                    if view.fsm_panel is not None:
                        view.fsm_panel.update()
                    if view.gripper_panel is not None:
                        view.gripper_panel.update()
                    if view.joint_panel is not None:
                        view.joint_panel.update()
                    if view.marker_manager is not None:
                        view.marker_manager.update()
                except Exception as e:
                    logger.debug(f"Client {view.client_id} panel update failed: {e}")

            time.sleep(update_period)
    
    def _map_joint_states(self, joint_names: list[str], positions: list[float]) -> dict[str, float]:
        """Map joint states from ROS2 topic to URDF actuated name→position dict."""
        if len(self.urdf_actuated_joint_names) == 0:
            raise ValueError("URDF actuated joint names are not initialized")

        actuated = set(self.urdf_actuated_joint_names)
        return {
            name: float(pos)
            for name, pos in zip(joint_names, positions)
            if name in actuated
        }
    
    def start(self):
        """Start the visualizer with an empty shared scene and optional auto robot load."""
        if self._running:
            logger.warning("Visualizer is already running")
            return
        
        self._running = True

        # Discover robots before browsers connect so pickers are populated.
        self._discovered_robot_topics = self._scan_robot_description_topics()

        # Non-default robot_description_topic from caller pre-selects that topic.
        configured = self.config.robot_description_topic
        if (
            configured
            and configured != "/robot_description"
            and configured in self._discovered_robot_topics
        ):
            self._selected_robot_topic = configured
        elif len(self._discovered_robot_topics) == 1:
            self._selected_robot_topic = self._discovered_robot_topics[0]
        else:
            self._selected_robot_topic = None

        # Shared scene only (grid + client connect hooks + update loop).
        # Per-browser GUI is created in on_client_connect.
        self._ensure_viser_server()

        if self._selected_robot_topic:
            logger.info(
                f"Auto-loading robot into shared scene: {self._selected_robot_topic}"
            )
            # Synchronous load so the model is in the scene before start() returns.
            # Panels bind later when each browser connects.
            self._switch_robot(self._selected_robot_topic)
        else:
            logger.info(
                "Empty workspace ready. Open the Viser page and select a robot "
                f"({len(self._discovered_robot_topics)} candidate(s) found)."
            )
        
        logger.info("✅ Visualizer started successfully")
    
    def stop(self):
        """Stop the visualizer and cleanup resources."""
        self._running = False
        
        # Tear down every browser's panels/markers first.
        for client_id in list(self._client_views.keys()):
            view = self._client_views.pop(client_id, None)
            if view is not None:
                self._cleanup_client_control_ui(view)
        self._cleanup_control_ui()
        
        # Wait for update thread first (it uses ros2_interface)
        if self._update_thread is not None:
            self._update_thread.join(timeout=2.0)
        
        # Wait a bit to let any in-flight ROS2 callbacks complete
        time.sleep(0.2)
        
        # Disconnect and remove every cached robot session
        self._destroy_all_sessions()
        self._selected_robot_topic = None
        
        if self._display_folder_handle is not None:
            try:
                self._display_folder_handle.remove()
            except Exception as e:
                logger.warning(f"Error removing display control panel: {e}")
            self._display_folder_handle = None
        
        logger.info("Visualizer stopped")
    
    def __enter__(self):
        """Context manager entry."""
        self.start()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit."""
        self.stop()
