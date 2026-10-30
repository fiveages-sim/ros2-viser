"""FSM Control Panel for ROS2 Viser."""

import logging
import time
from typing import Dict, List, Optional, Tuple

import viser

from arms_ros2_control_msgs.msg import WbcCurrentState

from ros2_robot_interface import ROS2RobotInterface

from ..i18n import get_translator
from ..config import ROS2ViserConfig

logger = logging.getLogger(__name__)

# Body mode display key → mode_command (aligned with RViz OCS2FSMPanel).
_BODY_MODE_OPTIONS: Tuple[Tuple[str, str, int], ...] = (
    ("wbc_body_free", "BODY_FREE", WbcCurrentState.BODY_FREE),
    ("wbc_body_vertical", "BODY_RELATIVE", WbcCurrentState.BODY_VERTICAL),
    ("wbc_body_tracking", "BODY_TRACKING", WbcCurrentState.BODY_TRACKING),
    ("wbc_body_locked", "BODY_LOCK", WbcCurrentState.BODY_LOCKED),
    ("wbc_body_custom", "BODY_CUSTOM_LOCK", WbcCurrentState.BODY_CUSTOM_LOCKED),
)

_HEAD_MODE_OPTIONS: Tuple[Tuple[str, str, int], ...] = (
    ("wbc_head_disabled", "HEAD_DISABLE", WbcCurrentState.HEAD_DISABLED),
    ("wbc_head_tracking", "HEAD_TRACKING", WbcCurrentState.HEAD_TRACKING),
    ("wbc_head_gaze", "HEAD_GAZE", WbcCurrentState.HEAD_GAZE),
    ("wbc_head_forward", "HEAD_FORWARD", WbcCurrentState.HEAD_FORWARD),
)


class FSMPanel:
    """FSM (Finite State Machine) control panel for robot state management.
    
    This panel provides a GUI interface for switching between robot operational
    states (HOME, HOLD, OCS2, MOVEJ) and performing special actions like pose switching.
    In OCS2 + WBC stacks it also exposes the same whole-body switches as RViz
    ``OCS2FSMPanel`` (base / arms / bimanual / home joint / body / head).
    """
    
    def __init__(
        self,
        server: viser.ViserServer,
        ros2_interface: ROS2RobotInterface,
        fsm_command_topic: str = "/fsm_command",
        config: Optional[ROS2ViserConfig] = None
    ):
        """Initialize FSM panel.
        
        Args:
            server: Viser server instance for GUI creation.
            ros2_interface: ROS2RobotInterface for sending commands and subscribing to topics.
            fsm_command_topic: ROS2 topic name for FSM commands (default: "/fsm_command").
            config: Optional ROS2ViserConfig for end-effector marker controls.
        """
        self.server = server
        self.ros2_interface = ros2_interface
        self.fsm_command_topic = fsm_command_topic
        self.config = config
        self.translator = get_translator()
        
        # FSM state tracking
        self._current_fsm_state: int = 0
        
        # GUI elements
        self._fsm_state_label: Optional[viser.GuiTextHandle] = None
        self._fsm_button_handles: dict[str, viser.GuiButtonHandle] = {}
        self._fsm_switch_pose_button: Optional[viser.GuiButtonHandle] = None
        self._home_confirm_modal: Optional[viser.GuiModalHandle] = None
        # End-effector marker controls (only visible in OCS2 mode)
        self._marker_publish_mode_dropdown: Optional[viser.GuiDropdownHandle] = None
        self._send_marker_pose_button: Optional[viser.GuiButtonHandle] = None
        self._marker_mode_callback = None
        self._marker_send_callback = None
        # Folder handle for cleanup
        self._folder_handle: Optional[viser.GuiFolderHandle] = None

        # WBC controls (RViz OCS2FSMPanel parity; OCS2 + is_wbc only)
        self._wbc_folder: Optional[viser.GuiFolderHandle] = None
        self._wbc_status_text: Optional[viser.GuiTextHandle] = None
        self._wbc_base_checkbox: Optional[viser.GuiCheckboxHandle] = None
        self._wbc_left_arm_checkbox: Optional[viser.GuiCheckboxHandle] = None
        self._wbc_right_arm_checkbox: Optional[viser.GuiCheckboxHandle] = None
        self._wbc_bimanual_checkbox: Optional[viser.GuiCheckboxHandle] = None
        self._wbc_home_pose_checkbox: Optional[viser.GuiCheckboxHandle] = None
        self._wbc_body_dropdown: Optional[viser.GuiDropdownHandle] = None
        self._wbc_head_dropdown: Optional[viser.GuiDropdownHandle] = None
        self._wbc_ui_syncing: bool = False
        self._wbc_body_label_to_cmd: Dict[str, str] = {}
        self._wbc_head_label_to_cmd: Dict[str, str] = {}
        self._wbc_last_visible: Optional[bool] = None
        
        # Initialization flag
        self._initialized = False
        
        # Cleanup flag to prevent callbacks from running after cleanup
        self._cleaned_up = False
    
    def initialize(self):
        """Initialize the FSM panel GUI and ROS2 subscriptions."""
        # If already initialized, cleanup first to avoid duplicates
        if self._initialized:
            logger.warning("FSM panel already initialized, cleaning up first...")
            self.cleanup()
        
        try:
            # Reset cleanup flag
            self._cleaned_up = False
            
            # Initialize GUI
            self._init_gui()
            self._current_fsm_state = 0
            self.update()
            
            self._initialized = True
            logger.debug("✅ FSM control panel initialized")
            
        except Exception as e:
            logger.error(f"Failed to initialize FSM panel: {e}", exc_info=True)
            raise
    
    def _init_gui(self):
        """Initialize FSM control panel GUI elements."""
        if self.server is None:
            return
        
        try:
            # Create folder and save reference for cleanup
            self._folder_handle = self.server.gui.add_folder(self.translator("fsm_control"))
            with self._folder_handle:
                # Current state display (read-only)
                self._fsm_state_label = self.server.gui.add_text(
                    self.translator("current_state"),
                    initial_value="INVALID",
                    disabled=True
                )
                
                # State transition buttons - show all possible target states as buttons
                # HOME button - Purple
                self._fsm_button_handles["to_home"] = self.server.gui.add_button("HOME", color=(128, 0, 128))
                self._fsm_button_handles["to_home"].on_click(
                    self._on_home_clicked
                )
                
                # HOLD button - Yellow
                self._fsm_button_handles["to_hold"] = self.server.gui.add_button("HOLD", color="yellow")
                self._fsm_button_handles["to_hold"].on_click(
                    lambda _: self._send_fsm_command(2, "HOLD")
                )
                
                # OCS2 button - Blue
                self._fsm_button_handles["to_ocs2"] = self.server.gui.add_button("OCS2", color="blue")
                self._fsm_button_handles["to_ocs2"].on_click(
                    lambda _: self._send_fsm_command(3, "OCS2")
                )
                
                # MOVEJ button - Cyan
                self._fsm_button_handles["to_movej"] = self.server.gui.add_button("MOVEJ", color="cyan")
                self._fsm_button_handles["to_movej"].on_click(
                    lambda _: self._send_fsm_command(4, "MOVEJ")
                )
                
                # Special action: Switch pose (only available in HOME state)
                # This sends command 100, then automatically sends 0 after 0.1s
                # Deep orange/red-orange for special action to distinguish from yellow
                switch_pose_text = self.translator("switch_pose")
                self._fsm_switch_pose_button = self.server.gui.add_button(
                    f"{switch_pose_text} (Home ↔ Rest)", 
                    color=(255, 100, 0)
                )
                self._fsm_switch_pose_button.on_click(self._on_switch_pose_clicked)
                self._create_marker_controls()
                self._create_wbc_controls()
                
        except Exception as e:
            logger.error(f"Failed to initialize FSM panel GUI: {e}", exc_info=True)
            raise
    
    def _on_switch_pose_clicked(self, _):
        """Handle switch pose button click.
        
        Sends command 100, then waits 0.1s and sends command 0.
        This is similar to the RViz panel implementation.
        """
        # Send command 100
        self._send_fsm_command(100, None)
        
        # Wait 0.1s then send command 0
        time.sleep(0.1)
        self._send_fsm_command(0, None)

    def _on_home_clicked(self, _):
        """Handle HOME button click with confirmation dialog."""
        if self.server is None or self._cleaned_up:
            return
        if self._home_confirm_modal is not None:
            return

        try:
            self._home_confirm_modal = self.server.gui.add_modal(
                self.translator("fsm_home_confirm_title")
            )
            with self._home_confirm_modal:
                confirm_label = self.translator("confirm")
                cancel_label = self.translator("cancel")
                action_buttons = self.server.gui.add_button_group(
                    "",
                    options=[confirm_label, cancel_label],
                )
                action_buttons.on_click(
                    lambda event: self._confirm_home_switch()
                    if event.target.value == confirm_label
                    else self._close_home_confirm_modal()
                )
        except Exception as e:
            logger.error(f"Failed to open HOME confirm dialog: {e}", exc_info=True)
            self._close_home_confirm_modal()

    def _confirm_home_switch(self):
        """Confirm and send HOME command."""
        self._send_fsm_command(1, "HOME")
        self._close_home_confirm_modal()

    def _close_home_confirm_modal(self):
        """Close HOME confirmation dialog if open."""
        try:
            if self._home_confirm_modal is not None:
                self._home_confirm_modal.close()
        except Exception as e:
            logger.debug(f"Could not close HOME confirm dialog: {e}")
        finally:
            self._home_confirm_modal = None
    
    def _send_fsm_command(self, command: int, expected_state: Optional[str] = None):
        """Send FSM command via ROS2RobotInterface."""
        if self.ros2_interface is None or not self.ros2_interface.is_connected:
            logger.warning("ROS2RobotInterface not connected, cannot send FSM command")
            return
        self.ros2_interface.send_fsm_command(command)

    def _send_mode_command(self, command: str) -> None:
        """Publish a WBC /mode_command string (same topic as RViz)."""
        if self.ros2_interface is None or not self.ros2_interface.is_connected:
            logger.warning("ROS2RobotInterface not connected, cannot send mode command")
            return
        try:
            self.ros2_interface.send_mode_command(command)
        except Exception as e:
            logger.error(f"Failed to send mode command {command!r}: {e}", exc_info=True)

    @staticmethod
    def _fsm_state_to_name(state: int) -> str:
        state_map = {
            1: "HOME",
            2: "HOLD",
            3: "OCS2",
            4: "MOVEJ",
        }
        return state_map.get(state, f"UNKNOWN({state})")
    
    def _create_marker_controls(self):
        """Create end-effector marker publish-mode dropdown and send button (OCS2 only)."""
        self._marker_publish_mode_dropdown = None
        self._send_marker_pose_button = None

        if self.config is None or not self.config.enable_end_effector_marker:
            return

        self._marker_publish_mode_dropdown = self.server.gui.add_dropdown(
            self.translator("marker_publish_mode"),
            options=[
                self.translator("continuous_publish"),
                self.translator("single_publish"),
            ],
            initial_value=(
                self.translator("continuous_publish")
                if self.config.marker_continuous_publish
                else self.translator("single_publish")
            ),
        )
        self._marker_publish_mode_dropdown.visible = False

        self._send_marker_pose_button = self.server.gui.add_button(
            self.translator("send_marker_pose"),
            color="green",
        )
        self._send_marker_pose_button.visible = False
        self._apply_marker_callbacks()

    def _create_wbc_controls(self):
        """Create whole-body control widgets (shown only in OCS2 when WBC is present)."""
        self._wbc_folder = None
        self._wbc_status_text = None
        self._wbc_base_checkbox = None
        self._wbc_left_arm_checkbox = None
        self._wbc_right_arm_checkbox = None
        self._wbc_bimanual_checkbox = None
        self._wbc_home_pose_checkbox = None
        self._wbc_body_dropdown = None
        self._wbc_head_dropdown = None
        self._wbc_body_label_to_cmd = {}
        self._wbc_head_label_to_cmd = {}
        self._wbc_last_visible = None

        if not getattr(self.ros2_interface, "is_wbc", False):
            return

        self._wbc_folder = self.server.gui.add_folder(self.translator("wbc_control"))
        with self._wbc_folder:
            self._wbc_status_text = self.server.gui.add_text(
                self.translator("status"),
                initial_value=self.translator("wbc_waiting_capability"),
                disabled=True,
            )
            self._wbc_base_checkbox = self.server.gui.add_checkbox(
                self.translator("wbc_base"),
                initial_value=False,
            )
            self._wbc_base_checkbox.on_update(self._on_wbc_base_toggled)

            self._wbc_left_arm_checkbox = self.server.gui.add_checkbox(
                self.translator("wbc_left_arm"),
                initial_value=True,
            )
            self._wbc_left_arm_checkbox.on_update(self._on_wbc_left_arm_toggled)

            self._wbc_right_arm_checkbox = self.server.gui.add_checkbox(
                self.translator("wbc_right_arm"),
                initial_value=True,
            )
            self._wbc_right_arm_checkbox.on_update(self._on_wbc_right_arm_toggled)

            self._wbc_bimanual_checkbox = self.server.gui.add_checkbox(
                self.translator("wbc_bimanual"),
                initial_value=False,
            )
            self._wbc_bimanual_checkbox.on_update(self._on_wbc_bimanual_toggled)

            self._wbc_home_pose_checkbox = self.server.gui.add_checkbox(
                self.translator("wbc_home_pose"),
                initial_value=False,
            )
            self._wbc_home_pose_checkbox.on_update(self._on_wbc_home_pose_toggled)

            body_labels, body_map = self._body_mode_options()
            self._wbc_body_label_to_cmd = body_map
            self._wbc_body_dropdown = self.server.gui.add_dropdown(
                self.translator("wbc_body_mode"),
                options=body_labels,
                initial_value=body_labels[0],
            )
            self._wbc_body_dropdown.on_update(self._on_wbc_body_mode_changed)

            head_labels, head_map = self._head_mode_options()
            self._wbc_head_label_to_cmd = head_map
            self._wbc_head_dropdown = self.server.gui.add_dropdown(
                self.translator("wbc_head_mode"),
                options=head_labels,
                initial_value=head_labels[0],
            )
            self._wbc_head_dropdown.on_update(self._on_wbc_head_mode_changed)

        self._set_wbc_controls_visible(False)

    def _body_mode_options(self) -> Tuple[List[str], Dict[str, str]]:
        """Return available body-mode labels and label→command map from capabilities."""
        cap = getattr(self.ros2_interface, "wbc_capability", None)
        labels: List[str] = []
        mapping: Dict[str, str] = {}
        for key, command, _state in _BODY_MODE_OPTIONS:
            if command == "BODY_RELATIVE" and cap is not None and not cap.has_body_relative_constraint:
                continue
            if command == "BODY_TRACKING" and cap is not None and not cap.body_tracking_ee_enabled:
                continue
            if command == "BODY_LOCK" and cap is not None and not cap.has_waist_lock:
                continue
            if command == "BODY_CUSTOM_LOCK" and cap is not None and not cap.has_custom_joint_lock:
                continue
            label = self.translator(key)
            labels.append(label)
            mapping[label] = command
        if not labels:
            label = self.translator("wbc_body_free")
            labels = [label]
            mapping = {label: "BODY_FREE"}
        return labels, mapping

    def _head_mode_options(self) -> Tuple[List[str], Dict[str, str]]:
        """Return available head-mode labels and label→command map from capabilities."""
        cap = getattr(self.ros2_interface, "wbc_capability", None)
        labels: List[str] = []
        mapping: Dict[str, str] = {}
        for key, command, _state in _HEAD_MODE_OPTIONS:
            if command == "HEAD_TRACKING" and cap is not None and not cap.head_tracking_ee_enabled:
                continue
            if command == "HEAD_GAZE" and cap is not None and not cap.head_midpoint_gaze_enabled:
                continue
            if command == "HEAD_FORWARD" and cap is not None and not cap.has_head_coupling:
                continue
            label = self.translator(key)
            labels.append(label)
            mapping[label] = command
        if not labels:
            label = self.translator("wbc_head_disabled")
            labels = [label]
            mapping = {label: "HEAD_DISABLE"}
        return labels, mapping

    def _set_wbc_controls_visible(self, visible: bool) -> None:
        if self._wbc_last_visible is visible:
            return
        self._wbc_last_visible = visible
        if self._wbc_folder is not None and hasattr(self._wbc_folder, "visible"):
            self._wbc_folder.visible = visible
        for handle in (
            self._wbc_status_text,
            self._wbc_base_checkbox,
            self._wbc_left_arm_checkbox,
            self._wbc_right_arm_checkbox,
            self._wbc_bimanual_checkbox,
            self._wbc_home_pose_checkbox,
            self._wbc_body_dropdown,
            self._wbc_head_dropdown,
        ):
            if handle is not None:
                handle.visible = visible

    def _set_checkbox(self, handle: Optional[viser.GuiCheckboxHandle], value: bool, disabled: bool) -> None:
        if handle is None:
            return
        handle.disabled = disabled
        if handle.value != value:
            handle.value = value

    def _refresh_wbc_ui(self) -> None:
        """Sync WBC widgets from capability + current_state (RViz refreshWbcUi)."""
        if self._wbc_folder is None:
            return

        state = getattr(self.ros2_interface, "wbc_state", None)
        cap = getattr(self.ros2_interface, "wbc_capability", None)
        if state is None:
            if self._wbc_status_text is not None:
                self._wbc_status_text.value = self.translator("wbc_waiting_capability")
            return

        left_enabled = state.left_arm_state == WbcCurrentState.ARM_ENABLED
        right_enabled = state.right_arm_state == WbcCurrentState.ARM_ENABLED
        bimanual_coupled = state.bimanual_state == WbcCurrentState.BIMANUAL_COUPLED
        base_unlocked = state.base_state == WbcCurrentState.BASE_UNLOCKED
        home_on = bool(state.home_joint_reference_enabled)
        head_forward = state.head_state == WbcCurrentState.HEAD_FORWARD

        has_base = True if cap is None else bool(cap.has_mobile_base)
        has_bimanual = True if cap is None else bool(cap.has_bimanual_coupling)
        has_home = True if cap is None else bool(cap.has_home_joint_reference)
        any_arm_disabled = (not left_enabled) or (not right_enabled)

        self._wbc_ui_syncing = True
        try:
            if self._wbc_status_text is not None:
                self._wbc_status_text.value = "WBC"

            self._set_checkbox(self._wbc_base_checkbox, base_unlocked, disabled=not has_base)

            if bimanual_coupled:
                self._set_checkbox(self._wbc_left_arm_checkbox, True, disabled=True)
                self._set_checkbox(self._wbc_right_arm_checkbox, True, disabled=True)
            else:
                self._set_checkbox(self._wbc_left_arm_checkbox, left_enabled, disabled=False)
                self._set_checkbox(self._wbc_right_arm_checkbox, right_enabled, disabled=False)

            bimanual_disabled = (not has_bimanual) or any_arm_disabled
            self._set_checkbox(
                self._wbc_bimanual_checkbox,
                bimanual_coupled and not bimanual_disabled,
                disabled=bimanual_disabled,
            )

            self._set_checkbox(self._wbc_home_pose_checkbox, home_on, disabled=not has_home)

            # Refresh body/head option lists from latest capability.
            body_labels, body_map = self._body_mode_options()
            self._wbc_body_label_to_cmd = body_map
            if self._wbc_body_dropdown is not None:
                self._wbc_body_dropdown.options = body_labels
                current_body_label = None
                for key, command, body_state in _BODY_MODE_OPTIONS:
                    if body_state == state.body_state:
                        current_body_label = self.translator(key)
                        break
                if current_body_label not in body_labels:
                    current_body_label = body_labels[0]
                if self._wbc_body_dropdown.value != current_body_label:
                    self._wbc_body_dropdown.value = current_body_label
                self._wbc_body_dropdown.disabled = len(body_labels) <= 1 or head_forward

            head_labels, head_map = self._head_mode_options()
            self._wbc_head_label_to_cmd = head_map
            if self._wbc_head_dropdown is not None:
                self._wbc_head_dropdown.options = head_labels
                current_head_label = None
                for key, command, head_state in _HEAD_MODE_OPTIONS:
                    if head_state == state.head_state:
                        current_head_label = self.translator(key)
                        break
                if current_head_label not in head_labels:
                    # Keep showing current mode even if capability filtered it out.
                    if current_head_label is not None:
                        head_labels = list(head_labels) + [current_head_label]
                        head_map[current_head_label] = next(
                            cmd for k, cmd, hs in _HEAD_MODE_OPTIONS if hs == state.head_state
                        )
                        self._wbc_head_label_to_cmd = head_map
                        self._wbc_head_dropdown.options = head_labels
                    else:
                        current_head_label = head_labels[0]
                if self._wbc_head_dropdown.value != current_head_label:
                    self._wbc_head_dropdown.value = current_head_label
        finally:
            self._wbc_ui_syncing = False

    def _on_wbc_base_toggled(self, _event) -> None:
        if self._cleaned_up or self._wbc_ui_syncing or self._wbc_base_checkbox is None:
            return
        cap = getattr(self.ros2_interface, "wbc_capability", None)
        if cap is not None and not cap.has_mobile_base:
            self._refresh_wbc_ui()
            return
        state = getattr(self.ros2_interface, "wbc_state", None)
        locked = state is None or state.base_state == WbcCurrentState.BASE_LOCKED
        desired = self._wbc_base_checkbox.value
        # Checkbox ON means unlocked.
        if desired and locked:
            self._send_mode_command("BASE_UNLOCK")
        elif (not desired) and (not locked):
            self._send_mode_command("BASE_LOCK")

    def _on_wbc_left_arm_toggled(self, _event) -> None:
        if self._cleaned_up or self._wbc_ui_syncing or self._wbc_left_arm_checkbox is None:
            return
        state = getattr(self.ros2_interface, "wbc_state", None)
        if state is not None and state.bimanual_state == WbcCurrentState.BIMANUAL_COUPLED:
            self._refresh_wbc_ui()
            return
        enabled = state is not None and state.left_arm_state == WbcCurrentState.ARM_ENABLED
        desired = self._wbc_left_arm_checkbox.value
        if desired and not enabled:
            self._send_mode_command("LEFT_ARM_ENABLE")
        elif (not desired) and enabled:
            self._send_mode_command("LEFT_ARM_DISABLE")

    def _on_wbc_right_arm_toggled(self, _event) -> None:
        if self._cleaned_up or self._wbc_ui_syncing or self._wbc_right_arm_checkbox is None:
            return
        state = getattr(self.ros2_interface, "wbc_state", None)
        if state is not None and state.bimanual_state == WbcCurrentState.BIMANUAL_COUPLED:
            self._refresh_wbc_ui()
            return
        enabled = state is not None and state.right_arm_state == WbcCurrentState.ARM_ENABLED
        desired = self._wbc_right_arm_checkbox.value
        if desired and not enabled:
            self._send_mode_command("RIGHT_ARM_ENABLE")
        elif (not desired) and enabled:
            self._send_mode_command("RIGHT_ARM_DISABLE")

    def _on_wbc_bimanual_toggled(self, _event) -> None:
        if self._cleaned_up or self._wbc_ui_syncing or self._wbc_bimanual_checkbox is None:
            return
        cap = getattr(self.ros2_interface, "wbc_capability", None)
        state = getattr(self.ros2_interface, "wbc_state", None)
        if cap is not None and not cap.has_bimanual_coupling:
            self._refresh_wbc_ui()
            return
        if state is not None and (
            state.left_arm_state != WbcCurrentState.ARM_ENABLED
            or state.right_arm_state != WbcCurrentState.ARM_ENABLED
        ):
            self._refresh_wbc_ui()
            return
        coupled = state is not None and state.bimanual_state == WbcCurrentState.BIMANUAL_COUPLED
        desired = self._wbc_bimanual_checkbox.value
        if desired and not coupled:
            self._send_mode_command("ARMS_COUPLED")
        elif (not desired) and coupled:
            self._send_mode_command("ARMS_INDEPENDENT")

    def _on_wbc_home_pose_toggled(self, _event) -> None:
        if self._cleaned_up or self._wbc_ui_syncing or self._wbc_home_pose_checkbox is None:
            return
        cap = getattr(self.ros2_interface, "wbc_capability", None)
        if cap is not None and not cap.has_home_joint_reference:
            self._refresh_wbc_ui()
            return
        state = getattr(self.ros2_interface, "wbc_state", None)
        on = state is not None and bool(state.home_joint_reference_enabled)
        desired = self._wbc_home_pose_checkbox.value
        if desired and not on:
            self._send_mode_command("HOME_JOINT_ON")
        elif (not desired) and on:
            self._send_mode_command("HOME_JOINT_OFF")

    def _on_wbc_body_mode_changed(self, _event) -> None:
        if self._cleaned_up or self._wbc_ui_syncing or self._wbc_body_dropdown is None:
            return
        command = self._wbc_body_label_to_cmd.get(self._wbc_body_dropdown.value)
        if not command:
            return
        state = getattr(self.ros2_interface, "wbc_state", None)
        if state is not None:
            for _key, cmd, body_state in _BODY_MODE_OPTIONS:
                if cmd == command and state.body_state == body_state:
                    return
        self._send_mode_command(command)

    def _on_wbc_head_mode_changed(self, _event) -> None:
        if self._cleaned_up or self._wbc_ui_syncing or self._wbc_head_dropdown is None:
            return
        command = self._wbc_head_label_to_cmd.get(self._wbc_head_dropdown.value)
        if not command:
            return
        state = getattr(self.ros2_interface, "wbc_state", None)
        if state is not None:
            for _key, cmd, head_state in _HEAD_MODE_OPTIONS:
                if cmd == command and state.head_state == head_state:
                    return
        self._send_mode_command(command)

    def _apply_marker_callbacks(self):
        """Re-bind stored callbacks to marker GUI controls."""
        if self._marker_mode_callback is not None and self._marker_publish_mode_dropdown is not None:
            self._marker_publish_mode_dropdown.on_update(self._marker_mode_callback)
        if self._marker_send_callback is not None and self._send_marker_pose_button is not None:
            self._send_marker_pose_button.on_click(self._marker_send_callback)

    def set_marker_callbacks(self, on_mode_changed, on_send_clicked):
        """Set callbacks for marker controls.
        
        Args:
            on_mode_changed: Callback function for publish mode dropdown change
            on_send_clicked: Callback function for send button click
        """
        self._marker_mode_callback = on_mode_changed
        self._marker_send_callback = on_send_clicked
        self._apply_marker_callbacks()
    
    def get_marker_controls(self):
        """Get marker control handles for connection to marker manager.
        
        Returns:
            Tuple of (publish_mode_dropdown, send_button) or (None, None) if not available.
        """
        return (self._marker_publish_mode_dropdown, self._send_marker_pose_button)
    
    def update(self):
        """Update FSM panel button visibility based on current state.
        
        This method should be called periodically from the main update loop
        (not from ROS2 callbacks) to avoid blocking the executor.
        Only updates button visibility when the state actually changes.
        """
        current_state = self.ros2_interface.get_fsm_state()
        # Update if state changed or if this is the first update (INVALID state)
        if current_state != self._current_fsm_state or self._current_fsm_state == 0:
            self._current_fsm_state = current_state
            if self._fsm_state_label is not None:
                self._fsm_state_label.value = self._fsm_state_to_name(current_state)
            
            # Only update button visibility when state changes
            if current_state == 1:
                self._fsm_button_handles["to_home"].visible = False
                self._fsm_button_handles["to_hold"].visible = True
                self._fsm_button_handles["to_ocs2"].visible = False
                self._fsm_button_handles["to_movej"].visible = False
                self._fsm_switch_pose_button.visible = True
            elif current_state == 2:
                self._fsm_button_handles["to_home"].visible = True
                self._fsm_button_handles["to_hold"].visible = False
                self._fsm_button_handles["to_ocs2"].visible = True
                self._fsm_button_handles["to_movej"].visible = True
                self._fsm_switch_pose_button.visible = False
            elif current_state == 3:
                self._fsm_button_handles["to_home"].visible = False
                self._fsm_button_handles["to_hold"].visible = True
                self._fsm_button_handles["to_ocs2"].visible = False
                self._fsm_button_handles["to_movej"].visible = False
                self._fsm_switch_pose_button.visible = False
                # Show marker controls in OCS2 mode (visibility managed by marker manager)
                # Marker manager will handle showing/hiding based on publish mode
            elif current_state == 4:
                # MOVEJ state: can only go to HOLD
                self._fsm_button_handles["to_home"].visible = False
                self._fsm_button_handles["to_hold"].visible = True
                self._fsm_button_handles["to_ocs2"].visible = False
                self._fsm_button_handles["to_movej"].visible = False
                self._fsm_switch_pose_button.visible = False
            else:
                logger.warning(f"Unknown FSM state: {current_state}")

        # WBC section: only while OCS2 + WBC (same as RViz OCS2FSMPanel).
        show_wbc = (
            current_state == 3
            and bool(getattr(self.ros2_interface, "is_wbc", False))
            and self._wbc_folder is not None
        )
        self._set_wbc_controls_visible(show_wbc)
        if show_wbc:
            self._refresh_wbc_ui()
    
    def update_gui_labels(self):
        """Update GUI labels with current language without recreating subscriptions.
        
        This method only updates the text/labels of GUI elements, preserving
        all ROS2 subscriptions and state.
        """
        if self._folder_handle is not None:
            old_folder = self._folder_handle
            self._folder_handle = self.server.gui.add_folder(self.translator("fsm_control"))
            with self._folder_handle:
                self._fsm_state_label = self.server.gui.add_text(
                    self.translator("current_state"),
                    initial_value=self._fsm_state_to_name(self._current_fsm_state),
                    disabled=True
                )
                
                # Recreate buttons
                self._fsm_button_handles["to_home"] = self.server.gui.add_button("HOME", color=(128, 0, 128))
                self._fsm_button_handles["to_home"].on_click(
                    self._on_home_clicked
                )
                
                self._fsm_button_handles["to_hold"] = self.server.gui.add_button("HOLD", color="yellow")
                self._fsm_button_handles["to_hold"].on_click(
                    lambda _: self._send_fsm_command(2, "HOLD")
                )
                
                self._fsm_button_handles["to_ocs2"] = self.server.gui.add_button("OCS2", color="blue")
                self._fsm_button_handles["to_ocs2"].on_click(
                    lambda _: self._send_fsm_command(3, "OCS2")
                )
                
                self._fsm_button_handles["to_movej"] = self.server.gui.add_button("MOVEJ", color="cyan")
                self._fsm_button_handles["to_movej"].on_click(
                    lambda _: self._send_fsm_command(4, "MOVEJ")
                )
                
                # Recreate switch pose button
                switch_pose_text = self.translator("switch_pose")
                self._fsm_switch_pose_button = self.server.gui.add_button(
                    f"{switch_pose_text} (Home ↔ Rest)",
                    color=(255, 100, 0)
                )
                self._fsm_switch_pose_button.on_click(self._on_switch_pose_clicked)
                self._create_marker_controls()
                self._create_wbc_controls()
            
            # Remove old folder
            try:
                old_folder.remove()
            except Exception as e:
                logger.debug(f"Could not remove old folder: {e}")
            
            # Force visibility refresh (marker controls recreated above)
            self._current_fsm_state = 0
            self.update()
            
            logger.debug("FSM panel GUI labels updated")
    
    def cleanup(self):
        """Cleanup resources (subscriptions, GUI elements, etc.)."""
        # Set cleanup flag first to prevent callbacks from running
        self._cleaned_up = True
        
        # Hide all GUI elements
        try:
            self._close_home_confirm_modal()
            if self._fsm_state_label is not None:
                self._fsm_state_label.visible = False
            for handle in self._fsm_button_handles.values():
                if handle is not None:
                    handle.visible = False
            if self._fsm_switch_pose_button is not None:
                self._fsm_switch_pose_button.visible = False
            self._set_wbc_controls_visible(False)
            # Hide folder if it has a visible attribute
            if self._folder_handle is not None:
                try:
                    if hasattr(self._folder_handle, 'visible'):
                        self._folder_handle.visible = False
                    elif hasattr(self._folder_handle, 'remove'):
                        self._folder_handle.remove()
                except Exception as e:
                    logger.debug(f"Could not hide/remove folder: {e}")
        except Exception as e:
            logger.warning(f"Error hiding FSM panel GUI elements: {e}")
        
        # Note: No direct subscription to cleanup - ros2_interface handles it
        
        # Clear references
        self._fsm_state_label = None
        self._fsm_button_handles.clear()
        self._fsm_switch_pose_button = None
        self._home_confirm_modal = None
        self._marker_publish_mode_dropdown = None
        self._send_marker_pose_button = None
        self._wbc_folder = None
        self._wbc_status_text = None
        self._wbc_base_checkbox = None
        self._wbc_left_arm_checkbox = None
        self._wbc_right_arm_checkbox = None
        self._wbc_bimanual_checkbox = None
        self._wbc_home_pose_checkbox = None
        self._wbc_body_dropdown = None
        self._wbc_head_dropdown = None
        self._wbc_body_label_to_cmd = {}
        self._wbc_head_label_to_cmd = {}
        self._wbc_last_visible = None
        self._folder_handle = None
        self._initialized = False
        logger.debug("FSM panel cleaned up")
