"""WBC checkbox handlers must send a mode command from the *event* value.

Regression context: viser dispatches browser-originated ``on_update`` callbacks
on a thread pool, while ``_refresh_wbc_ui`` rewrites ``handle.value`` from the
update loop. Reading ``handle.value`` inside the callback therefore observed the
refreshed value and either dropped the click or inverted the command. The
handlers now compare ``event.target.value`` against ``wbc_state``.
"""

from __future__ import annotations

import time
from types import SimpleNamespace
from unittest import mock

import pytest

msgs = pytest.importorskip(
    "arms_ros2_control_msgs.msg", reason="needs a sourced ROS workspace"
)
WbcCurrentState = msgs.WbcCurrentState

from ros2_viser.panels import fsm_panel as fsm_panel_module  # noqa: E402
from ros2_viser.panels.fsm_panel import FSMPanel  # noqa: E402


def _event(value: bool) -> SimpleNamespace:
    return SimpleNamespace(target=SimpleNamespace(value=value))


def _state(**overrides) -> SimpleNamespace:
    state = SimpleNamespace(
        base_state=WbcCurrentState.BASE_LOCKED,
        body_state=WbcCurrentState.BODY_FREE,
        bimanual_state=WbcCurrentState.BIMANUAL_INDEPENDENT,
        left_arm_state=WbcCurrentState.ARM_ENABLED,
        right_arm_state=WbcCurrentState.ARM_ENABLED,
        home_joint_reference_enabled=False,
        head_state=WbcCurrentState.HEAD_DISABLED,
    )
    for key, value in overrides.items():
        setattr(state, key, value)
    return state


def _capability(**overrides) -> SimpleNamespace:
    capability = SimpleNamespace(
        has_mobile_base=True,
        has_bimanual_coupling=True,
        has_home_joint_reference=True,
        has_body_relative_constraint=True,
        has_head_coupling=True,
        head_tracking_ee_enabled=True,
        head_midpoint_gaze_enabled=True,
    )
    for key, value in overrides.items():
        setattr(capability, key, value)
    return capability


def _make_panel(state=None, capability=None):
    """Build a panel without touching viser/ROS (only the handler paths are used)."""
    panel = object.__new__(FSMPanel)
    panel._cleaned_up = False
    panel._wbc_ui_syncing = False
    panel._wbc_status_text = None
    panel._wbc_base_checkbox = SimpleNamespace(value=False, disabled=False)
    panel._wbc_left_arm_checkbox = SimpleNamespace(value=True, disabled=False)
    panel._wbc_right_arm_checkbox = SimpleNamespace(value=True, disabled=False)
    panel._wbc_bimanual_checkbox = SimpleNamespace(value=False, disabled=False)
    panel._wbc_home_pose_checkbox = SimpleNamespace(value=False, disabled=False)
    panel._wbc_body_dropdown = None
    panel._wbc_head_dropdown = None
    panel._wbc_body_label_to_cmd = {}
    panel._wbc_head_label_to_cmd = {}
    panel._wbc_last_visible = None
    panel._wbc_state_missing_since = None
    panel._wbc_state_warn_ts = 0.0

    commands: list[str] = []
    refreshes: list[int] = []
    panel._send_mode_command = commands.append
    panel._refresh_wbc_ui = lambda: refreshes.append(1)
    panel.ros2_interface = SimpleNamespace(
        wbc_state=state,
        wbc_capability=capability,
        is_wbc=True,
        config=SimpleNamespace(topic_prefix="/g1d"),
    )
    return panel, commands, refreshes


# --------------------------------------------------------------------------- base


def test_base_click_unlocks_when_locked():
    panel, commands, _ = _make_panel(state=_state(base_state=WbcCurrentState.BASE_LOCKED))
    panel._on_wbc_base_toggled(_event(True))
    assert commands == ["BASE_UNLOCK"]


def test_base_click_locks_when_unlocked():
    panel, commands, _ = _make_panel(
        state=_state(base_state=WbcCurrentState.BASE_UNLOCKED)
    )
    panel._on_wbc_base_toggled(_event(False))
    assert commands == ["BASE_LOCK"]


def test_base_echo_of_refresh_sends_nothing():
    """A refresh-triggered callback carries the value already reflected by state."""
    panel, commands, _ = _make_panel(
        state=_state(base_state=WbcCurrentState.BASE_UNLOCKED)
    )
    panel._on_wbc_base_toggled(_event(True))
    assert commands == []


def test_base_ignored_without_mobile_base():
    panel, commands, refreshes = _make_panel(
        state=_state(base_state=WbcCurrentState.BASE_LOCKED),
        capability=_capability(has_mobile_base=False),
    )
    panel._on_wbc_base_toggled(_event(True))
    assert commands == []
    assert refreshes == [1]


def test_base_ignored_before_first_state():
    panel, commands, _ = _make_panel(state=None)
    panel._on_wbc_base_toggled(_event(True))
    assert commands == []


# ----------------------------------------------------------------------- arms


def test_left_arm_disable_on_uncheck():
    panel, commands, _ = _make_panel(
        state=_state(left_arm_state=WbcCurrentState.ARM_ENABLED)
    )
    panel._on_wbc_left_arm_toggled(_event(False))
    assert commands == ["LEFT_ARM_DISABLE"]


def test_left_arm_enable_on_check():
    panel, commands, _ = _make_panel(
        state=_state(left_arm_state=WbcCurrentState.ARM_DISABLED)
    )
    panel._on_wbc_left_arm_toggled(_event(True))
    assert commands == ["LEFT_ARM_ENABLE"]


def test_right_arm_uses_same_intent_rule():
    panel, commands, _ = _make_panel(
        state=_state(right_arm_state=WbcCurrentState.ARM_DISABLED)
    )
    panel._on_wbc_right_arm_toggled(_event(True))
    assert commands == ["RIGHT_ARM_ENABLE"]


def test_arm_toggle_refused_while_coupled():
    panel, commands, refreshes = _make_panel(
        state=_state(
            bimanual_state=WbcCurrentState.BIMANUAL_COUPLED,
            left_arm_state=WbcCurrentState.ARM_ENABLED,
        )
    )
    panel._on_wbc_left_arm_toggled(_event(False))
    assert commands == []
    assert refreshes == [1]


# ------------------------------------------------------------------- bimanual


def test_bimanual_couples_when_both_arms_enabled():
    panel, commands, _ = _make_panel(
        state=_state(
            left_arm_state=WbcCurrentState.ARM_ENABLED,
            right_arm_state=WbcCurrentState.ARM_ENABLED,
            bimanual_state=WbcCurrentState.BIMANUAL_INDEPENDENT,
        )
    )
    panel._on_wbc_bimanual_toggled(_event(True))
    assert commands == ["ARMS_COUPLED"]


def test_bimanual_uncouples_when_coupled():
    panel, commands, _ = _make_panel(
        state=_state(bimanual_state=WbcCurrentState.BIMANUAL_COUPLED)
    )
    panel._on_wbc_bimanual_toggled(_event(False))
    assert commands == ["ARMS_INDEPENDENT"]


def test_bimanual_refused_when_an_arm_is_disabled():
    panel, commands, refreshes = _make_panel(
        state=_state(
            left_arm_state=WbcCurrentState.ARM_ENABLED,
            right_arm_state=WbcCurrentState.ARM_DISABLED,
        )
    )
    panel._on_wbc_bimanual_toggled(_event(True))
    assert commands == []
    assert refreshes == [1]


# ------------------------------------------------------------------ home pose


def test_home_pose_on_and_off():
    panel, commands, _ = _make_panel(
        state=_state(home_joint_reference_enabled=False)
    )
    panel._on_wbc_home_pose_toggled(_event(True))
    assert commands == ["HOME_JOINT_ON"]

    panel, commands, _ = _make_panel(
        state=_state(home_joint_reference_enabled=True)
    )
    panel._on_wbc_home_pose_toggled(_event(False))
    assert commands == ["HOME_JOINT_OFF"]


# ------------------------------------------------------- missing-state warning


def _build_warning_panel(state=None):
    panel, _, _ = _make_panel(state=state)
    return panel


def test_missing_state_warning_fires_once_per_interval():
    panel = _build_warning_panel(state=None)
    panel._wbc_state_missing_since = time.monotonic() - 10.0
    panel._wbc_state_warn_ts = 0.0

    with mock.patch.object(fsm_panel_module.logger, "warning") as warning:
        panel._warn_if_wbc_state_missing()
        assert warning.call_count == 1
        panel._warn_if_wbc_state_missing()
        assert warning.call_count == 1, "warning must not repeat within the interval"

    args = warning.call_args.args
    assert "/g1d" in args, f"topic_prefix should be logged, got {args!r}"


def test_missing_state_warning_waits_for_threshold():
    panel = _build_warning_panel(state=None)
    with mock.patch.object(fsm_panel_module.logger, "warning") as warning:
        panel._warn_if_wbc_state_missing()  # starts the timer
        panel._warn_if_wbc_state_missing()  # still within threshold
    warning.assert_not_called()


def test_state_present_resets_missing_timer():
    panel = _build_warning_panel(state=_state())
    panel._wbc_state_missing_since = 123.0
    panel._warn_if_wbc_state_missing()
    assert panel._wbc_state_missing_since is None
