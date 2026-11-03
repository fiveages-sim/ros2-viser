"""In OCS2 mode a disabled arm must lose its interactive marker.

Regression context: the WBC panel can disable one arm, but the draggable
end-effector marker stayed visible and kept publishing poses for that arm.
"""

from __future__ import annotations

from types import SimpleNamespace

import pytest

msgs = pytest.importorskip(
    "arms_ros2_control_msgs.msg", reason="needs a sourced ROS workspace"
)
WbcCurrentState = msgs.WbcCurrentState

from ros2_viser.end_effector_marker import EndEffectorMarkerManager  # noqa: E402

ENABLED = WbcCurrentState.ARM_ENABLED
DISABLED = WbcCurrentState.ARM_DISABLED


def _state(left: int = ENABLED, right: int = ENABLED):
    return SimpleNamespace(
        left_arm_state=left,
        right_arm_state=right,
        bimanual_state=WbcCurrentState.BIMANUAL_INDEPENDENT,
        body_state=WbcCurrentState.BODY_FREE,
        head_state=WbcCurrentState.HEAD_DISABLED,
    )


def _manager(state=_state()):
    mgr = object.__new__(EndEffectorMarkerManager)
    mgr._left_ee_marker = SimpleNamespace(visible=False)
    mgr._right_ee_marker = SimpleNamespace(visible=False)
    mgr._body_marker = None
    mgr._head_marker = None
    mgr._last_arm_marker_visible = None
    mgr.ros2_interface = SimpleNamespace(wbc_state=state, is_connected=True)
    return mgr


def _visibility(mgr) -> tuple[bool, bool]:
    return mgr._left_ee_marker.visible, mgr._right_ee_marker.visible


def test_both_arms_enabled_show_both_markers():
    mgr = _manager()
    mgr._update_marker_visibility(True, show_body=False, show_head=False)
    assert _visibility(mgr) == (True, True)


def test_disabled_left_arm_hides_left_marker_only():
    mgr = _manager(_state(left=DISABLED))
    mgr._update_marker_visibility(True, show_body=False, show_head=False)
    assert _visibility(mgr) == (False, True)


def test_disabled_right_arm_hides_right_marker_only():
    mgr = _manager(_state(right=DISABLED))
    mgr._update_marker_visibility(True, show_body=False, show_head=False)
    assert _visibility(mgr) == (True, False)


def test_both_arms_disabled_hide_both_markers():
    mgr = _manager(_state(left=DISABLED, right=DISABLED))
    mgr._update_marker_visibility(True, show_body=False, show_head=False)
    assert _visibility(mgr) == (False, False)


def test_markers_follow_state_transitions():
    mgr = _manager()
    mgr._update_marker_visibility(True, show_body=False, show_head=False)
    assert _visibility(mgr) == (True, True)

    mgr.ros2_interface.wbc_state = _state(left=DISABLED)
    mgr._update_marker_visibility(True, show_body=False, show_head=False)
    assert _visibility(mgr) == (False, True)

    mgr.ros2_interface.wbc_state = _state()
    mgr._update_marker_visibility(True, show_body=False, show_head=False)
    assert _visibility(mgr) == (True, True)


def test_outside_ocs2_markers_stay_hidden():
    mgr = _manager()
    mgr._update_marker_visibility(False, show_body=False, show_head=False)
    assert _visibility(mgr) == (False, False)


def test_markers_visible_when_wbc_state_is_unknown():
    """Before the first current_state message, hiding would just flicker."""
    mgr = _manager(state=None)
    mgr._update_marker_visibility(True, show_body=False, show_head=False)
    assert _visibility(mgr) == (True, True)


def test_arm_marker_enabled_reads_wbc_state():
    mgr = _manager(_state(left=ENABLED, right=DISABLED))
    assert mgr._arm_marker_enabled("left") is True
    assert mgr._arm_marker_enabled("right") is False

    mgr.ros2_interface.wbc_state = None
    assert mgr._arm_marker_enabled("left") is True
    assert mgr._arm_marker_enabled("right") is True


def test_disabled_arm_target_axes_are_hidden_without_querying_the_handler():
    mgr = _manager(_state(left=DISABLED))
    mgr._show_current_target_axes = True
    mgr._left_current_target_axes = SimpleNamespace(visible=True)
    mgr._right_current_target_axes = SimpleNamespace(visible=True)
    mgr._last_left_current_target_display = None
    mgr._last_right_current_target_display = None

    def _must_not_be_called():
        raise AssertionError("disabled arm marker must not be queried")

    mgr.ros2_interface.left_arm_handler = SimpleNamespace(
        get_target_pose=_must_not_be_called,
        get_frame_id=lambda: "base",
    )
    mgr.ros2_interface.right_arm_handler = SimpleNamespace(
        get_target_pose=lambda: None,
        get_frame_id=lambda: "base",
    )

    mgr._update_current_target_axes()

    assert mgr._left_current_target_axes.visible is False
