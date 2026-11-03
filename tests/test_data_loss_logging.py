"""Missing joint_states / base TF must be logged once, not every few seconds.

Regression context: after the simulation stopped, the visualizer repeated
"No joint_states ..." and "No TF world <- base_footprint ..." every 5s forever.
"""

from __future__ import annotations

from types import SimpleNamespace
from unittest import mock

import pytest

pytest.importorskip("arms_ros2_control_msgs.msg", reason="needs a sourced ROS workspace")

from ros2_viser import visualizer as viz_module  # noqa: E402
from ros2_viser.visualizer import ROS2ViserVisualizer  # noqa: E402


def _visualizer() -> ROS2ViserVisualizer:
    viz = object.__new__(ROS2ViserVisualizer)
    viz._map_joint_states_for_session = lambda session, names, positions: {}
    return viz


def _session(*, joint_state=None, joint_state_ok: bool = True):
    return SimpleNamespace(
        robot_description_topic="/robot_description",
        urdf_root_link="/base_link",
        interface=SimpleNamespace(
            get_joint_state=lambda: joint_state,
            config=SimpleNamespace(
                joint_states_topic="/joint_states",
                namespace="",
            ),
        ),
        urdf_vis=SimpleNamespace(update_cfg=lambda cfg: None),
        robot_base_frame=SimpleNamespace(position=None, wxyz=None, visible=True),
        joint_state_ok=joint_state_ok,
        base_tf_ok=True,
        stale_hidden=False,
    )


# ------------------------------------------------------------- joint_states


def test_joint_state_outage_warns_once():
    viz = _visualizer()
    session = _session(joint_state=None)

    with mock.patch.object(viz_module.logger, "warning") as warning:
        assert viz._update_session_joint_state(session) is False
        assert viz._update_session_joint_state(session) is False
        assert viz._update_session_joint_state(session) is False

    assert warning.call_count == 1
    assert session.joint_state_ok is False


def test_joint_state_recovery_logs_once():
    viz = _visualizer()
    session = _session(joint_state={"names": ["a"], "positions": [0.0]}, joint_state_ok=False)

    with mock.patch.object(viz_module.logger, "info") as info:
        assert viz._update_session_joint_state(session) is True
        assert viz._update_session_joint_state(session) is True

    assert info.call_count == 1
    assert session.joint_state_ok is True


def test_healthy_joint_states_are_silent():
    viz = _visualizer()
    session = _session(joint_state={"names": ["a"], "positions": [0.0]})

    with (
        mock.patch.object(viz_module.logger, "warning") as warning,
        mock.patch.object(viz_module.logger, "info") as info,
    ):
        viz._update_session_joint_state(session)

    warning.assert_not_called()
    info.assert_not_called()


def test_returning_data_unhides_a_stale_robot():
    viz = _visualizer()
    session = _session(joint_state={"names": ["a"], "positions": [0.0]})
    session.stale_hidden = True
    session.robot_base_frame.visible = False

    viz._update_session_joint_state(session)

    assert session.stale_hidden is False
    assert session.robot_base_frame.visible is True


# ------------------------------------------------------------------ base TF


def test_missing_base_tf_warns_once_and_recovers():
    viz = _visualizer()
    viz._selected_viz_frame = "world"
    session = _session()

    with mock.patch.object(viz_module.logger, "warning") as warning:
        viz._lookup_transform_quiet = lambda *a, **k: None
        viz._apply_session_base_from_tf(session)
        viz._apply_session_base_from_tf(session)
        viz._apply_session_base_from_tf(session)

    assert warning.call_count == 1
    assert session.base_tf_ok is False

    transform = SimpleNamespace(
        transform=SimpleNamespace(
            translation=SimpleNamespace(x=1.0, y=2.0, z=3.0),
            rotation=SimpleNamespace(w=1.0, x=0.0, y=0.0, z=0.0),
        )
    )
    viz._lookup_transform_quiet = lambda *a, **k: transform
    with mock.patch.object(viz_module.logger, "info") as info:
        viz._apply_session_base_from_tf(session)
        viz._apply_session_base_from_tf(session)

    assert info.call_count == 1
    assert session.base_tf_ok is True
    assert session.robot_base_frame.position == (1.0, 2.0, 3.0)


def test_healthy_tf_is_quiet():
    viz = _visualizer()
    viz._selected_viz_frame = "world"
    session = _session()
    transform = SimpleNamespace(
        transform=SimpleNamespace(
            translation=SimpleNamespace(x=0.0, y=0.0, z=0.0),
            rotation=SimpleNamespace(w=1.0, x=0.0, y=0.0, z=0.0),
        )
    )
    viz._lookup_transform_quiet = lambda *a, **k: transform

    with (
        mock.patch.object(viz_module.logger, "warning") as warning,
        mock.patch.object(viz_module.logger, "info") as info,
    ):
        viz._apply_session_base_from_tf(session)

    warning.assert_not_called()
    info.assert_not_called()
