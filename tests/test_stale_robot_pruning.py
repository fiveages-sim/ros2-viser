"""Robots left behind by a stopped simulation must be pruned.

Regression context: the simulator was stopped, its ``robot_description`` publisher
disappeared, but the visualizer kept the cached session, so the robot stayed in
the scene (and the next run showed an extra one).
"""

from __future__ import annotations

import time
from types import SimpleNamespace

import pytest

pytest.importorskip("arms_ros2_control_msgs.msg", reason="needs a sourced ROS workspace")

from ros2_viser.visualizer import ROS2ViserVisualizer  # noqa: E402

TOPIC = "/g1d/robot_description"


def _session(
    *,
    last_msg_time: float = 0.0,
    tf_msg_time: float = 0.0,
    created_ts: float = 0.0,
    robot_node=object(),
):
    return SimpleNamespace(
        robot_description_topic=TOPIC,
        created_ts=created_ts,
        stale_hidden=False,
        robot_base_frame=SimpleNamespace(visible=True),
        interface=SimpleNamespace(
            robot_node=robot_node,
            get_last_joint_state_time=lambda: last_msg_time or None,
            tf_last_msg_time=tf_msg_time,
        ),
    )


def _visualizer(*, grace: float = 10.0, sessions=None):
    viz = object.__new__(ROS2ViserVisualizer)
    viz.config = SimpleNamespace(prune_stale_robots_after=grace)
    viz._robot_sessions = dict(sessions or {})
    viz._client_views = {}
    viz._selected_robot_topic = None
    viz._next_stale_check = 0.0
    viz._robot_dropdown = None
    viz._discovered_robot_topics = [TOPIC]

    viz.destroyed: list[str] = []
    viz.publisher_checks: list[str] = []
    viz.dropdown_writes: list[tuple[int, str | None]] = []
    viz.refreshed: list[int] = []

    def _destroy(topic):
        viz._robot_sessions.pop(topic, None)
        viz.destroyed.append(topic)

    viz._destroy_session = _destroy
    viz._cleanup_client_control_ui = lambda view: None
    viz._set_client_robot_dropdown = lambda view, topic: viz.dropdown_writes.append(
        (view.client_id, topic)
    )
    viz._refresh_client_robot_dropdown = lambda view: viz.refreshed.append(view.client_id)
    return viz


def test_stale_robot_without_publisher_is_removed():
    viz = _visualizer(sessions={TOPIC: _session()})
    viz._session_has_publisher = lambda session: False

    viz._maybe_prune_stale_sessions()

    assert viz.destroyed == [TOPIC]
    assert viz._robot_sessions == {}


def test_stale_grace_defaults_to_five_seconds():
    import dataclasses

    from ros2_viser.config import ROS2ViserConfig

    fields = {f.name: f for f in dataclasses.fields(ROS2ViserConfig)}
    assert fields["prune_stale_robots_after"].default == 5.0


def test_removal_refreshes_every_picker():
    """The removed namespace must disappear from the picker immediately."""
    view = SimpleNamespace(client_id=2, selected_robot_topic=None, switching=False)
    viz = _visualizer(sessions={TOPIC: _session()})
    viz._session_has_publisher = lambda session: False
    viz._client_views[2] = view

    viz._maybe_prune_stale_sessions()

    assert viz.refreshed == [2]
    assert TOPIC not in viz._discovered_robot_topics


def test_switching_view_is_not_refreshed_during_cleanup():
    view = SimpleNamespace(client_id=3, selected_robot_topic=None, switching=True)
    viz = _visualizer(sessions={TOPIC: _session()})
    viz._session_has_publisher = lambda session: False
    viz._client_views[3] = view

    viz._maybe_prune_stale_sessions()

    assert viz.refreshed == []


def test_silent_robot_with_publisher_is_hidden_not_removed():
    """Leftover simulation nodes keep publishing latching URDF but no data."""
    session = _session()
    viz = _visualizer(sessions={TOPIC: session})
    viz._session_has_publisher = lambda s: True

    viz._maybe_prune_stale_sessions()

    assert viz.destroyed == []
    assert session.stale_hidden is True
    assert session.robot_base_frame.visible is False


def test_fresh_data_skips_the_graph_query():
    viz = _visualizer(sessions={TOPIC: _session(last_msg_time=time.time())})
    viz._session_has_publisher = lambda session: viz.publisher_checks.append("x") or False

    viz._maybe_prune_stale_sessions()

    assert viz.destroyed == []
    assert viz.publisher_checks == []


def test_tf_message_time_alone_keeps_the_robot_alive():
    """A TF-only robot (no joint_states) must not be treated as gone."""
    viz = _visualizer(
        sessions={TOPIC: _session(last_msg_time=0.0, tf_msg_time=time.time())}
    )
    viz._session_has_publisher = lambda session: viz.publisher_checks.append("x") or False

    viz._maybe_prune_stale_sessions()

    assert viz.destroyed == []
    assert viz.publisher_checks == []


def test_pruning_can_be_disabled():
    viz = _visualizer(grace=0.0, sessions={TOPIC: _session()})
    viz._session_has_publisher = lambda session: False

    viz._maybe_prune_stale_sessions()

    assert viz.destroyed == []


def test_pruning_respects_the_cooldown():
    viz = _visualizer(sessions={TOPIC: _session()})
    viz._session_has_publisher = lambda session: False

    viz._maybe_prune_stale_sessions()
    viz._next_stale_check = time.time() + 60.0
    viz._robot_sessions[TOPIC] = _session()

    viz._maybe_prune_stale_sessions()

    assert viz.destroyed == [TOPIC]


def test_selected_robot_and_bound_views_are_cleared():
    view = SimpleNamespace(client_id=1, selected_robot_topic=TOPIC, switching=False)
    viz = _visualizer(sessions={TOPIC: _session()})
    viz._session_has_publisher = lambda session: False
    viz._client_views[1] = view
    viz._selected_robot_topic = TOPIC

    viz._maybe_prune_stale_sessions()

    assert viz._selected_robot_topic is None
    assert view.selected_robot_topic is None
    assert viz.refreshed == [1]
    assert TOPIC not in viz._discovered_robot_topics


# ------------------------------------------------------------ publisher probe


def test_publisher_probe_reads_count():
    class _Node:
        def __init__(self, count):
            self._count = count

        def count_publishers(self, topic):
            assert topic == TOPIC
            return self._count

    live = ROS2ViserVisualizer._session_has_publisher(
        None, _session(robot_node=_Node(1))
    )
    dead = ROS2ViserVisualizer._session_has_publisher(
        None, _session(robot_node=_Node(0))
    )

    assert live is True
    assert dead is False


def test_publisher_probe_is_conservative_on_errors():
    class _BoomNode:
        def count_publishers(self, topic):
            raise RuntimeError("rcl error")

    assert (
        ROS2ViserVisualizer._session_has_publisher(
            None, _session(robot_node=_BoomNode())
        )
        is True
    )


def test_publisher_probe_without_node_reports_dead():
    assert (
        ROS2ViserVisualizer._session_has_publisher(
            None, _session(robot_node=None)
        )
        is False
    )
