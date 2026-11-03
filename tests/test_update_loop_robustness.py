"""The update loop must survive a stuck switch or a stale-robot cleanup error.

Regression context: a session that raised inside the loop (or a ``_switching_robot``
flag left set by a worker that never started) silently froze panel updates and
prevented the stale-robot cleanup from ever running, so leftovers were never
released.
"""

from __future__ import annotations

import time
from types import SimpleNamespace
from unittest import mock

import pytest

pytest.importorskip("arms_ros2_control_msgs.msg", reason="needs a sourced ROS workspace")

from ros2_viser import visualizer as viz_module  # noqa: E402
from ros2_viser.visualizer import ROS2ViserVisualizer  # noqa: E402


def _visualizer():
    viz = object.__new__(ROS2ViserVisualizer)
    viz.config = SimpleNamespace(prune_stale_robots_after=10.0)
    viz._robot_sessions = {}
    viz._client_views = {}
    viz._switching_robot = False
    viz._switching_since = 0.0
    return viz


# ------------------------------------------------------- stuck-switch watchdog


def test_recent_switch_is_not_cleared():
    viz = _visualizer()
    viz._switching_robot = True
    viz._switching_since = time.monotonic()

    viz._recover_stuck_switch()

    assert viz._switching_robot is True


def test_stuck_switch_is_cleared_after_timeout():
    viz = _visualizer()
    viz._switching_robot = True
    viz._switching_since = time.monotonic() - 3600.0
    view = SimpleNamespace(client_id=1, switching=True)
    viz._client_views[1] = view

    with mock.patch.object(viz_module.logger, "error") as error:
        viz._recover_stuck_switch()

    assert viz._switching_robot is False
    assert viz._switching_since == 0.0
    assert view.switching is False
    assert error.call_count == 1


def test_idle_switch_is_a_noop():
    viz = _visualizer()
    viz._recover_stuck_switch()
    assert viz._switching_robot is False


# ------------------------------------------------ cleanup must not abort others


def test_cleanup_error_on_one_session_keeps_checking_the_rest():
    good = SimpleNamespace(
        robot_description_topic="/ok/robot_description",
        last_data_ts=0.0,
        created_ts=0.0,
        stale_hidden=False,
        robot_base_frame=SimpleNamespace(visible=True),
        interface=SimpleNamespace(robot_node=object()),
    )
    bad = SimpleNamespace(
        robot_description_topic="/bad/robot_description",
        last_data_ts=0.0,
        created_ts=0.0,
        stale_hidden=False,
        robot_base_frame=SimpleNamespace(visible=True),
        interface=SimpleNamespace(robot_node=object()),
    )

    viz = object.__new__(ROS2ViserVisualizer)
    viz.config = SimpleNamespace(prune_stale_robots_after=10.0)
    viz._client_views = {}
    viz._selected_robot_topic = None
    viz._next_stale_check = 0.0
    viz._robot_dropdown = None
    viz._discovered_robot_topics = ["/bad/robot_description", "/ok/robot_description"]
    viz._robot_sessions = {
        "/bad/robot_description": bad,
        "/ok/robot_description": good,
    }
    viz.destroyed: list[str] = []

    def _destroy(topic):
        viz._robot_sessions.pop(topic, None)
        viz.destroyed.append(topic)
        if topic == "/bad/robot_description":
            raise RuntimeError("boom")

    viz._destroy_session = _destroy
    viz._cleanup_client_control_ui = lambda view: None
    viz._set_client_robot_dropdown = lambda view, topic: None
    viz._refresh_client_robot_dropdown = lambda view: None
    viz._session_has_publisher = lambda session: False

    with mock.patch.object(viz_module.logger, "error"):
        viz._maybe_prune_stale_sessions()

    assert viz.destroyed == ["/bad/robot_description", "/ok/robot_description"]
