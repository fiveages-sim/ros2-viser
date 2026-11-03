"""A root-level /robot_description must not shadow namespaced robots.

Regression context: the simulator keeps a global ``robot_state_publisher`` on
``/robot_description`` while the driven robot lives at
``/g1d/robot_description``. Both were offered (and loaded), leaving an extra,
unrelated robot in the shared scene.
"""

from __future__ import annotations

from types import SimpleNamespace

import pytest

pytest.importorskip("arms_ros2_control_msgs.msg", reason="needs a sourced ROS workspace")

from ros2_viser.i18n import get_translator, set_global_language  # noqa: E402
from ros2_viser.visualizer import ROS2ViserVisualizer  # noqa: E402

ROOT = "/robot_description"
NAMESPACED = "/g1d/robot_description"


def _visualizer(*, prefer: bool = True, discovered=None, sessions=None):
    viz = object.__new__(ROS2ViserVisualizer)
    set_global_language("en")
    viz.translator = get_translator()
    viz.config = SimpleNamespace(prefer_namespaced_robots=prefer)
    viz._discovered_robot_topics = list(discovered or [])
    viz._robot_sessions = dict(sessions or {})
    viz._client_views = {}
    viz._selected_robot_topic = None
    viz._robot_dropdown = None

    viz.destroyed: list[str] = []
    viz.dropdown_writes: list[tuple[int, str | None]] = []
    viz.refreshed: list[int] = []

    def _destroy(topic):
        viz._robot_sessions.pop(topic, None)
        viz.destroyed.append(topic)

    viz._destroy_session = _destroy
    viz._refresh_client_robot_dropdown = lambda view: viz.refreshed.append(view.client_id)
    viz._set_client_robot_dropdown = lambda view, topic: viz.dropdown_writes.append(
        (view.client_id, topic)
    )
    viz._cleanup_client_control_ui = lambda view: None
    return viz


# --------------------------------------------------------------------- options


def test_picker_drops_root_when_namespaced_robot_exists():
    viz = _visualizer(discovered=[ROOT, NAMESPACED])
    assert viz._robot_picker_topics() == [NAMESPACED]


def test_picker_keeps_root_when_no_namespaced_robot():
    viz = _visualizer(discovered=[ROOT])
    assert viz._robot_picker_topics() == [ROOT]


def test_picker_keeps_root_when_preference_disabled():
    viz = _visualizer(prefer=False, discovered=[ROOT, NAMESPACED])
    assert viz._robot_picker_topics() == [ROOT, NAMESPACED]


def test_picker_keeps_cached_sessions_of_namespaced_robots():
    viz = _visualizer(discovered=[NAMESPACED], sessions={NAMESPACED: object()})
    assert viz._robot_picker_topics() == [NAMESPACED]


def test_default_workspace_label_maps_to_nothing_when_not_offered():
    viz = _visualizer(discovered=[ROOT, NAMESPACED])
    set_global_language("en")
    label = viz.translator("default_workspace")
    assert viz._topic_from_dropdown_value(label) is None


def test_default_workspace_label_maps_to_root_when_offered():
    viz = _visualizer(discovered=[ROOT])
    label = viz.translator("default_workspace")
    assert viz._topic_from_dropdown_value(label) == ROOT


# ---------------------------------------------------------------------- prune


def test_prune_removes_cached_root_robot():
    viz = _visualizer(
        discovered=[ROOT, NAMESPACED],
        sessions={ROOT: object(), NAMESPACED: object()},
    )
    viz._selected_robot_topic = ROOT

    viz._prune_root_robot_sessions()

    assert viz.destroyed == [ROOT]
    assert list(viz._robot_sessions) == [NAMESPACED]
    assert viz._selected_robot_topic is None


def test_prune_keeps_root_when_no_namespaced_robot():
    viz = _visualizer(discovered=[ROOT], sessions={ROOT: object()})
    viz._prune_root_robot_sessions()
    assert viz.destroyed == []


def test_prune_respects_the_opt_out_flag():
    viz = _visualizer(prefer=False, discovered=[ROOT, NAMESPACED], sessions={ROOT: object()})
    viz._prune_root_robot_sessions()
    assert viz.destroyed == []


def test_prune_clears_the_bound_client_view():
    view = SimpleNamespace(
        client_id=1,
        selected_robot_topic=ROOT,
        switching=False,
    )
    viz = _visualizer(discovered=[ROOT, NAMESPACED], sessions={ROOT: object()})
    viz._client_views[1] = view

    viz._prune_root_robot_sessions()

    assert view.selected_robot_topic is None
    assert viz.refreshed == [1]


def test_prune_skips_views_that_are_switching():
    view = SimpleNamespace(
        client_id=1,
        selected_robot_topic=ROOT,
        switching=True,
    )
    viz = _visualizer(discovered=[ROOT, NAMESPACED], sessions={ROOT: object()})
    viz._client_views[1] = view

    viz._prune_root_robot_sessions()

    assert view.selected_robot_topic == ROOT
    assert viz.refreshed == []
