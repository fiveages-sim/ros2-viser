"""Robot-switch bookkeeping: lock ownership, queued picks, empty-workspace guard.

Regression context: ``_switch_robot`` used to clear ``_switching_robot`` outside
``_robot_switch_lock`` and before re-enabling the picker, so a click landing in
that window could claim the flag and start a second switch thread (double bind /
panel flicker). The finalization block now runs under the lock, and a pick that
lost to the finished switch is re-asserted on the picker instead of being left
dangling.
"""

from __future__ import annotations

import threading
import time
from types import SimpleNamespace

import pytest

pytest.importorskip("arms_ros2_control_msgs.msg", reason="needs a sourced ROS workspace")

from ros2_viser.i18n import get_translator, set_global_language  # noqa: E402
from ros2_viser.visualizer import (  # noqa: E402
    _ROBOT_DROPDOWN_SNAP_GUARD_S,
    ClientView,
    ROS2ViserVisualizer,
)

ROBOT_A = "/g1d/robot_description"
ROBOT_B = "/g2/robot_description"


def _dropdown() -> SimpleNamespace:
    return SimpleNamespace(options=[""], value="", disabled=False)


def _view(topic: str | None = None, *, bound: bool = False) -> ClientView:
    view = ClientView(client_id=1, client=object())
    view.robot_dropdown = _dropdown()
    view.refresh_robots_button = SimpleNamespace(disabled=False)
    view.selected_robot_topic = topic
    view.controls_bound = bound
    return view


def _make_visualizer() -> ROS2ViserVisualizer:
    viz = object.__new__(ROS2ViserVisualizer)
    viz.config = SimpleNamespace()
    set_global_language("en")
    viz.translator = get_translator()
    viz._robot_switch_lock = threading.Lock()
    viz._switching_robot = False
    viz._client_views = {}
    viz._robot_sessions = {}
    viz._discovered_robot_topics = [ROBOT_A, ROBOT_B]
    viz._active_session_key = None
    viz._selected_robot_topic = None
    viz._robot_dropdown = None
    viz._skip_reconnect_detection = False

    viz._activations: list[str] = []
    viz._created: list[str] = []
    viz._requested: list[str | None] = []
    viz._dropdown_writes: list[str | None] = []

    viz._create_session = lambda topic: viz._created.append(topic) or object()

    def _activate(topic, client_view=None):
        viz._activations.append(topic)
        # Mirrors the real ``_activate_session`` bookkeeping for one browser.
        if client_view is not None:
            client_view.selected_robot_topic = topic
            client_view.controls_bound = True

    viz._activate_session = _activate
    viz._ensure_scene_environment = lambda: None
    viz._set_robot_picker_enabled = lambda enabled: None
    viz._set_client_robot_dropdown = lambda view, topic: viz._dropdown_writes.append(
        topic
    )
    viz._request_robot_switch = lambda topic, client_view=None: viz._requested.append(
        topic
    )
    return viz


# --------------------------------------------------------------- claim / queue


def test_request_switch_claims_flag_and_queues_when_busy(monkeypatch):
    started: list[object] = []

    class FakeThread:
        def __init__(self, target=None, kwargs=None, **rest):
            self.target = target
            self.kwargs = kwargs or {}
            started.append(self)

        def start(self):
            return None

    monkeypatch.setattr(
        "ros2_viser.visualizer.threading.Thread", FakeThread, raising=True
    )

    viz = _make_visualizer()
    view = _view()

    # Instance attribute is stubbed by ``_make_visualizer``; exercise the real one.
    request = ROS2ViserVisualizer._request_robot_switch
    request(viz, ROBOT_A, client_view=view)
    assert viz._switching_robot is True
    assert view.switching is True
    assert len(started) == 1

    request(viz, ROBOT_B, client_view=view)
    assert len(started) == 1, "busy switch must not spawn a second thread"
    assert view.has_queued_robot is True
    assert view.queued_robot_topic == ROBOT_B


# ------------------------------------------------------------- finalization


def test_switch_finally_releases_lock_and_redispatches_queue():
    viz = _make_visualizer()
    view = _view()
    view.switching = True
    view.has_queued_robot = True
    view.queued_robot_topic = ROBOT_B

    viz._switch_robot(ROBOT_A, client_view=view, switch_claimed=True)

    assert viz._switching_robot is False
    assert view.switching is False
    assert view.robot_dropdown.disabled is False
    assert view.selected_robot_topic == ROBOT_A
    assert view.has_queued_robot is False
    assert view.queued_robot_topic is None
    assert viz._requested == [ROBOT_B]


def test_switch_does_not_redispatch_the_topic_it_just_bound():
    viz = _make_visualizer()
    view = _view()
    view.has_queued_robot = True
    view.queued_robot_topic = ROBOT_A

    viz._switch_robot(ROBOT_A, client_view=view, switch_claimed=True)

    assert viz._requested == []
    # The picker is re-asserted so it cannot keep showing an unbound label.
    assert viz._dropdown_writes[-1] == ROBOT_A


def test_bound_picker_drops_the_empty_workspace_entry():
    """Reported bug: after loading, the picker still offered "empty workspace" and
    a browser echo of options[0] unloaded the robot again."""
    viz = _make_visualizer()
    empty_label = viz.translator("empty_workspace")
    view = _view()

    assert viz._client_robot_dropdown_options(view)[0] == empty_label

    viz._switch_robot(ROBOT_A, client_view=view, switch_claimed=True)

    options = viz._client_robot_dropdown_options(view)
    assert empty_label not in options
    assert "/g1d" in options


def test_clients_connected_before_load_finishes_are_bound():
    """Reported bug: panels only appear after refreshing the page.

    A browser can connect while ``start()`` is still loading the auto-detected
    robot, so it never receives panels; the global switch must adopt it.
    """
    viz = _make_visualizer()
    view = _view()
    viz._client_views[view.client_id] = view

    viz._switch_robot(ROBOT_A, switch_claimed=True)  # global (no client_view)

    assert view.selected_robot_topic == ROBOT_A
    assert view.controls_bound is True
    assert viz._dropdown_writes[-1] == ROBOT_A


def test_waiting_client_with_matching_queue_is_bound_not_dropped():
    viz = _make_visualizer()
    view = _view()
    view.has_queued_robot = True
    view.queued_robot_topic = ROBOT_A
    viz._client_views[view.client_id] = view

    viz._switch_robot(ROBOT_A, switch_claimed=True)

    assert view.controls_bound is True
    assert view.has_queued_robot is False
    assert view.queued_robot_topic is None


def test_failed_switch_clears_selection_and_picker():
    viz = _make_visualizer()

    def boom(topic, client_view=None):
        raise RuntimeError("bind failed")

    viz._activate_session = boom
    viz._create_session = lambda topic: object()  # pretend the session exists
    view = _view(topic=ROBOT_A)

    viz._switch_robot(ROBOT_A, client_view=view, switch_claimed=True)

    assert viz._switching_robot is False
    assert viz._requested == []
    assert viz._dropdown_writes[-1] is None


# ------------------------------------------------------- empty-workspace picks


def test_empty_pick_while_bound_is_ignored_and_restored():
    viz = _make_visualizer()
    view = _view(topic=ROBOT_A, bound=True)

    viz._on_client_robot_dropdown_changed(view, "")

    assert viz._requested == []
    assert viz._dropdown_writes[-1] == ROBOT_A
    assert view.restoring_dropdown is False


def test_empty_pick_while_bound_is_ignored_regardless_of_timing():
    """The empty entry is not offered while bound, so this is always an echo."""
    viz = _make_visualizer()
    view = _view(topic=ROBOT_A, bound=True)
    viz._switching_robot = False

    viz._on_client_robot_dropdown_changed(view, "")

    assert viz._requested == []
    assert view.selected_robot_topic == ROBOT_A


def test_empty_pick_when_nothing_bound_requests_unload():
    viz = _make_visualizer()
    view = _view(topic=None, bound=False)

    viz._on_client_robot_dropdown_changed(view, "")

    assert viz._requested == [None]


def test_selection_while_switching_is_queued_not_dropped():
    viz = _make_visualizer()
    viz._switching_robot = True
    view = _view()
    view.switching = True

    viz._on_client_robot_dropdown_changed(view, ROBOT_B)

    assert viz._requested == []
    assert view.has_queued_robot is True
    assert view.queued_robot_topic == ROBOT_B


def test_reselecting_the_bound_robot_is_a_no_op():
    viz = _make_visualizer()
    view = _view(topic=ROBOT_A, bound=True)

    viz._on_client_robot_dropdown_changed(view, ROBOT_A)

    assert viz._requested == []


# ------------------------------------------- picker writes arm the snap guard


def test_picker_write_arms_the_snap_guard():
    """An echo of options[0] must be filtered for a while after our own write."""
    viz = _make_visualizer()
    view = _view()

    before = time.monotonic()
    ROS2ViserVisualizer._set_client_robot_dropdown(viz, view, ROBOT_A)

    assert view.robot_dropdown.value == "/g1d"
    assert view.last_dropdown_write_ts >= before
    assert view.snap_guard_label == viz._client_robot_dropdown_options(view)[0]
    assert view.snap_guard_until >= before + _ROBOT_DROPDOWN_SNAP_GUARD_S - 0.5


def test_picker_write_to_empty_does_not_arm_the_snap_guard():
    """Resetting to the empty entry must not arm a guard."""
    viz = _make_visualizer()
    empty_label = viz.translator("empty_workspace")
    view = _view(topic=None)
    view.robot_dropdown.options = [empty_label, "/g1d"]
    view.robot_dropdown.value = "/g1d"

    ROS2ViserVisualizer._set_client_robot_dropdown(viz, view, None)

    assert view.robot_dropdown.value == empty_label
    assert view.snap_guard_until == 0.0


def test_snap_guard_ignores_first_option_echo():
    viz = _make_visualizer()
    view = _view(topic=ROBOT_A, bound=True)
    # First option label for this browser (robot A), echoed back by the browser.
    view.snap_guard_label = "/g1d"
    view.snap_guard_until = time.monotonic() + 2.0

    viz._on_client_robot_dropdown_changed(view, "/g1d")

    assert viz._requested == []
    assert view.selected_robot_topic == ROBOT_A
