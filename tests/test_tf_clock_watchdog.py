"""Simulator restarts reset sim time; the TF buffer must be rebuilt.

Regression context: after the simulation was stopped/restarted, tf2 rejected every
transform as "from the past" and spammed ``TF_OLD_DATA`` because the buffer still
cached the pre-restart timestamps.
"""

from __future__ import annotations

from types import SimpleNamespace

import pytest

pytest.importorskip("arms_ros2_control_msgs.msg", reason="needs a sourced ROS workspace")

from ros2_robot_interface import ros_interface as ri  # noqa: E402
from ros2_robot_interface.ros_interface import ROS2RobotInterface  # noqa: E402


def _msg(sec: float):
    whole = int(sec)
    return SimpleNamespace(
        header=SimpleNamespace(
            stamp=SimpleNamespace(sec=whole, nanosec=int((sec - whole) * 1e9))
        )
    )


def _iface() -> ROS2RobotInterface:
    iface = _bare_iface()
    resets: list[int] = []
    iface._reset_tf_buffer = lambda: resets.append(1)  # type: ignore[method-assign]
    iface.resets = resets  # type: ignore[attr-defined]
    return iface


def _bare_iface() -> ROS2RobotInterface:
    """Real methods (no stubbed ``_reset_tf_buffer``)."""
    iface = object.__new__(ROS2RobotInterface)
    iface.robot_node = object()
    iface.tf_buffer = None
    iface.tf_listener = None
    iface._last_header_stamp = None
    iface._tf_reset_count = 0
    return iface


# --------------------------------------------------------------------- watchdog


def test_first_stamp_only_seeds_the_baseline():
    iface = _iface()
    iface._watch_clock_regression(_msg(17.0))
    assert iface.resets == []
    assert iface._last_header_stamp == pytest.approx(17.0)


def test_forward_time_does_not_reset():
    iface = _iface()
    iface._watch_clock_regression(_msg(17.0))
    iface._watch_clock_regression(_msg(42.5))
    assert iface.resets == []


def test_small_backward_jitter_does_not_reset():
    iface = _iface()
    iface._watch_clock_regression(_msg(20.0))
    iface._watch_clock_regression(_msg(19.5))  # 0.5s < threshold
    assert iface.resets == []


def test_simulator_restart_rebuilds_the_buffer():
    iface = _iface()
    iface._watch_clock_regression(_msg(180.0))
    iface._watch_clock_regression(_msg(0.5))  # sim time restarted
    assert iface.resets == [1]
    assert iface._last_header_stamp == pytest.approx(0.5)


def test_zero_stamp_is_ignored():
    iface = _iface()
    iface._watch_clock_regression(_msg(180.0))
    iface._watch_clock_regression(_msg(0.0))
    assert iface.resets == []
    assert iface._last_header_stamp == pytest.approx(180.0)


def test_missing_header_is_ignored():
    iface = _iface()
    iface._watch_clock_regression(SimpleNamespace())  # no header at all
    assert iface.resets == []


def test_tf_buffer_regression_triggers_reset_even_without_stamped_messages():
    """TF may use a different clock source than joint_states."""
    iface = _iface()
    iface.tf_buffer = SimpleNamespace(clock_regressed=True)

    iface._watch_clock_regression(SimpleNamespace())  # message carries no stamp

    assert iface.resets == [1]


# ----------------------------------------------------- buffer stamp tracking


def _stamp(sec: float):
    whole = int(sec)
    return SimpleNamespace(sec=whole, nanosec=int((sec - whole) * 1e9))


def test_stamp_tracking_keeps_the_newest_value():
    newest = 0.0
    for sec in (10.0, 11.0, 10.5):
        newest, regressed = ri.note_stamp_regression(newest, _stamp(sec))
        assert regressed is False
    assert newest == pytest.approx(11.0)


def test_stamp_tracking_flags_sim_restart():
    newest = 180.0
    newest, regressed = ri.note_stamp_regression(newest, _stamp(0.5))
    assert regressed is True
    assert newest == pytest.approx(180.0)


def test_stamp_tracking_ignores_jitter_and_empty_stamps():
    newest = 20.0
    assert ri.note_stamp_regression(newest, _stamp(19.5)) == (20.0, False)
    assert ri.note_stamp_regression(newest, _stamp(0.0)) == (20.0, False)
    assert ri.note_stamp_regression(newest, None) == (20.0, False)


# ------------------------------------------------------------------ tf rebuild


class _FakeJumpHandle:
    def __init__(self, log):
        self._log = log

    def unregister(self):
        self._log.append("jump.unregister")


class _FakeSub:
    def __init__(self, log, name):
        self._log = log
        self._name = name

    def destroy(self):
        self._log.append(f"{self._name}.destroy")


def test_reset_releases_old_subscriptions_and_rebuilds(monkeypatch):
    log: list[str] = []
    old_listener = SimpleNamespace(
        tf_sub=_FakeSub(log, "tf_sub"),
        tf_static_sub=_FakeSub(log, "tf_static_sub"),
    )
    old_buffer = SimpleNamespace(jump_handle=_FakeJumpHandle(log))

    created: dict = {}

    class _Buffer:
        def __init__(self, *args, on_transform=None, **kwargs):
            self.jump_handle = None
            self.on_transform = on_transform
            created["buffer"] = self

    class _Listener:
        def __init__(self, buffer, node):
            created["listener"] = self
            created["buffer_arg"] = buffer
            created["node"] = node

    monkeypatch.setattr(ri, "_ClockWatchTfBuffer", _Buffer)
    monkeypatch.setattr(ri.tf2_ros, "TransformListener", _Listener)

    iface = _bare_iface()
    iface.tf_listener = old_listener
    iface.tf_buffer = old_buffer

    iface._reset_tf_buffer()

    assert log == ["tf_sub.destroy", "tf_static_sub.destroy", "jump.unregister"]
    assert created["buffer_arg"] is created["buffer"] is iface.tf_buffer
    assert created["listener"] is iface.tf_listener
    assert created["node"] is iface.robot_node
    assert iface._tf_reset_count == 1
    assert old_listener.tf_sub is None
    assert old_listener.tf_static_sub is None


def test_reset_is_noop_without_a_node():
    iface = _bare_iface()
    iface.robot_node = None
    iface.tf_buffer = object()
    iface.tf_listener = object()

    iface._reset_tf_buffer()

    assert iface._tf_reset_count == 0


def test_release_tf_listener_is_idempotent():
    iface = object.__new__(ROS2RobotInterface)
    iface.tf_buffer = None
    iface.tf_listener = None

    iface._release_tf_listener()
    iface._release_tf_listener()

    assert iface.tf_buffer is None
    assert iface.tf_listener is None


def test_release_tf_listener_survives_destroy_errors():
    class _BoomSub:
        def destroy(self):
            raise RuntimeError("already invalid")

    iface = object.__new__(ROS2RobotInterface)
    iface.tf_buffer = None
    iface.tf_listener = SimpleNamespace(tf_sub=_BoomSub(), tf_static_sub=_BoomSub())

    iface._release_tf_listener()  # must not raise

    assert iface.tf_listener is None
