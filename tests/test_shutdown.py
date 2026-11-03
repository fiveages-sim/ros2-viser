"""Shutdown must release resources and stay quiet on Ctrl+C.

Regression context: stopping the simulation / pressing Ctrl+C printed
"Exception in thread Thread-N (spin)" and left the rclpy context and the viser
server running, because teardown destroyed entities while the executor was still
iterating its wait set.
"""

from __future__ import annotations

import pytest

pytest.importorskip("arms_ros2_control_msgs.msg", reason="needs a sourced ROS workspace")

from ros2_robot_interface import ros_interface as ri  # noqa: E402
from ros2_robot_interface.ros_interface import ROS2RobotInterface  # noqa: E402
from ros2_robot_interface.utils import rclpy_context  # noqa: E402
from ros2_viser import visualizer as viz_module  # noqa: E402
from ros2_viser.visualizer import ROS2ViserVisualizer  # noqa: E402


# --------------------------------------------------------------------- spin loop


def test_spin_executor_swallows_teardown_exception():
    iface = object.__new__(ROS2RobotInterface)
    iface._disconnecting = True

    class _BoomExecutor:
        def spin(self):
            raise RuntimeError("generator raised StopIteration")

    iface.executor = _BoomExecutor()
    ROS2RobotInterface._spin_executor(iface)  # must not raise


def test_spin_executor_swallows_after_executor_cleared():
    iface = object.__new__(ROS2RobotInterface)
    iface._disconnecting = False
    iface.executor = None

    class _BoomExecutor:
        def spin(self):
            raise RuntimeError("InvalidHandle")

    executor = _BoomExecutor()
    iface.executor = executor
    ROS2RobotInterface._spin_executor(iface)  # must not raise


def test_spin_executor_returns_without_executor():
    iface = object.__new__(ROS2RobotInterface)
    iface._disconnecting = False
    iface.executor = None
    ROS2RobotInterface._spin_executor(iface)


# ------------------------------------------------------------------- disconnect


def test_disconnect_is_reentrant_safe():
    iface = object.__new__(ROS2RobotInterface)
    iface._disconnecting = True
    iface.disconnect()  # returns immediately instead of touching a half-torn state
    assert iface._disconnecting is True


def test_disconnect_stops_spin_thread_before_destroying_entities():
    log: list = []

    class _Executor:
        def remove_node(self, node):
            log.append("remove_node")

        def shutdown(self):
            log.append("shutdown")

    class _Thread:
        def join(self, timeout=None):
            log.append(("join", timeout))

        def is_alive(self):
            return False

    class _Sub:
        def destroy(self):
            log.append("sub.destroy")

    class _Iface(ROS2RobotInterface):
        def __init__(self):  # bypass the real constructor
            pass

        def __getattr__(self, name):
            if name.startswith("__"):
                raise AttributeError(name)
            return None  # every unset entity reads as None

    iface = _Iface()
    iface._disconnecting = False
    iface.executor = _Executor()
    iface.executor_thread = _Thread()
    iface.robot_node = object()
    iface.joint_state_sub = _Sub()

    iface._disconnect_resources()

    assert log == ["remove_node", "shutdown", ("join", 2.0), "sub.destroy"]
    assert iface.executor is None
    assert iface.executor_thread is None
    assert iface.robot_node is None
    assert iface.joint_state_sub is None


# ------------------------------------------------------------- rclpy ownership


def test_shutdown_rclpy_is_noop_when_caller_owns_context(monkeypatch):
    monkeypatch.setattr(rclpy_context, "_OWNED", False)
    calls: list[int] = []
    monkeypatch.setattr(rclpy_context.rclpy, "try_shutdown", lambda: calls.append(1))

    ri.shutdown_rclpy_if_owned()

    assert calls == []


def test_shutdown_rclpy_releases_owned_context(monkeypatch):
    monkeypatch.setattr(rclpy_context, "_OWNED", True)
    calls: list[int] = []
    monkeypatch.setattr(rclpy_context.rclpy, "try_shutdown", lambda: calls.append(1))

    ri.shutdown_rclpy_if_owned()

    assert calls == [1]
    assert rclpy_context.context_is_owned() is False


# ------------------------------------------------------------ visualizer.stop()


class _FakeServer:
    def __init__(self):
        self.stopped = False

    def stop(self):
        self.stopped = True


def _bare_visualizer() -> ROS2ViserVisualizer:
    viz = object.__new__(ROS2ViserVisualizer)
    viz._running = True
    viz._robot_sessions = {}
    viz._client_views = {}
    viz._update_thread = None
    viz._selected_robot_topic = None
    viz._display_folder_handle = None
    viz._marker_manager = None
    viz._fsm_panel = None
    viz._gripper_panel = None
    viz._joint_panel = None
    viz._hardware_panel = None
    viz._controller_panel = None
    viz.server = _FakeServer()
    return viz


def test_stop_releases_ros_context_after_sessions(monkeypatch):
    order: list[str] = []
    viz = _bare_visualizer()
    viz._destroy_all_sessions = lambda: order.append("destroy_sessions")
    monkeypatch.setattr(
        viz_module, "shutdown_rclpy_if_owned", lambda: order.append("rclpy_shutdown")
    )

    viz.stop()

    assert order == ["destroy_sessions", "rclpy_shutdown"]
    assert viz._running is False


def test_stop_closes_viser_server(monkeypatch):
    viz = _bare_visualizer()
    viz._destroy_all_sessions = lambda: None
    monkeypatch.setattr(viz_module, "shutdown_rclpy_if_owned", lambda: None)
    server = viz.server

    viz.stop()

    assert server.stopped is True
    assert viz.server is None


def test_stop_is_idempotent(monkeypatch):
    viz = _bare_visualizer()
    viz._destroy_all_sessions = lambda: None
    monkeypatch.setattr(viz_module, "shutdown_rclpy_if_owned", lambda: None)

    viz.stop()
    viz.stop()  # already stopped: must not raise or re-run teardown

    assert viz._running is False
    assert viz.server is None
