"""``safe_remove`` keeps Viser teardown quiet about redundant removals."""

from __future__ import annotations

import warnings
from types import SimpleNamespace

from ros2_viser.scene_utils import safe_remove


class _FakeNode:
    def __init__(self, *, removed: bool = False, warns: bool = False):
        self._impl = SimpleNamespace(removed=removed)
        self.calls = 0
        self._warns = warns

    def remove(self):
        self.calls += 1
        if self._warns:
            warnings.warn(
                f"Attempted to remove already removed node: {self!r}",
                UserWarning,
            )


def test_removes_once():
    node = _FakeNode()
    safe_remove(node)
    assert node.calls == 1


def test_skips_already_removed_node():
    node = _FakeNode(removed=True)
    safe_remove(node)
    assert node.calls == 0


def test_suppresses_already_removed_warning():
    node = _FakeNode(warns=True)
    with warnings.catch_warnings(record=True) as caught:
        warnings.simplefilter("always")
        safe_remove(node)
    assert node.calls == 1
    assert not [w for w in caught if "already removed node" in str(w.message)]


def test_other_warnings_still_propagate():
    class _NoisyNode:
        def remove(self):
            warnings.warn("something else", UserWarning)

    with warnings.catch_warnings(record=True) as caught:
        warnings.simplefilter("always")
        safe_remove(_NoisyNode())
    assert [w for w in caught if "something else" in str(w.message)]


def test_none_is_a_noop():
    safe_remove(None)


def test_remove_errors_are_swallowed():
    class _Boom:
        def remove(self):
            raise RuntimeError("node gone")

    safe_remove(_Boom())  # must not raise


def test_viser_urdf_style_double_removal_is_quiet():
    """ViserUrdf.remove() re-removes children already dropped by their parent."""

    class _UrdfLike:
        def __init__(self):
            self.nodes = [_FakeNode(), _FakeNode(removed=True)]

        def remove(self):
            for node in self.nodes:
                if node._impl.removed:
                    warnings.warn("Attempted to remove already removed node: x")
                else:
                    node.remove()
                    node._impl.removed = True

    handle = _UrdfLike()
    with warnings.catch_warnings(record=True) as caught:
        warnings.simplefilter("always")
        safe_remove(handle)
    assert not [w for w in caught if "already removed node" in str(w.message)]
