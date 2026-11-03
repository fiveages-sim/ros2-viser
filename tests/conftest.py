"""Test bootstrap for ``ros2-viser``.

The sibling ``ros2_robot_interface`` package lives in the same monorepo but is not
always pip-installed into the interpreter running the tests, so add its outer
directory to ``sys.path`` before the modules under test import it.

Tests that need the custom ROS message packages use ``pytest.importorskip`` so a
plain (non-sourced) Python environment skips them instead of failing.
"""

from __future__ import annotations

import sys
from pathlib import Path

_SIBLING_INTERFACE_DIR = Path(__file__).resolve().parents[2] / "ros2_robot_interface"

if _SIBLING_INTERFACE_DIR.is_dir():
    _path = str(_SIBLING_INTERFACE_DIR)
    if _path not in sys.path:
        sys.path.insert(0, _path)
