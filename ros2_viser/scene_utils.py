"""Small helpers for mutating the shared Viser scene safely."""

from __future__ import annotations

import logging
import warnings
from typing import Any

logger = logging.getLogger(__name__)

# Raised by viser when a node (or one of its descendants) is removed twice.
_ALREADY_REMOVED_MESSAGE = ".*already removed node.*"


def safe_remove(handle: Any, *, label: str = "") -> None:
    """Remove a Viser scene node / ``ViserUrdf`` without teardown noise.

    ``viser.extras.ViserUrdf.remove()`` walks *all* joint frames and meshes even
    though removing a parent frame already removed its children; viser then emits
    one ``UserWarning: Attempted to remove already removed node`` per redundant
    mesh. Rebuilding a URDF, destroying a session and relabelling panels all hit
    that path, so the warning is expected and not actionable here.
    """
    if handle is None:
        return

    impl = getattr(handle, "_impl", None)
    if impl is not None and getattr(impl, "removed", False):
        return

    try:
        with warnings.catch_warnings():
            warnings.filterwarnings("ignore", message=_ALREADY_REMOVED_MESSAGE)
            handle.remove()
    except Exception as e:
        logger.debug(f"Failed to remove {label or handle}: {e}")
