"""Public API for STEP cylindrical hole detection."""

from __future__ import annotations

from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from .find_holes import HoleCandidate, find_cylindrical_holes, load_step

__all__ = ["HoleCandidate", "find_cylindrical_holes", "load_step"]


def __getattr__(name: str) -> Any:
    if name in __all__:
        from . import find_holes

        return getattr(find_holes, name)
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
