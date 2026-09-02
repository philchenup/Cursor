"""Windows runtime helpers for GoTrack.

Upstream GoTrack is written for Linux (EGL + NCCL DDP + fork DataLoader).
On Windows those defaults crash before a pose is produced:

* ``PYOPENGL_PLATFORM=egl`` is Linux-only (pyrender/OpenGL)
* ``strategy: ddp`` needs NCCL, which PyTorch does not ship on Windows
* ``DataLoader(num_workers>0)`` and ``multiprocessing.Pool`` use spawn and
  interact badly with Hydra re-entry
* YAML ``root_dir: D:\\data\\...`` treats backslashes as escapes
"""

from __future__ import annotations

import os
import sys
from pathlib import Path
from typing import Any, Optional


def is_windows() -> bool:
    return sys.platform == "win32"


def configure_opengl_platform() -> None:
    """Select a pyrender backend. Call before ``import pyrender``."""
    if is_windows():
        platform = os.environ.get("PYOPENGL_PLATFORM", "").lower()
        if platform in ("egl", "osmesa"):
            os.environ.pop("PYOPENGL_PLATFORM", None)
        display = os.environ.get("DISPLAY", "")
        if display.startswith(":"):
            os.environ.pop("DISPLAY", None)
        return
    if sys.platform.startswith("linux"):
        os.environ.setdefault("PYOPENGL_PLATFORM", "egl")
        os.environ.setdefault("DISPLAY", os.environ.get("DISPLAY", ":0"))


def dataloader_num_workers(requested: Any) -> int:
    """Windows: always 0. Spawn workers + Hydra often deadlock or re-run main."""
    if is_windows():
        return 0
    try:
        return max(0, int(requested))
    except (TypeError, ValueError):
        return 0


def mp_pool_workers(requested: Any) -> int:
    """Return 0 to mean run inline (do not spawn a ``multiprocessing.Pool``)."""
    if is_windows():
        return 0
    try:
        count = int(requested)
    except (TypeError, ValueError):
        count = 1
    if count <= 1:
        return 0
    return count


def _as_filesystem_path(value: Any) -> Optional[str]:
    if value is None:
        return None
    text = str(value).strip().replace("\\", "/")
    if not text:
        return None
    return str(Path(text))


def normalize_windows_paths(cfg: Any) -> Any:
    """Turn Hydra path strings into real filesystem paths on Windows.

    Unquoted ``\\`` in YAML is an escape. Prefer ``D:/data/gotrack_project``.
    """
    if not is_windows() or cfg is None:
        return cfg

    def _set(node: Any, key: str) -> None:
        try:
            current = node.get(key) if hasattr(node, "get") else None
        except Exception:  # noqa: BLE001
            return
        fixed = _as_filesystem_path(current)
        if fixed is None:
            return
        try:
            node[key] = fixed
        except Exception:  # noqa: BLE001
            try:
                setattr(node, key, fixed)
            except Exception:  # noqa: BLE001
                pass

    _set(cfg, "save_dir")
    user = None
    machine = None
    try:
        user = cfg.get("user")
    except Exception:  # noqa: BLE001
        user = getattr(cfg, "user", None)
    try:
        machine = cfg.get("machine")
    except Exception:  # noqa: BLE001
        machine = getattr(cfg, "machine", None)
    if user is not None:
        _set(user, "root_dir")
    if machine is not None:
        _set(machine, "root_dir")
    return cfg


def hydra_full_error_hint() -> str:
    if is_windows():
        return (
            "To see Hydra's chained ImportError, run:\n"
            "  set HYDRA_FULL_ERROR=1\n"
            "  python -m scripts.inference_gotrack"
        )
    return (
        "To see Hydra's chained ImportError, run:\n"
        "  export HYDRA_FULL_ERROR=1\n"
        "  python -m scripts.inference_gotrack"
    )
