"""Matplotlib visualization of the groove and welding trajectory."""

from __future__ import annotations

from pathlib import Path

import numpy as np

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

from .pipeline import WeldSeamResult


def _subsample(points: np.ndarray, limit: int = 4000) -> np.ndarray:
    if len(points) <= limit:
        return points
    idx = np.linspace(0, len(points) - 1, limit, dtype=int)
    return points[idx]


def render_result(result: WeldSeamResult, out_path: str | Path, title: str = "Weld seam trajectory") -> Path:
    out_path = Path(out_path)
    out_path.parent.mkdir(parents=True, exist_ok=True)

    cloud = _subsample(result.cloud)
    groove = result.groove
    traj = result.trajectory

    fig = plt.figure(figsize=(14.0, 9.0), dpi=130)
    fig.suptitle(title, fontsize=14, fontweight="bold")

    ax3d = fig.add_subplot(2, 2, 1, projection="3d")
    ax3d.scatter(cloud[:, 0], cloud[:, 1], cloud[:, 2], s=4, c="#9aa0a6", alpha=0.28, label="point cloud")
    ax3d.scatter(groove[:, 0], groove[:, 1], groove[:, 2], s=12, c="#d62728", label="groove")
    ax3d.plot(traj[:, 0], traj[:, 1], traj[:, 2], color="#2ca02c", linewidth=2.6, label="trajectory")
    ax3d.scatter(traj[0, 0], traj[0, 1], traj[0, 2], s=50, c="#1f77b4", label="start")
    ax3d.scatter(traj[-1, 0], traj[-1, 1], traj[-1, 2], s=50, c="#ff7f0e", label="end")
    ax3d.set_xlabel("X")
    ax3d.set_ylabel("Y")
    ax3d.set_zlabel("Z")
    ax3d.legend(loc="upper left", fontsize=8)
    _readable_3d(ax3d, np.vstack((cloud, traj)))

    ax_xy = fig.add_subplot(2, 2, 2)
    ax_xy.scatter(cloud[:, 0], cloud[:, 1], s=5, c="#c8ccd0")
    ax_xy.scatter(groove[:, 0], groove[:, 1], s=10, c="#d62728")
    ax_xy.plot(traj[:, 0], traj[:, 1], "-o", color="#2ca02c", markersize=3.5, linewidth=2.0)
    ax_xy.set_title("Top view (XY)")
    ax_xy.set_xlabel("X")
    ax_xy.set_ylabel("Y")
    ax_xy.grid(True, alpha=0.25)

    ax_yz = fig.add_subplot(2, 2, 3)
    ax_yz.scatter(cloud[:, 1], cloud[:, 2], s=6, c="#c8ccd0")
    ax_yz.scatter(groove[:, 1], groove[:, 2], s=12, c="#d62728")
    ax_yz.plot(traj[:, 1], traj[:, 2], "o", color="#2ca02c", markersize=4)
    ax_yz.set_title("Groove section (YZ)")
    ax_yz.set_xlabel("Y")
    ax_yz.set_ylabel("Z")
    ax_yz.set_aspect("equal", adjustable="box")
    ax_yz.grid(True, alpha=0.25)

    ax_tbl = fig.add_subplot(2, 2, 4)
    ax_tbl.axis("off")
    preview = result.poses[: min(12, len(result.poses))]
    cell = [[f"{v:.4f}" for v in row[:6]] for row in preview]
    table = ax_tbl.table(
        cellText=cell,
        colLabels=["x", "y", "z", "rx", "ry", "rz"],
        loc="center",
        cellLoc="center",
    )
    table.auto_set_font_size(False)
    table.set_fontsize(8)
    table.scale(1.05, 1.25)
    ax_tbl.set_title(
        f"First {len(preview)} / {len(result.poses)} waypoints   |   "
        f"groove pts={len(result.groove)}   voxel={result.voxel_size:.4g}",
        fontsize=10,
    )

    fig.tight_layout()
    fig.savefig(out_path, bbox_inches="tight")
    plt.close(fig)
    return out_path


def _readable_3d(ax, points: np.ndarray) -> None:
    mins = points.min(axis=0)
    maxs = points.max(axis=0)
    spans = np.maximum(maxs - mins, 1e-6)
    # Stretch the short axes so a long seam is still readable in 3D.
    aspect = np.maximum(spans, spans.max() * 0.22)
    ax.set_box_aspect(aspect.tolist())
