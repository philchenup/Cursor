"""Interactive point picking on a displayed Open3D point cloud.

Open3D ``VisualizerWithEditing`` does **not** pick on a normal left click.
That rotates the view. Selection uses:

* Shift + left click  — pick the nearest rendered vertex (yellow sphere)
* Shift + right click — undo the last pick
* Q or Esc            — close the window, then read ``get_picked_points()``

``get_picked_points()`` returns **indices** into ``pcd.points``, not XYZ.
Index into the same cloud that was added to the visualizer.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np

from .ply_io import load_ply, save_csv, save_ply


PICK_HELP = """
Open3D point picking (window must be focused):
  Shift + left click   pick a point  (console: Picked point #i (x, y, z))
  Shift + right click  undo last pick
  Left drag            rotate
  Ctrl + left drag     pan
  Wheel                zoom
  Q / Esc              finish and return indices
"""


@dataclass
class PickedPoints:
    indices: np.ndarray
    xyz: np.ndarray

    def save(self, out_dir: str | Path) -> dict[str, Path]:
        out_dir = Path(out_dir)
        out_dir.mkdir(parents=True, exist_ok=True)
        csv_path = out_dir / "picked_points.csv"
        ply_path = out_dir / "picked_points.ply"
        save_csv(csv_path, np.column_stack((self.indices.astype(np.float64), self.xyz)), ("index", "x", "y", "z"))
        if len(self.xyz):
            save_ply(ply_path, self.xyz)
        return {"csv": csv_path, "ply": ply_path}


def _require_open3d():
    try:
        import open3d as o3d
    except ImportError as exc:
        raise SystemExit(
            "Open3D is required for interactive picking. Install with:\n"
            "  python3 -m pip install open3d"
        ) from exc
    return o3d


def cloud_from_points(
    points: np.ndarray,
    *,
    estimate_normals: bool = True,
    camera_location: tuple[float, float, float] = (0.0, 0.0, 0.0),
    color: tuple[float, float, float] = (0.8, 0.8, 0.8),
    radius: float = 0.01,
    max_nn: int = 30,
):
    """Build the same Open3D cloud the original demo painted before picking."""
    o3d = _require_open3d()
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(np.asarray(points, dtype=np.float64).reshape(-1, 3))
    if estimate_normals and len(points) >= 3:
        pcd.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=radius, max_nn=max_nn)
        )
        pcd.normalize_normals()
        pcd.orient_normals_towards_camera_location(camera_location=list(camera_location))
    pcd.paint_uniform_color(list(color))
    return pcd


def pick_points_interactive(
    pcd,
    *,
    window_name: str = "Shift+LMB pick points, Q to finish",
    point_size: float = 6.0,
) -> list[int]:
    """Show ``pcd`` and block until the window is closed.

    Returns the picked vertex indices in click order.
    """
    o3d = _require_open3d()
    print(PICK_HELP)
    vis = o3d.visualization.VisualizerWithEditing()
    vis.create_window(window_name=window_name)
    vis.add_geometry(pcd)
    render = vis.get_render_option()
    render.point_size = float(point_size)
    render.background_color = np.array([0.08, 0.08, 0.10])
    vis.run()
    vis.destroy_window()
    return list(vis.get_picked_points())


def picked_xyz(pcd, indices: list[int] | np.ndarray) -> np.ndarray:
    points = np.asarray(pcd.points)
    idx = np.asarray(indices, dtype=int)
    if idx.size == 0:
        return np.zeros((0, 3), dtype=np.float64)
    return points[idx]


def pick_from_ply(
    path: str | Path,
    *,
    voxel_size: float | None = None,
    camera_location: tuple[float, float, float] = (0.0, 0.0, 0.0),
    point_size: float = 6.0,
) -> PickedPoints:
    o3d = _require_open3d()
    points = load_ply(path)
    if voxel_size is not None and voxel_size > 0:
        raw = o3d.geometry.PointCloud()
        raw.points = o3d.utility.Vector3dVector(points)
        raw = raw.voxel_down_sample(voxel_size)
        points = np.asarray(raw.points)
    pcd = cloud_from_points(points, camera_location=camera_location)
    indices = pick_points_interactive(pcd, point_size=point_size)
    xyz = picked_xyz(pcd, indices)
    return PickedPoints(indices=np.asarray(indices, dtype=int), xyz=xyz)
