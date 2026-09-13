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

from .geometry import points_in_cylinder
from .ply_io import load_ply, save_csv, save_ply

ROI_PICK_HELP = """
This script expects exactly 3 picks, in this order:
  1) START   of the weld / first segment
  2) CORNER  (the kink between the two segments)
  3) END     of the weld / second segment

How to pick (window must be focused):
  Shift + left click    select a vertex (yellow sphere, console prints #i)
  Shift + right click   undo last pick
  Left drag             rotate view  (this does NOT pick)
  Q                     close window after the 3rd point
"""


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
    print_help: bool = True,
) -> list[int]:
    """Show ``pcd`` and block until the window is closed.

    Returns the picked vertex indices in click order.
    """
    o3d = _require_open3d()
    if print_help:
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


def make_line_set(
    points: np.ndarray,
    edges: list[list[int]] | np.ndarray,
    color: tuple[float, float, float] = (1.0, 0.0, 0.0),
):
    """Build an Open3D LineSet.

    ``points`` are XYZ. ``edges`` are integer index pairs such as ``[[0, 1], [1, 2]]``.
    Do not pass edges to ``Vector3dVector`` — that type is only for 3D floats and
    raises RuntimeError: Unable to cast Python instance of type list to C++ type.
    """
    o3d = _require_open3d()
    pts = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    idx = np.asarray(edges, dtype=np.int32).reshape(-1, 2)
    line = o3d.geometry.LineSet()
    line.points = o3d.utility.Vector3dVector(pts)
    line.lines = o3d.utility.Vector2iVector(idx)
    line.colors = o3d.utility.Vector3dVector(np.tile(np.asarray(color, dtype=np.float64), (len(idx), 1)))
    return line


def picked_xyz(pcd, indices: list[int] | np.ndarray) -> np.ndarray:
    points = np.asarray(pcd.points)
    idx = np.asarray(indices, dtype=int)
    if idx.size == 0:
        return np.zeros((0, 3), dtype=np.float64)
    return points[idx]


def three_point_cylinder_roi(
    cloud_points: np.ndarray,
    start: np.ndarray,
    corner: np.ndarray,
    end: np.ndarray,
    radius: float,
    corner_extend: float = 0.11,
) -> np.ndarray:
    """Jeffery ``detect_groove_workflow``: two cylinders start→corner and end→corner.

    The corner is pushed slightly past the true pick so the two tubes overlap
    at the kink (``corner + (corner - other) * 0.11``).
    """
    pts = np.asarray(cloud_points, dtype=np.float64).reshape(-1, 3)
    start = np.asarray(start, dtype=np.float64).reshape(3)
    corner = np.asarray(corner, dtype=np.float64).reshape(3)
    end = np.asarray(end, dtype=np.float64).reshape(3)
    corner1 = corner + (corner - start) * corner_extend
    corner2 = corner + (corner - end) * corner_extend
    roi1 = points_in_cylinder(start, corner1, radius, pts)
    roi2 = points_in_cylinder(end, corner2, radius, pts)
    if len(roi1) == 0:
        return roi2
    if len(roi2) == 0:
        return roi1
    return np.vstack((roi1, roi2))


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


def pick_three_point_roi_from_ply(
    path: str | Path,
    *,
    voxel_size: float = 0.001,
    radius_scale: float = 18.0,
    camera_location: tuple[float, float, float] = (0.0, 0.0, 0.0),
    point_size: float = 6.0,
    show_preview: bool = True,
) -> tuple[PickedPoints, np.ndarray]:
    """Interactive port of Jeffery ``detect_groove_workflow`` picking + cylinder ROI."""
    o3d = _require_open3d()
    pcd = cloud_from_points(load_ply(path), camera_location=camera_location)
    print(ROI_PICK_HELP)
    indices = pick_points_interactive(
        pcd,
        window_name="Pick 3 points: START, CORNER, END  then Q",
        point_size=point_size,
        print_help=False,
    )
    if len(indices) != 3:
        raise SystemExit(
            f"This workflow needs exactly 3 points (start, corner, end), got {len(indices)}. "
            "Shift+left click three vertices in that order, then press Q."
        )
    xyz = picked_xyz(pcd, indices)
    start, corner, end = xyz
    radius = voxel_size * radius_scale
    roi = three_point_cylinder_roi(np.asarray(pcd.points), start, corner, end, radius)
    if show_preview:
        roi_pcd = o3d.geometry.PointCloud()
        roi_pcd.points = o3d.utility.Vector3dVector(roi)
        roi_pcd.paint_uniform_color([1.0, 0.5, 0.5])
        overlay = o3d.geometry.PointCloud(pcd)
        overlay.paint_uniform_color([0.8, 0.8, 0.8])
        line = make_line_set(xyz, [[0, 1], [1, 2]], color=(1.0, 0.0, 0.0))
        o3d.visualization.draw_geometries([overlay, roi_pcd, line], window_name="ROI + polyline")
    return PickedPoints(indices=np.asarray(indices, dtype=int), xyz=xyz), roi
