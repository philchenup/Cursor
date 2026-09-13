"""End-to-end groove finding and trajectory export, without ROS."""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from .geometry import (
    estimate_normals,
    finite_points,
    infer_length_scale,
    remove_radius_outlier,
    remove_statistical_outlier,
    voxel_downsample,
)
from .groove import asymmetry_feature, cluster_groove, select_high_feature_points
from .multilayer import multilayer_passes, offset_along_normal
from .ply_io import load_ply, save_csv
from .trajectory import find_orientation, generate_trajectory, poses_to_table, trajectory_normal


@dataclass
class WeldSeamResult:
    cloud: np.ndarray
    groove: np.ndarray
    trajectory: np.ndarray
    rotvecs: np.ndarray
    quaternions: np.ndarray
    poses: np.ndarray
    left_poses: np.ndarray
    right_poses: np.ndarray
    normal: np.ndarray
    voxel_size: float
    scale: float
    extras: dict = field(default_factory=dict)

    def save(self, out_dir: str | Path) -> dict[str, Path]:
        out_dir = Path(out_dir)
        out_dir.mkdir(parents=True, exist_ok=True)
        header = ["x", "y", "z", "rx", "ry", "rz", "qx", "qy", "qz", "qw"]
        paths = {
            "trajectory_csv": out_dir / "trajectory.csv",
            "left_csv": out_dir / "trajectory_left.csv",
            "right_csv": out_dir / "trajectory_right.csv",
        }
        save_csv(paths["trajectory_csv"], self.poses, header)
        save_csv(paths["left_csv"], self.left_poses, header[:6])
        save_csv(paths["right_csv"], self.right_poses, header[:6])
        np.savez(
            out_dir / "weld_seam.npz",
            cloud=self.cloud,
            groove=self.groove,
            trajectory=self.trajectory,
            poses=self.poses,
            normal=self.normal,
        )
        paths["npz"] = out_dir / "weld_seam.npz"
        return paths


def _preprocess(
    points: np.ndarray,
    voxel_size: float,
    max_depth: float | None,
    camera_axis: int,
) -> np.ndarray:
    cloud = voxel_downsample(points, voxel_size)
    if max_depth is not None:
        cloud = cloud[cloud[:, camera_axis] < max_depth]
    cloud = remove_statistical_outlier(cloud, nb_neighbors=20, std_ratio=1.0)
    cloud = remove_radius_outlier(cloud, nb_points=12, radius=4.0 * voxel_size)
    if len(cloud) < 30:
        raise RuntimeError(f"Too few points after filtering: {len(cloud)}")
    return cloud


def detect_weld_seam(
    source: str | Path | np.ndarray,
    voxel_size: float | None = None,
    keep_ratio: float = 0.05,
    max_depth: float | None = None,
    camera: np.ndarray | None = None,
    apply_tcp_offset: bool = False,
    thickness: float | None = None,
    sort_distance: float | None = None,
) -> WeldSeamResult:
    """Detect a weld groove from a PLY path or Nx3 array and return a trajectory.

    Algorithm matches romi-lab/robotic-welding-demo ``detect_groove_workflow``:
    downsample → outliers → normals → asymmetry → top-5% → DBSCAN → thin/sort
    → B-spline → 6-DoF poses. Camera-to-base and UR execution are omitted;
    everything stays in the point-cloud frame.
    """
    if isinstance(source, (str, Path)):
        points = load_ply(source)
    else:
        points = np.asarray(source, dtype=np.float64)
    points = finite_points(points)
    if len(points) < 50:
        raise RuntimeError(f"Point cloud too small: {len(points)} points")

    scale = infer_length_scale(points)
    voxel = float(voxel_size if voxel_size is not None else 0.005 * scale)
    cloud = _preprocess(points, voxel, max_depth, camera_axis=2)

    if camera is None:
        extent = cloud.max(axis=0) - cloud.min(axis=0)
        camera = cloud.mean(axis=0) + np.array([0.0, 0.0, 1.5 * float(extent[2] + extent.max() * 0.25)])

    normals = estimate_normals(cloud, radius=max(2.0 * voxel, 0.01 * scale), max_nn=30, camera=camera)
    feature = asymmetry_feature(cloud, normals)
    candidates = select_high_feature_points(cloud, feature, keep_ratio=keep_ratio)
    groove = cluster_groove(candidates, voxel)

    groove_extent = groove.max(axis=0) - groove.min(axis=0)
    line_span = float(np.max(groove_extent))
    thick = float(thickness if thickness is not None else max(0.4 * line_span, 8.0 * voxel))
    step = float(sort_distance if sort_distance is not None else max(4.0 * voxel, 0.02 * scale))

    trajectory = generate_trajectory(groove, thickness=thick, sort_distance=step)
    if len(trajectory) < 2:
        raise RuntimeError("Failed to reconstruct a weld trajectory")

    normal = trajectory_normal(trajectory, cloud, scale=scale)
    traj_pts, rotvecs, quats = find_orientation(trajectory, normal)
    poses = poses_to_table(traj_pts, rotvecs, quats)
    if apply_tcp_offset:
        poses = offset_along_normal(poses, offset_z=-0.003 * scale, offset_y=-0.002 * scale)
        traj_pts = poses[:, :3]
        rotvecs = poses[:, 3:6]
    left, right = multilayer_passes(poses[:, :6], z_height=-0.004 * scale, y_height=-0.006 * scale)

    return WeldSeamResult(
        cloud=cloud,
        groove=groove,
        trajectory=traj_pts,
        rotvecs=rotvecs,
        quaternions=quats,
        poses=poses,
        left_poses=left,
        right_poses=right,
        normal=normal,
        voxel_size=voxel,
        scale=scale,
        extras={
            "feature": feature,
            "candidates": candidates,
            "thickness": thick,
            "sort_distance": step,
            "camera": np.asarray(camera, dtype=np.float64),
        },
    )
