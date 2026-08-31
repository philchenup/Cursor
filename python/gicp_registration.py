"""Open3D Generalized ICP (GICP) point cloud registration.

Usage as a library:

    from gicp_registration import register_gicp, load_point_cloud

    source = load_point_cloud("source.pcd")
    target = load_point_cloud("target.pcd")
    result = register_gicp(source, target, voxel_size=0.05)
    source.transform(result.transformation)

Usage from the command line:

    python gicp_registration.py source.pcd target.pcd --voxel-size 0.05
    python gicp_registration.py --self-test
"""

from __future__ import annotations

import argparse
import copy
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import open3d as o3d


@dataclass
class GICPResult:
    """GICP output: 4x4 transform that takes source into the target frame."""

    transformation: np.ndarray
    fitness: float
    inlier_rmse: float
    correspondence_set: np.ndarray

    @classmethod
    def from_open3d(
        cls, result: o3d.pipelines.registration.RegistrationResult
    ) -> GICPResult:
        return cls(
            transformation=np.asarray(result.transformation, dtype=np.float64),
            fitness=float(result.fitness),
            inlier_rmse=float(result.inlier_rmse),
            correspondence_set=np.asarray(result.correspondence_set),
        )


def load_point_cloud(path: str | Path) -> o3d.geometry.PointCloud:
    pcd = o3d.io.read_point_cloud(str(path))
    if pcd.is_empty():
        raise ValueError(f"Empty or unreadable point cloud: {path}")
    return pcd


def prepare_for_gicp(
    pcd: o3d.geometry.PointCloud,
    voxel_size: float,
) -> o3d.geometry.PointCloud:
    """Voxel-downsample and estimate normals used by GICP covariance estimation."""
    cloud = pcd.voxel_down_sample(voxel_size) if voxel_size > 0 else copy.deepcopy(pcd)
    radius = voxel_size * 2.0 if voxel_size > 0 else 0.1
    cloud.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius, max_nn=30)
    )
    cloud.orient_normals_towards_camera_location(camera_location=np.array([0.0, 0.0, 0.0]))
    return cloud


def register_gicp(
    source: o3d.geometry.PointCloud,
    target: o3d.geometry.PointCloud,
    *,
    voxel_size: float = 0.05,
    max_correspondence_distance: float | None = None,
    init: np.ndarray | None = None,
    max_iteration: int = 64,
    relative_fitness: float = 1e-6,
    relative_rmse: float = 1e-6,
    epsilon: float = 1e-3,
) -> GICPResult:
    """Align ``source`` to ``target`` with Open3D Generalized ICP.

    Parameters
    ----------
    source, target:
        Input clouds. They are not modified.
    voxel_size:
        Downsample leaf size. Also used to set the correspondence distance
        when ``max_correspondence_distance`` is omitted (3 * voxel_size).
    max_correspondence_distance:
        Reject correspondences farther than this (same unit as the clouds).
    init:
        4x4 initial guess. Identity if omitted. GICP is local; a coarse
        global alignment (FPFH + RANSAC) is recommended for large offsets.
    max_iteration, relative_fitness, relative_rmse:
        ICPConvergenceCriteria.
    epsilon:
        Covariance regularization in TransformationEstimationForGeneralizedICP.

    Returns
    -------
    GICPResult
        ``transformation`` maps source points into the target frame.
    """
    if source.is_empty() or target.is_empty():
        raise ValueError("Source and target point clouds must be non-empty")

    if init is None:
        init = np.identity(4, dtype=np.float64)
    else:
        init = np.asarray(init, dtype=np.float64).reshape(4, 4)

    if max_correspondence_distance is None:
        max_correspondence_distance = voxel_size * 3.0 if voxel_size > 0 else 0.1

    source_d = prepare_for_gicp(source, voxel_size)
    target_d = prepare_for_gicp(target, voxel_size)

    estimation = o3d.pipelines.registration.TransformationEstimationForGeneralizedICP(
        epsilon
    )
    criteria = o3d.pipelines.registration.ICPConvergenceCriteria(
        relative_fitness=relative_fitness,
        relative_rmse=relative_rmse,
        max_iteration=max_iteration,
    )

    raw = o3d.pipelines.registration.registration_generalized_icp(
        source_d,
        target_d,
        max_correspondence_distance,
        init,
        estimation,
        criteria,
    )
    return GICPResult.from_open3d(raw)


def register_gicp_multiscale(
    source: o3d.geometry.PointCloud,
    target: o3d.geometry.PointCloud,
    voxel_sizes: tuple[float, ...] = (0.2, 0.1, 0.05),
    *,
    init: np.ndarray | None = None,
    max_iteration: int = 64,
) -> GICPResult:
    """Coarse-to-fine GICP. Each stage uses the previous transform as init."""
    current = np.identity(4, dtype=np.float64) if init is None else np.asarray(init)
    result: GICPResult | None = None
    for voxel_size in voxel_sizes:
        result = register_gicp(
            source,
            target,
            voxel_size=voxel_size,
            init=current,
            max_iteration=max_iteration,
        )
        current = result.transformation
    assert result is not None
    return result


def apply_transform(
    source: o3d.geometry.PointCloud,
    transformation: np.ndarray,
) -> o3d.geometry.PointCloud:
    aligned = copy.deepcopy(source)
    aligned.transform(transformation)
    return aligned


def make_synthetic_pair(
    n_points: int = 4000,
    translation: tuple[float, float, float] = (0.12, -0.08, 0.05),
    yaw_deg: float = 12.0,
    seed: int = 0,
) -> tuple[o3d.geometry.PointCloud, o3d.geometry.PointCloud, np.ndarray]:
    """Target is a noisy unit cube; source is the same cloud after a known pose."""
    rng = np.random.default_rng(seed)
    pts = rng.uniform(-0.5, 0.5, size=(n_points, 3))

    target = o3d.geometry.PointCloud()
    target.points = o3d.utility.Vector3dVector(pts)

    yaw = np.deg2rad(yaw_deg)
    gt = np.identity(4, dtype=np.float64)
    gt[0, 0] = np.cos(yaw)
    gt[0, 1] = -np.sin(yaw)
    gt[1, 0] = np.sin(yaw)
    gt[1, 1] = np.cos(yaw)
    gt[0:3, 3] = translation

    source = o3d.geometry.PointCloud()
    # source = inv(gt) * target, so GICP should recover gt
    source.points = o3d.utility.Vector3dVector(pts)
    source.transform(np.linalg.inv(gt))
    source.points = o3d.utility.Vector3dVector(
        np.asarray(source.points) + rng.normal(0.0, 0.002, size=(n_points, 3))
    )
    return source, target, gt


def _print_result(result: GICPResult) -> None:
    np.set_printoptions(precision=6, suppress=True)
    print("fitness     :", result.fitness)
    print("inlier_rmse :", result.inlier_rmse)
    print("correspondences:", len(result.correspondence_set))
    print("transformation:\n", result.transformation)


def _self_test() -> int:
    source, target, gt = make_synthetic_pair()
    result = register_gicp_multiscale(source, target, voxel_sizes=(0.08, 0.04, 0.02))
    _print_result(result)

    rot_err = np.linalg.norm(result.transformation[:3, :3] - gt[:3, :3], ord="fro")
    trans_err = np.linalg.norm(result.transformation[:3, 3] - gt[:3, 3])
    print(f"rotation Frobenius error: {rot_err:.6f}")
    print(f"translation L2 error    : {trans_err:.6f}")

    if result.fitness < 0.5:
        print("FAIL: fitness too low")
        return 1
    if trans_err > 0.02:
        print("FAIL: translation error too large")
        return 1
    print("PASS")
    return 0


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Open3D GICP point cloud registration")
    parser.add_argument("source", nargs="?", help="source point cloud (.pcd/.ply/...)")
    parser.add_argument("target", nargs="?", help="target point cloud (.pcd/.ply/...)")
    parser.add_argument("--voxel-size", type=float, default=0.05)
    parser.add_argument("--max-corr-dist", type=float, default=None)
    parser.add_argument("--max-iteration", type=int, default=64)
    parser.add_argument(
        "--multiscale",
        action="store_true",
        help="run coarse-to-fine GICP (voxel_size * {4, 2, 1})",
    )
    parser.add_argument("--output", type=str, default=None, help="save aligned source")
    parser.add_argument("--self-test", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = _parse_args()
    if args.self_test:
        return _self_test()

    if not args.source or not args.target:
        raise SystemExit("source and target paths are required (or pass --self-test)")

    source = load_point_cloud(args.source)
    target = load_point_cloud(args.target)

    if args.multiscale:
        voxel_sizes = (args.voxel_size * 4.0, args.voxel_size * 2.0, args.voxel_size)
        result = register_gicp_multiscale(
            source,
            target,
            voxel_sizes=voxel_sizes,
            max_iteration=args.max_iteration,
        )
    else:
        result = register_gicp(
            source,
            target,
            voxel_size=args.voxel_size,
            max_correspondence_distance=args.max_corr_dist,
            max_iteration=args.max_iteration,
        )

    _print_result(result)

    if args.output:
        aligned = apply_transform(source, result.transformation)
        o3d.io.write_point_cloud(args.output, aligned)
        print("wrote", args.output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
