"""
Open3D 点云配准算法对比测试。

算法:
  1. register_fast_global
  2. register_fpfh_ransac
  3. register_icp_generalized
  4. register_icp_point_to_plane
  5. register_icp_point_to_point

读取当前目录下的 src.ply（源）与 target.ply（目标），输出每种算法耗时。

用法:
  python registration_benchmark.py
  python registration_benchmark.py --no-show
"""

from __future__ import annotations

import argparse
import time
from pathlib import Path

import numpy as np
import open3d as o3d


# DemoICPPointClouds 常用初始变换（ICP 需要较好初值）
TRANS_INIT = np.asarray(
    [
        [0.862, 0.011, -0.507, 0.5],
        [-0.139, 0.967, -0.215, 0.7],
        [0.487, 0.255, 0.835, -1.4],
        [0.0, 0.0, 0.0, 1.0],
    ],
    dtype=float,
)


def load_point_clouds(src_path: Path, target_path: Path):
    if not src_path.is_file():
        raise FileNotFoundError(f"找不到源点云: {src_path}")
    if not target_path.is_file():
        raise FileNotFoundError(f"找不到目标点云: {target_path}")

    source = o3d.io.read_point_cloud(str(src_path))
    target = o3d.io.read_point_cloud(str(target_path))
    if source.is_empty() or target.is_empty():
        raise RuntimeError("src.ply 或 target.ply 为空")

    print(f"[INFO] source: {len(source.points)} points <- {src_path}")
    print(f"[INFO] target: {len(target.points)} points <- {target_path}")
    return source, target


def preprocess_point_cloud(pcd: o3d.geometry.PointCloud, voxel_size: float):
    pcd_down = pcd.voxel_down_sample(voxel_size)
    pcd_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 2.0, max_nn=30)
    )
    fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        pcd_down,
        o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 5.0, max_nn=100),
    )
    return pcd_down, fpfh


def prepare_for_registration(source, target, voxel_size: float):
    print(f"[INFO] 预处理 voxel_size={voxel_size} ...")
    t0 = time.perf_counter()

    source_down, source_fpfh = preprocess_point_cloud(source, voxel_size)
    target_down, target_fpfh = preprocess_point_cloud(target, voxel_size)

    source_icp = o3d.geometry.PointCloud(source)
    target_icp = o3d.geometry.PointCloud(target)
    normal_param = o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 2.0, max_nn=30)
    if not source_icp.has_normals():
        source_icp.estimate_normals(normal_param)
    if not target_icp.has_normals():
        target_icp.estimate_normals(normal_param)
    source_icp.estimate_covariances(normal_param)
    target_icp.estimate_covariances(normal_param)

    print(f"[INFO] 预处理完成: {time.perf_counter() - t0:.3f}s")
    print(f"[INFO] downsampled src={len(source_down.points)}, tgt={len(target_down.points)}")
    return source_down, target_down, source_fpfh, target_fpfh, source_icp, target_icp


# ---------------------------------------------------------------------------
# 五种配准算法
# ---------------------------------------------------------------------------

def register_fast_global(source_down, target_down, source_fpfh, target_fpfh, voxel_size):
    distance_threshold = voxel_size * 0.5
    # Open3D >=0.17: registration_fgr_based_on_feature_matching
    return o3d.pipelines.registration.registration_fgr_based_on_feature_matching(
        source_down,
        target_down,
        source_fpfh,
        target_fpfh,
        o3d.pipelines.registration.FastGlobalRegistrationOption(
            maximum_correspondence_distance=distance_threshold
        ),
    )


def register_fpfh_ransac(source_down, target_down, source_fpfh, target_fpfh, voxel_size):
    distance_threshold = voxel_size * 1.5
    return o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        source_down,
        target_down,
        source_fpfh,
        target_fpfh,
        True,
        distance_threshold,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
        3,
        [
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(distance_threshold),
        ],
        o3d.pipelines.registration.RANSACConvergenceCriteria(100000, 0.999),
    )


def register_icp_generalized(source, target, threshold, init):
    return o3d.pipelines.registration.registration_generalized_icp(
        source,
        target,
        threshold,
        init,
        o3d.pipelines.registration.TransformationEstimationForGeneralizedICP(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=50),
    )


def register_icp_point_to_plane(source, target, threshold, init):
    return o3d.pipelines.registration.registration_icp(
        source,
        target,
        threshold,
        init,
        o3d.pipelines.registration.TransformationEstimationPointToPlane(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=50),
    )


def register_icp_point_to_point(source, target, threshold, init):
    return o3d.pipelines.registration.registration_icp(
        source,
        target,
        threshold,
        init,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=50),
    )


def evaluate(source, target, transformation, threshold):
    return o3d.pipelines.registration.evaluate_registration(
        source, target, threshold, transformation
    )


def draw_result(source, target, transformation, window_name: str):
    src = o3d.geometry.PointCloud(source)
    tgt = o3d.geometry.PointCloud(target)
    src.paint_uniform_color([1, 0.706, 0])
    tgt.paint_uniform_color([0, 0.651, 0.929])
    src.transform(transformation)
    o3d.visualization.draw_geometries([src, tgt], window_name=window_name, width=960, height=720)


def run_benchmark(source, target, voxel_size: float, show: bool):
    (
        source_down,
        target_down,
        source_fpfh,
        target_fpfh,
        source_icp,
        target_icp,
    ) = prepare_for_registration(source, target, voxel_size)

    icp_threshold = voxel_size * 0.4
    eval_threshold = voxel_size * 1.5

    algorithms = [
        (
            "register_fast_global",
            lambda: register_fast_global(
                source_down, target_down, source_fpfh, target_fpfh, voxel_size
            ),
        ),
        (
            "register_fpfh_ransac",
            lambda: register_fpfh_ransac(
                source_down, target_down, source_fpfh, target_fpfh, voxel_size
            ),
        ),
        (
            "register_icp_generalized",
            lambda: register_icp_generalized(source_icp, target_icp, icp_threshold, TRANS_INIT),
        ),
        (
            "register_icp_point_to_plane",
            lambda: register_icp_point_to_plane(source_icp, target_icp, icp_threshold, TRANS_INIT),
        ),
        (
            "register_icp_point_to_point",
            lambda: register_icp_point_to_point(source_icp, target_icp, icp_threshold, TRANS_INIT),
        ),
    ]

    rows = []
    print("\n" + "=" * 72)
    print(f"{'Algorithm':<30} {'Time(s)':>10} {'Fitness':>10} {'InlierRMSE':>12}")
    print("=" * 72)

    for name, fn in algorithms:
        t0 = time.perf_counter()
        result = fn()
        elapsed = time.perf_counter() - t0

        evaluation = evaluate(source, target, result.transformation, eval_threshold)
        rows.append(
            {
                "name": name,
                "time_s": elapsed,
                "fitness": evaluation.fitness,
                "inlier_rmse": evaluation.inlier_rmse,
                "transformation": np.asarray(result.transformation),
            }
        )
        print(
            f"{name:<30} {elapsed:>10.4f} "
            f"{evaluation.fitness:>10.4f} {evaluation.inlier_rmse:>12.6f}"
        )
        if show:
            draw_result(source, target, result.transformation, name)

    print("=" * 72)
    return rows


def save_report(rows, out_path: Path):
    out_path.parent.mkdir(parents=True, exist_ok=True)
    lines = [
        "Open3D Registration Benchmark",
        "=" * 60,
        f"{'Algorithm':<30} {'Time(s)':>10} {'Fitness':>10} {'InlierRMSE':>12}",
        "-" * 60,
    ]
    for r in rows:
        lines.append(
            f"{r['name']:<30} {r['time_s']:>10.4f} {r['fitness']:>10.4f} {r['inlier_rmse']:>12.6f}"
        )
    lines.append("-" * 60)
    lines.append("")
    for r in rows:
        lines.append(f"[{r['name']}] transformation:")
        lines.append(np.array2string(r["transformation"], precision=6, suppress_small=True))
        lines.append("")
    out_path.write_text("\n".join(lines), encoding="utf-8")
    print(f"[INFO] 报告已保存: {out_path.resolve()}")


def main():
    parser = argparse.ArgumentParser(description="Open3D 五种点云配准算法计时测试")
    parser.add_argument("--src", type=Path, default=Path("src.ply"))
    parser.add_argument("--target", type=Path, default=Path("target.ply"))
    parser.add_argument("--voxel-size", type=float, default=0.05)
    parser.add_argument("--report", type=Path, default=Path("output/registration_benchmark.txt"))
    parser.add_argument("--no-show", action="store_true", help="不弹出可视化窗口")
    args = parser.parse_args()

    source, target = load_point_clouds(args.src, args.target)
    rows = run_benchmark(source, target, args.voxel_size, show=not args.no_show)
    save_report(rows, args.report)


if __name__ == "__main__":
    main()
