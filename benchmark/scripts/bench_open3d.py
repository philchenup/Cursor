#!/usr/bin/env python3
"""Open3D timing benchmark for the requested algorithm catalog."""

from __future__ import annotations

import json
import time
from pathlib import Path
from typing import Any, Callable

import numpy as np
import open3d as o3d

ROOT = Path(__file__).resolve().parents[1]
DATA = ROOT / "data"
OUT = ROOT / "results" / "open3d_timings.json"
WARMUP = 1
REPEATS = 5


def timed(fn: Callable[[], Any], warmup: int = WARMUP, repeats: int = REPEATS) -> dict:
    for _ in range(warmup):
        fn()
    samples = []
    result = None
    for _ in range(repeats):
        t0 = time.perf_counter()
        result = fn()
        samples.append((time.perf_counter() - t0) * 1000.0)
    return {
        "mean_ms": float(np.mean(samples)),
        "std_ms": float(np.std(samples)),
        "min_ms": float(np.min(samples)),
        "max_ms": float(np.max(samples)),
        "samples_ms": [float(x) for x in samples],
        "supported": True,
    }


def unsupported(reason: str) -> dict:
    return {"supported": False, "reason": reason}


def load_meta():
    return json.loads((DATA / "meta.json").read_text())


def main() -> None:
    meta = load_meta()
    intr = meta["intrinsics"]
    results: dict[str, dict] = {
        "library": "Open3D",
        "version": o3d.__version__,
        "n_points": meta["n_points"],
        "categories": {},
    }

    pcd = o3d.io.read_point_cloud(str(DATA / "cloud.ply"))
    pts = np.asarray(pcd.points)
    src = o3d.io.read_point_cloud(str(DATA / "cloud_source.ply"))
    tgt = o3d.io.read_point_cloud(str(DATA / "cloud_target.ply"))
    depth_img = o3d.io.read_image(str(DATA / "depth.png"))
    color_img = o3d.io.read_image(str(DATA / "color.png"))
    mask_img = o3d.io.read_image(str(DATA / "mask.png"))
    cam = o3d.camera.PinholeCameraIntrinsic(
        intr["width"],
        intr["height"],
        intr["fx"],
        intr["fy"],
        intr["cx"],
        intr["cy"],
    )

    # ---------- IO / conversion ----------
    io_cat: dict[str, Any] = {}

    def load_cloud():
        return o3d.io.read_point_cloud(str(DATA / "cloud.ply"))

    io_cat["load_cloud"] = timed(load_cloud)

    def save_cloud():
        o3d.io.write_point_cloud(str(DATA / "_tmp_o3d.ply"), pcd)

    io_cat["save_cloud"] = timed(save_cloud)

    def txt_to_cloud():
        arr = np.loadtxt(DATA / "cloud.txt")
        c = o3d.geometry.PointCloud()
        c.points = o3d.utility.Vector3dVector(arr)
        return c

    io_cat["txt_to_cloud"] = timed(txt_to_cloud)

    def cloud_to_txt():
        np.savetxt(DATA / "_tmp_o3d.txt", np.asarray(pcd.points), fmt="%.6f")

    io_cat["cloud_to_txt"] = timed(cloud_to_txt)

    def rescale_units():
        c = o3d.geometry.PointCloud(pcd)
        c.scale(1000.0, center=np.zeros(3))
        return c

    io_cat["rescale_units"] = timed(rescale_units)

    def transform_cloud():
        c = o3d.geometry.PointCloud(pcd)
        T = np.eye(4)
        T[:3, :3] = o3d.geometry.get_rotation_matrix_from_xyz((0.1, 0.2, 0.05))
        T[:3, 3] = [0.01, -0.02, 0.03]
        c.transform(T)
        return c

    io_cat["transform_cloud"] = timed(transform_cloud)

    def depth_to_cloud():
        return o3d.geometry.PointCloud.create_from_depth_image(
            depth_img, cam, depth_scale=1000.0, depth_trunc=3.0
        )

    io_cat["depth_to_cloud"] = timed(depth_to_cloud)

    def rgbd_to_cloud():
        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
            color_img, depth_img, depth_scale=1000.0, depth_trunc=3.0, convert_rgb_to_intensity=False
        )
        return o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, cam)

    io_cat["rgbd_to_cloud"] = timed(rgbd_to_cloud)

    def extract_cloud_by_mask():
        # Project points to image plane and keep those under mask
        depth = np.asarray(depth_img)
        mask = np.asarray(mask_img)
        if mask.ndim == 3:
            mask = mask[..., 0]
        cloud = o3d.geometry.PointCloud.create_from_depth_image(
            depth_img, cam, depth_scale=1000.0, depth_trunc=3.0
        )
        # Approximate: crop by AABB of masked depth pixels mapped to 3D bbox
        ys, xs = np.where(mask > 0)
        if len(xs) == 0:
            return cloud
        u0, u1 = xs.min(), xs.max()
        v0, v1 = ys.min(), ys.max()
        # Keep points whose projection falls in mask bbox (fast proxy)
        pts_c = np.asarray(cloud.points)
        if len(pts_c) == 0:
            return cloud
        fx, fy, cx, cy = intr["fx"], intr["fy"], intr["cx"], intr["cy"]
        u = (pts_c[:, 0] / pts_c[:, 2]) * fx + cx
        v = (pts_c[:, 1] / pts_c[:, 2]) * fy + cy
        keep = (u >= u0) & (u <= u1) & (v >= v0) & (v <= v1)
        out = o3d.geometry.PointCloud()
        out.points = o3d.utility.Vector3dVector(pts_c[keep])
        return out

    io_cat["extract_cloud_by_mask"] = timed(extract_cloud_by_mask)

    def concat_clouds():
        return pcd + pcd

    io_cat["concat_clouds"] = timed(concat_clouds)
    results["categories"]["io_conversion"] = io_cat

    # ---------- preprocess ----------
    pre: dict[str, Any] = {}
    voxel = 0.05

    pre["downsample_voxel"] = timed(lambda: pcd.voxel_down_sample(voxel))
    pre["downsample_uniform"] = timed(lambda: pcd.uniform_down_sample(every_k_points=10))
    pre["downsample_random"] = timed(lambda: pcd.random_down_sample(sampling_ratio=0.2))
    pre["downsample_fps"] = timed(
        lambda: pcd.farthest_point_down_sample(num_samples=min(5000, len(pts)))
    )
    pre["downsample_to_count"] = timed(
        lambda: pcd.farthest_point_down_sample(num_samples=min(8000, len(pts)))
    )
    pre["downsample_normal_space"] = unsupported(
        "Open3D has no Normal Space Sampling (NSS) API"
    )

    aabb = pcd.get_axis_aligned_bounding_box()
    min_b, max_b = aabb.get_min_bound(), aabb.get_max_bound()
    z_mid = 0.5 * (min_b[2] + max_b[2])

    def filter_axis_range():
        bbox = o3d.geometry.AxisAlignedBoundingBox(
            min_bound=[min_b[0], min_b[1], min_b[2]],
            max_bound=[max_b[0], max_b[1], z_mid],
        )
        return pcd.crop(bbox)

    pre["filter_axis_range"] = timed(filter_axis_range)

    def filter_axis_threshold():
        arr = np.asarray(pcd.points)
        keep = arr[:, 2] > z_mid
        out = o3d.geometry.PointCloud()
        out.points = o3d.utility.Vector3dVector(arr[keep])
        return out

    pre["filter_axis_threshold"] = timed(filter_axis_threshold)
    pre["filter_by_direction"] = unsupported(
        "No dedicated direction-filter; would require custom normal/dot logic"
    )
    pre["remove_statistical_outliers"] = timed(
        lambda: pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)[0]
    )
    pre["remove_radius_outliers"] = timed(
        lambda: pcd.remove_radius_outlier(nb_points=16, radius=0.05)[0]
    )

    pcd_n = o3d.geometry.PointCloud(pcd)

    def estimate_normals():
        c = o3d.geometry.PointCloud(pcd)
        c.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.08, max_nn=30)
        )
        return c

    pre["estimate_normals"] = timed(estimate_normals)

    def orient_normals():
        c = o3d.geometry.PointCloud(pcd)
        c.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.08, max_nn=30)
        )
        c.orient_normals_consistent_tangent_plane(k=15)
        return c

    pre["orient_normals"] = timed(orient_normals)

    def flip_normals():
        c = o3d.geometry.PointCloud(pcd)
        if not c.has_normals():
            c.estimate_normals(
                search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.08, max_nn=30)
            )
        c.normals = o3d.utility.Vector3dVector(-np.asarray(c.normals))
        return c

    pre["flip_normals"] = timed(flip_normals)

    def compute_fpfh():
        c = o3d.geometry.PointCloud(pcd)
        c = c.voxel_down_sample(0.05)
        c.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30)
        )
        return o3d.pipelines.registration.compute_fpfh_feature(
            c, o3d.geometry.KDTreeSearchParamHybrid(radius=0.25, max_nn=100)
        )

    pre["compute_fpfh"] = timed(compute_fpfh)

    def compute_curvature():
        # PCA-based curvature via covariance eigenvalues on neighborhoods
        c = o3d.geometry.PointCloud(pcd)
        c = c.voxel_down_sample(0.04)
        tree = o3d.geometry.KDTreeFlann(c)
        pts_c = np.asarray(c.points)
        curv = np.zeros(len(pts_c))
        for i in range(len(pts_c)):
            _, idx, _ = tree.search_knn_vector_3d(c.points[i], 20)
            neigh = pts_c[idx]
            cov = np.cov(neigh.T)
            w = np.linalg.eigvalsh(cov)
            w = np.clip(w, 1e-12, None)
            curv[i] = w[0] / w.sum()
        return curv

    pre["compute_curvature"] = timed(compute_curvature)
    results["categories"]["preprocess"] = pre

    # ---------- geometry / segmentation ----------
    geo: dict[str, Any] = {}

    def compute_obb():
        return pcd.get_oriented_bounding_box()

    geo["compute_obb"] = timed(compute_obb)

    def compute_convex_hull():
        return pcd.compute_convex_hull()

    geo["compute_convex_hull"] = timed(compute_convex_hull)

    def compute_centroid():
        return pcd.get_center()

    geo["compute_centroid"] = timed(compute_centroid)

    def compute_aabb():
        return pcd.get_axis_aligned_bounding_box()

    geo["compute_aabb"] = timed(compute_aabb)

    def select_by_mask():
        n = len(pcd.points)
        m = np.zeros(n, dtype=bool)
        m[::2] = True
        return pcd.select_by_index(np.where(m)[0])

    geo["select_by_mask"] = timed(select_by_mask)

    def select_by_index():
        idx = np.arange(0, len(pcd.points), 5)
        return pcd.select_by_index(idx)

    geo["select_by_index"] = timed(select_by_index)

    def segment_plane():
        return pcd.segment_plane(distance_threshold=0.02, ransac_n=3, num_iterations=500)

    geo["segment_plane"] = timed(segment_plane)

    def cluster_dbscan():
        return np.array(pcd.cluster_dbscan(eps=0.05, min_points=10, print_progress=False))

    geo["cluster_dbscan"] = timed(cluster_dbscan)

    def split_by_labels():
        labels = np.array(pcd.cluster_dbscan(eps=0.05, min_points=10, print_progress=False))
        clouds = []
        for lab in np.unique(labels):
            if lab < 0:
                continue
            clouds.append(pcd.select_by_index(np.where(labels == lab)[0]))
        return clouds

    geo["split_by_labels"] = timed(split_by_labels)

    def fit_line():
        # PCA principal axis as line fit proxy
        arr = np.asarray(pcd.points)
        c = arr.mean(axis=0)
        _, _, vt = np.linalg.svd(arr - c, full_matrices=False)
        return c, vt[0]

    geo["fit_line"] = timed(fit_line)
    geo["fit_sphere"] = unsupported("Open3D has no built-in sphere RANSAC fit API")

    def fit_plane():
        return pcd.segment_plane(distance_threshold=0.02, ransac_n=3, num_iterations=300)

    geo["fit_plane"] = timed(fit_plane)

    def plane_from_points():
        arr = np.asarray(pcd.points)[:3]
        v1 = arr[1] - arr[0]
        v2 = arr[2] - arr[0]
        n = np.cross(v1, v2)
        n /= np.linalg.norm(n) + 1e-12
        d = -np.dot(n, arr[0])
        return n, d

    geo["plane_from_points"] = timed(plane_from_points)

    def plane_normal():
        model, *_ = pcd.segment_plane(distance_threshold=0.02, ransac_n=3, num_iterations=200)
        n = np.asarray(model[:3])
        return n / (np.linalg.norm(n) + 1e-12)

    geo["plane_normal"] = timed(plane_normal)

    def split_by_plane():
        model, inliers = pcd.segment_plane(
            distance_threshold=0.02, ransac_n=3, num_iterations=300
        )
        inlier = pcd.select_by_index(inliers)
        outlier = pcd.select_by_index(inliers, invert=True)
        return inlier, outlier

    geo["split_by_plane"] = timed(split_by_plane)

    def extract_boundary_points():
        c = o3d.geometry.PointCloud(pcd).voxel_down_sample(0.03)
        c.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.08, max_nn=30)
        )
        # Boundary approx via low neighbor count
        tree = o3d.geometry.KDTreeFlann(c)
        pts_c = np.asarray(c.points)
        border = []
        for i in range(len(pts_c)):
            k, idx, _ = tree.search_radius_vector_3d(c.points[i], 0.06)
            if k < 12:
                border.append(i)
        return c.select_by_index(border)

    geo["extract_boundary_points"] = timed(extract_boundary_points)
    results["categories"]["geometry_segmentation"] = geo

    # ---------- registration ----------
    reg: dict[str, Any] = {}
    voxel_r = 0.05
    src_d = src.voxel_down_sample(voxel_r)
    tgt_d = tgt.voxel_down_sample(voxel_r)
    src_d.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_r * 2, max_nn=30)
    )
    tgt_d.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_r * 2, max_nn=30)
    )
    src_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        src_d, o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_r * 5, max_nn=100)
    )
    tgt_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        tgt_d, o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_r * 5, max_nn=100)
    )

    def make_init_pose():
        return np.eye(4)

    reg["make_init_pose"] = timed(make_init_pose)

    def register_fpfh_ransac():
        return o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
            src_d,
            tgt_d,
            src_fpfh,
            tgt_fpfh,
            mutual_filter=True,
            max_correspondence_distance=voxel_r * 1.5,
            estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(
                False
            ),
            ransac_n=3,
            checkers=[
                o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
                o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(voxel_r * 1.5),
            ],
            criteria=o3d.pipelines.registration.RANSACConvergenceCriteria(50_000, 0.999),
        )

    reg["register_fpfh_ransac"] = timed(register_fpfh_ransac)

    def register_fast_global():
        return o3d.pipelines.registration.registration_fgr_based_on_feature_matching(
            src_d,
            tgt_d,
            src_fpfh,
            tgt_fpfh,
            o3d.pipelines.registration.FastGlobalRegistrationOption(
                maximum_correspondence_distance=voxel_r * 1.5
            ),
        )

    reg["register_fast_global"] = timed(register_fast_global)
    reg["register_ppf"] = unsupported("Open3D has no PPF registration pipeline API")
    reg["relative_sampling_step"] = unsupported(
        "Parameter utility, not a standalone Open3D algorithm"
    )

    init = np.eye(4)

    def register_icp_point_to_point():
        return o3d.pipelines.registration.registration_icp(
            src_d,
            tgt_d,
            max_correspondence_distance=voxel_r * 2,
            init=init,
            estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(),
            criteria=o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=30),
        )

    reg["register_icp_point_to_point"] = timed(register_icp_point_to_point)

    def register_icp_point_to_plane():
        return o3d.pipelines.registration.registration_icp(
            src_d,
            tgt_d,
            max_correspondence_distance=voxel_r * 2,
            init=init,
            estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPlane(),
            criteria=o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=30),
        )

    reg["register_icp_point_to_plane"] = timed(register_icp_point_to_plane)

    def register_icp_generalized():
        return o3d.pipelines.registration.registration_generalized_icp(
            src_d,
            tgt_d,
            max_correspondence_distance=voxel_r * 2,
            init=init,
            criteria=o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=30),
        )

    reg["register_icp_generalized"] = timed(register_icp_generalized)

    # Colored ICP needs colors
    src_c = o3d.geometry.PointCloud(src_d)
    tgt_c = o3d.geometry.PointCloud(tgt_d)
    if not src_c.has_colors():
        src_c.paint_uniform_color([1, 0.7, 0])
    if not tgt_c.has_colors():
        tgt_c.paint_uniform_color([0, 0.65, 0.9])

    def register_icp_colored():
        return o3d.pipelines.registration.registration_colored_icp(
            src_c,
            tgt_c,
            max_correspondence_distance=voxel_r * 2,
            init=init,
            criteria=o3d.pipelines.registration.ICPConvergenceCriteria(
                relative_fitness=1e-6, relative_rmse=1e-6, max_iteration=20
            ),
        )

    reg["register_icp_colored"] = timed(register_icp_colored)
    reg["register_filterreg"] = unsupported("FilterReg not in Open3D")
    reg["register_cpd"] = unsupported("CPD not in Open3D core")
    reg["register_gmmtree"] = unsupported("GMMTree registration not in Open3D")

    def evaluate_registration():
        return o3d.pipelines.registration.evaluate_registration(
            src_d, tgt_d, voxel_r * 2, init
        )

    reg["evaluate_registration"] = timed(evaluate_registration)

    def is_registration_valid():
        ev = o3d.pipelines.registration.evaluate_registration(
            src_d, tgt_d, voxel_r * 2, init
        )
        return bool(ev.fitness > 0.3 and ev.inlier_rmse < 0.05)

    reg["is_registration_valid"] = timed(is_registration_valid)
    results["categories"]["registration"] = reg

    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(json.dumps(results, indent=2))
    print(json.dumps(results, indent=2))


if __name__ == "__main__":
    main()
