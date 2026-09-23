#!/usr/bin/env python3
"""Separate the three workpiece planes in a welding three-plane butt / triplanar scene.

Vanilla sequential RANSAC often glues two faces together or steals points from
the groove. This pipeline:

  1) ROI + voxel + statistical filter + normals
  2) sequential RANSAC with a normal-consistency inlier test
  3) merge nearly coplanar fragments (plate split by occlusion)
  4) keep the three largest distinct orientations
  5) globally reassign every point (distance AND normal)
  6) drop the ambiguous band around plane intersections
  7) SVD-refine each plane on the cleaned inliers

Typical geometries
  - triplanar fillet: three faces ~pairwise 70–110 deg
  - V-groove butt: two bevels + bottom / backing, dihedral ~30–70 deg
  - I-butt of two nearly coplanar plates: RANSAC cannot split them; use a gap /
    density split instead (see --mode ibutt)
"""

from __future__ import annotations

import argparse
import copy
from dataclasses import dataclass

import numpy as np
import open3d as o3d


@dataclass
class Plane:
    n: np.ndarray  # unit normal
    d: float  # n·x + d = 0
    indices: np.ndarray

    def distance(self, pts: np.ndarray) -> np.ndarray:
        return pts @ self.n + self.d

    def flip_toward(self, centroid: np.ndarray) -> None:
        if float(self.n @ centroid + self.d) > 0.0:
            self.n = -self.n
            self.d = -self.d


def estimate_normals(pcd, radius: float) -> None:
    pcd.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=radius, max_nn=40))
    pcd.orient_normals_consistent_tangent_plane(30)


def fit_plane_svd(pts: np.ndarray) -> tuple[np.ndarray, float]:
    c = pts.mean(axis=0)
    _, _, vt = np.linalg.svd(pts - c, full_matrices=False)
    n = vt[-1]
    n = n / (np.linalg.norm(n) + 1e-12)
    d = -float(n @ c)
    return n, d


def ransac_plane_normal_aware(pcd, dist_th: float, normal_th: float, n_iter: int, min_inliers: int):
    pts = np.asarray(pcd.points)
    nrm = np.asarray(pcd.normals)
    if len(pts) < 3:
        return None

    model, inliers = pcd.segment_plane(distance_threshold=dist_th, ransac_n=3, num_iterations=n_iter)
    a, b, c, d = model
    n = np.array([a, b, c], dtype=float)
    nn = np.linalg.norm(n)
    if nn < 1e-9:
        return None
    n = n / nn
    d = d / nn

    idx = np.asarray(inliers, dtype=int)
    if idx.size == 0:
        return None
    keep = np.abs(nrm[idx] @ n) >= normal_th
    idx = idx[keep]
    if idx.size < min_inliers:
        return None

    n, d = fit_plane_svd(pts[idx])
    plane = Plane(n=n, d=d, indices=idx)
    plane.flip_toward(pts.mean(axis=0))
    return plane


def sequential_planes(pcd, k: int, dist_th: float, normal_th: float, n_iter: int, min_inliers: int):
    remaining = copy.deepcopy(pcd)
    planes = []
    used = np.zeros(len(pcd.points), dtype=bool)
    global_ids = np.arange(len(pcd.points))

    for _ in range(k):
        if len(remaining.points) < max(min_inliers, 20):
            break
        plane = ransac_plane_normal_aware(remaining, dist_th, normal_th, n_iter, min_inliers)
        if plane is None:
            break
        local = plane.indices
        plane.indices = global_ids[local]
        planes.append(plane)
        used_mask = np.ones(len(remaining.points), dtype=bool)
        used_mask[local] = False
        remaining = remaining.select_by_index(np.where(used_mask)[0])
        global_ids = global_ids[used_mask]
    return planes


def merge_coplanar(planes: list[Plane], pts: np.ndarray, n_dot: float, d_th: float) -> list[Plane]:
    used = [False] * len(planes)
    merged = []
    for i, p in enumerate(planes):
        if used[i]:
            continue
        idxs = [p.indices]
        n, d = p.n.copy(), p.d
        used[i] = True
        for j in range(i + 1, len(planes)):
            if used[j]:
                continue
            q = planes[j]
            if abs(float(n @ q.n)) < n_dot:
                continue
            # same side / same offset, allowing opposite normal signs
            sign = 1.0 if float(n @ q.n) > 0 else -1.0
            if abs((d) - sign * q.d) > d_th:
                continue
            idxs.append(q.indices)
            used[j] = True
        all_idx = np.unique(np.concatenate(idxs))
        n, d = fit_plane_svd(pts[all_idx])
        mp = Plane(n=n, d=d, indices=all_idx)
        mp.flip_toward(pts.mean(axis=0))
        merged.append(mp)
    merged.sort(key=lambda x: len(x.indices), reverse=True)
    return merged


def pick_three_distinct(planes: list[Plane], min_dot: float) -> list[Plane]:
    """Keep planes whose normals are not almost parallel."""
    chosen = []
    for p in planes:
        if all(abs(float(p.n @ q.n)) < min_dot for q in chosen):
            chosen.append(p)
        if len(chosen) == 3:
            break
    return chosen


def reassign(pts: np.ndarray, nrm: np.ndarray, planes: list[Plane], dist_th: float, normal_th: float):
    labels = -np.ones(len(pts), dtype=int)
    if not planes:
        return labels
    dist = np.stack([np.abs(p.distance(pts)) for p in planes], axis=1)
    ndot = np.stack([np.abs(nrm @ p.n) for p in planes], axis=1)
    ok = (dist <= dist_th) & (ndot >= normal_th)
    dist_masked = np.where(ok, dist, np.inf)
    best = np.argmin(dist_masked, axis=1)
    finite = np.isfinite(dist_masked[np.arange(len(pts)), best])
    labels[finite] = best[finite]
    for i, p in enumerate(planes):
        p.indices = np.where(labels == i)[0]
        if p.indices.size >= 10:
            p.n, p.d = fit_plane_svd(pts[p.indices])
            p.flip_toward(pts.mean(axis=0))
    return labels


def strip_intersection_band(pts: np.ndarray, planes: list[Plane], labels: np.ndarray, band: float):
    """Points close to two planes at once sit on the weld and confuse both faces."""
    if len(planes) < 2:
        return labels
    dist = np.stack([np.abs(p.distance(pts)) for p in planes], axis=1)
    close = dist < band
    n_close = close.sum(axis=1)
    labels = labels.copy()
    labels[n_close >= 2] = -1
    for i, p in enumerate(planes):
        p.indices = np.where(labels == i)[0]
        if p.indices.size >= 10:
            p.n, p.d = fit_plane_svd(pts[p.indices])
            p.flip_toward(pts.mean(axis=0))
    return labels


def plane_intersection(p: Plane, q: Plane):
    d = np.cross(p.n, q.n)
    ln = np.linalg.norm(d)
    if ln < 1e-6:
        return None
    d = d / ln
    a = np.stack([p.n, q.n, d], axis=0)
    b = np.array([-p.d, -q.d, 0.0])
    try:
        p0 = np.linalg.solve(a, b)
    except np.linalg.LinAlgError:
        return None
    return p0, d


def three_plane_corner(planes: list[Plane]):
    if len(planes) != 3:
        return None
    a = np.stack([p.n for p in planes], axis=0)
    b = -np.array([p.d for p in planes])
    if abs(np.linalg.det(a)) < 1e-8:
        return None
    return np.linalg.solve(a, b)


def colorize(pcd, labels: np.ndarray) -> o3d.geometry.PointCloud:
    colors = np.array(
        [
            [0.90, 0.20, 0.20],
            [0.20, 0.75, 0.25],
            [0.20, 0.40, 0.95],
            [0.70, 0.70, 0.70],
        ]
    )
    out = copy.deepcopy(pcd)
    c = np.full((len(labels), 3), 0.55)
    for i in range(3):
        c[labels == i] = colors[i]
    c[labels < 0] = colors[3]
    out.colors = o3d.utility.Vector3dVector(c)
    return out


def preprocess(pcd, voxel: float, roi_z=None):
    if roi_z is not None:
        zmin, zmax = roi_z
        bbox = pcd.get_axis_aligned_bounding_box()
        minb = np.array(bbox.min_bound)
        maxb = np.array(bbox.max_bound)
        minb[2], maxb[2] = zmin, zmax
        pcd = pcd.crop(o3d.geometry.AxisAlignedBoundingBox(minb, maxb))
    if voxel > 0:
        pcd = pcd.voxel_down_sample(voxel)
    pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=1.8)
    return pcd


def split_ibutt_gap(pcd, plane: Plane, gap_width: float):
    """Last-resort split of two nearly coplanar plates across a groove/gap."""
    pts = np.asarray(pcd.points)
    # project onto the fitted plane, then along the in-plane axis of max spread
    n = plane.n
    # build an in-plane frame
    t1 = np.cross(n, np.array([1.0, 0.0, 0.0]))
    if np.linalg.norm(t1) < 1e-3:
        t1 = np.cross(n, np.array([0.0, 1.0, 0.0]))
    t1 /= np.linalg.norm(t1)
    t2 = np.cross(n, t1)
    uv = np.stack([pts @ t1, pts @ t2], axis=1)
    # histogram along the direction with a density valley
    best_axis, best_score, best_cut = 0, -1.0, 0.0
    for axis in (0, 1):
        vals = uv[:, axis]
        hist, edges = np.histogram(vals, bins=64)
        # ignore empty borders
        interior = hist[4:-4]
        if interior.size == 0:
            continue
        valley = int(np.argmin(interior)) + 4
        peak = max(hist.max(), 1)
        score = 1.0 - hist[valley] / peak
        if score > best_score:
            best_score, best_axis, best_cut = score, axis, 0.5 * (edges[valley] + edges[valley + 1])
    side = uv[:, best_axis] < best_cut
    left = np.where(side & (np.abs(uv[:, best_axis] - best_cut) > 0.5 * gap_width))[0]
    right = np.where((~side) & (np.abs(uv[:, best_axis] - best_cut) > 0.5 * gap_width))[0]
    return left, right, best_cut, best_axis


def run(args):
    pcd = o3d.io.read_point_cloud(args.input)
    if pcd.is_empty():
        raise SystemExit(f"empty or unreadable cloud: {args.input}")

    pcd = preprocess(pcd, args.voxel, None if args.zmin is None else (args.zmin, args.zmax))
    estimate_normals(pcd, radius=max(3.0 * args.voxel, 1e-3))
    pts = np.asarray(pcd.points)
    nrm = np.asarray(pcd.normals)

    if args.mode == "ibutt":
        plane = ransac_plane_normal_aware(pcd, args.dist, 0.80, args.iters, args.min_inliers)
        if plane is None:
            raise SystemExit("failed to fit the dominant plate plane")
        left, right, cut, axis = split_ibutt_gap(pcd, plane, args.gap)
        labels = -np.ones(len(pts), dtype=int)
        labels[left] = 0
        labels[right] = 1
        print(f"I-butt gap split along axis {axis}, cut={cut:.4f}")
        print(f"  plate0={left.size}  plate1={right.size}")
    else:
        raw = sequential_planes(
            pcd,
            k=args.candidates,
            dist_th=args.dist,
            normal_th=args.normal_th,
            n_iter=args.iters,
            min_inliers=args.min_inliers,
        )
        raw = merge_coplanar(raw, pts, n_dot=args.merge_n, d_th=args.merge_d)
        planes = pick_three_distinct(raw, min_dot=args.distinct_n)
        if len(planes) < 3:
            print(f"warning: only {len(planes)} distinct planes, check --dist / --normal-th / ROI")
        labels = reassign(pts, nrm, planes, dist_th=args.dist * 1.2, normal_th=args.normal_th)
        labels = strip_intersection_band(pts, planes, labels, band=args.band)

        print("planes (n, d, inliers):")
        for i, p in enumerate(planes):
            print(f"  [{i}] n={np.round(p.n, 4)}  d={p.d:.4f}  n={p.indices.size}")
        print("pairwise dihedral (deg):")
        for i in range(len(planes)):
            for j in range(i + 1, len(planes)):
                ang = np.degrees(np.arccos(np.clip(abs(float(planes[i].n @ planes[j].n)), 0, 1)))
                print(f"  {i}-{j}: {ang:.1f}")
                inter = plane_intersection(planes[i], planes[j])
                if inter:
                    print(f"       weld line dir={np.round(inter[1], 4)}  through={np.round(inter[0], 4)}")
        corner = three_plane_corner(planes)
        if corner is not None:
            print(f"three-plane corner (weld start): {np.round(corner, 4)}")

        for i, p in enumerate(planes):
            cloud = pcd.select_by_index(p.indices.tolist())
            o3d.io.write_point_cloud(f"{args.prefix}_plane{i}.ply", cloud)

    vis = colorize(pcd, labels)
    o3d.io.write_point_cloud(f"{args.prefix}_labeled.ply", vis)
    print(f"wrote {args.prefix}_labeled.ply and per-plane clouds")
    if args.show:
        o3d.visualization.draw_geometries([vis], window_name="three-plane split")


def main():
    p = argparse.ArgumentParser(description="Split three planes in a welding butt / fillet cloud")
    p.add_argument("--input", required=True, help="input .ply / .pcd")
    p.add_argument("--prefix", default="three_plane")
    p.add_argument("--mode", choices=["auto", "ibutt"], default="auto")
    p.add_argument("--voxel", type=float, default=0.001, help="voxel size in metres (1 mm)")
    p.add_argument("--dist", type=float, default=0.0012, help="RANSAC distance threshold (m)")
    p.add_argument("--normal-th", type=float, default=0.90, help="min |n·n_plane|, 0.90 ≈ 26 deg")
    p.add_argument("--iters", type=int, default=1000)
    p.add_argument("--min-inliers", type=int, default=200)
    p.add_argument("--candidates", type=int, default=6, help="extract extra planes, then merge")
    p.add_argument("--merge-n", type=float, default=0.985, help="coplanar merge: |n1·n2|")
    p.add_argument("--merge-d", type=float, default=0.003, help="coplanar merge: offset (m)")
    p.add_argument("--distinct-n", type=float, default=0.92, help="reject a 4th almost-parallel plane")
    p.add_argument("--band", type=float, default=0.0015, help="intersection dead-band (m)")
    p.add_argument("--gap", type=float, default=0.002, help="I-butt groove half-width (m)")
    p.add_argument("--zmin", type=float, default=None)
    p.add_argument("--zmax", type=float, default=None)
    p.add_argument("--show", action="store_true")
    run(p.parse_args())


if __name__ == "__main__":
    main()
