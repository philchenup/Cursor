"""Point-cloud helpers that replace the Open3D calls in demo_all.py."""

from __future__ import annotations

import numpy as np
from scipy.spatial import cKDTree


def finite_points(points: np.ndarray) -> np.ndarray:
    pts = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    return pts[np.isfinite(pts).all(axis=1)]


def infer_length_scale(points: np.ndarray) -> float:
    """1.0 for metre-scale clouds, 1000.0 when the bbox looks like millimetres."""
    pts = finite_points(points)
    extent = pts.max(axis=0) - pts.min(axis=0)
    return 1000.0 if float(extent.max()) > 20.0 else 1.0


def voxel_downsample(points: np.ndarray, voxel_size: float) -> np.ndarray:
    pts = finite_points(points)
    if len(pts) == 0:
        return pts
    keys = np.floor((pts - pts.min(axis=0)) / max(voxel_size, 1e-12)).astype(np.int64)
    _, index = np.unique(keys, axis=0, return_index=True)
    return pts[np.sort(index)]


def remove_statistical_outlier(
    points: np.ndarray, nb_neighbors: int = 20, std_ratio: float = 1.0
) -> np.ndarray:
    pts = finite_points(points)
    if len(pts) <= nb_neighbors:
        return pts
    tree = cKDTree(pts)
    dists, _ = tree.query(pts, k=nb_neighbors + 1)
    mean_d = dists[:, 1:].mean(axis=1)
    threshold = mean_d.mean() + std_ratio * mean_d.std()
    return pts[mean_d <= threshold]


def remove_radius_outlier(points: np.ndarray, nb_points: int, radius: float) -> np.ndarray:
    pts = finite_points(points)
    if len(pts) == 0:
        return pts
    tree = cKDTree(pts)
    keep = np.array([len(tree.query_ball_point(p, radius)) >= nb_points for p in pts])
    return pts[keep]


def estimate_normals(
    points: np.ndarray, radius: float, max_nn: int = 30, camera: np.ndarray | None = None
) -> np.ndarray:
    pts = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    normals = np.zeros_like(pts)
    if len(pts) == 0:
        return normals

    tree = cKDTree(pts)
    k = min(max_nn, len(pts))
    for i, point in enumerate(pts):
        idx = tree.query_ball_point(point, radius)
        if len(idx) < 3:
            _, idx = tree.query(point, k=k)
            idx = np.atleast_1d(idx)
        neighborhood = pts[np.asarray(idx, dtype=int)]
        centered = neighborhood - neighborhood.mean(axis=0)
        cov = centered.T @ centered / max(len(neighborhood), 1)
        _, vecs = np.linalg.eigh(cov)
        normals[i] = vecs[:, 0]

    norms = np.linalg.norm(normals, axis=1, keepdims=True)
    normals = normals / np.clip(norms, 1e-12, None)

    if camera is None:
        camera = pts.mean(axis=0) + np.array([0.0, 0.0, 1.0]) * (
            np.linalg.norm(pts.max(axis=0) - pts.min(axis=0)) + 1e-6
        )
    to_camera = np.asarray(camera, dtype=np.float64).reshape(1, 3) - pts
    flip = np.sum(normals * to_camera, axis=1) < 0.0
    normals[flip] *= -1.0
    return normals


def ransac_plane(
    points: np.ndarray,
    distance_threshold: float = 0.003,
    ransac_n: int = 20,
    num_iterations: int = 100,
    rng: np.random.Generator | None = None,
) -> np.ndarray:
    pts = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    if len(pts) < 3:
        return np.array([0.0, 0.0, 1.0, 0.0])

    rng = rng or np.random.default_rng(0)
    best_model = np.array([0.0, 0.0, 1.0, -float(pts[:, 2].mean())])
    best_count = -1
    n_sample = min(max(ransac_n, 3), len(pts))

    for _ in range(num_iterations):
        sample = pts[rng.choice(len(pts), size=n_sample, replace=False)]
        centered = sample - sample.mean(axis=0)
        try:
            _, _, vt = np.linalg.svd(centered, full_matrices=False)
        except np.linalg.LinAlgError:
            continue
        normal = vt[-1]
        norm = np.linalg.norm(normal)
        if norm < 1e-12:
            continue
        normal = normal / norm
        d = -float(normal @ sample.mean(axis=0))
        dist = np.abs(pts @ normal + d)
        count = int(np.count_nonzero(dist < distance_threshold))
        if count > best_count:
            best_count = count
            best_model = np.append(normal, d)
    return best_model


def angle_deg(a: np.ndarray, b: np.ndarray) -> float:
    na = np.linalg.norm(a)
    nb = np.linalg.norm(b)
    if na < 1e-12 or nb < 1e-12:
        return 180.0
    cos = float(np.clip(np.dot(a, b) / (na * nb), -1.0, 1.0))
    return float(np.degrees(np.arccos(cos)))
