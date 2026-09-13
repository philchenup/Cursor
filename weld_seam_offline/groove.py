"""Groove detection: asymmetry feature + DBSCAN (demo_all.py)."""

from __future__ import annotations

import numpy as np
from scipy.spatial import cKDTree
from sklearn.cluster import DBSCAN


def normalize_feature(values: np.ndarray) -> np.ndarray:
    values = np.asarray(values, dtype=np.float64)
    span = values.max() - values.min()
    if span < 1e-12:
        return np.zeros_like(values)
    return (values - values.min()) / span


def asymmetry_feature(points: np.ndarray, normals: np.ndarray) -> np.ndarray:
    """Local normal deviation used by robotic-welding-demo / robotic-sealing."""
    pts = np.asarray(points, dtype=np.float64)
    nrm = np.asarray(normals, dtype=np.float64)
    tree = cKDTree(pts)
    neighbor = min(max(len(pts) // 100, 8), 30)
    neighbor = min(neighbor, len(pts))
    values = np.zeros(len(pts), dtype=np.float64)
    for i, point in enumerate(pts):
        _, idx = tree.query(point, k=neighbor)
        idx = np.atleast_1d(idx)
        vector = nrm[idx].mean(axis=0)
        self_n = nrm[i]
        proj = self_n * (np.dot(vector, self_n) / max(np.linalg.norm(self_n), 1e-12))
        values[i] = np.linalg.norm(vector - proj)
    return values


def _principal_axis(points: np.ndarray) -> np.ndarray:
    centered = points - points.mean(axis=0)
    _, _, vt = np.linalg.svd(centered, full_matrices=False)
    return vt[0]


def cluster_groove(points: np.ndarray, voxel_size: float, min_points: int = 10) -> np.ndarray:
    """DBSCAN the high-feature seeds, then merge collinear seam clusters.

    The original demo kept only the largest cluster. On a cropped PLY the seam
    is often split into a few aligned pieces, so those pieces are joined when
    they share the same principal direction.
    """
    pts = np.asarray(points, dtype=np.float64)
    if len(pts) == 0:
        raise RuntimeError("No candidate groove points to cluster")

    eps = 6.0 * voxel_size
    min_samples = min(min_points, max(len(pts) // 8, 3))
    labels = DBSCAN(eps=eps, min_samples=min_samples).fit(pts).labels_
    unique, counts = np.unique(labels, return_counts=True)
    valid = [int(label) for label in unique[np.argsort(counts)[::-1]] if label != -1]
    if not valid:
        if np.all(labels == -1):
            raise RuntimeError("Cannot find a valid groove cluster")
        return pts

    clusters = [pts[labels == label] for label in valid]
    # Prefer the longest thin cluster (a seam), not merely the densest blob.
    scores = [_seam_score(cluster) for cluster in clusters]
    seed = clusters[int(np.argmax(scores))]
    axis = _principal_axis(seed)
    merged = [seed]
    for cluster in clusters:
        if cluster is seed or len(cluster) < 3:
            continue
        direction = _principal_axis(cluster)
        if abs(float(np.dot(direction, axis))) < 0.85:
            continue
        if _mean_perp_distance(seed, cluster, axis) > 3.0 * voxel_size:
            continue
        gap = _cluster_gap(np.vstack(merged), cluster)
        if gap <= 8.0 * voxel_size:
            merged.append(cluster)
    return np.vstack(merged)


def _seam_score(cluster: np.ndarray) -> float:
    if len(cluster) < 3:
        return 0.0
    centered = cluster - cluster.mean(axis=0)
    try:
        _, singular, _ = np.linalg.svd(centered, full_matrices=False)
    except np.linalg.LinAlgError:
        return 0.0
    length = float(singular[0])
    thickness = float(max(singular[1], 1e-9))
    return length * (length / thickness)


def _mean_perp_distance(seed: np.ndarray, cluster: np.ndarray, axis: np.ndarray) -> float:
    origin = seed.mean(axis=0)
    rel = cluster.mean(axis=0) - origin
    proj = rel - np.dot(rel, axis) * axis
    return float(np.linalg.norm(proj))


def _cluster_gap(a: np.ndarray, b: np.ndarray) -> float:
    tree = cKDTree(a)
    dists, _ = tree.query(b, k=1)
    return float(np.min(dists))


def select_high_feature_points(
    points: np.ndarray,
    feature: np.ndarray,
    keep_ratio: float = 0.05,
    min_keep: int = 80,
    max_ratio: float = 0.25,
) -> np.ndarray:
    """Keep the strongest asymmetry points.

    demo_all.py used a fixed top 5%. RealSense scenes are dense; a cropped PLY
    can be only a few hundred voxels, so we also enforce a minimum seed count.
    """
    values = normalize_feature(feature)
    keep = int(len(points) * keep_ratio)
    keep = max(keep, min(min_keep, len(points)))
    keep = min(keep, int(len(points) * max_ratio), len(points))
    keep = max(keep, min(20, len(points)))
    idx = np.argsort(values)[-keep:]
    return points[idx]
