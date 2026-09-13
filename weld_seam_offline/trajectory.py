"""Centerline thinning, ordering, spline, and torch orientation from demo_all.py."""

from __future__ import annotations

import numpy as np
from scipy import interpolate
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation as R

from .geometry import angle_deg, estimate_normals, ransac_plane


def thin_line(
    points: np.ndarray, point_cloud_thickness: float = 0.5
) -> tuple[np.ndarray, list[np.ndarray]]:
    """Project each point onto the local 3D PCA line (Lee / curve-extraction)."""
    pts = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    tree = cKDTree(pts)
    thinned = np.zeros_like(pts)
    lines: list[np.ndarray] = []
    for i, point in enumerate(pts):
        idx = tree.query_ball_point(point, point_cloud_thickness)
        if len(idx) < 2:
            _, idx = tree.query(point, k=min(8, len(pts)))
            idx = np.atleast_1d(idx)
        neighborhood = pts[np.asarray(idx, dtype=int)]
        mean = neighborhood.mean(axis=0)
        centered = neighborhood - mean
        try:
            _, _, vt = np.linalg.svd(centered, full_matrices=False)
            direction = vt[0]
        except np.linalg.LinAlgError:
            direction = np.array([1.0, 0.0, 0.0])
        line = np.vstack((mean - direction, mean + direction))
        lines.append(line)
        ap = point - line[0]
        ab = line[1] - line[0]
        denom = float(np.dot(ab, ab))
        thinned[i] = line[0] + (np.dot(ap, ab) / denom) * ab if denom > 1e-12 else point
    return thinned, lines


def sort_points(
    points: np.ndarray,
    regression_lines: list[np.ndarray],
    sorted_point_distance: float = 0.02,
) -> np.ndarray:
    """Walk along local regression directions from the seed point, both ways."""
    pts = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    if len(pts) <= 2:
        return pts.copy()

    tree = cKDTree(pts)

    def walk(direction_sign: int) -> list[int]:
        visited = {0}
        current = 0
        prev = regression_lines[0][1] - regression_lines[0][0]
        order: list[int] = []
        for _ in range(len(pts)):
            vec = regression_lines[current][1] - regression_lines[current][0]
            if np.dot(prev, vec) < 0:
                vec = -vec
            prev = vec
            norm = np.linalg.norm(vec)
            if norm < 1e-12:
                break
            target = pts[current] + direction_sign * (vec / norm) * sorted_point_distance
            radius = sorted_point_distance / (1.5 if direction_sign > 0 else 3.0)
            idx = tree.query_ball_point(target, radius)
            idx = [i for i in idx if i not in visited]
            if not idx:
                break
            target_vec = target - pts[current]
            nearest = min(idx, key=lambda i: angle_deg(target_vec, pts[i] - pts[current]))
            visited.add(nearest)
            order.append(nearest)
            current = nearest
        return order

    left = [0] + walk(+1)
    right = walk(-1)
    combined = right[::-1] + left
    unique: list[int] = []
    seen: set[int] = set()
    for i in combined:
        if i not in seen:
            unique.append(i)
            seen.add(i)
    ordered = pts[unique]
    if len(ordered) < 3:
        # Fallback: sort along the global principal axis of the groove.
        mean = pts.mean(axis=0)
        _, _, vt = np.linalg.svd(pts - mean, full_matrices=False)
        order = np.argsort(pts @ vt[0])
        ordered = pts[order]
    return ordered


def fit_spline(points: np.ndarray, samples_per_point: int = 2) -> np.ndarray:
    pts = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    # Drop near-duplicates so splprep stays stable.
    keep = [0]
    for i in range(1, len(pts)):
        if np.linalg.norm(pts[i] - pts[keep[-1]]) > 1e-8:
            keep.append(i)
    pts = pts[keep]
    if len(pts) < 2:
        return pts
    if len(pts) < 4:
        t = np.linspace(0.0, 1.0, max(len(pts) * samples_per_point, 8))
        return np.column_stack([np.interp(t, np.linspace(0.0, 1.0, len(pts)), pts[:, j]) for j in range(3)])

    k = min(3, len(pts) - 1)
    try:
        tck, _ = interpolate.splprep([pts[:, 0], pts[:, 1], pts[:, 2]], s=float("inf"), k=k)
        u = np.linspace(0.0, 1.0, max(len(pts) * samples_per_point, 16))
        xyz = interpolate.splev(u, tck)
        return np.vstack(xyz).T
    except (TypeError, ValueError):
        return pts


def generate_trajectory(
    groove_points: np.ndarray, thickness: float, sort_distance: float
) -> np.ndarray:
    thinned, lines = thin_line(groove_points, point_cloud_thickness=thickness)
    ordered = sort_points(thinned, lines, sorted_point_distance=sort_distance)
    return fit_spline(ordered)


def trajectory_normal(trajectory: np.ndarray, cloud: np.ndarray, scale: float = 1.0) -> np.ndarray:
    """Average trajectory normal, oriented against the dominant plane (find_normal)."""
    plane = ransac_plane(
        cloud,
        distance_threshold=0.003 * scale,
        ransac_n=min(20, len(cloud)),
        num_iterations=80,
    )
    plane_normal = plane[:3]
    plane_normal = plane_normal / max(np.linalg.norm(plane_normal), 1e-12)

    merged = np.vstack((trajectory, cloud))
    normals = estimate_normals(merged, radius=0.02 * scale, max_nn=80, camera=None)
    # Orient like Open3D orient_normals_to_align_with_direction(-plane_normal).
    if np.dot(normals.mean(axis=0), -plane_normal) < 0:
        normals = -normals
    traj_normals = normals[: len(trajectory)]
    normal = traj_normals.mean(axis=0)
    nrm = np.linalg.norm(normal)
    if nrm < 1e-12:
        normal = -plane_normal
        nrm = np.linalg.norm(normal)
    return normal / nrm


def find_orientation(trajectory: np.ndarray, normal: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Build a right-handed torch frame: X along seam, Z along surface normal."""
    points = np.asarray(trajectory, dtype=np.float64).reshape(-1, 3)
    z_dir = np.asarray(normal, dtype=np.float64)
    z_dir = z_dir / max(np.linalg.norm(z_dir), 1e-12)

    rotvecs = []
    quats = []
    for i in range(len(points)):
        if i + 1 < len(points):
            pos_diff = points[i + 1] - points[i]
        else:
            pos_diff = points[i] - points[i - 1]
        x_dir = pos_diff - np.dot(pos_diff, z_dir) * z_dir
        if np.linalg.norm(x_dir) < 1e-9:
            fallback = np.array([1.0, 0.0, 0.0])
            x_dir = fallback - np.dot(fallback, z_dir) * z_dir
        x_dir = x_dir / np.linalg.norm(x_dir)
        y_dir = np.cross(z_dir, x_dir)
        y_dir = y_dir / max(np.linalg.norm(y_dir), 1e-12)
        rotation = R.from_matrix(np.vstack((x_dir, y_dir, z_dir)).T)
        rotvecs.append(rotation.as_rotvec())
        quats.append(rotation.as_quat())  # x, y, z, w
    return points, np.asarray(rotvecs), np.asarray(quats)


def poses_to_table(points: np.ndarray, rotvecs: np.ndarray, quats: np.ndarray) -> np.ndarray:
    return np.hstack((points, rotvecs, quats))
