"""T 型板焊缝定位。

俯视点云里，中间焊缝和两侧焊趾都由板件几何决定，不依赖坡口里有没有点：

1. 去掉与图像四角同色的无回波背景。
2. 底板是前景里的主色，其余连通域是板件。
3. 只配对长边近似平行、中间留有窄缝、并且沿缝方向有足够重叠的两块板。
4. 沿焊缝方向分箱，取相向一侧的高分位棱边。缝宽用中位数，被缺口或飞溅带偏的箱子用 MAD 丢掉。
5. 两条焊趾是两侧棱边，中间焊缝是它们的中线。端点取两块板沿缝方向的重叠段。

立板本身有点时走 ``tjoint_seams_3d``：RANSAC 抽出翼板，把立板投影到翼板上，
再用同一套分箱包络得到中心线和两条角焊缝。两面都有点时，点数多的那一面不会把中心线拉偏。
只有一面有点时，结果是这一面的焊趾，另一侧要用已知板厚再偏置。
"""

from __future__ import annotations

import argparse
import json
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

_FONT = "/usr/share/fonts/truetype/wqy/wqy-microhei.ttc"


@dataclass
class Seam2D:
    """图像平面上的一条接头。坐标原点在左上角，x 向右，y 向下，单位是像素。"""

    centerline: np.ndarray  # (2, 2)，起点到终点
    toe_a: np.ndarray
    toe_b: np.ndarray
    direction: np.ndarray  # 单位向量，y 分量 >= 0
    normal: np.ndarray  # 从 toe_a 指向 toe_b
    width: float
    length: float
    parallel_angle_deg: float
    gap_mad: float
    edge_std: float
    side_a_rgb: tuple[int, int, int] | None = None
    side_b_rgb: tuple[int, int, int] | None = None

    @property
    def direction_deg(self) -> float:
        """与 +x 轴的夹角，y 向下，90° 为竖直向下。"""
        return float(np.degrees(np.arctan2(self.direction[1], self.direction[0])))

    def to_dict(self) -> dict:
        data = {
            "centerline": np.round(self.centerline, 2).tolist(),
            "toe_a": np.round(self.toe_a, 2).tolist(),
            "toe_b": np.round(self.toe_b, 2).tolist(),
            "direction": np.round(self.direction, 6).tolist(),
            "direction_deg": round(self.direction_deg, 2),
            "width_px": round(self.width, 2),
            "length_px": round(self.length, 2),
            "parallel_angle_deg": round(self.parallel_angle_deg, 3),
            "gap_mad_px": round(self.gap_mad, 3),
            "edge_std_px": round(self.edge_std, 3),
        }
        if self.side_a_rgb is not None:
            data["side_a_rgb"] = list(self.side_a_rgb)
            data["side_b_rgb"] = list(self.side_b_rgb)
        return data


@dataclass
class Seam3D:
    """翼板平面上的 T 型接头，单位与输入点云一致。"""

    centerline: np.ndarray  # (2, 3)
    toe_a: np.ndarray
    toe_b: np.ndarray
    direction: np.ndarray
    width: float
    length: float
    flange_normal: np.ndarray

    def to_dict(self) -> dict:
        return {
            "centerline": np.round(self.centerline, 4).tolist(),
            "toe_a": np.round(self.toe_a, 4).tolist(),
            "toe_b": np.round(self.toe_b, 4).tolist(),
            "direction": np.round(self.direction, 6).tolist(),
            "width": round(self.width, 4),
            "length": round(self.length, 4),
            "flange_normal": np.round(self.flange_normal, 6).tolist(),
        }


def _unit(vector: np.ndarray) -> np.ndarray:
    norm = float(np.linalg.norm(vector))
    if norm < 1e-12:
        raise ValueError("零向量不能归一化")
    return vector / norm


def _pca_axes(points: np.ndarray) -> tuple[np.ndarray, np.ndarray, float, float]:
    center = points.mean(axis=0)
    centered = points - center
    covariance = np.cov(centered.T)
    eigenvalues, eigenvectors = np.linalg.eigh(covariance)
    major = eigenvectors[:, int(np.argmax(eigenvalues))]
    return center, _unit(major), float(eigenvalues.max()), float(eigenvalues.min())


def _parallel_angle_deg(first: np.ndarray, second: np.ndarray) -> float:
    cosine = float(np.clip(abs(np.dot(_unit(first), _unit(second))), 0.0, 1.0))
    return float(np.degrees(np.arccos(cosine)))


def _orient_down(direction: np.ndarray) -> np.ndarray:
    if direction[1] < 0 or (abs(direction[1]) < 1e-8 and direction[0] < 0):
        return -direction
    return direction


def _binned_edges(
    along: np.ndarray,
    across: np.ndarray,
    t_lo: float,
    t_hi: float,
    bin_size: float,
    edge_percentile: float,
    min_bin_points: int,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """每个箱子里沿缝方向的低侧、高侧棱边。返回 (bin_center, low, high)。"""
    if t_hi - t_lo <= bin_size:
        return np.empty(0), np.empty(0), np.empty(0)
    edges = np.arange(t_lo, t_hi + bin_size, bin_size)
    centers: list[float] = []
    low: list[float] = []
    high: list[float] = []
    low_q = 100.0 - edge_percentile
    for start, stop in zip(edges[:-1], edges[1:]):
        chosen = (along >= start) & (along < stop)
        if int(chosen.sum()) < min_bin_points:
            continue
        values = across[chosen]
        centers.append(0.5 * (start + stop))
        low.append(float(np.percentile(values, low_q)))
        high.append(float(np.percentile(values, edge_percentile)))
    if not centers:
        return np.empty(0), np.empty(0), np.empty(0)
    return np.asarray(centers), np.asarray(low), np.asarray(high)


def _mad_keep(values: np.ndarray, floor: float) -> np.ndarray:
    median = float(np.median(values))
    mad = float(np.median(np.abs(values - median))) + 1e-9
    limit = max(floor, 3.0 * 1.4826 * mad)
    return np.abs(values - median) <= limit


def fit_two_plate_seam(
    points_a: np.ndarray,
    points_b: np.ndarray,
    *,
    bin_size: float = 8.0,
    edge_percentile: float = 99.5,
    min_bin_points: int = 30,
    max_parallel_deg: float = 15.0,
    min_gap: float = 3.0,
    max_gap_to_width: float = 1.25,
    trim_percentile: float = 1.0,
    max_gap_mad_ratio: float = 0.08,
) -> Seam2D | None:
    """由两块板的点求中间焊缝和两条焊趾。几何不合格时返回 None。"""
    if len(points_a) < 50 or len(points_b) < 50:
        return None

    center_a, major_a, _, _ = _pca_axes(points_a)
    center_b, major_b, _, _ = _pca_axes(points_b)
    parallel_angle = _parallel_angle_deg(major_a, major_b)
    if parallel_angle > max_parallel_deg:
        return None

    same_way = 1.0 if float(np.dot(major_a, major_b)) >= 0.0 else -1.0
    direction = _orient_down(_unit(major_a + same_way * major_b))
    normal = np.array([-direction[1], direction[0]], dtype=np.float64)
    if float(np.dot(center_b - center_a, normal)) < 0:
        normal = -normal

    origin = 0.5 * (center_a + center_b)
    along_a = (points_a - origin) @ direction
    across_a = (points_a - origin) @ normal
    along_b = (points_b - origin) @ direction
    across_b = (points_b - origin) @ normal

    # 两块板应当隔缝相对，而不是首尾相接。
    separation = center_b - center_a
    if abs(float(np.dot(separation, direction))) > abs(float(np.dot(separation, normal))):
        return None

    t_lo = max(
        float(np.percentile(along_a, trim_percentile)),
        float(np.percentile(along_b, trim_percentile)),
    )
    t_hi = min(
        float(np.percentile(along_a, 100.0 - trim_percentile)),
        float(np.percentile(along_b, 100.0 - trim_percentile)),
    )
    length_a = float(np.percentile(along_a, 99) - np.percentile(along_a, 1))
    length_b = float(np.percentile(along_b, 99) - np.percentile(along_b, 1))
    if t_hi - t_lo < 0.25 * min(length_a, length_b):
        return None

    bin_t, edge_a, edge_b = _facing_edges(
        along_a,
        across_a,
        along_b,
        across_b,
        t_lo,
        t_hi,
        bin_size,
        edge_percentile,
        min_bin_points,
    )
    if len(bin_t) < 5:
        return None

    gap = edge_b - edge_a
    keep = _mad_keep(gap, floor=4.0)
    if int(keep.sum()) < 5:
        return None
    gap_mad = float(np.median(np.abs(gap[keep] - np.median(gap[keep]))))
    toe_a_s = float(np.median(edge_a[keep]))
    toe_b_s = float(np.median(edge_b[keep]))
    width = toe_b_s - toe_a_s
    width_a = float(np.percentile(across_a, 99) - np.percentile(across_a, 1))
    width_b = float(np.percentile(across_b, 99) - np.percentile(across_b, 1))
    if width < min_gap or width > max_gap_to_width * min(width_a, width_b):
        return None
    if gap_mad > max(2.0, max_gap_mad_ratio * width):
        return None

    edge_std = float(
        np.hypot(np.std(edge_a[keep] - toe_a_s), np.std(edge_b[keep] - toe_b_s))
    )
    mid_s = 0.5 * (toe_a_s + toe_b_s)

    def point_at(along: float, across: float) -> np.ndarray:
        return origin + along * direction + across * normal

    return Seam2D(
        centerline=np.stack([point_at(t_lo, mid_s), point_at(t_hi, mid_s)]),
        toe_a=np.stack([point_at(t_lo, toe_a_s), point_at(t_hi, toe_a_s)]),
        toe_b=np.stack([point_at(t_lo, toe_b_s), point_at(t_hi, toe_b_s)]),
        direction=direction,
        normal=normal,
        width=float(width),
        length=float(t_hi - t_lo),
        parallel_angle_deg=parallel_angle,
        gap_mad=gap_mad,
        edge_std=edge_std,
    )


def _facing_edges(
    along_a: np.ndarray,
    across_a: np.ndarray,
    along_b: np.ndarray,
    across_b: np.ndarray,
    t_lo: float,
    t_hi: float,
    bin_size: float,
    edge_percentile: float,
    min_bin_points: int,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """A 在法向负侧，相向棱边是高分位；B 的相向棱边是低分位。"""
    edges = np.arange(t_lo, t_hi + bin_size, bin_size)
    centers: list[float] = []
    edge_a: list[float] = []
    edge_b: list[float] = []
    low_q = 100.0 - edge_percentile
    for start, stop in zip(edges[:-1], edges[1:]):
        chosen_a = (along_a >= start) & (along_a < stop)
        chosen_b = (along_b >= start) & (along_b < stop)
        if int(chosen_a.sum()) < min_bin_points or int(chosen_b.sum()) < min_bin_points:
            continue
        centers.append(0.5 * (start + stop))
        edge_a.append(float(np.percentile(across_a[chosen_a], edge_percentile)))
        edge_b.append(float(np.percentile(across_b[chosen_b], low_q)))
    if not centers:
        return np.empty(0), np.empty(0), np.empty(0)
    return np.asarray(centers), np.asarray(edge_a), np.asarray(edge_b)


def fit_strip_seam(
    points: np.ndarray,
    support: np.ndarray | None = None,
    *,
    bin_size: float = 4.0,
    edge_percentile: float = 99.5,
    min_bin_points: int = 8,
    trim_percentile: float = 1.0,
    min_aspect: float = 2.0,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, float, float] | None:
    """一条细长点带的两侧边缘和中线。返回 (center, toe_lo, toe_hi, direction, width, length)。"""
    if len(points) < 30:
        return None
    center, major, major_var, minor_var = _pca_axes(points)
    if minor_var <= 1e-9 or np.sqrt(major_var / minor_var) < min_aspect:
        return None
    direction = _orient_down(major)
    normal = np.array([-direction[1], direction[0]], dtype=np.float64)
    along = (points - center) @ direction
    across = (points - center) @ normal
    t_lo = float(np.percentile(along, trim_percentile))
    t_hi = float(np.percentile(along, 100.0 - trim_percentile))
    if support is not None and len(support) > 10:
        support_along = (support - center) @ direction
        t_lo = max(t_lo, float(np.percentile(support_along, trim_percentile)))
        t_hi = min(t_hi, float(np.percentile(support_along, 100.0 - trim_percentile)))
    bin_t, low, high = _binned_edges(
        along, across, t_lo, t_hi, bin_size, edge_percentile, min_bin_points
    )
    if len(bin_t) < 5:
        return None
    width_each = high - low
    keep = _mad_keep(width_each, floor=1.0)
    if int(keep.sum()) < 5:
        return None
    toe_lo = float(np.median(low[keep]))
    toe_hi = float(np.median(high[keep]))
    mid = 0.5 * (toe_lo + toe_hi)

    def point_at(along_value: float, across_value: float) -> np.ndarray:
        return center + along_value * direction + across_value * normal

    return (
        np.stack([point_at(t_lo, mid), point_at(t_hi, mid)]),
        np.stack([point_at(t_lo, toe_lo), point_at(t_hi, toe_lo)]),
        np.stack([point_at(t_lo, toe_hi), point_at(t_hi, toe_hi)]),
        direction,
        float(toe_hi - toe_lo),
        float(t_hi - t_lo),
    )


def _plane_from_three(p0: np.ndarray, p1: np.ndarray, p2: np.ndarray) -> np.ndarray | None:
    normal = np.cross(p1 - p0, p2 - p0)
    norm = float(np.linalg.norm(normal))
    if norm < 1e-9:
        return None
    return normal / norm


def _fit_plane_svd(points: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    center = points.mean(axis=0)
    _, _, vh = np.linalg.svd(points - center, full_matrices=False)
    normal = _unit(vh[-1])
    if normal[2] < 0:
        normal = -normal
    return center, normal


def ransac_plane(
    points: np.ndarray,
    *,
    threshold: float = 0.4,
    iterations: int = 250,
    seed: int = 0,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """确定性 RANSAC 平面。返回 (中心, 法向, 内点掩码)，法向尽量朝 +z。"""
    if len(points) < 3:
        raise ValueError("拟合平面至少需要 3 个点")
    rng = np.random.default_rng(seed)
    best_inliers: np.ndarray | None = None
    best_count = 0
    for _ in range(iterations):
        sample = points[rng.choice(len(points), 3, replace=False)]
        normal = _plane_from_three(sample[0], sample[1], sample[2])
        if normal is None:
            continue
        distance = np.abs((points - sample[0]) @ normal)
        inliers = distance <= threshold
        count = int(inliers.sum())
        if count > best_count:
            best_count = count
            best_inliers = inliers
    if best_inliers is None or best_count < 3:
        center, normal = _fit_plane_svd(points)
        distance = np.abs((points - center) @ normal)
        return center, normal, distance <= threshold
    center, normal = _fit_plane_svd(points[best_inliers])
    distance = np.abs((points - center) @ normal)
    return center, normal, distance <= threshold


def _plane_basis(normal: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    helper = np.array([1.0, 0.0, 0.0]) if abs(normal[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
    axis_u = _unit(np.cross(normal, helper))
    axis_v = _unit(np.cross(normal, axis_u))
    return axis_u, axis_v


def tjoint_seams_3d(
    points: np.ndarray,
    *,
    plane_threshold: float = 0.4,
    web_height: float = 1.5,
    ransac_iterations: int = 250,
    seed: int = 0,
    bin_size: float = 2.0,
) -> Seam3D | None:
    """立板有点时，求翼板平面上的中间焊缝和两条角焊缝。

    立板点先投影到翼板，再取投影带两侧的稳健边缘。边缘由分箱高分位决定，
    所以一面点多、一面点少时中心线不会跟着质心漂。
    """
    points = np.asarray(points, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3 or len(points) < 50:
        return None
    center, normal, inliers = ransac_plane(
        points, threshold=plane_threshold, iterations=ransac_iterations, seed=seed
    )
    signed = (points - center) @ normal
    web = points[(~inliers) & (np.abs(signed) >= web_height)]
    if len(web) < 40:
        return None
    axis_u, axis_v = _plane_basis(normal)
    web_2d = np.stack([(web - center) @ axis_u, (web - center) @ axis_v], axis=1)
    flange = points[inliers]
    flange_2d = np.stack([(flange - center) @ axis_u, (flange - center) @ axis_v], axis=1)
    fitted = fit_strip_seam(web_2d, support=flange_2d, bin_size=bin_size)
    if fitted is None:
        return None
    centerline, toe_a, toe_b, direction_2d, width, length = fitted

    def lift(segment: np.ndarray) -> np.ndarray:
        return center + segment[:, 0:1] * axis_u + segment[:, 1:2] * axis_v

    direction = _unit(lift(np.stack([[0.0, 0.0], direction_2d]))[1] - center)
    return Seam3D(
        centerline=lift(centerline),
        toe_a=lift(toe_a),
        toe_b=lift(toe_b),
        direction=direction,
        width=width,
        length=length,
        flange_normal=normal,
    )


def _backdrop_mask(rgb: np.ndarray, threshold: float) -> np.ndarray:
    corners = np.stack(
        [rgb[0, 0], rgb[0, -1], rgb[-1, 0], rgb[-1, -1]],
        axis=0,
    ).astype(np.float32)
    backdrop = corners[int(np.argmin(corners.max(axis=1)))]
    return np.linalg.norm(rgb.astype(np.float32) - backdrop, axis=2) < threshold


def segment_plates(
    rgb: np.ndarray,
    *,
    backdrop_threshold: float = 40.0,
    min_aspect: float = 2.0,
    min_area_frac: float = 0.008,
    open_radius: int = 2,
    close_radius: int = 5,
) -> list[dict]:
    """从俯视彩色点云图里分割板件。闭运算核必须小于坡口宽度，否则两块板会粘在一起。"""
    rgb = np.asarray(rgb)
    if rgb.ndim != 3 or rgb.shape[2] < 3:
        raise ValueError("需要 RGB 图像")
    rgb = rgb[:, :, :3]
    foreground = ~_backdrop_mask(rgb, backdrop_threshold)
    if int(foreground.sum()) < 100:
        return []

    lab = cv2.cvtColor(rgb, cv2.COLOR_RGB2LAB).astype(np.float32)
    base_color = np.median(lab[foreground], axis=0)
    distance = np.linalg.norm(lab - base_color, axis=2)
    distance_u8 = np.clip(distance, 0, 255).astype(np.uint8)
    threshold, _ = cv2.threshold(
        distance_u8[foreground], 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU
    )
    plates = ((distance > threshold) & foreground).astype(np.uint8) * 255
    base_count = int((foreground & (distance <= threshold)).sum())
    if base_count < int(foreground.sum()) * 0.4:
        raise ValueError("底板不是视场主色，拒绝用颜色中值分割")

    open_kernel = cv2.getStructuringElement(
        cv2.MORPH_ELLIPSE, (open_radius * 2 + 1, open_radius * 2 + 1)
    )
    close_kernel = cv2.getStructuringElement(
        cv2.MORPH_ELLIPSE, (close_radius * 2 + 1, close_radius * 2 + 1)
    )
    plates = cv2.morphologyEx(plates, cv2.MORPH_OPEN, open_kernel)
    plates = cv2.morphologyEx(plates, cv2.MORPH_CLOSE, close_kernel)

    count, labels, stats, _ = cv2.connectedComponentsWithStats(plates, 8)
    min_area = max(500, int(min_area_frac * foreground.sum()))
    found: list[dict] = []
    for index in range(1, count):
        area = int(stats[index, cv2.CC_STAT_AREA])
        if area < min_area:
            continue
        ys, xs = np.where(labels == index)
        points = np.stack([xs, ys], axis=1).astype(np.float64)
        _, _, major_var, minor_var = _pca_axes(points)
        if minor_var <= 1e-9 or np.sqrt(major_var / minor_var) < min_aspect:
            continue
        color = rgb[ys, xs].mean(axis=0)
        found.append(
            {
                "points": points,
                "rgb": (int(round(color[0])), int(round(color[1])), int(round(color[2]))),
            }
        )
    return found


def find_seams_in_image(rgb: np.ndarray, **segment_kwargs) -> list[Seam2D]:
    """分割板件并配对，返回按中点 x 从小到大排序的接头。"""
    plates = segment_plates(rgb, **segment_kwargs)
    candidates: list[tuple[float, int, int, Seam2D]] = []
    for i in range(len(plates)):
        for j in range(i + 1, len(plates)):
            seam = fit_two_plate_seam(plates[i]["points"], plates[j]["points"])
            if seam is None:
                continue
            seam.side_a_rgb = plates[i]["rgb"]
            seam.side_b_rgb = plates[j]["rgb"]
            candidates.append((seam.width, i, j, seam))
    candidates.sort(key=lambda item: item[0])
    used: set[int] = set()
    seams: list[Seam2D] = []
    for _, i, j, seam in candidates:
        if i in used or j in used:
            continue
        used.add(i)
        used.add(j)
        seams.append(seam)
    seams.sort(key=lambda seam: float(seam.centerline.mean(axis=0)[0]))
    return seams


def _draw_segment(image: np.ndarray, segment: np.ndarray, color: tuple[int, int, int], thickness: int) -> None:
    start = tuple(int(round(v)) for v in segment[0])
    end = tuple(int(round(v)) for v in segment[1])
    cv2.line(image, start, end, color, thickness, cv2.LINE_AA)
    for point in (start, end):
        cv2.circle(image, point, 7, color, -1, cv2.LINE_AA)
        cv2.circle(image, point, 7, (255, 255, 255), 1, cv2.LINE_AA)


def draw_seams(rgb: np.ndarray, seams: list[Seam2D]) -> np.ndarray:
    """把中间焊缝画成红色，两侧焊趾画成橙色和青色。"""
    canvas = cv2.cvtColor(rgb[:, :, :3].copy(), cv2.COLOR_RGB2BGR)
    for seam in seams:
        _draw_segment(canvas, seam.toe_a, (0, 140, 255), 2)
        _draw_segment(canvas, seam.toe_b, (255, 220, 0), 2)
        _draw_segment(canvas, seam.centerline, (40, 40, 255), 3)
    rgb_out = cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB)
    try:
        from PIL import Image, ImageDraw, ImageFont

        image = Image.fromarray(rgb_out)
        draw = ImageDraw.Draw(image)
        font = ImageFont.truetype(_FONT, 28)
        small = ImageFont.truetype(_FONT, 22)
        draw.rectangle((16, 16, 430, 132), fill=(20, 20, 20))
        draw.line((32, 48, 92, 48), fill=(255, 40, 40), width=4)
        draw.text((104, 32), "中间焊缝", font=font, fill=(255, 255, 255))
        draw.line((32, 88, 92, 88), fill=(255, 140, 0), width=4)
        draw.line((32, 108, 92, 108), fill=(0, 210, 255), width=4)
        draw.text((104, 82), "两侧焊趾", font=small, fill=(255, 255, 255))
        return np.array(image)
    except (ImportError, OSError):
        return rgb_out


def _load_rgb(path: Path) -> np.ndarray:
    image = cv2.imread(str(path), cv2.IMREAD_UNCHANGED)
    if image is None:
        raise FileNotFoundError(path)
    if image.ndim == 2:
        image = cv2.cvtColor(image, cv2.COLOR_GRAY2RGB)
    elif image.shape[2] == 4:
        image = cv2.cvtColor(image, cv2.COLOR_BGRA2RGB)
    else:
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    return image


def main() -> None:
    parser = argparse.ArgumentParser(description="定位俯视点云中 T 型板的中间焊缝和两侧焊趾")
    parser.add_argument("image", type=Path, help="俯视点云渲染图")
    parser.add_argument("-o", "--output", type=Path, help="标注图输出路径")
    parser.add_argument("--json", type=Path, help="焊缝坐标 JSON 输出路径")
    args = parser.parse_args()

    rgb = _load_rgb(args.image)
    seams = find_seams_in_image(rgb)
    payload = {
        "image": str(args.image),
        "width": int(rgb.shape[1]),
        "height": int(rgb.shape[0]),
        "coordinate_system": "像素，原点在左上角，x 向右，y 向下",
        "seams": [seam.to_dict() for seam in seams],
    }
    text = json.dumps(payload, ensure_ascii=False, indent=2)
    print(text)
    if args.json:
        args.json.write_text(text + "\n", encoding="utf-8")
    if args.output:
        drawn = draw_seams(rgb, seams)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(args.output), cv2.cvtColor(drawn, cv2.COLOR_RGB2BGR))
