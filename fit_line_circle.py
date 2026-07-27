"""
使用 Open3D 生成直线/圆点云，拟合直线与圆，并可视化结果。

用法:
  python fit_line_circle.py              # 交互可视化（需要显示器）
  python fit_line_circle.py --no-show    # 仅拟合并保存结果图（适合无界面环境）
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import open3d as o3d


# ---------------------------------------------------------------------------
# 点云生成
# ---------------------------------------------------------------------------

def generate_line_point_cloud(
    point_on_line: np.ndarray,
    direction: np.ndarray,
    n_points: int = 300,
    length: float = 4.0,
    noise_std: float = 0.05,
    seed: int = 0,
) -> o3d.geometry.PointCloud:
    """沿一条直线采样，并叠加高斯噪声，得到线状点云。"""
    rng = np.random.default_rng(seed)
    direction = direction / np.linalg.norm(direction)
    t = rng.uniform(-length / 2, length / 2, size=n_points)
    points = point_on_line + t[:, None] * direction + rng.normal(0.0, noise_std, size=(n_points, 3))

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    pcd.paint_uniform_color([0.2, 0.55, 0.95])
    return pcd


def generate_circle_point_cloud(
    center: np.ndarray,
    radius: float,
    normal: np.ndarray,
    n_points: int = 400,
    noise_std: float = 0.04,
    seed: int = 1,
) -> o3d.geometry.PointCloud:
    """在指定平面上采样圆，并叠加高斯噪声，得到圆环点云。"""
    rng = np.random.default_rng(seed)
    normal = normal / np.linalg.norm(normal)

    # 构造平面正交基
    helper = np.array([1.0, 0.0, 0.0]) if abs(normal[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
    u = np.cross(normal, helper)
    u /= np.linalg.norm(u)
    v = np.cross(normal, u)

    theta = rng.uniform(0.0, 2.0 * math.pi, size=n_points)
    points = (
        center
        + radius * (np.cos(theta)[:, None] * u + np.sin(theta)[:, None] * v)
        + rng.normal(0.0, noise_std, size=(n_points, 3))
    )

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    pcd.paint_uniform_color([0.95, 0.45, 0.2])
    return pcd


# ---------------------------------------------------------------------------
# 拟合
# ---------------------------------------------------------------------------

def fit_line_pca(pcd: o3d.geometry.PointCloud) -> tuple[np.ndarray, np.ndarray]:
    """
    用 PCA 拟合 3D 直线。
    返回: (直线上一点, 单位方向向量)
    """
    points = np.asarray(pcd.points)
    centroid = points.mean(axis=0)
    _, _, vt = np.linalg.svd(points - centroid, full_matrices=False)
    direction = vt[0]
    direction /= np.linalg.norm(direction)
    return centroid, direction


def fit_circle_least_squares(
    pcd: o3d.geometry.PointCloud,
    plane_normal: np.ndarray | None = None,
) -> tuple[np.ndarray, float, np.ndarray]:
    """
    拟合 3D 圆:
      1) 用点云 PCA 估计平面法向（也可传入已知法向）
      2) 投影到平面后做代数最小二乘圆拟合
    返回: (圆心, 半径, 单位法向)
    """
    points = np.asarray(pcd.points)
    centroid = points.mean(axis=0)

    if plane_normal is None:
        _, _, vt = np.linalg.svd(points - centroid, full_matrices=False)
        normal = vt[2]
    else:
        normal = np.asarray(plane_normal, dtype=float)
    normal = normal / np.linalg.norm(normal)

    helper = np.array([1.0, 0.0, 0.0]) if abs(normal[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
    u = np.cross(normal, helper)
    u /= np.linalg.norm(u)
    v = np.cross(normal, u)

    # 投影到局部 2D 坐标
    rel = points - centroid
    xy = np.column_stack([rel @ u, rel @ v])

    # 代数圆拟合: x^2 + y^2 + d x + e y + f = 0
    x, y = xy[:, 0], xy[:, 1]
    a = np.column_stack([2 * x, 2 * y, np.ones_like(x)])
    b = x**2 + y**2
    sol, *_ = np.linalg.lstsq(a, b, rcond=None)
    cx_local, cy_local, c = sol
    radius = math.sqrt(max(cx_local**2 + cy_local**2 + c, 0.0))
    center = centroid + cx_local * u + cy_local * v
    return center, radius, normal


# ---------------------------------------------------------------------------
# 可视化几何构造
# ---------------------------------------------------------------------------

def make_fitted_line_geometry(
    point: np.ndarray,
    direction: np.ndarray,
    half_length: float = 2.5,
) -> o3d.geometry.LineSet:
    """根据拟合参数生成可绘制的直线段。"""
    p0 = point - half_length * direction
    p1 = point + half_length * direction
    line_set = o3d.geometry.LineSet(
        points=o3d.utility.Vector3dVector([p0, p1]),
        lines=o3d.utility.Vector2iVector([[0, 1]]),
    )
    line_set.colors = o3d.utility.Vector3dVector([[0.05, 0.85, 0.25]])
    return line_set


def make_fitted_circle_geometry(
    center: np.ndarray,
    radius: float,
    normal: np.ndarray,
    n_segments: int = 128,
) -> o3d.geometry.LineSet:
    """根据拟合参数生成可绘制的圆折线。"""
    normal = normal / np.linalg.norm(normal)
    helper = np.array([1.0, 0.0, 0.0]) if abs(normal[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
    u = np.cross(normal, helper)
    u /= np.linalg.norm(u)
    v = np.cross(normal, u)

    angles = np.linspace(0.0, 2.0 * math.pi, n_segments, endpoint=False)
    pts = center + radius * (np.cos(angles)[:, None] * u + np.sin(angles)[:, None] * v)
    lines = [[i, (i + 1) % n_segments] for i in range(n_segments)]

    circle = o3d.geometry.LineSet(
        points=o3d.utility.Vector3dVector(pts),
        lines=o3d.utility.Vector2iVector(lines),
    )
    circle.colors = o3d.utility.Vector3dVector([[0.9, 0.15, 0.15]] * len(lines))
    return circle


def make_center_sphere(center: np.ndarray, radius: float = 0.05) -> o3d.geometry.TriangleMesh:
    sphere = o3d.geometry.TriangleMesh.create_sphere(radius=radius)
    sphere.translate(center)
    sphere.paint_uniform_color([0.9, 0.15, 0.15])
    sphere.compute_vertex_normals()
    return sphere


# ---------------------------------------------------------------------------
# 保存 2D 投影图（便于无显示器环境检查结果）
# ---------------------------------------------------------------------------

def save_result_figure(
    line_pcd: o3d.geometry.PointCloud,
    line_point: np.ndarray,
    line_dir: np.ndarray,
    circle_pcd: o3d.geometry.PointCloud,
    circle_center: np.ndarray,
    circle_radius: float,
    out_path: Path,
) -> None:
    line_pts = np.asarray(line_pcd.points)
    circle_pts = np.asarray(circle_pcd.points)

    fig, axes = plt.subplots(1, 2, figsize=(12, 5))

    # 直线：XY 投影
    ax = axes[0]
    ax.scatter(line_pts[:, 0], line_pts[:, 1], s=8, c="#3488f0", alpha=0.65, label="line points")
    t = np.linspace(-2.5, 2.5, 100)
    fitted = line_point[None, :] + t[:, None] * line_dir[None, :]
    ax.plot(fitted[:, 0], fitted[:, 1], c="#14d940", lw=2.5, label="fitted line")
    ax.set_title("Line Fit (XY)")
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best")

    # 圆：XY 投影
    ax = axes[1]
    ax.scatter(circle_pts[:, 0], circle_pts[:, 1], s=8, c="#f27333", alpha=0.65, label="circle points")
    theta = np.linspace(0.0, 2.0 * math.pi, 200)
    # 圆在 XY 平面附近时，直接用圆心+半径画；若圆不在 XY 平面，这是近似投影
    ax.plot(
        circle_center[0] + circle_radius * np.cos(theta),
        circle_center[1] + circle_radius * np.sin(theta),
        c="#e52626",
        lw=2.5,
        label="fitted circle",
    )
    ax.scatter([circle_center[0]], [circle_center[1]], c="#e52626", s=40, zorder=5, label="center")
    ax.set_title("Circle Fit (XY)")
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best")

    fig.tight_layout()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=160)
    plt.close(fig)


# ---------------------------------------------------------------------------
# 主流程
# ---------------------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(description="Open3D 直线/圆点云生成、拟合与可视化")
    parser.add_argument("--no-show", action="store_true", help="不弹出 Open3D 窗口，仅保存结果图")
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("output/line_circle_fit.png"),
        help="结果图保存路径",
    )
    args = parser.parse_args()

    # 1) 生成线点云并拟合直线
    true_line_point = np.array([0.0, 0.0, 0.0])
    true_line_dir = np.array([1.0, 0.4, 0.2])
    line_pcd = generate_line_point_cloud(
        point_on_line=true_line_point,
        direction=true_line_dir,
        n_points=300,
        length=4.0,
        noise_std=0.05,
    )
    line_point, line_dir = fit_line_pca(line_pcd)

    # 2) 生成圆点云并拟合圆
    true_circle_center = np.array([3.5, 0.0, 0.0])
    true_circle_radius = 1.2
    true_circle_normal = np.array([0.0, 0.0, 1.0])
    circle_pcd = generate_circle_point_cloud(
        center=true_circle_center,
        radius=true_circle_radius,
        normal=true_circle_normal,
        n_points=400,
        noise_std=0.04,
    )
    circle_center, circle_radius, circle_normal = fit_circle_least_squares(circle_pcd)

    print("=== 直线拟合结果 ===")
    print(f"点: {line_point}")
    print(f"方向: {line_dir}")
    print(
        "与真值方向夹角(度): "
        f"{math.degrees(math.acos(np.clip(abs(np.dot(line_dir, true_line_dir / np.linalg.norm(true_line_dir))), 0, 1))):.3f}"
    )

    print("\n=== 圆拟合结果 ===")
    print(f"圆心: {circle_center}")
    print(f"半径: {circle_radius:.6f}")
    print(f"法向: {circle_normal}")
    print(f"圆心误差: {np.linalg.norm(circle_center - true_circle_center):.6f}")
    print(f"半径误差: {abs(circle_radius - true_circle_radius):.6f}")

    # 3) 构造可视化对象
    fitted_line = make_fitted_line_geometry(line_point, line_dir, half_length=2.5)
    fitted_circle = make_fitted_circle_geometry(circle_center, circle_radius, circle_normal)
    center_sphere = make_center_sphere(circle_center, radius=0.06)
    frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.8)

    # 保存 2D 结果图
    save_result_figure(
        line_pcd,
        line_point,
        line_dir,
        circle_pcd,
        circle_center,
        circle_radius,
        args.output,
    )
    print(f"\n结果图已保存: {args.output.resolve()}")

    if args.no_show:
        return

    # 4) Open3D 交互可视化：蓝线点云 + 绿拟合线；橙圆点云 + 红拟合圆
    o3d.visualization.draw_geometries(
        [line_pcd, fitted_line, circle_pcd, fitted_circle, center_sphere, frame],
        window_name="Open3D Line & Circle Fit",
        width=1280,
        height=800,
    )


if __name__ == "__main__":
    main()
