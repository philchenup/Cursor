#!/usr/bin/env python3
"""从 RGB PNG 与 16 位深度 PNG，按针孔相机内参恢复彩色点云（Open3D）。

针孔模型（相机坐标系：x 右、y 下、z 前，深度为 Z）：

    X = (u - cx) * Z / fx
    Y = (v - cy) * Z / fy
    Z = depth / depth_scale

16 位深度图常见单位是毫米，此时 ``depth_scale=1000``，点云单位为米。
"""

from __future__ import annotations

import argparse
import json
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import numpy as np
import open3d as o3d
from PIL import Image


@dataclass
class CameraIntrinsics:
    """针孔相机内参。"""

    fx: float
    fy: float
    cx: float
    cy: float
    width: Optional[int] = None
    height: Optional[int] = None

    def to_o3d(self, width: int, height: int) -> o3d.camera.PinholeCameraIntrinsic:
        return o3d.camera.PinholeCameraIntrinsic(
            width=width,
            height=height,
            fx=self.fx,
            fy=self.fy,
            cx=self.cx,
            cy=self.cy,
        )

    @classmethod
    def from_dict(cls, data: dict) -> "CameraIntrinsics":
        if "K" in data:
            k = np.asarray(data["K"], dtype=np.float64)
            if k.shape != (3, 3):
                raise ValueError("内参矩阵 K 必须是 3x3")
            fx, fy, cx, cy = float(k[0, 0]), float(k[1, 1]), float(k[0, 2]), float(k[1, 2])
        else:
            required = ("fx", "fy", "cx", "cy")
            missing = [name for name in required if name not in data]
            if missing:
                raise ValueError(f"内参缺少字段: {', '.join(missing)}")
            fx, fy, cx, cy = (float(data[name]) for name in required)
        return cls(
            fx=fx,
            fy=fy,
            cx=cx,
            cy=cy,
            width=int(data["width"]) if data.get("width") is not None else None,
            height=int(data["height"]) if data.get("height") is not None else None,
        )

    @classmethod
    def from_json(cls, path: str | Path) -> "CameraIntrinsics":
        with open(path, "r", encoding="utf-8") as f:
            return cls.from_dict(json.load(f))


def load_rgb_png(path: str | Path) -> np.ndarray:
    """读取 RGB PNG，返回 HxWx3 的 uint8 数组。"""
    image = Image.open(path)
    if image.mode == "RGBA":
        image = image.convert("RGB")
    elif image.mode == "L":
        image = image.convert("RGB")
    elif image.mode != "RGB":
        image = image.convert("RGB")
    rgb = np.asarray(image, dtype=np.uint8)
    if rgb.ndim != 3 or rgb.shape[2] != 3:
        raise ValueError(f"RGB 图形状异常: {rgb.shape}，期望 HxWx3")
    return np.ascontiguousarray(rgb)


def load_depth_png16(path: str | Path) -> np.ndarray:
    """读取 16 位深度 PNG，返回 HxW 的 uint16 数组。

    支持常见模式：``I;16`` / ``I;16B`` / ``I`` / ``F``。
    若文件实际是 8 位，会给出提示并按 uint16 解释（数值范围 0–255）。
    """
    image = Image.open(path)
    raw = np.array(image)
    if raw.ndim != 2:
        # 有些深度图被存成三通道但每个通道相同
        if raw.ndim == 3 and raw.shape[2] in (3, 4):
            raw = raw[..., 0]
        else:
            raise ValueError(f"深度图必须是单通道，当前形状: {raw.shape}")

    if raw.dtype == np.uint16:
        depth = raw
    elif raw.dtype == np.uint8:
        print(
            f"警告: {path} 是 8 位 PNG，不是 16 位深度图。将按 uint8 提升为 uint16。",
            file=sys.stderr,
        )
        depth = raw.astype(np.uint16)
    elif np.issubdtype(raw.dtype, np.floating):
        if raw.max() <= 0:
            depth = np.zeros(raw.shape, dtype=np.uint16)
        elif raw.max() <= 65535:
            depth = np.clip(np.rint(raw), 0, 65535).astype(np.uint16)
        else:
            raise ValueError(f"浮点深度图数值过大: max={raw.max()}")
    elif np.issubdtype(raw.dtype, np.integer):
        depth = np.clip(raw, 0, 65535).astype(np.uint16)
    else:
        raise ValueError(f"不支持的深度图 dtype: {raw.dtype}")

    return np.ascontiguousarray(depth)


def _check_same_size(rgb: np.ndarray, depth: np.ndarray) -> None:
    if rgb.shape[:2] != depth.shape[:2]:
        raise ValueError(
            f"RGB 与深度图分辨率不一致: RGB={rgb.shape[:2]}, depth={depth.shape[:2]}"
        )


def reconstruct_with_open3d(
    rgb: np.ndarray,
    depth_u16: np.ndarray,
    intrinsics: CameraIntrinsics,
    depth_scale: float = 1000.0,
    depth_trunc: float = 10.0,
    flip_gl: bool = False,
) -> o3d.geometry.PointCloud:
    """用 Open3D 的 RGBDImage + 针孔内参生成点云。

    ``depth_scale``: 原始深度除以该值得到米（毫米深度图用 1000）。
    ``depth_trunc``: 超过该米数的深度丢弃。
    ``flip_gl``: 转到 OpenGL/可视化常用坐标系（Y 上、Z 后）。
    """
    _check_same_size(rgb, depth_u16)
    height, width = depth_u16.shape

    color_image = o3d.geometry.Image(np.ascontiguousarray(rgb, dtype=np.uint8))
    depth_image = o3d.geometry.Image(np.ascontiguousarray(depth_u16, dtype=np.uint16))
    rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
        color_image,
        depth_image,
        depth_scale=float(depth_scale),
        depth_trunc=float(depth_trunc),
        convert_rgb_to_intensity=False,
    )

    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
        rgbd,
        intrinsics.to_o3d(width, height),
        np.eye(4),
    )
    if flip_gl:
        pcd.transform(
            np.array(
                [
                    [1, 0, 0, 0],
                    [0, -1, 0, 0],
                    [0, 0, -1, 0],
                    [0, 0, 0, 1],
                ],
                dtype=np.float64,
            )
        )
    return pcd


def reconstruct_with_numpy(
    rgb: np.ndarray,
    depth_u16: np.ndarray,
    intrinsics: CameraIntrinsics,
    depth_scale: float = 1000.0,
    depth_trunc: float = 10.0,
    min_depth: float = 1e-6,
    flip_gl: bool = False,
) -> o3d.geometry.PointCloud:
    """向量化反投影，结果写入 Open3D PointCloud（便于对照 Open3D 内置实现）。"""
    _check_same_size(rgb, depth_u16)

    z = depth_u16.astype(np.float64) / float(depth_scale)
    valid = np.isfinite(z) & (z > min_depth) & (z < depth_trunc)
    if not np.any(valid):
        return o3d.geometry.PointCloud()

    height, width = depth_u16.shape
    u = np.arange(width, dtype=np.float64)
    v = np.arange(height, dtype=np.float64)
    uu, vv = np.meshgrid(u, v)

    z_valid = z[valid]
    x = (uu[valid] - intrinsics.cx) * z_valid / intrinsics.fx
    y = (vv[valid] - intrinsics.cy) * z_valid / intrinsics.fy
    points = np.stack((x, y, z_valid), axis=1)
    colors = rgb[valid].astype(np.float64) / 255.0

    if flip_gl:
        points[:, 1] *= -1.0
        points[:, 2] *= -1.0

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    pcd.colors = o3d.utility.Vector3dVector(colors)
    return pcd


def reconstruct_point_cloud(
    rgb: np.ndarray,
    depth_u16: np.ndarray,
    intrinsics: CameraIntrinsics,
    depth_scale: float = 1000.0,
    depth_trunc: float = 10.0,
    backend: str = "open3d",
    flip_gl: bool = False,
) -> o3d.geometry.PointCloud:
    backend = backend.lower()
    if backend == "open3d":
        return reconstruct_with_open3d(
            rgb, depth_u16, intrinsics, depth_scale, depth_trunc, flip_gl
        )
    if backend == "numpy":
        return reconstruct_with_numpy(
            rgb, depth_u16, intrinsics, depth_scale, depth_trunc, flip_gl=flip_gl
        )
    raise ValueError(f"未知 backend: {backend}（可选 open3d / numpy）")


def save_point_cloud(pcd: o3d.geometry.PointCloud, path: str | Path) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    ok = o3d.io.write_point_cloud(str(path), pcd, write_ascii=False, compressed=False)
    if not ok:
        raise IOError(f"写入点云失败: {path}")


def generate_demo_rgbd(
    width: int = 320,
    height: int = 240,
    fx: float = 250.0,
    fy: float = 250.0,
    plane_z_m: float = 1.5,
    depth_scale: float = 1000.0,
) -> tuple[np.ndarray, np.ndarray, CameraIntrinsics]:
    """生成一张彩色图 + 16 位平面深度图，便于无真实数据时试跑。"""
    cx = (width - 1) / 2.0
    cy = (height - 1) / 2.0
    uu, vv = np.meshgrid(np.arange(width), np.arange(height))
    rgb = np.zeros((height, width, 3), dtype=np.uint8)
    rgb[..., 0] = (uu * 255 / max(width - 1, 1)).astype(np.uint8)
    rgb[..., 1] = (vv * 255 / max(height - 1, 1)).astype(np.uint8)
    rgb[..., 2] = 180

    depth_u16 = np.full((height, width), int(round(plane_z_m * depth_scale)), dtype=np.uint16)
    # 四周挖一圈无效深度，模拟真实深度图的空洞
    depth_u16[:4, :] = 0
    depth_u16[-4:, :] = 0
    depth_u16[:, :4] = 0
    depth_u16[:, -4:] = 0

    intrinsics = CameraIntrinsics(fx=fx, fy=fy, cx=cx, cy=cy, width=width, height=height)
    return rgb, depth_u16, intrinsics


def save_demo_images(
    rgb: np.ndarray,
    depth_u16: np.ndarray,
    out_dir: str | Path,
) -> tuple[Path, Path]:
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    rgb_path = out_dir / "demo_rgb.png"
    depth_path = out_dir / "demo_depth16.png"
    Image.fromarray(rgb).save(rgb_path)
    Image.fromarray(depth_u16).save(depth_path)
    return rgb_path, depth_path


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="读取 RGB PNG 与 16 位深度 PNG，按内参恢复点云并写出（Open3D）"
    )
    parser.add_argument("--rgb", type=str, help="RGB 彩色 PNG 路径")
    parser.add_argument("--depth", type=str, help="16 位深度 PNG 路径")
    parser.add_argument("--fx", type=float, help="焦距 fx（像素）")
    parser.add_argument("--fy", type=float, help="焦距 fy（像素）")
    parser.add_argument("--cx", type=float, help="主点 cx（像素）")
    parser.add_argument("--cy", type=float, help="主点 cy（像素）")
    parser.add_argument(
        "--intrinsics",
        type=str,
        help="内参 JSON（含 fx/fy/cx/cy，或 3x3 的 K；可含 depth_scale）",
    )
    parser.add_argument(
        "--depth-scale",
        type=float,
        default=None,
        help="深度除数，毫米深度图用 1000（默认 1000）",
    )
    parser.add_argument(
        "--depth-trunc",
        type=float,
        default=10.0,
        help="超过该距离（米）的深度丢弃，默认 10",
    )
    parser.add_argument(
        "--backend",
        choices=("open3d", "numpy"),
        default="open3d",
        help="反投影实现，默认 open3d",
    )
    parser.add_argument(
        "--flip-gl",
        action="store_true",
        help="将点云翻到 OpenGL 坐标系（Y 上 Z 后），方便可视化",
    )
    parser.add_argument(
        "--output",
        "-o",
        type=str,
        default="cloud.ply",
        help="输出点云路径（.ply / .pcd / .xyz）",
    )
    parser.add_argument(
        "--demo",
        action="store_true",
        help="用合成 RGB + 16 位平面深度图跑一遍完整流程",
    )
    parser.add_argument(
        "--demo-dir",
        type=str,
        default="python/demo_output",
        help="--demo 时保存合成图的目录",
    )
    parser.add_argument(
        "--visualize",
        action="store_true",
        help="用 Open3D 窗口显示点云（无显示环境会跳过）",
    )
    return parser


def _intrinsics_from_args(
    args: argparse.Namespace,
    rgb: np.ndarray,
    json_extra: dict,
) -> CameraIntrinsics:
    height, width = rgb.shape[:2]
    if args.intrinsics:
        intrinsics = CameraIntrinsics.from_dict(json_extra)
        if args.fx is not None:
            intrinsics.fx = args.fx
        if args.fy is not None:
            intrinsics.fy = args.fy
        if args.cx is not None:
            intrinsics.cx = args.cx
        if args.cy is not None:
            intrinsics.cy = args.cy
    else:
        if None in (args.fx, args.fy, args.cx, args.cy):
            raise SystemExit("请提供 --fx --fy --cx --cy，或使用 --intrinsics JSON / --demo")
        intrinsics = CameraIntrinsics(
            fx=args.fx, fy=args.fy, cx=args.cx, cy=args.cy, width=width, height=height
        )

    if intrinsics.width is None:
        intrinsics.width = width
    if intrinsics.height is None:
        intrinsics.height = height
    if (intrinsics.width, intrinsics.height) != (width, height):
        print(
            f"警告: 内参分辨率 {intrinsics.width}x{intrinsics.height} "
            f"与图像 {width}x{height} 不一致，按图像尺寸重建。",
            file=sys.stderr,
        )
    return intrinsics


def main(argv: Optional[list[str]] = None) -> int:
    parser = _build_parser()
    args = parser.parse_args(argv)

    json_extra: dict = {}
    if args.intrinsics:
        with open(args.intrinsics, "r", encoding="utf-8") as f:
            json_extra = json.load(f)

    depth_scale = args.depth_scale
    if depth_scale is None:
        depth_scale = float(json_extra.get("depth_scale", 1000.0))

    if args.demo:
        rgb, depth_u16, intrinsics = generate_demo_rgbd(depth_scale=depth_scale)
        rgb_path, depth_path = save_demo_images(rgb, depth_u16, args.demo_dir)
        print(f"已生成合成图: {rgb_path}")
        print(f"已生成 16 位深度图: {depth_path}")
        # 走真实读盘路径，保证与用户用法一致
        rgb = load_rgb_png(rgb_path)
        depth_u16 = load_depth_png16(depth_path)
        k_path = Path(args.demo_dir) / "demo_intrinsics.json"
        k_path.write_text(
            json.dumps(
                {
                    "fx": intrinsics.fx,
                    "fy": intrinsics.fy,
                    "cx": intrinsics.cx,
                    "cy": intrinsics.cy,
                    "width": rgb.shape[1],
                    "height": rgb.shape[0],
                    "depth_scale": depth_scale,
                },
                indent=2,
            ),
            encoding="utf-8",
        )
        print(f"已写入内参: {k_path}")
    else:
        if not args.rgb or not args.depth:
            parser.error("必须提供 --rgb 与 --depth，或使用 --demo")
        rgb = load_rgb_png(args.rgb)
        depth_u16 = load_depth_png16(args.depth)
        intrinsics = _intrinsics_from_args(args, rgb, json_extra)

    pcd = reconstruct_point_cloud(
        rgb=rgb,
        depth_u16=depth_u16,
        intrinsics=intrinsics,
        depth_scale=depth_scale,
        depth_trunc=args.depth_trunc,
        backend=args.backend,
        flip_gl=args.flip_gl,
    )

    n = len(pcd.points)
    print(
        f"点云点数: {n}  |  图像 {rgb.shape[1]}x{rgb.shape[0]}  |  "
        f"fx={intrinsics.fx:.3f} fy={intrinsics.fy:.3f} "
        f"cx={intrinsics.cx:.3f} cy={intrinsics.cy:.3f}  |  "
        f"depth_scale={depth_scale}  backend={args.backend}"
    )
    if n == 0:
        print("警告: 点云为空，请检查深度图是否全 0，或 depth_scale / depth_trunc 是否合理。", file=sys.stderr)
    else:
        pts = np.asarray(pcd.points)
        print(
            "XYZ 范围: "
            f"X[{pts[:, 0].min():.4f}, {pts[:, 0].max():.4f}]  "
            f"Y[{pts[:, 1].min():.4f}, {pts[:, 1].max():.4f}]  "
            f"Z[{pts[:, 2].min():.4f}, {pts[:, 2].max():.4f}]"
        )

    save_point_cloud(pcd, args.output)
    print(f"已写出点云: {args.output}")

    if args.visualize and n > 0:
        try:
            o3d.visualization.draw_geometries([pcd], window_name="RGB-D Point Cloud")
        except Exception as exc:  # 无显示设备时不要让 CLI 失败
            print(f"可视化跳过: {exc}", file=sys.stderr)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
