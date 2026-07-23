#!/usr/bin/env python3
"""Aggregate Open3D / PCL timings + Hikvision VM3D availability into category reports."""

from __future__ import annotations

import json
import platform
from datetime import datetime, timezone
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
RES = ROOT / "results"

# Hikvision VisionMaster 3D (VM3D) operator availability mapped from official docs:
# https://pinfo.hikrobotics.com/hkws/unzip/20240919112534_17943_doc/
# Timing cannot be measured here: VM3D is proprietary Windows software (dongle/license).
VM3D = {
    "io_conversion": {
        "load_cloud": {
            "supported": True,
            "module": "3D图像源 / 本地图像",
            "note": "流程内加载点云/深度图本地文件",
        },
        "save_cloud": {
            "supported": True,
            "module": "3D输出图像 / 文本保存",
            "note": "可导出结果图像/数据",
        },
        "txt_to_cloud": {"supported": False, "reason": "无独立 txt→点云算子（通常走图像源/脚本）"},
        "cloud_to_txt": {"supported": False, "reason": "无独立点云→txt算子"},
        "rescale_units": {"supported": True, "module": "尺度变换", "note": "3D测量类尺度变换"},
        "transform_cloud": {
            "supported": True,
            "module": "点云坐标系转换 / 坐标系变换-深度图",
        },
        "depth_to_cloud": {"supported": True, "module": "转点云-深度图"},
        "rgbd_to_cloud": {
            "supported": True,
            "module": "RGBD等间距转换-深度图",
            "note": "偏深度图/RGBD流程，非通用开源 RGBD→PCD API",
        },
        "extract_cloud_by_mask": {
            "supported": True,
            "module": "点云截取 / 截取-深度图 / ROI转掩膜",
        },
        "concat_clouds": {"supported": True, "module": "点云合并"},
    },
    "preprocess": {
        "downsample_voxel": {
            "supported": True,
            "module": "点云降采样",
            "note": "文档为统一“点云降采样”，未公开细分为 voxel/fps/nss 等独立算子名",
        },
        "downsample_uniform": {
            "supported": "partial",
            "module": "点云降采样",
            "note": "可能作为降采样模式，文档未单独列出",
        },
        "downsample_random": {"supported": False, "reason": "文档无随机降采样独立算子"},
        "downsample_fps": {"supported": False, "reason": "文档无 FPS 独立算子"},
        "downsample_to_count": {
            "supported": "partial",
            "module": "点云降采样",
            "note": "可能通过目标点数参数实现，未单独命名",
        },
        "downsample_normal_space": {"supported": False, "reason": "文档无 Normal Space Sampling"},
        "filter_axis_range": {
            "supported": True,
            "module": "点云截取 / 截取-深度图",
        },
        "filter_axis_threshold": {
            "supported": True,
            "module": "点云截取 / 高度抽取",
        },
        "filter_by_direction": {
            "supported": True,
            "module": "法向量滤除-深度图",
            "note": "深度图法向滤除，非通用点云方向滤波 API",
        },
        "remove_statistical_outliers": {
            "supported": "partial",
            "module": "杂点过滤-深度图 / 滤波-深度图",
            "note": "深度图域离群点过滤，非 PCL SOR 同名实现",
        },
        "remove_radius_outliers": {
            "supported": "partial",
            "module": "杂点过滤-深度图",
        },
        "estimate_normals": {
            "supported": True,
            "module": "法向量灰度图-深度图",
            "note": "深度图法向可视化/计算，非通用点云 NormalEstimation API",
        },
        "orient_normals": {"supported": False, "reason": "文档无法线一致定向独立算子"},
        "flip_normals": {"supported": False, "reason": "文档无法线翻转独立算子"},
        "compute_fpfh": {"supported": False, "reason": "文档无 FPFH 特征算子"},
        "compute_curvature": {"supported": False, "reason": "文档无曲率计算独立算子"},
    },
    "geometry_segmentation": {
        "compute_obb": {"supported": False, "reason": "文档无 OBB 独立算子"},
        "compute_convex_hull": {"supported": False, "reason": "文档无凸包独立算子"},
        "compute_centroid": {
            "supported": True,
            "module": "平面点云属性计算 / 单团点云几何查找",
        },
        "compute_aabb": {
            "supported": "partial",
            "module": "单团点云几何查找",
            "note": "几何属性中可能含包围信息，未单列 AABB",
        },
        "select_by_mask": {"supported": True, "module": "点云截取 / ROI / 掩膜相关"},
        "select_by_index": {"supported": False, "reason": "面向流程算子，无索引选择 API"},
        "segment_plane": {
            "supported": True,
            "module": "平面检测-深度图 / 平面拟合",
        },
        "cluster_dbscan": {"supported": False, "reason": "文档无 DBSCAN 点云聚类算子"},
        "split_by_labels": {"supported": False, "reason": "文档无按标签拆分点云算子"},
        "fit_line": {
            "supported": True,
            "module": "直线查找-深度图 / 直线查找-轮廓图",
        },
        "fit_sphere": {"supported": False, "reason": "文档无球体拟合独立算子"},
        "fit_plane": {"supported": True, "module": "平面拟合 / 平面检测-深度图"},
        "plane_from_points": {"supported": False, "reason": "无三点构面独立底层 API"},
        "plane_normal": {
            "supported": True,
            "module": "平面拟合 / 平面点云属性计算",
        },
        "split_by_plane": {
            "supported": "partial",
            "module": "点云截取 + 平面检测组合",
        },
        "extract_boundary_points": {
            "supported": "partial",
            "module": "轮廓相关 / 边缘类深度图工具",
            "note": "偏轮廓图/深度图边缘，非通用边界点提取",
        },
    },
    "registration": {
        "make_init_pose": {
            "supported": "partial",
            "module": "位置修正-深度图 / 标定与坐标转换",
        },
        "register_fpfh_ransac": {"supported": False, "reason": "无 FPFH+RANSAC 点云配准算子"},
        "register_fast_global": {"supported": False, "reason": "无 Fast Global Registration"},
        "register_ppf": {"supported": False, "reason": "无 PPF 配准算子"},
        "relative_sampling_step": {"supported": False, "reason": "非独立算子"},
        "register_icp_point_to_point": {
            "supported": False,
            "reason": "文档无经典点到点 ICP 点云算子名",
        },
        "register_icp_point_to_plane": {
            "supported": False,
            "reason": "文档无点到面 ICP 点云算子名",
        },
        "register_icp_generalized": {"supported": False, "reason": "无 GICP"},
        "register_icp_colored": {"supported": False, "reason": "无 Colored ICP"},
        "register_filterreg": {"supported": False, "reason": "无 FilterReg"},
        "register_cpd": {"supported": False, "reason": "无 CPD"},
        "register_gmmtree": {"supported": False, "reason": "无 GMMTree"},
        "evaluate_registration": {
            "supported": "partial",
            "module": "配准定位-深度图 / 数据统计",
            "note": "深度图配准定位结果评估，非 Open3D evaluate_registration",
        },
        "is_registration_valid": {
            "supported": "partial",
            "module": "条件检测 / 逻辑工具",
        },
        "_vm3d_registration_note": {
            "supported": True,
            "module": "配准定位-深度图 / 匹配-深度图 / 尺度匹配-深度图",
            "note": "VM3D 配准主路径是深度图模板匹配/定位，与 PCL/Open3D 点云 ICP/FPFH 体系不同",
        },
    },
}

CAT_TITLE = {
    "io_conversion": "一、IO / 转换类 (io_conversion)",
    "preprocess": "二、预处理类 (preprocess)",
    "geometry_segmentation": "三、几何 / 分割类 (geometry_segmentation)",
    "registration": "四、配准类 (registration)",
}


def mean_or_none(entry):
    if not entry or not entry.get("supported"):
        return None
    return entry.get("mean_ms")


def fmt_ms(v):
    if v is None:
        return "—"
    if v < 0.01:
        return f"{v:.4f}"
    if v < 10:
        return f"{v:.3f}"
    if v < 1000:
        return f"{v:.2f}"
    return f"{v:.1f}"


def vm_cell(entry):
    if entry is None:
        return "—"
    s = entry.get("supported")
    if s is True:
        mod = entry.get("module", "有")
        return f"有（{mod}）; 耗时未测"
    if s == "partial":
        mod = entry.get("module", "")
        return f"部分（{mod}）; 耗时未测"
    return "—"


def main():
    o3d = json.loads((RES / "open3d_timings.json").read_text())
    pcl = json.loads((RES / "pcl_timings.json").read_text())

    env = {
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "host": platform.node(),
        "platform": platform.platform(),
        "processor": platform.processor() or platform.machine(),
        "python": platform.python_version(),
        "n_points": o3d.get("n_points"),
        "warmup": 1,
        "repeats": 5,
        "open3d_version": o3d.get("version"),
        "pcl_version": pcl.get("version"),
        "vm3d_timing": "unavailable_in_this_environment",
        "vm3d_docs": "https://pinfo.hikrobotics.com/hkws/unzip/20240919112534_17943_doc/",
        "notes": [
            "同一合成点云 N=100000，相同参数口径尽量对齐（体素0.05、SOR k=20/std=2、ROR r=0.05/n=16 等）",
            "库不存在的算法不输出耗时（表中为 —）",
            "海康 VM3D 为 Windows 商业软件（加密狗/授权），本 Linux 环境无法安装实测；仅据官方模块文档标注是否具备对应能力",
            "部分同名算法实现不同：fit_line(Open3D=PCA, PCL=RANSAC)；downsample_to_count(Open3D=FPS, PCL=RandomSample)；orient_normals(Open3D=consistent tangent plane, PCL=viewpoint)",
            "配准类 ICP 在体素降采样后的点云上运行（voxel=0.05），迭代上限约 30",
        ],
    }

    tables = {}
    md = []
    md.append("# 点云算法处理耗时对比：PCL / Open3D / 海康 VM3D")
    md.append("")
    md.append("## 测试环境")
    md.append("")
    md.append(f"- 时间(UTC): `{env['generated_at_utc']}`")
    md.append(f"- 平台: `{env['platform']}`")
    md.append(f"- 点数: **{env['n_points']}**（合成同一数据集）")
    md.append(f"- Open3D: `{env['open3d_version']}`；PCL: `{env['pcl_version']}`（1.14.x）")
    md.append("- 计时: warmup=1, repeats=5, 单位 ms（mean）")
    md.append("- 海康 VM3D: **本环境无法运行**，仅输出能力映射，不输出处理耗时")
    md.append("")
    md.append("## 按类性能结果")
    md.append("")

    for cat, title in CAT_TITLE.items():
        md.append(f"### {title}")
        md.append("")
        md.append("| 算法 | Open3D (ms) | PCL (ms) | 海康 VM3D |")
        md.append("|---|---:|---:|---|")
        rows = []
        keys = sorted(
            set(o3d["categories"][cat].keys())
            | set(pcl["categories"][cat].keys())
            | set(k for k in VM3D[cat].keys() if not k.startswith("_"))
        )
        for algo in keys:
            o = mean_or_none(o3d["categories"][cat].get(algo))
            p = mean_or_none(pcl["categories"][cat].get(algo))
            # Skip rows where none of the three produce a meaningful result
            vm = VM3D[cat].get(algo)
            if o is None and p is None and (vm is None or vm.get("supported") is False):
                continue
            md.append(
                f"| `{algo}` | {fmt_ms(o)} | {fmt_ms(p)} | {vm_cell(vm)} |"
            )
            rows.append(
                {
                    "algorithm": algo,
                    "open3d_ms": o,
                    "pcl_ms": p,
                    "vm3d": vm,
                }
            )
        tables[cat] = rows
        md.append("")

    md.append("## 简要结论")
    md.append("")
    md.append("- **IO/变换**: PCL 在 `save_cloud`/`transform_cloud`/`concat_clouds` 更快；Open3D 在 `load_cloud`/`txt_to_cloud` 更快，并独有 `depth_to_cloud`/`rgbd_to_cloud`。")
    md.append("- **预处理**: 体素降采样两者接近；法线/SOR 上 Open3D 通常更快；PCL 独有 `downsample_normal_space`；Open3D 独有 `downsample_fps`。")
    md.append("- **几何分割**: Open3D 在 `segment_plane`/`cluster_dbscan`/`fit_plane` 更快；PCL 独有 `fit_sphere`；PCL `fit_line`(RANSAC@100k) 极慢。")
    md.append("- **配准**: Open3D 在 ICP/GICP/FPFH-RANSAC/FGR/ColoredICP 全面更快，且覆盖更广；PCL 无 FGR/ColoredICP。")
    md.append("- **海康 VM3D**: 强项在工业深度图流程（转点云、降采样、截取合并、平面拟合、深度图配准定位）；缺少开源库中的 FPFH/ICP/DBSCAN/FPS 等细粒度点云算法同名实现，故这些项不输出耗时。")
    md.append("")
    md.append("## 复现")
    md.append("")
    md.append("```bash")
    md.append("cd benchmark")
    md.append("python3 scripts/generate_data.py")
    md.append("python3 scripts/bench_open3d.py")
    md.append("cmake -S . -B build && cmake --build build -j")
    md.append("./build/bench_pcl data results/pcl_timings.json")
    md.append("python3 scripts/make_report.py")
    md.append("```")
    md.append("")

    report = {
        "env": env,
        "vm3d_availability": VM3D,
        "tables": tables,
        "open3d": o3d,
        "pcl": pcl,
    }
    (RES / "vm3d_availability.json").write_text(json.dumps(VM3D, indent=2, ensure_ascii=False))
    (RES / "report.json").write_text(json.dumps(report, indent=2, ensure_ascii=False))
    (RES / "REPORT.md").write_text("\n".join(md), encoding="utf-8")
    (ROOT / "REPORT.md").write_text("\n".join(md), encoding="utf-8")
    print("\n".join(md))


if __name__ == "__main__":
    main()
