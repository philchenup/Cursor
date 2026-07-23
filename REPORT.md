# 点云算法处理耗时对比：PCL / Open3D / 海康 VM3D

## 测试环境

- 时间(UTC): `2026-07-23T03:46:28.915830+00:00`
- 平台: `Linux-6.12.94+-x86_64-with-glibc2.39`
- 点数: **100000**（合成同一数据集）
- Open3D: `0.19.0`；PCL: `101400`（1.14.x）
- 计时: warmup=1, repeats=5, 单位 ms（mean）
- 海康 VM3D: **本环境无法运行**，仅输出能力映射，不输出处理耗时

## 按类性能结果

### 一、IO / 转换类 (io_conversion)

| 算法 | Open3D (ms) | PCL (ms) | 海康 VM3D |
|---|---:|---:|---|
| `cloud_to_txt` | 108.04 | 47.23 | — |
| `concat_clouds` | 7.778 | 0.394 | 有（点云合并）; 耗时未测 |
| `depth_to_cloud` | 0.423 | — | 有（转点云-深度图）; 耗时未测 |
| `extract_cloud_by_mask` | 0.997 | — | 有（点云截取 / 截取-深度图 / ROI转掩膜）; 耗时未测 |
| `load_cloud` | 13.51 | 62.33 | 有（3D图像源 / 本地图像）; 耗时未测 |
| `rescale_units` | 1.523 | 0.556 | 有（尺度变换）; 耗时未测 |
| `rgbd_to_cloud` | 0.488 | — | 有（RGBD等间距转换-深度图）; 耗时未测 |
| `save_cloud` | 17.58 | 1.417 | 有（3D输出图像 / 文本保存）; 耗时未测 |
| `transform_cloud` | 1.340 | 0.188 | 有（点云坐标系转换 / 坐标系变换-深度图）; 耗时未测 |
| `txt_to_cloud` | 14.92 | 48.22 | — |

### 二、预处理类 (preprocess)

| 算法 | Open3D (ms) | PCL (ms) | 海康 VM3D |
|---|---:|---:|---|
| `compute_curvature` | 424.33 | 47.55 | — |
| `compute_fpfh` | 70.53 | 226.88 | — |
| `downsample_fps` | 499.69 | — | — |
| `downsample_normal_space` | — | 25.02 | — |
| `downsample_random` | 1.393 | 1.380 | — |
| `downsample_to_count` | 798.53 | 1.335 | 部分（点云降采样）; 耗时未测 |
| `downsample_uniform` | 0.239 | 4.821 | 部分（点云降采样）; 耗时未测 |
| `downsample_voxel` | 3.854 | 3.389 | 有（点云降采样）; 耗时未测 |
| `estimate_normals` | 124.04 | 368.30 | 有（法向量灰度图-深度图）; 耗时未测 |
| `filter_axis_range` | 2.874 | 0.531 | 有（点云截取 / 截取-深度图）; 耗时未测 |
| `filter_axis_threshold` | 0.521 | 0.413 | 有（点云截取 / 高度抽取）; 耗时未测 |
| `filter_by_direction` | — | — | 有（法向量滤除-深度图）; 耗时未测 |
| `flip_normals` | 1.081 | 0.383 | — |
| `orient_normals` | 2202.7 | 385.45 | — |
| `remove_radius_outliers` | 167.32 | 203.77 | 部分（杂点过滤-深度图）; 耗时未测 |
| `remove_statistical_outliers` | 93.53 | 229.99 | 部分（杂点过滤-深度图 / 滤波-深度图）; 耗时未测 |

### 三、几何 / 分割类 (geometry_segmentation)

| 算法 | Open3D (ms) | PCL (ms) | 海康 VM3D |
|---|---:|---:|---|
| `cluster_dbscan` | 300.15 | 451.57 | — |
| `compute_aabb` | 0.208 | 0.100 | 部分（单团点云几何查找）; 耗时未测 |
| `compute_centroid` | 0.065 | 0.0000 | 有（平面点云属性计算 / 单团点云几何查找）; 耗时未测 |
| `compute_convex_hull` | 8.298 | 10.13 | — |
| `compute_obb` | 8.295 | 1.069 | — |
| `extract_boundary_points` | 155.25 | 135.44 | 部分（轮廓相关 / 边缘类深度图工具）; 耗时未测 |
| `fit_line` | 3.627 | 10629.8 | 有（直线查找-深度图 / 直线查找-轮廓图）; 耗时未测 |
| `fit_plane` | 70.90 | 152.55 | 有（平面拟合 / 平面检测-深度图）; 耗时未测 |
| `fit_sphere` | — | 224.40 | — |
| `plane_from_points` | 0.024 | 0.0000 | — |
| `plane_normal` | 47.91 | 153.82 | 有（平面拟合 / 平面点云属性计算）; 耗时未测 |
| `segment_plane` | 117.45 | 153.74 | 有（平面检测-深度图 / 平面拟合）; 耗时未测 |
| `select_by_index` | 2.862 | 0.051 | — |
| `select_by_mask` | 6.919 | 0.135 | 有（点云截取 / ROI / 掩膜相关）; 耗时未测 |
| `split_by_labels` | 316.14 | 452.33 | — |
| `split_by_plane` | 73.34 | 154.20 | 部分（点云截取 + 平面检测组合）; 耗时未测 |

### 四、配准类 (registration)

| 算法 | Open3D (ms) | PCL (ms) | 海康 VM3D |
|---|---:|---:|---|
| `evaluate_registration` | 2.142 | 10.51 | 部分（配准定位-深度图 / 数据统计）; 耗时未测 |
| `is_registration_valid` | 2.139 | 29.28 | 部分（条件检测 / 逻辑工具）; 耗时未测 |
| `make_init_pose` | 0.0016 | 0.0000 | 部分（位置修正-深度图 / 标定与坐标转换）; 耗时未测 |
| `register_fast_global` | 781.46 | — | — |
| `register_fpfh_ransac` | 832.25 | 5626.3 | — |
| `register_icp_colored` | 40.32 | — | — |
| `register_icp_generalized` | 15.98 | 73.08 | — |
| `register_icp_point_to_plane` | 16.22 | 145.96 | — |
| `register_icp_point_to_point` | 15.48 | 139.58 | — |

## 简要结论

- **IO/变换**: PCL 在 `save_cloud`/`transform_cloud`/`concat_clouds` 更快；Open3D 在 `load_cloud`/`txt_to_cloud` 更快，并独有 `depth_to_cloud`/`rgbd_to_cloud`。
- **预处理**: 体素降采样两者接近；法线/SOR 上 Open3D 通常更快；PCL 独有 `downsample_normal_space`；Open3D 独有 `downsample_fps`。
- **几何分割**: Open3D 在 `segment_plane`/`cluster_dbscan`/`fit_plane` 更快；PCL 独有 `fit_sphere`；PCL `fit_line`(RANSAC@100k) 极慢。
- **配准**: Open3D 在 ICP/GICP/FPFH-RANSAC/FGR/ColoredICP 全面更快，且覆盖更广；PCL 无 FGR/ColoredICP。
- **海康 VM3D**: 强项在工业深度图流程（转点云、降采样、截取合并、平面拟合、深度图配准定位）；缺少开源库中的 FPFH/ICP/DBSCAN/FPS 等细粒度点云算法同名实现，故这些项不输出耗时。

## 复现

```bash
cd benchmark
python3 scripts/generate_data.py
python3 scripts/bench_open3d.py
cmake -S . -B build && cmake --build build -j
./build/bench_pcl data results/pcl_timings.json
python3 scripts/make_report.py
```
