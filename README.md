# Cursor

## CED 焊缝点云检测

参考 [CED_Detector](https://github.com/UCR-Robotics/CED_Detector) 的质心距离显著性，但只输出**两个点云面之间的焊缝**，不输出单独一块点云的自由边缘。

原始 CED 会把两类点都当成关键点：

1. **自由边**：邻域被截成半盘，质心偏向面内，显著性高。
2. **焊缝 / 两面交线**：邻域同时落到两个平面上，质心也会偏移。

本仓库在 CED 预筛选之后增加「双表面支撑」判定：邻域法向必须能分成两个平面簇，二面角落在焊缝范围内，并且该点靠近两平面交线。只有一个平面支撑的边缘会被丢掉。默认不做非极大值抑制，因此结果是一条稠密焊缝点云。

```cpp
#include "ced_weld_seam.h"

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr seam (new pcl::PointCloud<pcl::PointXYZ>);

pcl::CEDWeldSeamDetector<pcl::PointXYZ, pcl::PointXYZ> detector;
detector.setRadiusSearch (30.0);          // mm：约 3~5 倍点间距
detector.setCentroidThreshold (0.10);     // 无量纲，毫米点云也不用改
detector.setSupportRadius (40.0);         // mm：可省略，默认 1.3 * radius
detector.setDihedralAngleRange (25.0, 155.0);
detector.setInputCloud (cloud);
detector.compute (*seam);
```

对比原始 CED-3D（会包含自由边）：

```cpp
#include "ced_3d.h"

pcl::CEDKeypoint3D<pcl::PointXYZ, pcl::PointXYZ> ced3d;
ced3d.setRadiusSearch (50.0);             // mm
ced3d.setNonMaxRadius (50.0);
ced3d.setCentroidThreshold (0.2);
ced3d.setInputCloud (cloud);
ced3d.compute (*keypoints);
```

### 构建与运行

需要 PCL >= 1.8（本环境已验证 PCL 1.14）。

```bash
mkdir -p build && cd build
cmake ..
make -j
./test_ced_weld_seam
./detect_weld_seam --demo weld_seam.pcd --save-ced ced_keypoints.pcd --save-demo demo_cloud.pcd
./detect_weld_seam input.pcd weld_seam.pcd --unit mm --radius 30 --centroid 0.10
```

`--demo` 会生成一个 L 形对接点云：两块互相垂直的平板沿 Y 轴相交。原始 CED 会标出所有自由边，焊缝检测器只保留交线上的点。

### 点云单位为 mm 时怎么改参数

长度类参数必须和点云坐标用同一单位；比例、角度、点数不用乘 1000。未显式设置的半径/带宽会按 `setRadiusSearch` 自动缩放，所以毫米点云通常只改这一项。

| 参数 | 米（上一版示例） | 毫米 | 是否要改 |
| --- | --- | --- | --- |
| `setRadiusSearch` | 0.03 | **30** | 要，约 3~5 倍点间距 |
| `setSupportRadius` | 0.04 | **40** | 要；也可不设，默认 1.3×半径 |
| `setNormalRadius` | 0.018 | **18** | 可省略，默认 0.6×半径 |
| `setPlaneDistanceThreshold` | 0.0054 | **5.4** | 可省略，默认 0.18×半径 |
| `setSeamBandWidth` | 0.0105 | **10.5** | 可省略，默认 0.35×半径 |
| `setClusterGapRadius` | 0.03 | **30** | 可省略，默认等于半径 |
| `setCentroidThreshold` | 0.10 | 0.10 | 不用改 |
| `setDihedralAngleRange` | 25, 155 | 25, 155 | 不用改 |
| `setMinNeighbors` / `setMinSeamClusterSize` | 8 / 12 | 8 / 12 | 不用改 |

若实际点间距不是约 10mm，不要套用 30，按 `半径 ≈ 3~5 × 点间距` 重算。例如点间距 1mm 时用 `setRadiusSearch(4.0)`。

命令行对应：

```bash
./detect_weld_seam scan_mm.pcd weld_seam.pcd --unit mm --radius 30
```

`--unit mm` 只影响默认半径（0.05 → 50）；只要写了 `--radius`，就按你给的毫米值用。

### 主要参数

| 参数 | 含义 | 建议 |
| --- | --- | --- |
| `setRadiusSearch` | CED 邻域半径（与点云同单位） | 点间距的 3~5 倍 |
| `setCentroidThreshold` | 质心偏移 / 半径 | 0.08~0.15，过大会漏焊缝 |
| `setSupportRadius` | 双平面分析半径 | 略大于搜索半径 |
| `setDihedralAngleRange` | 两面夹角范围（度） | 常见坡口 25~155 |
| `setSeamBandWidth` | 允许偏离交线的宽度 | 控制缝带厚度 |
| `setMinSeamClusterSize` | 最小连通焊缝点数 | 去掉零星误检 |
| `setNonMaxSuppression` | 是否再做稀疏关键点 | 默认关闭 |

## 耿玉森 RCIM 2022 焊缝与轨迹

复现 Geng et al., *Robotics and Computer-Integrated Manufacturing*, 2022, 79:102433：

1. 改进 RANSAC：在种子点邻域内采样，顺序抽出多个平面并最小二乘精化  
2. 相交平面求交线，用两侧支撑点截出焊缝点云  
3. 沿交线等间距采样得到焊接轨迹（mm）  
4. 用二面角角平分线作为焊枪接近方向（轨迹点的 `normal`）

```cpp
#include "geng_rcim2022.h"

GengRcim2022 detector;
detector.setInputCloud(cloud_mm);   // 坐标单位 mm
detector.compute();
auto seam = detector.seamCloud();   // 焊缝点云，mm
auto traj = detector.trajectoryCloud();  // 有序轨迹，normal = 焊枪 Z
```

```bash
./geng_rcim2022_demo --demo l geng_seam_mm.pcd geng_traj_mm.pcd
./geng_rcim2022_demo --demo t
./geng_rcim2022_demo --demo box
./geng_rcim2022_demo scan_mm.pcd seam.pcd traj.pcd
./test_geng_rcim2022
```

默认参数按 mm、点间距约 2–4 mm 设置。`trajectory` 中每个点的 `xyz` 是焊点，`normal` 是二面角角平分线（焊枪接近方向）。

## ScaleAISShapeBy1000

将 OpenCASCADE 的 `AIS_Shape*` 缩小 1000 倍，并返回一个新的 `AIS_Shape*`。

```cpp
#include "ScaleAISShape.h"

AIS_Shape* ais = /* 已有对象 */;
AIS_Shape* scaled = ScaleAISShapeBy1000(ais);

// 推荐用 Handle 接管返回值，避免泄漏
Handle(AIS_Shape) scaledHandle = ScaleAISShapeBy1000(ais);
```
