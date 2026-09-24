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
detector.setRadiusSearch (0.03);          // 约 3~5 倍点间距
detector.setCentroidThreshold (0.10);     // 低于原版 CED，以免漏掉焊缝点
detector.setSupportRadius (0.04);         // 双平面分析半径
detector.setDihedralAngleRange (25.0, 155.0);
detector.setInputCloud (cloud);
detector.compute (*seam);
```

对比原始 CED-3D（会包含自由边）：

```cpp
#include "ced_3d.h"

pcl::CEDKeypoint3D<pcl::PointXYZ, pcl::PointXYZ> ced3d;
ced3d.setRadiusSearch (0.05);
ced3d.setNonMaxRadius (0.05);
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
./detect_weld_seam input.pcd weld_seam.pcd --radius 0.03 --centroid 0.10
```

`--demo` 会生成一个 L 形对接点云：两块互相垂直的平板沿 Y 轴相交。原始 CED 会标出所有自由边，焊缝检测器只保留交线上的点。

### 主要参数

| 参数 | 含义 | 建议 |
| --- | --- | --- |
| `setRadiusSearch` | CED 邻域半径 | 点间距的 3~5 倍 |
| `setCentroidThreshold` | 质心偏移 / 半径 | 0.08~0.15，过大会漏焊缝 |
| `setSupportRadius` | 双平面分析半径 | 略大于搜索半径 |
| `setDihedralAngleRange` | 两面夹角范围（度） | 常见坡口 25~155 |
| `setSeamBandWidth` | 允许偏离交线的宽度 | 控制缝带厚度 |
| `setMinSeamClusterSize` | 最小连通焊缝点数 | 去掉零星误检 |
| `setNonMaxSuppression` | 是否再做稀疏关键点 | 默认关闭 |

## ScaleAISShapeBy1000

将 OpenCASCADE 的 `AIS_Shape*` 缩小 1000 倍，并返回一个新的 `AIS_Shape*`。

```cpp
#include "ScaleAISShape.h"

AIS_Shape* ais = /* 已有对象 */;
AIS_Shape* scaled = ScaleAISShapeBy1000(ais);

// 推荐用 Handle 接管返回值，避免泄漏
Handle(AIS_Shape) scaledHandle = ScaleAISShapeBy1000(ais);
```
