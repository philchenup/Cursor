# Cursor

## 耿玉森 RCIM 2022 焊缝与轨迹

复现 Geng et al., *Robotics and Computer-Integrated Manufacturing*, 2022, 79:102433：

1. 改进 RANSAC：在种子点邻域内采样，顺序抽出多个平面并最小二乘精化
2. 相交平面求交线，用两侧支撑点截出焊缝点云
3. 沿交线等间距采样得到焊接轨迹（mm）
4. 用二面角角平分线作为焊枪接近方向（轨迹点的 `normal`）

```cpp
#include "geng_rcim2022.h"

GengRcim2022 detector;
detector.setInputCloud(cloud_mm);        // 坐标单位 mm
detector.compute();
auto seam = detector.seamCloud();        // 焊缝点云，mm
auto traj = detector.trajectoryCloud();  // 有序轨迹，normal = 焊枪 Z
```

```bash
mkdir -p build && cd build
cmake ..
make -j
./test_geng_rcim2022
./geng_rcim2022_demo --demo l geng_seam_mm.pcd geng_traj_mm.pcd
./geng_rcim2022_demo --demo t
./geng_rcim2022_demo --demo box
./geng_rcim2022_demo scan_mm.pcd seam.pcd traj.pcd
```

默认参数按 mm、点间距约 2–4 mm。`trajectory` 中每个点的 `xyz` 是焊点，`normal` 是二面角角平分线。

论文：https://doi.org/10.1016/j.rcim.2022.102433

## ScaleAISShapeBy1000

将 OpenCASCADE 的 `AIS_Shape*` 缩小 1000 倍，并返回一个新的 `AIS_Shape*`。

```cpp
#include "ScaleAISShape.h"

AIS_Shape* ais = /* 已有对象 */;
AIS_Shape* scaled = ScaleAISShapeBy1000(ais);

// 推荐用 Handle 接管返回值，避免泄漏
Handle(AIS_Shape) scaledHandle = ScaleAISShapeBy1000(ais);
```
