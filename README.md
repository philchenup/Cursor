# Cursor

## 耿玉森 RCIM 2022 焊缝与轨迹

复现 Geng et al., *Robotics and Computer-Integrated Manufacturing*, 2022, 79:102433：

1. 改进 RANSAC：在种子点邻域内采样，顺序抽出多个平面并最小二乘精化
2. 分割后清洗每个面：点归最近平面、局部法向与面法向不一致的丢掉、只留最大连通块（去掉共面但不属于这块面的点）
3. 先把各分割面点云拼起来，再只从这两张面的点中求交线抽缝（不从原始点云抽）
4. 沿交线等间距采样得到焊接轨迹（mm）
5. 用二面角角平分线作为焊枪接近方向（轨迹点的 `normal`）
6. 起点、终点的焊枪姿态再朝焊缝内部倾斜 45°（`end_tilt_deg`），避开端头其它面。`tiltTorchInward` 为 `static`，可在 `const` 成员函数中调用。
7. 焊缝长度取两张分割面沿交线的完整重叠区间，并延伸到附近的第三面交点，保证箱角等端头接到焊缝末端。

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

默认参数按 mm、点间距约 2–4 mm。`trajectory` 中每个点的 `xyz` 是焊点；中间点 `normal` 是二面角角平分线，起终点再向缝内倾 `end_tilt_deg`（默认 45°）。焊缝起终点取两侧支撑点沿交线的重叠区间，不再额外裁两端。某块分割面里混进其它面的点时，收紧 `max_normal_dev_deg`（默认 30°）或 `plane_cluster_tol_mm`（默认 8 mm）。

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
