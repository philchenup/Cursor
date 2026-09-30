# Cursor

## computeTwoPointPoses

输入 `pcl::PointCloud<pcl::PointNormal>`（`points[0]` / `points[1]` 为起终点，法向为该端枪轴初值）。

免示教里 **路径方向** 和 **枪头朝向** 必须拆开。鹅颈枪头大致沿工具 +X，若 TCP-X 绑死行走，X 朝天则枪头朝天。

1. **工艺定行走**：默认 `PreferDownhill`（高→低，ISO PG）。厚板改 `PreferUphill`。不要用绕 Z 转 180° 选方向。
2. **重力定行走角**：绕工具 Y 倾 `travel_angle_deg`（默认 10°），符号取枪头更朝下的一侧，上坡下坡都适用。
3. **正交 TCP**：X 是行走在 ⊥Z 上的投影。陡缝默认不翻 X。
4. **枪略倾**：法向与 `preferred_torch`（默认 −Z）同侧时最多再倾 25°。

```cpp
ComputeTwoPointPosesOptions opt;
opt.travel_policy = WeldTravelPolicy::PreferDownhill;  // 或 KeepGiven / PreferUphill
opt.travel_angle_deg = 10.f;
opt.tool_head_axis = Eigen::Vector3f::UnitX();         // 鹅颈；喷嘴沿 Z 则 UnitZ()
computeTwoPointPoses(trajectory, pose_start, pose_end, opt);
```

## ScaleAISShapeBy1000

将 OpenCASCADE 的 `AIS_Shape*` 缩小 1000 倍，并返回一个新的 `AIS_Shape*`。

```cpp
#include "ScaleAISShape.h"

AIS_Shape* ais = /* 已有对象 */;
AIS_Shape* scaled = ScaleAISShapeBy1000(ais);

// 推荐用 Handle 接管返回值，避免泄漏
Handle(AIS_Shape) scaledHandle = ScaleAISShapeBy1000(ais);
```
