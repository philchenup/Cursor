# Cursor

## computeTwoPointPoses

输入 `pcl::PointCloud<pcl::PointNormal>`（`points[0]` / `points[1]` 为起终点，法向为该端枪轴初值）。

工业上先按焊缝相对重力分类，再给不同的 TCP 轴：

| 位置 | 判定 | TCP | 行走 |
|---|---|---|---|
| **平焊 PA** | 缝几乎水平 | **X = 沿缝**，Y 侧向，Z 指向工件 | 水平；斜板可用下坡 |
| **立焊 PF** | `|缝方向 · 上|` 大 | **Y = 从下到上沿缝**，**X 水平**（鹅颈），Z 指向墙 | 下→上 |

立缝若仍用 X 作竖直行走，鹅颈（工具 +X）会朝天。把行走换到 Y，X 留在水平面，与平焊共用同一把枪的 TCP 定义（Z 永远是枪尖）。

```cpp
ComputeTwoPointPosesOptions opt;
opt.weld_position = WeldPosition::Auto;                 // 或 Flat / Vertical
opt.vertical_seam_abs_cos = 0.5f;                       // 约 60° 起算立缝
opt.travel_policy = WeldTravelPolicy::PreferDownhill;   // 只作用于平焊/斜板
opt.travel_angle_deg = 10.f;                            // 平焊绕 Y，立焊绕 X
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
