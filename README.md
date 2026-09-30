# Cursor

## computeTwoPointPoses

输入 `pcl::PointCloud<pcl::PointNormal>`（`points[0]` / `points[1]` 为起终点，法向为该端枪轴初值）。

竖直或倾斜焊缝时，为避免机械臂拧姿态：

1. **正交 TCP**：X 是焊缝方向在 ⊥Z 上的投影，再 `Y = Z × X`，`Z = X × Y`。倾斜缝若把 X 设成三维起点→终点，旋转不正交，IK 会拧腕。
2. **陡焊缝不反行走**：`|travel · world_up|` 较大时禁止用 `Y·up < 0` 把 X、Y 一起取反。
3. **两端同号**：是否绕 Z 翻 180° 只决定一次，避免路径中 180° 扭转。
4. **枪略倾**：法向与 `preferred_torch`（默认世界 -Z）同侧时，最多再倾 `max_torch_tilt_deg`（立墙焊缝枪口略朝下）。异侧不倾，以免枪穿到工件背面。

```cpp
#include "ComputeTwoPointPoses.h"

pcl::PointCloud<pcl::PointNormal> trajectory;
Eigen::Affine3f pose_start, pose_end;
ComputeTwoPointPosesOptions opt;          // 可改 steep_seam_abs_cos / max_torch_tilt_deg
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
