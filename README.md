# Cursor

## computeTwoPointPoses

输入 `pcl::PointCloud<pcl::PointNormal>` 轨迹云（`points[0]` 起点、`points[1]` 终点，法向即该端 Z）。  
X 为 `normalize(终点 − 起点)`，不向 ⊥Z 投影；`Y = Z × X`，若 `Y·world_Z < 0` 则 X、Y 一起取反。

```cpp
#include "ComputeTwoPointPoses.h"

pcl::PointCloud<pcl::PointNormal> trajectory;  // 至少两个点
Eigen::Affine3f pose_start, pose_end;
if (computeTwoPointPoses(trajectory, pose_start, pose_end)) {
    // pose_start / pose_end 的 X 沿起点→终点
}
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
