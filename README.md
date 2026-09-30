# Cursor

## computeTwoPointPoses / computeWeldTcpStartEnd

工件放在地面上采点云（`world_up = +Z`）。焊点 **Y 朝上** 用来确定起点/终点。

- **平焊 PA**：X 沿缝；起终点绕 Y 向焊缝内倾 30°，不绕 X，避免枪体撞端壁/侧壁。
- **立焊 PF**：Y 自下而上沿缝，X 水平；不内倾。

```cpp
ComputeTwoPointPosesOptions opt;
computeTwoPointPoses(pose_start, pose_end, opt);

computeWeldTcpStartEnd(tcp_weld_start, tcp_weld_end);  // 默认内倾 30°
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
