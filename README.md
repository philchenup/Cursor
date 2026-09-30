# Cursor

## SeamExtra / trajectoryCloud

点云提取焊缝后，起终点 TCP 已按地面工件规则修正。结果直接从 `trajectoryCloud()` 取。

- **平焊**：Y 朝上定顺序，两端绕 Y 内倾 30°。
- **立焊**：自下而上；起终点绕 X 同向朝上坡倾 30°（终点不反向朝起点）。

```cpp
#include "seamExtractor.h"

SeamExtra extractor;
extractor.setInputCloud(cloud);
if (!extractor.compute())
    return;

for (const auto& se : extractor.trajectoryCloud()) {
    const Eigen::Affine3f& start = se.first;  // 焊枪起点 TCP
    const Eigen::Affine3f& end = se.second;   // 焊枪终点 TCP
}
```

`computeWeldTcpStartEnd` 是 `computeTwoPointPoses` 的薄封装，`SeamExtra::fillSeamPoses` 内部调用它。

## computeTwoPointPoses

工件放在地面上采点云（`world_up = +Z`）。焊点 **Y 朝上** 用来确定起点/终点。

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
