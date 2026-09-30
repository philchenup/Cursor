# Cursor

## computeTwoPointPoses

就地修正起点/终点 TCP，无需点云。平移是焊点，Z 是枪轴。

```cpp
ComputeTwoPointPosesOptions opt;
opt.torch_x = tcp.linear().col(0);  // 当前焊枪 +X；不填则不锁 X
computeTwoPointPoses(pose_start, pose_end, opt);
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
