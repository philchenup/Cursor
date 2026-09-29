# Cursor

## M3T RealSense 单物体跟踪

Windows 上对应 pym3t `run_realsense_example.py` 的 C++ 实现见 [`m3t_realsense/`](m3t_realsense/README.md)。OpenCV 与 RealSense 已安装时，用 M3T 的 `RealSenseColorCamera` / `RegionModality` / `DepthModality` 实时跟踪 `.obj` 物体。

## ScaleAISShapeBy1000

将 OpenCASCADE 的 `AIS_Shape*` 缩小 1000 倍，并返回一个新的 `AIS_Shape*`。

```cpp
#include "ScaleAISShape.h"

AIS_Shape* ais = /* 已有对象 */;
AIS_Shape* scaled = ScaleAISShapeBy1000(ais);

// 推荐用 Handle 接管返回值，避免泄漏
Handle(AIS_Shape) scaledHandle = ScaleAISShapeBy1000(ais);
```
