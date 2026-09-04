# Cursor

## pcl_modules

不依赖 Qt 的 PCL 点云处理模块（滤波 / 特征 / 分割 / 曲面），仅用 PCL 即可在 Windows 上 CMake 编译。见 [pcl_modules/README.md](pcl_modules/README.md)。

## ScaleAISShapeBy1000

将 OpenCASCADE 的 `AIS_Shape*` 缩小 1000 倍，并返回一个新的 `AIS_Shape*`。

```cpp
#include "ScaleAISShape.h"

AIS_Shape* ais = /* 已有对象 */;
AIS_Shape* scaled = ScaleAISShapeBy1000(ais);

// 推荐用 Handle 接管返回值，避免泄漏
Handle(AIS_Shape) scaledHandle = ScaleAISShapeBy1000(ais);
```
