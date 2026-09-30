# Cursor

## 缩放时冻结 3D 视口尺寸

只停绘制不够：放大时 Inventor / VTK 仍按中间尺寸重建 FBO。缩放过程中 `setFixedSize` 锁住两个 3D 窗口，并关掉 Windows 最大化动画；150ms 后再解开只刷新一次。见 `snippets/MainWindow_pause_3d_on_resize.cpp`。

## ScaleAISShapeBy1000

将 OpenCASCADE 的 `AIS_Shape*` 缩小 1000 倍，并返回一个新的 `AIS_Shape*`。

```cpp
#include "ScaleAISShape.h"

AIS_Shape* ais = /* 已有对象 */;
AIS_Shape* scaled = ScaleAISShapeBy1000(ais);

// 推荐用 Handle 接管返回值，避免泄漏
Handle(AIS_Shape) scaledHandle = ScaleAISShapeBy1000(ais);
```
