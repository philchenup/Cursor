# Cursor

## 缩放时暂停 3D 刷新

缩小快、放大卡：放大时 Windows 过渡会连续给出更大的中间尺寸，Inventor / VTK 每次都重新分配 FBO。上一版只停了绘制，40ms 定时器还会在动画中途整帧渲染。

这次：关掉窗口过渡；放大过程吞掉两个 3D 视口的 `Resize`；120ms 内不再变尺寸才分配一次 FBO 并刷新。

见 `snippets/MainWindow_pause_3d_on_resize.*`。构造函数里用 `setGeometry(availableGeometry())` 替换 `resize(1600, 1200)`。

## ScaleAISShapeBy1000

将 OpenCASCADE 的 `AIS_Shape*` 缩小 1000 倍，并返回一个新的 `AIS_Shape*`。

```cpp
#include "ScaleAISShape.h"

AIS_Shape* ais = /* 已有对象 */;
AIS_Shape* scaled = ScaleAISShapeBy1000(ais);

// 推荐用 Handle 接管返回值，避免泄漏
Handle(AIS_Shape) scaledHandle = ScaleAISShapeBy1000(ais);
```
