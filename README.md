# Cursor

## 缩放时暂停 3D 刷新

在 `MainWindow` 里检测最大化 / 缩小：`resizeEvent` 和 `WindowStateChange` 时关掉 OpenInventor（`viewer->viewer->setAutoRedraw`）和 VTK（`cloudview` 中止渲染），尺寸稳定后再刷一次。

把头文件声明拷进 `mainwindow.h`，实现拷进 `mainwindow.cpp`，构造函数 `resize(1600, 1200)` 之后接上定时器。见 `snippets/MainWindow_pause_3d_on_resize.*`。

```cpp
void MainWindow::resizeEvent(QResizeEvent* event)
{
    pause3DRefresh();
    QMainWindow::resizeEvent(event);
    m_resizeIdle.start();
}
```

`main` 仍用 `w.showMaximized()`。

## ScaleAISShapeBy1000

将 OpenCASCADE 的 `AIS_Shape*` 缩小 1000 倍，并返回一个新的 `AIS_Shape*`。

```cpp
#include "ScaleAISShape.h"

AIS_Shape* ais = /* 已有对象 */;
AIS_Shape* scaled = ScaleAISShapeBy1000(ais);

// 推荐用 Handle 接管返回值，避免泄漏
Handle(AIS_Shape) scaledHandle = ScaleAISShapeBy1000(ais);
```
