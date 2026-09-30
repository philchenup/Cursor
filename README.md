# Cursor

## 缩放时暂停 3D 刷新

`resizeEvent` 里关掉 Inventor / VTK 刷新，100ms 内不再变尺寸再打开。见 `snippets/MainWindow_pause_3d_on_resize.cpp`。

```cpp
void MainWindow::set3DRefresh(bool on)
{
    if (viewer && viewer->viewer) {
        viewer->viewer->setAutoRedraw(on);
        if (on) viewer->viewer->render();
    }
    if (ui->cloudview) {
        ui->cloudview->setUpdatesEnabled(on);
        if (on) ui->cloudview->update();
    }
}

void MainWindow::resizeEvent(QResizeEvent* e)
{
    set3DRefresh(false);
    QMainWindow::resizeEvent(e);
    m_resizeIdle.start(100);
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
