# Cursor

## 启动即最大化（且不卡顿）

`MainWindow` 构造里 `resize(1600, 1200)`，`main` 再 `showMaximized()`。Windows 过渡动画会连续 `Resize`，OpenInventor（`w.viewer` / SoQt）和 VTK 点云（`ui->cloudview`）每次都整帧重绘，所以从小窗口拉到最大化会卡。

```cpp
#include "fast_maximize.h"

QApplication a(argc, argv);
MainWindow w;
showMaximizedFast(&w, w.viewer, w.findChild<QWidget *>("cloudview"));
```

示例：`include/fast_maximize.h`、`src/qt_show_maximized_main.cpp`。建议删掉构造函数里的 `this->resize(1600, 1200);`。

## ScaleAISShapeBy1000

将 OpenCASCADE 的 `AIS_Shape*` 缩小 1000 倍，并返回一个新的 `AIS_Shape*`。

```cpp
#include "ScaleAISShape.h"

AIS_Shape* ais = /* 已有对象 */;
AIS_Shape* scaled = ScaleAISShapeBy1000(ais);

// 推荐用 Handle 接管返回值，避免泄漏
Handle(AIS_Shape) scaledHandle = ScaleAISShapeBy1000(ais);
```
