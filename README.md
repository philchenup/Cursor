# Cursor

## 启动即最大化（且不卡顿）

从小窗口拉到最大化会卡，是因为 Windows 过渡动画会连续发出几十次 `Resize`。每次都做完整布局，VTK / OCC / OpenGL 视口还会整帧重绘。

做法：关掉窗口过渡；先铺满工作区再 `showMaximized()`；缩放过程冻结重绘，3D 视口只在结束时渲染一次。

```cpp
#include "fast_maximize.h"

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;
    showMaximizedFast(&w, occView);  // 无 3D 视口可只传 &w
    return a.exec();
}
```

示例：`include/fast_maximize.h`、`src/qt_show_maximized_main.cpp`。

## ScaleAISShapeBy1000

将 OpenCASCADE 的 `AIS_Shape*` 缩小 1000 倍，并返回一个新的 `AIS_Shape*`。

```cpp
#include "ScaleAISShape.h"

AIS_Shape* ais = /* 已有对象 */;
AIS_Shape* scaled = ScaleAISShapeBy1000(ais);

// 推荐用 Handle 接管返回值，避免泄漏
Handle(AIS_Shape) scaledHandle = ScaleAISShapeBy1000(ais);
```
