# Cursor

## NexusVIT Command Workspace UI

原界面是左右堆叠面板 + 彩色大图标工具条。新界面改成视口优先的 Command Workspace（底部控制甲板），**所有原按钮保留、objectName 不变**，背景仍为炭灰。

详见 [README_UI.md](README_UI.md)，交互预览：`preview/nexusvit-workspace.html`。

## ScaleAISShapeBy1000

将 OpenCASCADE 的 `AIS_Shape*` 缩小 1000 倍，并返回一个新的 `AIS_Shape*`。

```cpp
#include "ScaleAISShape.h"

AIS_Shape* ais = /* 已有对象 */;
AIS_Shape* scaled = ScaleAISShapeBy1000(ais);

// 推荐用 Handle 接管返回值，避免泄漏
Handle(AIS_Shape) scaledHandle = ScaleAISShapeBy1000(ais);
```
