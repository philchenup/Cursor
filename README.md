# Cursor

## ScaleAISShapeBy1000

将 OpenCASCADE 的 `AIS_Shape*` 缩小 1000 倍，并返回一个新的 `AIS_Shape*`。

```cpp
#include "ScaleAISShape.h"

AIS_Shape* ais = /* 已有对象 */;
AIS_Shape* scaled = ScaleAISShapeBy1000(ais);

Handle(AIS_Shape) scaledHandle = ScaleAISShapeBy1000(ais);
```

## NexusVIT UI

`nexusvit_ui/` 是对照截图还原的 Qt 深色工业风界面（仅布局与风格，无业务功能）。构建见 `nexusvit_ui/README.md`。
