# Cursor

## ScaleAISShape

以原点缩放 `AIS_Shape*`，并把颜色同步到新对象上。

缩放会生成新的 BRep，原来绑在面上的 `AIS_ColoredShape` 自定义颜色必须按
`BRepBuilderAPI_Transform::ModifiedShape` 映射到新拓扑，否则缩小后只剩底色。

```cpp
#include "ScaleAISShape.h"

AIS_Shape* ais = /* 已有对象，可以是 AIS_ColoredShape */;
AIS_Shape* scaled = ScaleAISShape(ais, 0.001f);   // 缩小 1000 倍
AIS_Shape* same = ScaleAISShapeBy1000(ais);       // 同上

// 推荐用 Handle 接管返回值，避免泄漏
Handle(AIS_Shape) scaledHandle = ScaleAISShape(ais, 0.001f);
```

作为 `ReadModel::ScaleAISShape` 使用时，函数体与 `ScaleAISShape()` 相同：
整体 `SetColor` + 分面 `SetCustomColor` 都会跟到缩放后的形状上。
