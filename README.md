# Cursor

## gpTrsfToRl

不要调用 `gp_Trsf::Value()`（1-based，越界会 `Standard_OutOfRange`），也不要用 Eigen `<<` 去喂 `Value(1,1)`。对基点做 `Transformed`：

```cpp
const gp_Pnt o  = gp_Pnt(0, 0, 0).Transformed(trsf); // 平移
const gp_Pnt px = gp_Pnt(1, 0, 0).Transformed(trsf);
const gp_Pnt py = gp_Pnt(0, 1, 0).Transformed(trsf);
const gp_Pnt pz = gp_Pnt(0, 0, 1).Transformed(trsf);
// linear 的三列 = px-o, py-o, pz-o；translation = o
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
