# Cursor

## gpTrsfToRl

`gp_Trsf` ↔ `rl::math::Transform` 按 3×4 逐元素互逆（与 `gp_Trsf::SetValues` 同一套系数）：

```cpp
rl::math::Transform tf = gpTrsfToRl(ais->Transformation());

gp_Trsf T0;
T0.SetValues(tf(0, 0), tf(0, 1), tf(0, 2), tf(0, 3),
	tf(1, 0), tf(1, 1), tf(1, 2), tf(1, 3),
	tf(2, 0), tf(2, 1), tf(2, 2), tf(2, 3));
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
