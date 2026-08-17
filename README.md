# Cursor

## gpTrsfToRl

只用 `gp_Trsf::Value(row, col)`（1-based 的 3×4）。无参 `GetRotation()` 在旧版 OCCT 不存在，`TranslationPart()` 返回 `const gp_XYZ&`，拷贝都会编不过。

```cpp
rl::math::Transform T;
T.setIdentity();
T.linear() <<
	trsf.Value(1, 1), trsf.Value(1, 2), trsf.Value(1, 3),
	trsf.Value(2, 1), trsf.Value(2, 2), trsf.Value(2, 3),
	trsf.Value(3, 1), trsf.Value(3, 2), trsf.Value(3, 3);
T.translation() = rl::math::Vector3(
	trsf.Value(1, 4), trsf.Value(2, 4), trsf.Value(3, 4));
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
