# Cursor

## gpTrsfToRl

把 OpenCASCADE 的 `gp_Trsf` 转成 Robotics Library 的 `rl::math::Transform`。只输出刚体（单位旋转 + 平移），供 `body->setFrame()` 使用。

旧写法把 `VectorialPart()`（旋转乘上 scale）逐元素写进 `T(row, col)`。3x3 一旦带比例、反射或 NaN，Inventor / Bullet 分解矩阵就会崩溃。

```cpp
#include "AisRlBodyBinding.h"

rl::math::Transform T = gpTrsfToRl(ais->Transformation());
body->setFrame(T);
```

失败（非有限分量、旋转无法正交化、OCCT 异常）时返回 `Identity`，不把异常抛给调用方。几何上的 1/1000 缩放应先烤进拓扑（`ScaleAISShapeBy1000`），不要放进 RL 的 frame。

## ScaleAISShapeBy1000

将 OpenCASCADE 的 `AIS_Shape*` 缩小 1000 倍，并返回一个新的 `AIS_Shape*`。

```cpp
#include "ScaleAISShape.h"

AIS_Shape* ais = /* 已有对象 */;
AIS_Shape* scaled = ScaleAISShapeBy1000(ais);

// 推荐用 Handle 接管返回值，避免泄漏
Handle(AIS_Shape) scaledHandle = ScaleAISShapeBy1000(ais);
```
