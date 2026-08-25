# Cursor

## rlVectorToStdVector

将 Robotics Library 的 `rl::math::Vector`（Eigen 动态列向量，元素为 `double`）转换为 `std::vector<float>`。

```cpp
#include "RlVectorToStdVector.h"

rl::math::Vector q(6);
q << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6;

std::vector<float> joints = rlVectorToStdVector(q);
```

`rl::math::Vector3` / `Vector6` 等固定维度向量同样可转。空向量返回空的 `std::vector<float>`。

## ScaleAISShapeBy1000

将 OpenCASCADE 的 `AIS_Shape*` 缩小 1000 倍，并返回一个新的 `AIS_Shape*`。

```cpp
#include "ScaleAISShape.h"

AIS_Shape* ais = /* 已有对象 */;
AIS_Shape* scaled = ScaleAISShapeBy1000(ais);

// 推荐用 Handle 接管返回值，避免泄漏
Handle(AIS_Shape) scaledHandle = ScaleAISShapeBy1000(ais);
```
