# Cursor

## 法兰位姿（mm + ABC）

```cpp
mdl->forwardPosition();
const auto& T = mdl->getOperationalPosition(0);
const auto p = T.translation() * 1000.0;
const auto abc = T.rotation().eulerAngles(2, 1, 0).reverse() * rl::math::RAD2DEG;
std::cout << p.x() << " " << p.y() << " " << p.z() << " "
          << abc.x() << " " << abc.y() << " " << abc.z() << "\n";
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
