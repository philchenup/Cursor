# Cursor

## getFlangePoseMmAbc

用 `MainWindow::mdl`（`rl::mdl::Kinematic`）取末端法兰位姿，输出 **mm + ABC 度**。与 `OperationalModel::data` 同一套欧拉角。

```cpp
// 关节更新后
mdl->setPosition(q);
mdl->forwardPosition();

FlangePoseMmAbc pose;
auto* kin = dynamic_cast<rl::mdl::Kinematic*>(mdl.get());
getFlangePoseMmAbc(kin, pose);   // X,Y,Z [mm], A,B,C [deg]
// R = Rz(C) * Ry(B) * Rx(A)
```

MainWindow 封装见 `snippets/MainWindow_getFlangePoseMmAbc.cpp`。

RL 平移是米，函数内 `* 1000`。若 rlmdl 本身已是毫米，不要再乘。

## ScaleAISShapeBy1000

将 OpenCASCADE 的 `AIS_Shape*` 缩小 1000 倍，并返回一个新的 `AIS_Shape*`。

```cpp
#include "ScaleAISShape.h"

AIS_Shape* ais = /* 已有对象 */;
AIS_Shape* scaled = ScaleAISShapeBy1000(ais);

// 推荐用 Handle 接管返回值，避免泄漏
Handle(AIS_Shape) scaledHandle = ScaleAISShapeBy1000(ais);
```
