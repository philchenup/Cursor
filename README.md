# Cursor

## unwrapJointsNearestToSeed

`rl::mdl::JacobianInverseKinematics` 成功后会调用 `kinematic->normalize(q)`。转动关节用 `std::remainder(q, 2π)` 折到 **[-π, π]（±180°）**。限位即使是 ±360°，归一化后的值仍在限位内，解就不会回到另一圈。

同一末端位姿下，当前关节在 200°、IK 给出 -160°，关节空间规划会走出约 360° 的大幅度轨迹。

在 `solveIk` 成功后按 seed 展开回限位内最近圈：

```cpp
#include "UnwrapJointsNearest.h"

qOut = kinematic->getPosition();
unwrapJointsNearestToSeed(qOut, qSeed,
                          kinematic->getMinimum(),
                          kinematic->getMaximum());
```

带地轨时跳过第 0 轴：

```cpp
unwrapJointsNearestToSeed(qOut, qSeed,
                          kinematic->getMinimum(),
                          kinematic->getMaximum(),
                          /*prismaticCount=*/1);
```

角度仍是弧度。需要发给 JAKA（度）时，先展开再 `* 180 / π`，不要先折到 ±180°。

## ScaleAISShapeBy1000

将 OpenCASCADE 的 `AIS_Shape*` 缩小 1000 倍，并返回一个新的 `AIS_Shape*`。

```cpp
#include "ScaleAISShape.h"

AIS_Shape* ais = /* 已有对象 */;
AIS_Shape* scaled = ScaleAISShapeBy1000(ais);

// 推荐用 Handle 接管返回值，避免泄漏
Handle(AIS_Shape) scaledHandle = ScaleAISShapeBy1000(ais);
```
