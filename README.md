# Cursor

## ScaleAISShapeBy1000

将 OpenCASCADE 的 `AIS_Shape*` 缩小 1000 倍，并返回一个新的 `AIS_Shape*`。

```cpp
#include "ScaleAISShape.h"

AIS_Shape* ais = /* 已有对象 */;
AIS_Shape* scaled = ScaleAISShapeBy1000(ais);

Handle(AIS_Shape) scaledHandle = ScaleAISShapeBy1000(ais);
```

## JAKA Zu12 轨迹跟踪

机械臂已登录并使能、JAKA SDK 与 RL 已集成时，直接调用：

```cpp
#include "JakaZu12Trajectory.h"

errno_t ret = runJakaZu12Trajectory(robot, traj);
```

`traj` 为已经按 8ms 加密过的关节轨迹（每个点 6 维，单位 rad）。函数不再二次插值，按点以 `servo_j(ABS, 1)` 直接下发。
