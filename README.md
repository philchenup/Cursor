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

`traj` 为 `std::vector<rl::math::Vector>`，每个点 6 维关节角（rad）。函数按 JAKA 官方 8ms `servo_j` 周期线性加密后连续下发，并在进入伺服前设置 `servo_move_use_joint_LPF(0.5)`。
