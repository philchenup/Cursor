# Cursor

## JAKA 按轨迹伺服运行（C++ SDK V2.1.14）

轨迹类型为 `std::vector<std::vector<float>>`，且已经插值加密。控制器伺服周期为 8ms，按 **10 倍周期（80ms）** 发送一次数据。

```cpp
#include "JakaTrajectoryRunner.h"

std::vector<std::vector<float>> traj;  // 每点 6 个数；笛卡尔 [x,y,z,rx,ry,rz]（mm + rad）
JAKAZuRobot robot;
robot.login_in("192.168.2.194");
robot.power_on();
robot.enable_robot();

jaka_traj::RunOptions opt;
opt.space = jaka_traj::Space::Cartesian;  // 关节轨迹用 Space::Joint
jaka_traj::RunTrajectory(robot, traj, opt);
```

核心下发方式（手册 `servo_p` / `servo_j` 扩展接口，`step_num=10` → 80ms）：

```cpp
robot.servo_move_enable(TRUE);
for (const auto& pt : traj) {
    CartesianPose pose;  // 或 JointValue
    // ... 填入 pt[0..5]
    robot.servo_p(&pose, ABS, 10);  // 10 * 8ms = 80ms
    // 按 80ms 节拍发送
}
robot.servo_move_enable(FALSE);
```

- 头文件：`include/JakaTrajectoryRunner.h`
- 实现：`src/JakaTrajectoryRunner.cpp`
- 完整示例：`src/JakaTrajectoryExample.cpp`

编译时需链接本机 JAKA SDK（`JAKAZuRobot.h` / `jakaAPI`）。

## ScaleAISShapeBy1000

将 OpenCASCADE 的 `AIS_Shape*` 缩小 1000 倍，并返回一个新的 `AIS_Shape*`。

```cpp
#include "ScaleAISShape.h"

AIS_Shape* ais = /* 已有对象 */;
AIS_Shape* scaled = ScaleAISShapeBy1000(ais);

// 推荐用 Handle 接管返回值，避免泄漏
Handle(AIS_Shape) scaledHandle = ScaleAISShapeBy1000(ais);
```
