# Cursor

## ScaleAISShapeBy1000

将 OpenCASCADE 的 `AIS_Shape*` 缩小 1000 倍，并返回一个新的 `AIS_Shape*`。

```cpp
#include "ScaleAISShape.h"

AIS_Shape* ais = /* 已有对象 */;
AIS_Shape* scaled = ScaleAISShapeBy1000(ais);

// 推荐用 Handle 接管返回值，避免泄漏
Handle(AIS_Shape) scaledHandle = ScaleAISShapeBy1000(ais);
```

## JAKA Zu12 流畅跟踪 `rl::math::Vector` 轨迹

给定 `std::vector<rl::math::Vector>` 轨迹点，调用 `jaka::runJakaZu12Trajectory` 即可让 Zu12 连续、平滑地跑完这段轨迹。

实现对照 JAKA 官方仓库：

- [jakasdk-cpp](https://github.com/JAKARobotics/jakasdk-cpp)：`servo_move_enable` + `servo_j`
- [jaka_ros2 `moveit_server.cpp`](https://github.com/JAKARobotics/jaka_ros2/blob/main/src/jaka_planner/src/moveit_server.cpp)：`servo_move_use_joint_LPF(0.5)`，再按 `step_num = dt / 0.008` 下发 `servo_j(ABS)`
- [jaka_ros2 `servoj_demo.cpp`](https://github.com/JAKARobotics/jaka_ros2/blob/main/src/jaka_driver/src/servoj_demo.cpp)：先使能伺服，再连续发送目标

JAKA 控制器在伺服模式下**不做规划**，周期为 **8ms**。本函数因此会：

1. 在关节空间做端点速度为 0 的钳制三次样条
2. 按 8ms 重采样，并限制关节速度（Zu12 常用 120/180 deg/s，默认再乘 0.5）
3. 必要时先 `joint_move` 到首点
4. 设置 LPF / 速度前视滤波后，按时序调用 `servo_j`

```cpp
#include "JakaZu12Trajectory.h"

JAKAZuRobot robot;
robot.login_in("192.168.137.100");
robot.power_on();
robot.enable_robot();

std::vector<rl::math::Vector> traj = /* 6 维关节角，单位 rad */;
jaka::JakaZu12TrajectoryOptions opt;
opt.space = jaka::JakaZu12Space::Joint;          // 笛卡尔用 Cartesian，平移默认米
opt.velocity_scale = 0.4;

errno_t ret = jaka::runJakaZu12Trajectory(robot, traj, opt);
```

规划本身不依赖真机，可单独测试：

```bash
cmake -S . -B build -DCMAKE_CXX_COMPILER=g++
cmake --build build
./build/jaka_zu12_traj_test
```
