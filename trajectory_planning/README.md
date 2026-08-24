# Thread：以法兰位姿 `Eigen::Affine3f` 做轨迹规划

在原 rlPlanDemo `Thread` 上增加 **目标点法兰位姿** 输入。规划不再要求事先写好 `planner->goal` 关节角：`run()` 先对 `T_base_flange` 做 Jacobian IK，再走原来的 RRT `verify()` / `solve()` / 优化 / 动画。

## 输入

| 项目 | 类型 | 含义 |
| --- | --- | --- |
| 目标 | `Eigen::Affine3f` | 基座系下的法兰位姿 `T_base_flange`（不是 TCP） |
| 起点 | `rl::math::Vector`（可选） | 起始关节角；未设置时用 `mdl->getPosition()` |
| IK 超时 | `int` ms | 默认 500 |

`Affine3f` 会 `cast` 成 `rl::math::Transform`（`rl::math::Real`，一般为 double）后交给 `JacobianInverseKinematics::addGoal(..., 0)`。单位与现有 RL 模型一致（工程里通常是 mm）。

## 流程

```
planToFlange(T_base_flange) / start()
        │
        ▼
  Affine3f → rl::math::Transform
        │
        ▼
  JacobianInverseKinematics(q_start)
        │
        ▼
  起止点碰撞检查 (SimpleModel)
        │
        ▼
  planner->start / goal = qStart / qGoal
        │
        ▼
  verify() → RRT solve() → optimizer → draw / animate
        │
        ▼
  planningFinished(path, solved, plannerMs)
```

未调用 `setTargetFlangePose` 时，行为与原 Thread 相同：使用已经写在 `planner->start` / `planner->goal` 上的关节角。

## 合入工程

用本目录的 `Thread.h` / `Thread.cpp` 替换工程中的同名文件（仍依赖 `MainWindow.h`、`Viewer.h`）。

主线程在得到目标法兰位姿后：

```cpp
this->thread->setStartConfiguration(this->mdl->getPosition()); // 或 getHomePosition()
this->thread->setIkTimeoutMs(500);
this->thread->planToFlange(T_base_flange);  // Eigen::Affine3f
```

或分步：

```cpp
this->thread->stop();
this->thread->setTargetFlangePose(T_base_flange);
this->thread->start();
```

结果：

- `Thread::lastPath` / `lastSolved` / `lastPlannerMs`
- 信号 `planningFinished(const rl::plan::VectorList& path, bool solved, double plannerMs)`

若手头是 TCP 位姿，先换成法兰再传入：

```cpp
// T_base_flange = T_base_tcp * T_flange_tcp^{-1}
rl::math::Transform T_base_flange = T_base_tcp * this->tcp_transform.inverse();
Eigen::Affine3f pose;
pose.matrix() = T_base_flange.matrix().cast<float>();
this->thread->planToFlange(pose);
```

完整调用示例见 [`snippets/MainWindow_planFromFlange.cpp`](snippets/MainWindow_planFromFlange.cpp)。

## `qStart = getPosition()` 崩在 `handmade_aligned_free`

这不是 `getPosition()` 本身写坏了内存，而是 **赋值左侧 `qStart` 里的堆指针已经是脏的**，Eigen 在扩容时去 `free` 那个指针。

根因：`Thread` 继承 `QThread`/`QObject`，Qt 的 `operator new` **不保证 16 字节对齐**。把 `Eigen::Affine3f`（内部是对齐的 4×4 float，会走 SSE/AVX 写）做成成员后：

1. `setTargetFlangePose()` 或构造时对 `Affine3f` 做对齐 SIMD 写
2. 写越界，破坏紧挨着的 `rl::math::Vector qStart` 的 `data()` 指针
3. `this->qStart = kinematic->getPosition()` 要先释放旧 buffer → `handmade_aligned_free` 读 `*(ptr-1)` 崩溃

因此法兰矩阵改为 `Eigen::Matrix<float, 4, 4, Eigen::DontAlign>` 存储；关节角用按元素拷贝，避免跨 RL DLL 时 Eigen 对动态 `Vector` 做 move/对齐释放。

**不要**在 `QObject` 子类上用 `EIGEN_MAKE_ALIGNED_OPERATOR_NEW` 来“补”对齐，那会和 Qt 自己的 `operator new` 冲突。

若去掉 `Affine3f` 成员后仍崩在同一处，再查工程与 Robotics Library 是否用了不同的 `/arch:AVX`（Eigen 的 `aligned_malloc`/`aligned_free` 会不一致）。

## 说明

- 本仓库无 Qt / RL 构建环境，需把文件拷回原工程后在 MSVC 下编译。
- `planner->start` / `goal` 指向 Thread 自己的 `qStart` / `qGoal`，避免 `solve()` 期间指针失效。
- 动画循环、`rl::plan::Viewer` 信号保持不变；不需要往返播放时把 `thread->animate = false`。
