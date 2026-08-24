# Thread：按 `doGoToStart` 规划法兰位姿

对照 `IKWorker::doGoToStart`，把 `Thread::run()` 改成同一条路径。输入由 `DiscretePoint startPoint`（TCP）换成 **`Eigen::Affine3f T_base_flange`**（法兰），IK 超时 **500ms**。

把 `include/GlobalDefs_IKGoToStartParams.h` 粘进工程 `GlobalDefs.h`，替换原 `IKGoToStartParams`。

## 参数（与 `IKGoToStartParams` 对应）

| 字段 | 类型 | 默认 | 含义 |
| --- | --- | --- | --- |
| `q_home` | `rl::math::Vector` | `mdl->getHomePosition()` | Home 关节角（含地轨） |
| `T_base_flange` | `Eigen::Affine3f` | 必填 | 目标点法兰位姿，替代 `DiscretePoint` |
| `T_flange_to_tcp` | `rl::math::Transform` | Identity | 法兰 → TCP，只用于地轨 Y 对齐 |
| `railStepLen` | double | 5.0 | path1 地轨步长 (mm) |
| `cartStepLen` | double | 5.0 | path2 插值步长 |
| `timeoutMs` | int | **500** | Jacobian IK 超时 |

## 流程（与 `doGoToStart` 一致）

```
T_base_flange (Affine3f)
        │
        ├─ T_tcp = T_flange * T_flange_to_tcp
        ├─ yTarget = clamp(T_tcp.y, rail min/max)
        │
        ▼
path1  只动 Joint0，其它关节保持 q_home
        │
        ▼
锁 Joint0 = yTarget ± 1e-6
        │
        ▼
IK 500ms  addGoal(T_base_flange)   ← 法兰，不再 DiscretePoint / TCP
        │
        ▼
path2  RRT：path1.back() → qGoal（Joint0 锁定，optimizer 可选）
        │
        ▼
path2 各点 q(0) = yTarget，恢复地轨限位
        │
        ▼
拼接 path1 + path2 → lastPath → 动画
```

## 调用

```cpp
IKGoToStartParams p;
p.q_home = this->mdl->getHomePosition();
p.T_base_flange = T_base_flange;          // Eigen::Affine3f
p.T_flange_to_tcp = this->tcp_transform;
p.railStepLen = 5.0;
p.cartStepLen = 5.0;
p.timeoutMs = 500;

this->thread->planGoToStart(
    p.q_home, p.T_base_flange, p.T_flange_to_tcp,
    p.railStepLen, p.cartStepLen, p.timeoutMs);
```

未设置法兰位姿时，`Thread::run()` 仍走原来的 `planner->verify()/solve()`。

## 对齐

`Thread` 是 `QThread`，不能把 `Affine3f` / `rl::math::Transform` 做成成员。法兰和 TCP 偏移存在 `DontAlign` 矩阵里；关节角按元素拷贝。详见上一节 `handmade_aligned_free` 说明。
