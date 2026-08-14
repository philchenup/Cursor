# 原点 → 焊接起点

`IKWorker::doGoToStart` 分两段：

1. **path1**：只动地轨 Joint0，其它关节保持 Home，插值步长 10 mm，剩余不足 10 mm 直接到终点（与焊点 Y 重合）。
2. **path2**：path1 终点为起点、焊接起点为终点，按 `Thread::run` 调用 `planner->verify/solve/getPath`，再用 `model->interpolate` 加密。
3. 拼接后对每个关节点算工具系原点 `T_flange * T_flange_to_tcp` 的 XYZ。

`finished_start` 增加 `toolPoints`，MainWindow 需改连接：

```cpp
connect(ikwork, &IKWorker::finished_start, this,
    [=](const std::vector<rl::math::Vector>& jointTrajectory,
        const std::vector<rl::math::Vector3>& toolPoints,
        const double& ratio) {
        if (jointTrajectory.size() < 1) return;
        wholeTrajectory.clear();
        wholeTrajectory.insert(wholeTrajectory.end(),
            jointTrajectory.begin(), jointTrajectory.end());
        // toolPoints[i] 与 jointTrajectory[i] 一一对应
        ...
    }, Qt::QueuedConnection);
```
