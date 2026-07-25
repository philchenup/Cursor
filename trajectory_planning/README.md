# 起点 → 预焊接点 轨迹规划架构重构说明

## 一、为什么原 `Thread` 不适合当前工程

原 `Thread.h / Thread.cpp` 是从 RL 库示例程序 **rlPlanDemo** 移植的,它的设计前提与本焊接工程完全不同:

| 问题 | 原 Thread 的做法 | 与本工程的冲突 |
| --- | --- | --- |
| 线程模型 | 继承 `QThread` 并重写 `run()` | 工程内所有后台任务(`IKWorker`、`ProcessWorker`、`KukaCommunicator`、`ILaser`)统一采用 **QObject worker + `moveToThread`** 模式,由参数结构体 + `QMetaObject::invokeMethod` 驱动;两套模式混用增加维护成本,且 `QThread` 子类化方式无法方便地排队多次请求 |
| 数据获取 | 全程 `MainWindow::instance()->planner/model/...` 单例直取 | 强耦合、无法单元测试;且 worker 线程直接读写主窗口成员,线程边界不清晰 |
| 可视化 | 实现 `rl::plan::Viewer` 接口,发出 20 余个 `drawXxx` 信号给 Coin3D/SoQt 的 Viewer | 本工程用 **OCCT (`occtUpdate` + `flushSceneTimer`)** 渲染,不存在 SoQt Viewer,这些信号全部无消费者 |
| 结果输出 | 只把 `rl::plan::VectorList`(`std::list`)交给 Viewer 绘制,并在 `run()` 里用 `usleep` 死循环做往返动画 | 本工程的回放机制是主线程 `flushTrajTimer(20ms)` → `execSimulation()` 逐点消费 `std::vector<rl::math::Vector> wholeTrajectory`;需要的是**数据**而不是动画副作用 |
| 附加逻辑 | 写 `benchmark.csv`、`QApplication::quit()`、2 秒 `usleep` 展示起止位形 | 生产软件不需要 benchmark/退出逻辑,阻塞式 sleep 更不可接受 |

## 二、新架构:`PlanWorker`

新增 `tool/PlanWorker.h / .cpp`,**完全替代** `Thread.h / Thread.cpp`(旧文件可从工程中删除)。设计与 `IKWorker` 保持同构,做到工程内后台任务架构统一:

```
主线程 (MainWindow)                                worker 线程 (m_thread_planwork)
─────────────────────                              ────────────────────────────────
Trajectory()
  ├─ 构造 PlanToPreWeldParams
  │    q_start / mergedTraj[0] / tcp_transform ...
  └─ invokeMethod(doPlanToPreWeld) ────────────▶  doPlanToPreWeld(params)
                                                    ├─ QMutexLocker(MainWindow::mutex)
started ──▶ 停 flushSceneTimer、进度条清零 ◀──────  ├─ emit started()
                                                    ├─ 1. 预焊接点位姿 = 焊缝起点沿 -Z 后退 offset
                                                    ├─ 2. JacobianInverseKinematics 求 q_goal
                                                    ├─ 3. 起点/目标碰撞校验 (SimpleModel)
                                                    ├─ 4. AddRrtConCon::solve()   ← loadRobotWidget 初始化的规划器
                                                    ├─ 5. AdvancedOptimizer::process(path)
                                                    ├─ 6. VectorList → 按 interpStep 插值 →
                                                    │      std::vector<rl::math::Vector>
finished(trajectory, ms) ◀──────────────────────── └─ emit finished(...)
  ├─ 恢复 flushSceneTimer
  ├─ wholeTrajectory = trajectory        ← 第一段轨迹
  └─ invokeMethod(ikwork, "doSolve")     ← 用 trajectory.back() 作为焊缝段逆解的 q_initial,
                                            与原 finished_start 的链式逻辑一致
```

### 关键设计决策

1. **worker-object 而非 QThread 子类**:与工程现有模式统一;槽函数在 worker 线程事件循环中执行,天然支持排队、`isBusy()` 拒绝重入、析构时 `quit()+wait()` 优雅退出。
2. **上下文注入代替单例**:`loadRobotWidget()` 中初始化好的 `mdl / model(SimpleModel+ODE场景) / planner(AddRrtConCon) / optimizer(AdvancedOptimizer)` 通过 `setContext()` 一次性注入(shared_ptr 共享所有权),worker 不再反向依赖 `MainWindow`。
3. **共享模型的互斥**:RL 的 `SimpleModel`/`Kinematic` 非线程安全,规划期间 worker 持有 `MainWindow::mutex`,同时主线程收到 `started` 后暂停 `flushSceneTimer`(`occtUpdate` 会调用 `kinematic->forwardPosition()`),规划结束后恢复。规划结束前 worker 会把模型位姿复位到 `q_start`,避免画面跳变。
4. **输出即 `std::vector<rl::math::Vector>`**:RRT 输出的稀疏路径(`rl::plan::VectorList`,即 `std::list`)经 `model->distance / interpolate` 按 `interpStep` 插值成稠密关节轨迹,直接匹配 `execSimulation()` 每 20ms 消费一个点的回放机制,也可直接下发给 `KukaCommunicator`。
5. **失败路径全部显式化**:IK 失败 / 起止点碰撞 / `verify()` 失败 / RRT 超时 / 空轨迹分别通过 `failed(QString)` 报告到 console,进度通过 `progress(int)` 更新 `trajProgressBar`。
6. **预焊接点定义**:`preWeldBackOffset`(默认 50mm)表示从 `mergedTraj.front()` 沿焊接坐标系 `-Z` 方向后退的安全接近距离,设为 0 则直接以焊缝起点为目标。

## 三、MainWindow 集成改动清单

- `MainWindow.h`
  - 前置声明 `class PlanWorker;`
  - 新增成员 `PlanWorker* planworker; QThread* m_thread_planwork;`
- `MainWindow.cpp`
  - 构造函数初始化列表创建 `planworker` 与 `m_thread_planwork`
  - 析构函数中 `requestAbort()` + `quit()/wait()/delete`(与 `ikwork` 相同的清理顺序)
  - `initSimulation()`:`moveToThread`、`setContext(mdl, model, planner, optimizer, &mutex)`、连接 `started/progress/finished/failed/message` 信号;`finished` 中写入 `wholeTrajectory` 并链式触发 `ikwork->doSolve`(焊缝段)
  - `Trajectory()`:原来调用 `ikwork->doGoToStart`(关节步进式逼近)的分支替换为构造 `PlanToPreWeldParams` 并异步调用 `planworker->doPlanToPreWeld`
  - 删除了 `IKWorker::finished_start` 的连接(其职责由 `PlanWorker::finished` 承接;`IKWorker::doGoToStart` / `IKGoToStartParams` 不再使用,可后续清理)

## 四、迁移步骤

1. 将 `tool/PlanWorker.h / PlanWorker.cpp` 加入工程(qmake/CMake/vcxproj),放在与 `tool/IKWorker.*` 相同目录。
2. 从工程中移除 `Thread.h / Thread.cpp`(以及 rlPlanDemo 的 `Viewer.h/.cpp`,若无其他引用)。
3. 按本目录下的 `MainWindow.h / MainWindow.cpp` 同步集成改动。
4. 若工程尚未注册元类型,`PlanWorker` 构造函数已调用 `qRegisterMetaType<PlanToPreWeldParams>` 与 `qRegisterMetaType<std::vector<rl::math::Vector>>`,无需额外处理。

## 五、规划/优化算法选型(基于 [roboticslibrary/rl](https://github.com/roboticslibrary/rl))

约束条件:焊接场景为**静态已知环境**(工件 + 龙门 KR16),任务是**单查询**(起点 → 预焊接点)规划;`loadRobotWidget` 使用 **ODE 碰撞引擎**(`rl::sg::ode::Scene` 为 `SimpleScene`),只支持布尔碰撞查询,**不支持距离查询**。

| RL 算法 | 评估 | 结论 |
| --- | --- | --- |
| `RrtConCon`(RRT-Connect 双树) | 起点/目标各长一棵树交替贪心互连,静态场景单查询收敛最快、成功率最高,仅 `delta`/`epsilon` 两个参数 | **规划器选用** |
| `AddRrtConCon`(原用) | 自适应动态域采样只在窄通道场景占优;额外的 `alpha/lower/radius` 在混合单位空间难以整定(原 `radius=500°` 即失配值) | 兼容保留,窄通道时回退 |
| `Prm` / `PrmUtilityGuided` | 多查询路线图,工件更换即失效,前期采样开销大 | 不选 |
| `Rrt` / `RrtGoalBias` / `RrtCon` / `RrtExtCon` / `RrtExtExt` | 单树或保守双树,收敛慢于 Con-Con | 不选 |
| `Eet` | 面向工业场景的工作空间引导规划,但依赖 `DistanceModel` 距离查询与 `WorkspaceSphereExplorer`,ODE 引擎下不可用 | 换 PQP/Bullet/FCL 引擎后可再评估 |
| `AdvancedOptimizer` | RL 中唯一带细分的部分捷径优化,路径质量最好,mm 尺度调参后耗时可控 | **优化器选用**(参数见下节) |
| `SimpleOptimizer` | 贪心去点,最快但路径较长 | XML `<simpleOptimizer>` 开关备选 |

`loadRobotWidget` 现按 XML 节点名实例化规划器(`rrtConCon` / `addRrtConCon`),不再硬编码;`RrtConCon` 与 `AddRrtConCon` 同属 `RrtDual` 派生体系,近邻结构(双 GNAT)与 `PlanWorker` 调用的 `Planner` 接口均无需改动。

## 六、优化器调优:`optimizer->process(path)` 慢的根因与参数设置

`AdvancedOptimizer::process` 的实际算法(RL 源码):先把路径所有段**细分到 ≤ `length`**,然后反复扫描所有相邻三点 `(i, j, k)`,**每个三元组都先调一次 `verifier->isColliding(i, k)`**(RecursiveVerifier 以 `delta` 步长做碰撞检测),可行且中间点偏离直连线超过 `ratio·|ik|` 时删除 `j`;去点、细分交替进行直至收敛。

本工程关节空间是**混合单位**(地轨 mm + 6 转轴 rad),`model->distance` 被 mm 量级的地轨行程主导,而原配置的参数带 `unit="deg"`(按纯转轴机器人写的),换算后严重失配:

| 参数 | 原值 | 实际含义 | 后果 |
| --- | --- | --- | --- |
| `recursiveVerifier/delta` | 1° = **0.0175** | 地轨方向每 0.017mm 碰撞检测一次 | 每条候选捷径上万次碰撞查询;比规划器自身的检测分辨率(`delta` 10° = 0.175)还细 10 倍,毫无必要 |
| `length` | 500° = **8.73** | 每 ~9 单位(≈9mm 地轨)插一个细分点 | 几米长的路径 → 数百个路点,三元组数量爆炸,且每遍扫描每个三元组都要做一次完整 verifier 检测 |
| `ratio` | 0.3 | 偏差超过 30% 才去点 | 合理,保持 |

**推荐配置(见 `config/kuka16.xml`)**:

```xml
<advancedOptimizer>
    <recursiveVerifier>
        <delta>0.15</delta>   <!-- 原 0.0175, 与规划器分辨率同量级; 可调 0.1~0.2 -->
    </recursiveVerifier>
    <length>150</length>      <!-- 原 8.73, 约 150mm 一段; 可调 100~300 -->
    <ratio>0.3</ratio>
</advancedOptimizer>
```

碰撞查询量约下降 `(0.15/0.0175) × (150/8.73) ≈ 8.6 × 17 ≈ 150 倍`,优化耗时从"分钟级"降到"亚秒~秒级"。路径变粗不影响回放:`PlanWorker` 输出前会按 `interpStep` 重新做稠密插值。

**更快的选项**:`loadRobotWidget` 已支持在 XML 中用 `<simpleOptimizer>` 节点替换 `<advancedOptimizer>`,`SimpleOptimizer` 只做贪心去点(无细分、无 ratio 判断),通常几十次碰撞查询即可完成,适合对路径长度最优性要求不高的场合;也可在 `PlanToPreWeldParams` 中直接置 `optimize = false` 跳过优化。

## 七、可调参数

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `preWeldBackOffset` | 50.0 mm | 预焊接点沿焊接系 -Z 的后退距离 |
| `interpStep` | 1.0 | 输出轨迹插值步长(与 `model->distance` 同量纲,含地轨 mm 与关节 rad 的加权);越小回放越慢越平滑 |
| `plannerTimeoutSec` | 30 s | RRT 求解时间上限(原 Thread 硬编码 100s) |
| `optimize` | true | 是否执行 `AdvancedOptimizer` 路径缩短/平滑 |
| `ikTimeoutMs` | 500 ms | 预焊接点逆解超时 |
