#ifndef PLANWORKER_H
#define PLANWORKER_H

//
// PlanWorker: 起点(当前关节角/Home) -> 预焊接点 的关节空间轨迹规划 worker。
//
// 用于替代原 rlPlanDemo 移植过来的 Thread 类(QThread + rl::plan::Viewer + MainWindow 单例):
//   - 与工程内 IKWorker / ProcessWorker / KukaCommunicator 保持一致的
//     "QObject worker + moveToThread + 参数结构体 + 信号槽回主线程" 架构;
//   - 不再依赖 MainWindow::instance() 单例, 规划上下文(模型/规划器/优化器)
//     由主线程在初始化时通过 setContext() 一次性注入;
//   - 不再实现 rl::plan::Viewer 的逐顶点/逐边可视化(本工程使用 OCCT + 定时器刷新,
//     无 Coin3D SoQt 规划过程可视化的需求), 也去掉了 benchmark.csv 与 usleep 动画循环;
//   - 规划结果统一整理成 std::vector<rl::math::Vector> 关节轨迹,
//     通过 finished 信号(Qt::QueuedConnection)发送到主线程,
//     主线程直接写入 wholeTrajectory 并由 flushTrajTimer/execSimulation 播放。
//

#include <atomic>
#include <memory>
#include <vector>

#include <QMutex>
#include <QObject>

#include <rl/math/Transform.h>
#include <rl/math/Vector.h>
#include <rl/mdl/Dynamic.h>
#include <rl/plan/Optimizer.h>
#include <rl/plan/Planner.h>
#include <rl/plan/SimpleModel.h>
#include <rl/plan/VectorList.h>

#include "GlobalDefs.h"

// 起点 -> 预焊接点 规划请求参数(与 IKSolveParams / IKGoToStartParams 风格一致)
struct PlanToPreWeldParams
{
    rl::math::Vector q_start;              // 起始关节角(含地轨), 一般取 mdl->getPosition() 或 HomePosition
    DiscretePoint    preWeldPoint;         // 焊缝起点的 TCP 位姿(mergedTraj.front())
    rl::math::Transform T_flange_to_tcp;   // 法兰 -> TCP

    double preWeldBackOffset = 50.0;       // 预焊接点: 沿焊接坐标系 -Z 方向后退的安全距离(mm), 0 表示直接用 preWeldPoint
    double interpStep        = 1.0;        // 输出轨迹的关节空间插值步长(与 model->distance 同量纲, 决定播放密度)
    double plannerTimeoutSec = 30.0;       // RRT 规划超时(s)
    bool   optimize          = true;       // 是否用 AdvancedOptimizer 做路径优化
    int    ikTimeoutMs       = 500;        // 预焊接点逆解超时(ms)
};
Q_DECLARE_METATYPE(PlanToPreWeldParams)

class PlanWorker : public QObject
{
    Q_OBJECT

public:
    explicit PlanWorker(QObject* parent = nullptr);
    ~PlanWorker() override;

    // 主线程在 loadRobotWidget() 完成、规划对象创建好之后调用一次。
    // sceneMutex 即 MainWindow::mutex, 用于与主线程共享 mdl/model/scene 时互斥。
    void setContext(const std::shared_ptr<rl::mdl::Dynamic>& mdl,
                    const std::shared_ptr<rl::plan::SimpleModel>& model,
                    const std::shared_ptr<rl::plan::Planner>& planner,
                    const std::shared_ptr<rl::plan::Optimizer>& optimizer,
                    QMutex* sceneMutex);

    bool isBusy() const { return this->busy.load(); }

    // 线程安全; RRT solve 内部无法打断, 通过 plannerTimeoutSec 控制上限,
    // abort 在阶段边界(IK / solve / optimize / 插值)生效。
    void requestAbort() { this->abortFlag.store(true); }

public slots:
    // 通过 QMetaObject::invokeMethod(planworker, "doPlanToPreWeld",
    //     Qt::QueuedConnection, Q_ARG(PlanToPreWeldParams, p)) 触发, 在 worker 线程执行。
    void doPlanToPreWeld(const PlanToPreWeldParams& params);

signals:
    void started();

    void progress(int percent);

    // 起点 -> 预焊接点 的稠密关节轨迹(已插值), 供主线程写入 wholeTrajectory 并链式触发焊缝段逆解
    void finished(const std::vector<rl::math::Vector>& jointTrajectory, double plannerMs);

    void failed(const QString& reason);

    // level: 0-info 1-warning 2-error, 由主线程转发到 console
    void message(int level, const QString& msg);

private:
    // 由 DiscretePoint(含预焊接后退偏移)计算法兰系目标位姿
    rl::math::Transform flangeGoalFromPreWeldPoint(const PlanToPreWeldParams& params) const;

    // 预焊接点逆解, 成功时 qGoal 为目标关节角
    bool solveGoalConfiguration(const PlanToPreWeldParams& params, rl::math::Vector& qGoal);

    // 碰撞检测: q 处是否与场景干涉
    bool isCollidingAt(const rl::math::Vector& q);

    // 把 RRT 输出的稀疏路径(VectorList)按 interpStep 插值成稠密 vector 轨迹
    std::vector<rl::math::Vector> interpolatePath(const rl::plan::VectorList& path, double stepSize) const;

    std::shared_ptr<rl::mdl::Dynamic> mdl;
    std::shared_ptr<rl::plan::SimpleModel> model;
    std::shared_ptr<rl::plan::Planner> planner;
    std::shared_ptr<rl::plan::Optimizer> optimizer;
    QMutex* sceneMutex = nullptr;

    // planner->start / planner->goal 是裸指针, solve 期间必须保证其指向的内存有效
    rl::math::Vector qStart;
    rl::math::Vector qGoal;

    std::atomic<bool> busy{false};
    std::atomic<bool> abortFlag{false};
};

#endif // PLANWORKER_H
