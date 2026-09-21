#include "PlanWorker.h"

#include <chrono>
#include <cmath>

#include <QMutexLocker>

#include <rl/mdl/JacobianInverseKinematics.h>
#include <rl/mdl/Kinematic.h>

namespace
{

// busy 标志的 RAII, 保证任何 return 路径都会复位
struct BusyGuard
{
    explicit BusyGuard(std::atomic<bool>& flag) : flag(flag) {}
    ~BusyGuard() { flag.store(false); }
    std::atomic<bool>& flag;
};

} // namespace

PlanWorker::PlanWorker(QObject* parent) :
    QObject(parent)
{
    qRegisterMetaType<PlanToPreWeldParams>("PlanToPreWeldParams");
    // IKWorker 已注册过该类型时重复注册无副作用
    qRegisterMetaType<std::vector<rl::math::Vector>>("std::vector<rl::math::Vector>");
}

PlanWorker::~PlanWorker() = default;

void PlanWorker::setContext(const std::shared_ptr<rl::mdl::Dynamic>& mdl,
                            const std::shared_ptr<rl::plan::SimpleModel>& model,
                            const std::shared_ptr<rl::plan::Planner>& planner,
                            const std::shared_ptr<rl::plan::Optimizer>& optimizer,
                            QMutex* sceneMutex)
{
    this->mdl = mdl;
    this->model = model;
    this->planner = planner;
    this->optimizer = optimizer;
    this->sceneMutex = sceneMutex;
}

rl::math::Transform PlanWorker::flangeGoalFromPreWeldPoint(const PlanToPreWeldParams& params) const
{
    const DiscretePoint& pt = params.preWeldPoint;

    // 预焊接点 = 焊缝起点沿焊接坐标系 -Z 后退 preWeldBackOffset
    const gp_Pnt pos(
        pt.position.X() - params.preWeldBackOffset * pt.zDir.X(),
        pt.position.Y() - params.preWeldBackOffset * pt.zDir.Y(),
        pt.position.Z() - params.preWeldBackOffset * pt.zDir.Z());

    rl::math::Transform tcpGoal;
    tcpGoal.setIdentity();
    tcpGoal.linear().col(0) << pt.xDir.X(), pt.xDir.Y(), pt.xDir.Z();
    tcpGoal.linear().col(1) << pt.yDir.X(), pt.yDir.Y(), pt.yDir.Z();
    tcpGoal.linear().col(2) << pt.zDir.X(), pt.zDir.Y(), pt.zDir.Z();
    tcpGoal.translation() << pos.X(), pos.Y(), pos.Z();

    // T_base_tcp = T_base_flange * T_flange_tcp  =>  T_base_flange = T_base_tcp * T_flange_tcp^-1
    return tcpGoal * params.T_flange_to_tcp.inverse();
}

bool PlanWorker::solveGoalConfiguration(const PlanToPreWeldParams& params, rl::math::Vector& qGoal)
{
    rl::mdl::Kinematic* kinematic = dynamic_cast<rl::mdl::Kinematic*>(this->mdl.get());
    if (nullptr == kinematic)
    {
        return false;
    }

    kinematic->setPosition(params.q_start);
    kinematic->forwardPosition();

    rl::mdl::JacobianInverseKinematics ik(kinematic);
    ik.setDuration(std::chrono::milliseconds(params.ikTimeoutMs));
    ik.addGoal(this->flangeGoalFromPreWeldPoint(params), 0);

    if (!ik.solve())
    {
        return false;
    }

    qGoal = kinematic->getPosition();
    return true;
}

bool PlanWorker::isCollidingAt(const rl::math::Vector& q)
{
    this->model->setPosition(q);
    this->model->updateFrames();
    return this->model->isColliding();
}

std::vector<rl::math::Vector> PlanWorker::interpolatePath(const rl::plan::VectorList& path, double stepSize) const
{
    std::vector<rl::math::Vector> trajectory;

    if (path.empty())
    {
        return trajectory;
    }

    rl::plan::VectorList::const_iterator i = path.begin();
    rl::plan::VectorList::const_iterator j = ++path.begin();

    trajectory.push_back(*i);

    rl::math::Vector inter(this->model->getDofPosition());

    for (; i != path.end() && j != path.end(); ++i, ++j)
    {
        const rl::math::Real dist = this->model->distance(*i, *j);
        const std::size_t steps = std::max<std::size_t>(1, static_cast<std::size_t>(std::ceil(dist / stepSize)));

        for (std::size_t k = 1; k <= steps; ++k)
        {
            this->model->interpolate(*i, *j, static_cast<rl::math::Real>(k) / static_cast<rl::math::Real>(steps), inter);
            trajectory.push_back(inter);
        }
    }

    return trajectory;
}

void PlanWorker::doPlanToPreWeld(const PlanToPreWeldParams& params)
{
    if (this->busy.exchange(true))
    {
        emit failed(QStringLiteral("PlanWorker is busy, request ignored."));
        return;
    }

    BusyGuard busyGuard(this->busy);
    this->abortFlag.store(false);

    if (nullptr == this->mdl || nullptr == this->model || nullptr == this->planner || nullptr == this->sceneMutex)
    {
        emit failed(QStringLiteral("PlanWorker context is not initialized, call setContext() first."));
        return;
    }

    emit started();

    // 与主线程互斥: 规划期间独占 mdl/model/scene(主线程侧在 started 信号里暂停 flushSceneTimer)
    QMutexLocker locker(this->sceneMutex);

    // 1. 预焊接点逆解, 得到目标关节角
    this->qStart = params.q_start;

    if (!this->solveGoalConfiguration(params, this->qGoal))
    {
        emit failed(QStringLiteral("Pre-weld point IK failed, target pose may be unreachable."));
        return;
    }

    emit progress(20);

    if (this->abortFlag.load())
    {
        emit failed(QStringLiteral("Planning aborted."));
        return;
    }

    // 2. 起点 / 目标点碰撞校验
    if (this->isCollidingAt(this->qStart))
    {
        emit failed(QStringLiteral("Start configuration is in collision."));
        return;
    }

    if (this->isCollidingAt(this->qGoal))
    {
        emit failed(QStringLiteral("Pre-weld goal configuration is in collision."));
        return;
    }

    emit progress(30);

    // 3. RRT(AddRrtConCon) 关节空间规划
    this->planner->start = &this->qStart;
    this->planner->goal = &this->qGoal;
    this->planner->duration = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(params.plannerTimeoutSec));
    this->planner->viewer = nullptr;
    this->planner->reset();

    if (!this->planner->verify())
    {
        emit failed(QStringLiteral("Planner verify failed: invalid start or goal configuration."));
        return;
    }

    const std::chrono::steady_clock::time_point solveBegin = std::chrono::steady_clock::now();
    const bool solved = this->planner->solve();
    const std::chrono::steady_clock::time_point solveEnd = std::chrono::steady_clock::now();
    const double plannerMs = std::chrono::duration_cast<std::chrono::duration<double>>(solveEnd - solveBegin).count() * 1000.0;

    if (!solved)
    {
        emit failed(QStringLiteral("Planner failed in %1 ms, no collision-free path to pre-weld point.")
            .arg(QString::number(plannerMs, 'f', 1)));
        return;
    }

    emit message(0, QStringLiteral("Planner succeeded in %1 ms.").arg(QString::number(plannerMs, 'f', 1)));
    emit progress(70);

    if (this->abortFlag.load())
    {
        emit failed(QStringLiteral("Planning aborted."));
        return;
    }

    // 4. 路径优化(缩短 + 平滑)
    rl::plan::VectorList path = this->planner->getPath();

    if (params.optimize && nullptr != this->optimizer)
    {
        const std::chrono::steady_clock::time_point optimizeBegin = std::chrono::steady_clock::now();
        this->optimizer->process(path);
        const std::chrono::steady_clock::time_point optimizeEnd = std::chrono::steady_clock::now();
        const double optimizerMs = std::chrono::duration_cast<std::chrono::duration<double>>(optimizeEnd - optimizeBegin).count() * 1000.0;

        emit message(0, QStringLiteral("Optimizer finished in %1 ms.").arg(QString::number(optimizerMs, 'f', 1)));
    }

    emit progress(85);

    if (this->abortFlag.load())
    {
        emit failed(QStringLiteral("Planning aborted."));
        return;
    }

    // 5. 稀疏路径 -> 稠密 std::vector<rl::math::Vector> 关节轨迹
    std::vector<rl::math::Vector> trajectory = this->interpolatePath(path, params.interpStep);

    if (trajectory.empty())
    {
        emit failed(QStringLiteral("Planned path is empty after interpolation."));
        return;
    }

    // 复位共享模型到起始位姿, 避免主线程恢复刷新后画面跳变
    this->model->setPosition(this->qStart);
    this->model->updateFrames();

    emit progress(100);
    emit finished(trajectory, plannerMs);
}
