
#include "tool/IKWorker.h"
#include "MainWindow.h"

#include <QElapsedTimer>
#include <QMutexLocker>
#include <chrono>
#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <rl/mdl/Kinematic.h>
#include <rl/mdl/Joint.h>
#include <rl/mdl/JacobianInverseKinematics.h>
#include <rl/math/Unit.h>
#include <rl/plan/Optimizer.h>
#include <rl/plan/Planner.h>
#include <rl/plan/SimpleModel.h>

namespace
{
    constexpr double kMillimeterToMeter = 0.001;

    bool PlanCollisionFree(MainWindow* mw,
        const rl::math::Vector& qStart,
        const rl::math::Vector& qGoal,
        double step,
        std::vector<rl::math::Vector>& path)
    {
        if (mw == nullptr || mw->planner == nullptr || mw->model == nullptr)
            return false;

        QMutexLocker lock(&mw->mutex);
        if (!mw->start)
            mw->start = std::make_shared<rl::math::Vector>(qStart);
        else
            *mw->start = qStart;
        if (!mw->goal)
            mw->goal = std::make_shared<rl::math::Vector>(qGoal);
        else
            *mw->goal = qGoal;

        mw->planner->start = mw->start.get();
        mw->planner->goal = mw->goal.get();
        mw->planner->viewer = nullptr;

        if (!mw->planner->verify() || !mw->planner->solve())
            return false;

        rl::plan::VectorList sparse = mw->planner->getPath();
        if (mw->optimizer)
        {
            mw->optimizer->setViewer(nullptr);
            mw->optimizer->process(sparse);
        }
        if (sparse.empty())
            return false;

        path.clear();
        const double delta = (step > 0.0) ? step : 0.05;
        rl::math::Vector inter(mw->model->getDofPosition());
        path.push_back(*sparse.begin());

        rl::plan::VectorList::iterator it = sparse.begin();
        rl::plan::VectorList::iterator jt = sparse.begin();
        if (jt != sparse.end())
            ++jt;
        for (; it != sparse.end() && jt != sparse.end(); ++it, ++jt)
        {
            const rl::math::Real steps = std::ceil(mw->model->distance(*it, *jt) / delta);
            const rl::math::Real n = (steps < 1.0) ? 1.0 : steps;
            for (std::size_t k = 1; k < static_cast<std::size_t>(n) + 1; ++k)
            {
                mw->model->interpolate(*it, *jt, static_cast<rl::math::Real>(k) / n, inter);
                path.push_back(inter);
            }
        }
        return !path.empty();
    }
}

IKWorker::IKWorker(QObject * parent) : QObject(parent)
{
    qRegisterMetaType<rl::math::Vector>("rl::math::Vector");
    qRegisterMetaType<std::vector<rl::math::Vector>>("std::vector<rl::math::Vector>");
    qRegisterMetaType<std::vector<rl::math::Vector3>>("std::vector<rl::math::Vector3>");
    qRegisterMetaType<IKSolveParams>("IKSolveParams");
    qRegisterMetaType<IKGoToStartParams>("IKGoToStartParams");
    qRegisterMetaType<IKTransitParams>("IKTransitParams");
}

IKWorker::~IKWorker() = default;

void IKWorker::setKinematic(const std::shared_ptr<rl::mdl::Dynamic>& mdl)
{
    QMutexLocker locker(&m_kinMutex);
    if (m_running.load())
    {
        return;
    }
    m_kinematic = std::dynamic_pointer_cast<rl::mdl::Kinematic>(mdl);
}

void IKWorker::requestStop()
{
    m_stopRequested.store(true);
}

rl::math::Transform IKWorker::pointToTransform(const DiscretePoint& pt)
{
    rl::math::Transform T = rl::math::Transform::Identity();

    // 焊缝点来自 OpenCascade，单位毫米；规划场景单位是米。
    T.translation() = rl::math::Vector3(
        pt.position.X() * kMillimeterToMeter,
        pt.position.Y() * kMillimeterToMeter,
        pt.position.Z() * kMillimeterToMeter);

    rl::math::Matrix33 R;
    R.col(0) = rl::math::Vector3(pt.xDir.X(), pt.xDir.Y(), pt.xDir.Z());
    R.col(1) = rl::math::Vector3(pt.yDir.X(), pt.yDir.Y(), pt.yDir.Z());
    R.col(2) = rl::math::Vector3(pt.zDir.X(), pt.zDir.Y(), pt.zDir.Z());

    R.col(0).normalize();
    R.col(2) = R.col(2).normalized();
    R.col(1) = R.col(2).cross(R.col(0)).normalized();
    R.col(2) = R.col(0).cross(R.col(1)).normalized();

    T.linear() = R;
    return T;
}

void IKWorker::doSolve(const IKSolveParams& params)
{
    m_running.store(true);
    m_stopRequested.store(false);

    // 取出 kinematic（拷贝 shared_ptr，引用计数+1，保证求解期间对象不会被销毁）
    std::shared_ptr<rl::mdl::Kinematic> kinematic;
    {
        QMutexLocker locker(&m_kinMutex);
        kinematic = m_kinematic;
    }

    try
    {
        if (!kinematic)
            throw std::runtime_error("Kinematic not set. Call setKinematic() before doSolve().");

        const std::size_t dof = kinematic->getDof();
        const int totalPoints = static_cast<int>(params.trajectory.size());

        if (params.q_initial.size() != static_cast<Eigen::Index>(dof))
            throw std::runtime_error("Initial joint vector size mismatch with model DOF.");

        if (totalPoints == 0)
            throw std::runtime_error("Trajectory is empty.");

        if (params.railWindow <= 0.0)
            throw std::runtime_error("railWindow must be positive for interpolation.");

        emit started();

        const rl::math::Transform T_tcp_to_flange = params.T_flange_to_tcp.inverse();

        rl::mdl::Joint* railJoint = (params.constrainRail && dof >= 1) ? kinematic->getJoint(0) : nullptr;
        rl::math::Vector railMin0, railMax0;
        if (railJoint)
        {
            railMin0 = railJoint->getMinimum();
            railMax0 = railJoint->getMaximum();
        }

        std::vector<rl::math::Vector> jointTrajectory;
        jointTrajectory.reserve(static_cast<std::size_t>(totalPoints) * 2); // 粗略预留

        rl::math::Vector q_current = params.q_initial;
        kinematic->setPosition(q_current);
        kinematic->forwardPosition();

        int successCount = 0, failCount = 0, lastReportedPercent = -1;

        // —— 单点 IK 求解 lambda：成功/失败计数、地轨窗口、轨迹写入都封装在内 ——
        // 返回 false 表示用户请求停止（外层需要尽快退出）
        auto solveOne = [&](const rl::math::Transform& T_world_tcp) -> bool
            {
                if (m_stopRequested.load())
                    return false;

                rl::math::Transform T_world_flange = T_world_tcp * T_tcp_to_flange;

                // 地轨约束（直接操作 Joint(0)）
                if (railJoint && railJoint->getDofPosition() == 1)
                {
                    double lo = std::max(railMin0(0), T_world_tcp.translation().y());
                    double hi = std::min(railMax0(0), T_world_tcp.translation().y());

                    rl::math::Vector railMin(1), railMax(1);
                    railMin << lo;
                    railMax << hi;
                    railJoint->setMinimum(railMin);
                    railJoint->setMaximum(railMax);
                }

                rl::mdl::JacobianInverseKinematics ik(kinematic.get());
                ik.setDuration(std::chrono::milliseconds(params.timeoutMs));
                ik.addGoal(T_world_flange, 0);

                kinematic->setPosition(q_current);
                bool ok = ik.solve();

                if (ok)
                {
                    q_current = kinematic->getPosition();
                    jointTrajectory.push_back(q_current);
                    ++successCount;
                }
                else
                {
                    ++failCount;
                    jointTrajectory.push_back(q_current); // 沿用上一个解，保持轨迹长度
                }
                return true;
            };

        // 上一段终点 TCP 位姿（用于判断是否需要插值）。第一个点没有“上一点”，直接求解。
        rl::math::Transform T_prev_tcp = rl::math::Transform::Identity();
        bool hasPrev = false;

        for (int k = 0; k < totalPoints; ++k)
        {
            if (m_stopRequested.load())
            {
                if (railJoint)
                {
                    railJoint->setMinimum(railMin0);
                    railJoint->setMaximum(railMax0);
                }
                m_running.store(false);
                emit aborted();
                return;
            }

            const rl::math::Transform T_curr_tcp = pointToTransform(params.trajectory[k]);

            // —— 与上一个原始点之间做插值 ——
            if (hasPrev)
            {
                const rl::math::Vector3 p0 = T_prev_tcp.translation();
                const rl::math::Vector3 p1 = T_curr_tcp.translation();
                const double dist = (p1 - p0).norm();

                if (dist > params.railWindow)
                {
                    // 段数：保证每一段的步长 <= railWindow
                    const int segments = static_cast<int>(std::ceil(dist / params.railWindow));
                    // 姿态用四元数 SLERP，避免方向向量线性插值破坏正交性
                    const Eigen::Quaterniond q0(T_prev_tcp.linear());
                    const Eigen::Quaterniond q1(T_curr_tcp.linear());

                    // 生成 segments-1 个中间点（不含两端）
                    for (int i = 1; i < segments; ++i)
                    {
                        if (m_stopRequested.load())
                        {
                            if (railJoint)
                            {
                                railJoint->setMinimum(railMin0);
                                railJoint->setMaximum(railMax0);
                            }
                            m_running.store(false);
                            emit aborted();
                            return;
                        }

                        const double t = static_cast<double>(i) / static_cast<double>(segments);

                        rl::math::Transform T_mid = rl::math::Transform::Identity();
                        T_mid.translation() = (1.0 - t) * p0 + t * p1;
                        T_mid.linear() = q0.slerp(t, q1).normalized().toRotationMatrix();

                        if (!solveOne(T_mid))
                        {
                            if (railJoint)
                            {
                                railJoint->setMinimum(railMin0);
                                railJoint->setMaximum(railMax0);
                            }
                            m_running.store(false);
                            emit aborted();
                            return;
                        }
                    }
                }
            }

            // —— 求解原始第 k 个点 ——
            if (!solveOne(T_curr_tcp))
            {
                if (railJoint)
                {
                    railJoint->setMinimum(railMin0);
                    railJoint->setMaximum(railMax0);
                }
                m_running.store(false);
                emit aborted();
                return;
            }

            T_prev_tcp = T_curr_tcp;
            hasPrev = true;

            // 进度仍以原始点为基准，不会因插值数量抖动
            int percent = static_cast<int>((k + 1) * 100LL / totalPoints);
            if (percent != lastReportedPercent)
            {
                emit progress(percent);
                lastReportedPercent = percent;
            }
        }

        // 还原地轨限位
        if (railJoint)
        {
            railJoint->setMinimum(railMin0);
            railJoint->setMaximum(railMax0);
        }

        kinematic->setPosition(kinematic->getHomePosition());
        kinematic->forwardPosition();

        m_running.store(false);
        const int attempts = successCount + failCount;
        const double ratio = (attempts > 0) ? (static_cast<double>(successCount) / attempts) : 0.0;
        emit finished(jointTrajectory, ratio * 100, params.trajectory[0]);
    }
    catch (const std::exception& e)
    {
        m_running.store(false);
        emit failed(QString::fromUtf8(e.what()));
    }
    catch (...)
    {
        m_running.store(false);
        emit failed(QStringLiteral("Unknown exception in IK worker."));
    }
}

void IKWorker::doReturnHome(const IKReturnHomeParams& params)
{
    m_running.store(true);
    m_stopRequested.store(false);

    std::shared_ptr<rl::mdl::Kinematic> kinematic;
    {
        QMutexLocker locker(&m_kinMutex);
        kinematic = m_kinematic;
    }

    rl::math::Vector qFrozen;
    bool sceneFrozen = false;

    try
    {
        MainWindow* mw = MainWindow::instance();
        if (!mw || !mw->planner || !mw->model || !kinematic) throw std::runtime_error("Kinematic not set. Call setKinematic() before doReturnHome().");

        const std::size_t dof = kinematic->getDof();
        if (params.q_home.size() != static_cast<Eigen::Index>(dof)) throw std::runtime_error("q_home size mismatch with model DOF.");

        emit started();
        qFrozen = kinematic->getPosition();
        sceneFrozen = true;

        auto toolXyz = [&](const rl::math::Vector& q) -> rl::math::Vector3 {
            kinematic->setPosition(q);
            kinematic->forwardPosition();
            const rl::math::Transform T_tcp =
                kinematic->getOperationalPosition(0) * params.T_flange_to_tcp;
            return T_tcp.translation();
        };

        std::vector<rl::math::Vector> jointTrajectory;
        if ((params.q_current - params.q_home).cwiseAbs().maxCoeff() <= 1e-9)
        {
            jointTrajectory.push_back(params.q_home);
        }
        else if (!PlanCollisionFree(mw, params.q_current, params.q_home, params.jointStepRad, jointTrajectory))
        {
            kinematic->setPosition(qFrozen);
            kinematic->forwardPosition();
            m_running.store(false);
            emit failed(QStringLiteral("Planner failed: return home is in collision or has no path."));
            return;
        }

        std::vector<rl::math::Vector3> toolPoints;
        toolPoints.reserve(jointTrajectory.size());
        for (const rl::math::Vector& q : jointTrajectory)
            toolPoints.push_back(toolXyz(q));

        kinematic->setPosition(qFrozen);
        kinematic->forwardPosition();

        emit progress(100);
        m_running.store(false);
        emit finished_return(jointTrajectory, toolPoints, 100.0);
    }
    catch (const std::exception& e)
    {
        if (sceneFrozen && kinematic && qFrozen.size() > 0)
        {
            kinematic->setPosition(qFrozen);
            kinematic->forwardPosition();
        }
        m_running.store(false);
        emit failed(QString::fromUtf8(e.what()));
    }
    catch (...)
    {
        if (sceneFrozen && kinematic && qFrozen.size() > 0)
        {
            kinematic->setPosition(qFrozen);
            kinematic->forwardPosition();
        }
        m_running.store(false);
        emit failed(QStringLiteral("Unknown exception in IK worker (return home)."));
    }
}

void IKWorker::doGoToStart(const IKGoToStartParams& params)
{
    m_running.store(true);
    m_stopRequested.store(false);

    std::shared_ptr<rl::mdl::Kinematic> kinematic;
    {
        QMutexLocker locker(&m_kinMutex);
        kinematic = m_kinematic;
    }

    rl::mdl::Joint* railJoint = nullptr;
    rl::math::Vector railMin0, railMax0;

    try
    {
        MainWindow* mw = MainWindow::instance();
        if (!mw || !mw->planner || !mw->model || !kinematic) throw std::runtime_error("Kinematic not set. Call setKinematic() before doGoToStart().");

        const std::size_t dof = kinematic->getDof();
        if (params.q_home.size() != static_cast<Eigen::Index>(dof)) throw std::runtime_error("q_home size mismatch with model DOF.");

        emit started();

        const rl::math::Transform T_tcp_to_flange = params.T_flange_to_tcp.inverse();
        const rl::math::Transform T_world_tcp_start = pointToTransform(params.startPoint);
        const double yWeld = T_world_tcp_start.translation().y();

        railJoint = kinematic->getJoint(0);
        if (railJoint)
        {
            railMin0 = railJoint->getMinimum();
            railMax0 = railJoint->getMaximum();
        }

        auto restoreRail = [&]() {
            if (railJoint)
            {
                railJoint->setMinimum(railMin0);
                railJoint->setMaximum(railMax0);
            }
        };

        int lastReportedPercent = -1;
        auto reportProgress = [&](int percent) {
            percent = std::max(0, std::min(100, percent));
            if (percent != lastReportedPercent)
            {
                emit progress(percent);
                lastReportedPercent = percent;
            }
        };

        auto toolXyz = [&](const rl::math::Vector& q) -> rl::math::Vector3 {
            kinematic->setPosition(q);
            kinematic->forwardPosition();
            const rl::math::Transform T_tcp =
                kinematic->getOperationalPosition(0) * params.T_flange_to_tcp;
            return T_tcp.translation();
            };

        auto checkStop = [&]() -> bool {
            if (!m_stopRequested.load())
                return false;
            restoreRail();
            m_running.store(false);
            emit aborted();
            return true;
            };

        // ========== path1：只动地轨 Joint0，对齐焊接起点 Y，其它关节保持 Home ==========
        double kRailStepMm = params.railStepLen;
        double yTarget = yWeld;
        if (railJoint && railJoint->getDofPosition() == 1)
        {
            yTarget = std::min(std::max(yWeld, railMin0(0)), railMax0(0));
        }

        std::vector<rl::math::Vector> path1;
        rl::math::Vector qRail = params.q_home;
        path1.push_back(qRail);

        const double y0 = params.q_home(0);
        const double yDir = (yTarget >= y0) ? 1.0 : -1.0;
        double y = y0;
        while (std::abs(yTarget - y) > kRailStepMm)
        {
            if (checkStop())
                return;
            y += yDir * kRailStepMm;
            qRail(0) = y;
            path1.push_back(qRail);
        }
        if (std::abs(yTarget - y) > 1e-9)
        {
            qRail(0) = yTarget;
            path1.push_back(qRail);
        }

        reportProgress(20);

        // ========== 固定 Joint0，后续 IK / RRT 不再采样地轨 ==========
        if (railJoint && railJoint->getDofPosition() == 1)
        {
            const double eps = 1e-6;
            rl::math::Vector lo(1), hi(1);
            lo << (yTarget - eps);
            hi << (yTarget + eps);
            railJoint->setMinimum(lo);
            railJoint->setMaximum(hi);
        }

        // ========== 焊接起点关节解（path2 终点）==========
        rl::mdl::JacobianInverseKinematics ik(kinematic.get());
        ik.setDuration(std::chrono::milliseconds(params.timeoutMs));
        ik.addGoal(T_world_tcp_start * T_tcp_to_flange, 0);
        kinematic->setPosition(path1.back());
        if (!ik.solve())
        {
            restoreRail();
            m_running.store(false);
            emit failed(QStringLiteral("IK failed: weld start configuration."));
            return;
        }
        rl::math::Vector qGoal = kinematic->getPosition();
        qGoal(0) = yTarget;

        reportProgress(30);

        // ========== path2：Joint0 锁定后的 RRT（path1 终点 → 焊接起点）==========
        std::vector<rl::math::Vector> path2;
        {
            QMutexLocker lock(&mw->mutex);

            if (!mw->start)
                mw->start = std::make_shared<rl::math::Vector>(path1.back());
            else
                *mw->start = path1.back();
            if (!mw->goal)
                mw->goal = std::make_shared<rl::math::Vector>(qGoal);
            else
                *mw->goal = qGoal;

            mw->planner->start = mw->start.get();
            mw->planner->goal = mw->goal.get();
            mw->planner->viewer = nullptr;

            if (!mw->planner->verify())
            {
                restoreRail();
                m_running.store(false);
                emit failed(QStringLiteral("Invalid start or goal configuration."));
                return;
            }

            if (checkStop())
                return;

            const bool solved = mw->planner->solve();
            if (!solved)
            {
                restoreRail();
                m_running.store(false);
                emit failed(QStringLiteral("Planner failed: home-rail pose to weld start."));
                return;
            }

            rl::plan::VectorList sparse = mw->planner->getPath();
            if (mw->optimizer)
            {
                mw->optimizer->setViewer(nullptr);
                mw->optimizer->process(sparse);
            }

            const double delta = (params.cartStepLen > 0.0) ? params.cartStepLen : 1.0;
            rl::math::Vector inter(mw->model->getDofPosition());
            if (!sparse.empty())
                path2.push_back(*sparse.begin());

            rl::plan::VectorList::iterator it = sparse.begin();
            rl::plan::VectorList::iterator jt = sparse.begin();
            if (jt != sparse.end())
                ++jt;
            for (; it != sparse.end() && jt != sparse.end(); ++it, ++jt)
            {
                if (checkStop())
                    return;
                const rl::math::Real steps =
                    std::ceil(mw->model->distance(*it, *jt) / delta);
                const rl::math::Real n = (steps < 1.0) ? 1.0 : steps;
                for (std::size_t k = 1; k < static_cast<std::size_t>(n) + 1; ++k)
                {
                    mw->model->interpolate(*it, *jt, static_cast<rl::math::Real>(k) / n, inter);
                    path2.push_back(inter);
                }
            }
        }

        // 地轨保持 yTarget，避免限位窗口 eps 漂移
        for (rl::math::Vector& q : path2)
            q(0) = yTarget;

        restoreRail();
        reportProgress(90);

        // ========== 拼接 path1 + path2 ==========
        std::vector<rl::math::Vector> jointTrajectory = path1;
        std::size_t p2Begin = 0;
        if (!path2.empty() && !path1.empty()
            && (path2.front() - path1.back()).cwiseAbs().maxCoeff() < 1e-6)
        {
            p2Begin = 1;
        }
        jointTrajectory.insert(jointTrajectory.end(), path2.begin() + static_cast<std::ptrdiff_t>(p2Begin), path2.end());

        std::vector<rl::math::Vector3> toolPoints;
        toolPoints.reserve(jointTrajectory.size());
        for (const rl::math::Vector& q : jointTrajectory)
        {
            if (checkStop()) return;
            toolPoints.push_back(toolXyz(q));
        }

        reportProgress(100);

        kinematic->setPosition(kinematic->getHomePosition());
        kinematic->forwardPosition();

        m_running.store(false);
        emit finished_start(jointTrajectory, toolPoints, 100.0);
    }
    catch (const std::exception& e)
    {
        if (railJoint && railMin0.size() > 0)
        {
            railJoint->setMinimum(railMin0);
            railJoint->setMaximum(railMax0);
        }
        m_running.store(false);
        emit failed(QString::fromUtf8(e.what()));
    }
    catch (...)
    {
        if (railJoint && railMin0.size() > 0)
        {
            railJoint->setMinimum(railMin0);
            railJoint->setMaximum(railMax0);
        }
        m_running.store(false);
        emit failed(QStringLiteral("Unknown exception in IK worker (go to start)."));
    }
}

void IKWorker::doTransit(const IKTransitParams& params)
{
    m_running.store(true);
    m_stopRequested.store(false);

    std::shared_ptr<rl::mdl::Kinematic> kinematic;
    {
        QMutexLocker locker(&m_kinMutex);
        kinematic = m_kinematic;
    }

    rl::mdl::Joint* railJoint = nullptr;
    rl::math::Vector railMin0, railMax0;

    try
    {
        MainWindow* mw = MainWindow::instance();
        if (!mw || !mw->planner || !mw->model || !kinematic)
            throw std::runtime_error("Kinematic not set. Call setKinematic() before doTransit().");
        if (params.q_from.size() != static_cast<Eigen::Index>(kinematic->getDof()))
            throw std::runtime_error("q_from size mismatch with model DOF.");

        emit started();

        const rl::math::Transform T_tcp_to_flange = params.T_flange_to_tcp.inverse();
        const rl::math::Transform T_world_tcp = pointToTransform(params.goalPoint);
        const double yWeld = T_world_tcp.translation().y();

        railJoint = kinematic->getJoint(0);
        if (railJoint)
        {
            railMin0 = railJoint->getMinimum();
            railMax0 = railJoint->getMaximum();
        }
        auto restoreRail = [&]() {
            if (railJoint)
            {
                railJoint->setMinimum(railMin0);
                railJoint->setMaximum(railMax0);
            }
        };

        double yTarget = yWeld;
        if (railJoint && railJoint->getDofPosition() == 1)
        {
            yTarget = std::min(std::max(yWeld, railMin0(0)), railMax0(0));
            const double eps = 1e-6;
            rl::math::Vector lo(1), hi(1);
            lo << (yTarget - eps);
            hi << (yTarget + eps);
            railJoint->setMinimum(lo);
            railJoint->setMaximum(hi);
        }

        rl::mdl::JacobianInverseKinematics ik(kinematic.get());
        ik.setDuration(std::chrono::milliseconds(params.timeoutMs));
        ik.addGoal(T_world_tcp * T_tcp_to_flange, 0);
        kinematic->setPosition(params.q_from);
        if (!ik.solve())
        {
            restoreRail();
            m_running.store(false);
            emit failed(QStringLiteral("IK failed: next weld start configuration."));
            return;
        }
        rl::math::Vector qGoal = kinematic->getPosition();
        if (qGoal.size() > 0)
            qGoal(0) = yTarget;
        restoreRail();

        std::vector<rl::math::Vector> jointTrajectory;
        if (!PlanCollisionFree(mw, params.q_from, qGoal, params.cartStepLen, jointTrajectory))
        {
            kinematic->setPosition(params.q_from);
            kinematic->forwardPosition();
            m_running.store(false);
            emit failed(QStringLiteral("Planner failed: transit between welds is in collision or has no path."));
            return;
        }

        auto toolXyz = [&](const rl::math::Vector& q) -> rl::math::Vector3 {
            kinematic->setPosition(q);
            kinematic->forwardPosition();
            return (kinematic->getOperationalPosition(0) * params.T_flange_to_tcp).translation();
        };
        std::vector<rl::math::Vector3> toolPoints;
        toolPoints.reserve(jointTrajectory.size());
        for (const rl::math::Vector& q : jointTrajectory)
            toolPoints.push_back(toolXyz(q));

        kinematic->setPosition(params.q_from);
        kinematic->forwardPosition();
        emit progress(100);
        m_running.store(false);
        emit finished_start(jointTrajectory, toolPoints, 100.0);
    }
    catch (const std::exception& e)
    {
        if (railJoint && railMin0.size() > 0)
        {
            railJoint->setMinimum(railMin0);
            railJoint->setMaximum(railMax0);
        }
        m_running.store(false);
        emit failed(QString::fromUtf8(e.what()));
    }
    catch (...)
    {
        if (railJoint && railMin0.size() > 0)
        {
            railJoint->setMinimum(railMin0);
            railJoint->setMaximum(railMax0);
        }
        m_running.store(false);
        emit failed(QStringLiteral("Unknown exception in IK worker (transit)."));
    }
}