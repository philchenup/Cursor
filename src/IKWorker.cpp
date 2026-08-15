#include "IKWorker.h"
#include "MainWindow.h"

#include <QElapsedTimer>
#include <QMetaObject>
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

IKWorker::IKWorker(QObject* parent) : QObject(parent)
{
    qRegisterMetaType<rl::math::Vector>("rl::math::Vector");
    qRegisterMetaType<std::vector<rl::math::Vector>>("std::vector<rl::math::Vector>");
    qRegisterMetaType<std::vector<rl::math::Vector3>>("std::vector<rl::math::Vector3>");
    qRegisterMetaType<IKSolveParams>("IKSolveParams");
    qRegisterMetaType<IKGoToStartParams>("IKGoToStartParams");
}

IKWorker::~IKWorker() = default;

void IKWorker::setKinematic(const std::shared_ptr<rl::mdl::Dynamic>&mdl)
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

rl::math::Transform IKWorker::pointToTransform(const DiscretePoint & pt)
{
    rl::math::Transform T = rl::math::Transform::Identity();

    T.translation() = rl::math::Vector3(pt.position.X(), pt.position.Y(), pt.position.Z());

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

void IKWorker::doSolve(const IKSolveParams & params)
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
                double lo = std::max(railMin0(0), T_world_tcp.translation().y() - params.railWindow);
                double hi = std::min(railMax0(0), T_world_tcp.translation().y() + params.railWindow);

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

//void IKWorker::doReturnHome(const IKReturnHomeParams& params)
//{
//    m_running.store(true);
//    m_stopRequested.store(false);
//
//    std::shared_ptr<rl::mdl::Kinematic> kinematic;
//    {
//        QMutexLocker locker(&m_kinMutex);
//        kinematic = m_kinematic;
//    }
//
//    try
//    {
//        if (!kinematic)
//            throw std::runtime_error(
//                "Kinematic not set. Call setKinematic() before doReturnHome().");
//
//        const std::size_t dof = kinematic->getDof();
//
//        if (params.q_current.size() != static_cast<Eigen::Index>(dof))
//            throw std::runtime_error("q_current size mismatch with model DOF.");
//        if (params.q_home.size() != static_cast<Eigen::Index>(dof))
//            throw std::runtime_error("q_home size mismatch with model DOF.");
//        if (dof < 1)
//            throw std::runtime_error("Model DOF < 1.");
//        if (params.jointStepRad <= 0.0 || params.railStepLen <= 0.0)
//            throw std::runtime_error("Interpolation step must be positive.");
//        if (params.tcpBackDistance <= 0.0)
//            throw std::runtime_error("tcpBackDistance must be positive.");
//
//        emit started();
//
//        const rl::math::Transform T_tcp_to_flange = params.T_flange_to_tcp.inverse();
//
//        // 备份地轨原始限位（结束/异常时还原）
//        rl::mdl::Joint* railJoint = (dof >= 1) ? kinematic->getJoint(0) : nullptr;
//        rl::math::Vector railMin0, railMax0;
//        if (railJoint)
//        {
//            railMin0 = railJoint->getMinimum();
//            railMax0 = railJoint->getMaximum();
//        }
//
//        auto restoreRail = [&]() {
//            if (railJoint)
//            {
//                railJoint->setMinimum(railMin0);
//                railJoint->setMaximum(railMax0);
//            }
//        };
//
//        // ===== 工具：把地轨锁定在指定值附近（极窄窗口，相当于不可动）=====
//        auto lockRailAt = [&](double railValue) {
//            if (!railJoint || railJoint->getDofPosition() != 1) return;
//            const double eps = 1e-6;
//            rl::math::Vector lo(1), hi(1);
//            // 不超出原始限位
//            const double v = std::min(std::max(railValue, railMin0(0)), railMax0(0));
//            lo << (v - eps);
//            hi << (v + eps);
//            railJoint->setMinimum(lo);
//            railJoint->setMaximum(hi);
//        };
//
//        std::vector<rl::math::Vector> jointTrajectory;
//        jointTrajectory.reserve(256);
//
//        int successCount = 0, failCount = 0;
//        int lastReportedPercent = -1;
//
//        auto reportProgress = [&](int percent) {
//            if (percent < 0) percent = 0;
//            if (percent > 100) percent = 100;
//            if (percent != lastReportedPercent)
//            {
//                emit progress(percent);
//                lastReportedPercent = percent;
//            }
//        };
//
//        // ===== 工具：用给定 seed，对一个 TCP 目标位姿求 IK =====
//        // 返回是否成功；成功时 q_out 写入解，并 push 到轨迹；失败时不 push，不更新 seed。
//        auto solveTcpGoal = [&](const rl::math::Transform& T_world_tcp,
//            const rl::math::Vector& seed,
//            rl::math::Vector& q_out) -> bool
//        {
//            const rl::math::Transform T_world_flange = T_world_tcp * T_tcp_to_flange;
//
//            rl::mdl::JacobianInverseKinematics ik(kinematic.get());
//            ik.setDuration(std::chrono::milliseconds(params.timeoutMs));
//            ik.addGoal(T_world_flange, 0);
//
//            kinematic->setPosition(seed);
//            const bool ok = ik.solve();
//            if (ok)
//            {
//                q_out = kinematic->getPosition();
//                ++successCount;
//                return true;
//            }
//            ++failCount;
//            return false;
//        };
//
//        // —— 第 0 步：取“当前 TCP 位姿”，并判断是否已靠近 Home ——
//        kinematic->setPosition(params.q_current);
//        kinematic->forwardPosition();
//        const rl::math::Transform T_world_flange_now = kinematic->getOperationalPosition(0);
//        const rl::math::Transform T_world_tcp_now = T_world_flange_now * params.T_flange_to_tcp;
//
//        kinematic->setPosition(params.q_home);
//        kinematic->forwardPosition();
//        const rl::math::Transform T_world_flange_home = kinematic->getOperationalPosition(0);
//        const rl::math::Transform T_world_tcp_home = T_world_flange_home * params.T_flange_to_tcp;
//
//        // 与 Home 的 TCP 平移距离；小于 tcpBackDistance(100) 则直接关节插值到 Home
//        const double distToHome =
//            (T_world_tcp_now.translation() - T_world_tcp_home.translation()).norm();
//        const bool nearHome = (distToHome < std::abs(params.tcpBackDistance));
//
//        // 把当前关节也作为轨迹首点，便于下游平滑播放
//        jointTrajectory.push_back(params.q_current);
//
//        if (nearHome)
//        {
//            // 已靠近 Home：跳过 TCP 回退及分步 staging，直接 q_current → q_home 关节插值
//            double maxJointDelta = 0.0;
//            for (Eigen::Index j = 1; j < params.q_current.size(); ++j)
//            {
//                const double d = std::abs(params.q_home(j) - params.q_current(j));
//                if (d > maxJointDelta) maxJointDelta = d;
//            }
//            const double railDelta = std::abs(params.q_home(0) - params.q_current(0));
//            const int segJoint = std::max(1, static_cast<int>(std::ceil(maxJointDelta / params.jointStepRad)));
//            const int segRail = std::max(1, static_cast<int>(std::ceil(railDelta / params.railStepLen)));
//            const int seg = std::max(segJoint, segRail);
//
//            for (int i = 1; i <= seg; ++i)
//            {
//                if (m_stopRequested.load())
//                {
//                    m_running.store(false);
//                    emit aborted();
//                    return;
//                }
//                const double t = static_cast<double>(i) / static_cast<double>(seg);
//                rl::math::Vector q_interp = (1.0 - t) * params.q_current + t * params.q_home;
//                jointTrajectory.push_back(q_interp);
//                reportProgress(static_cast<int>(100.0 * i / seg));
//            }
//
//            reportProgress(100);
//            m_running.store(false);
//            emit finished_return(jointTrajectory, 100.0);
//            return;
//        }
//
//        // —— 远离 Home：沿 TCP -Z 后退，再分步关节/地轨/J5 回 Home ——
//        if (m_stopRequested.load())
//        {
//            restoreRail();
//            m_running.store(false);
//            emit aborted();
//            return;
//        }
//
//        lockRailAt(params.q_current(0));
//
//        const double backDist = std::abs(params.tcpBackDistance);
//        const int seg1 = std::max(1, static_cast<int>(std::ceil(backDist / params.railStepLen)));
//
//        rl::math::Vector q_seed = params.q_current;
//        rl::math::Vector q_step1 = params.q_current;
//
//        for (int i = 1; i <= seg1; ++i)
//        {
//            if (m_stopRequested.load())
//            {
//                restoreRail();
//                m_running.store(false);
//                emit aborted();
//                return;
//            }
//
//            const double t = static_cast<double>(i) / static_cast<double>(seg1);
//            rl::math::Transform T_back_in_tcp = rl::math::Transform::Identity();
//            T_back_in_tcp.translation() = rl::math::Vector3(0.0, 0.0, -t * backDist);
//            const rl::math::Transform T_world_tcp_back = T_world_tcp_now * T_back_in_tcp;
//
//            if (!solveTcpGoal(T_world_tcp_back, q_seed, q_step1))
//            {
//                restoreRail();
//                m_running.store(false);
//                emit failed(QStringLiteral("IK failed: TCP -Z back-off (step 1)."));
//                return;
//            }
//            // 地轨保持锁定值，避免 IK 窗口 eps 漂移
//            q_step1(0) = params.q_current(0);
//            jointTrajectory.push_back(q_step1);
//            q_seed = q_step1;
//
//            // 0% → 30%
//            reportProgress(static_cast<int>(30.0 * i / seg1));
//        }
//
//        // 后续是关节空间插值，无需再约束地轨；恢复原始限位
//        restoreRail();
//
//        // ===== Joint5 分段控制相关常量 =====
//        // Joint5 在 RL 模型中的索引（地轨=0，机械臂 J1..J6 = 1..6 → Joint5 = 5）
//        const Eigen::Index J5_INDEX = 5;
//        if (static_cast<std::size_t>(J5_INDEX) >= dof)
//            throw std::runtime_error("J5_INDEX out of range for model DOF.");
//
//        constexpr double kPi = 3.14159265358979323846;
//        const double kJoint5FinalRad = -kPi / 2.0; // -90°；若 Home 中 J5 本就是该值，可改用 params.q_home(J5_INDEX)
//
//        // —— 第 2 步：从 q_step1 关节插值到“暂存 Home”（地轨不动，Joint5 先到 0）——
//        // 暂存目标：除地轨外取 q_home，但 Joint5 强制为 0；地轨保持 q_step1(0)
//        rl::math::Vector q_home_staging = params.q_home;
//        q_home_staging(0) = q_step1(0);
//        q_home_staging(J5_INDEX) = 0.0; // Joint5 先到 0 位
//
//        // 段数：取除地轨外各关节最大角度差 / jointStepRad
//        double maxJointDelta = 0.0;
//        for (Eigen::Index j = 1; j < q_step1.size(); ++j)
//        {
//            const double d = std::abs(q_home_staging(j) - q_step1(j));
//            if (d > maxJointDelta) maxJointDelta = d;
//        }
//        int seg2 = std::max(1, static_cast<int>(std::ceil(maxJointDelta / params.jointStepRad)));
//
//        for (int i = 1; i <= seg2; ++i)
//        {
//            if (m_stopRequested.load())
//            {
//                m_running.store(false);
//                emit aborted();
//                return;
//            }
//            const double t = static_cast<double>(i) / static_cast<double>(seg2);
//            rl::math::Vector q_interp = (1.0 - t) * q_step1 + t * q_home_staging;
//            jointTrajectory.push_back(q_interp);
//
//            // 30% → 70%
//            reportProgress(30 + static_cast<int>(40.0 * i / seg2));
//        }
//
//        // —— 第 3 步：仅移动地轨到 q_home(0)，其余关节保持 Home 值，Joint5 仍保持 0 ——
//        const double railStart = q_home_staging(0);
//        const double railEnd = params.q_home(0);
//        const double railDelta = std::abs(railEnd - railStart);
//
//        int seg3 = std::max(1, static_cast<int>(std::ceil(railDelta / params.railStepLen)));
//
//        for (int i = 1; i <= seg3; ++i)
//        {
//            if (m_stopRequested.load())
//            {
//                m_running.store(false);
//                emit aborted();
//                return;
//            }
//            const double t = static_cast<double>(i) / static_cast<double>(seg3);
//            rl::math::Vector q_interp = params.q_home;          // 其余关节直接取 Home
//            q_interp(0) = (1.0 - t) * railStart + t * railEnd;   // 地轨线性
//            q_interp(J5_INDEX) = 0.0;                            // Joint5 等地轨到位前保持 0
//            jointTrajectory.push_back(q_interp);
//
//            // 70% → 90%
//            reportProgress(70 + static_cast<int>(20.0 * i / seg3));
//        }
//
//        // —— 第 4 步：地轨到位后，Joint5 从 0 插值到 -90°（其余关节保持 Home，地轨保持到位）——
//        const double j5Start = 0.0;
//        const double j5End = kJoint5FinalRad;
//        const double j5Delta = std::abs(j5End - j5Start);
//
//        int seg4 = std::max(1, static_cast<int>(std::ceil(j5Delta / params.jointStepRad)));
//
//        for (int i = 1; i <= seg4; ++i)
//        {
//            if (m_stopRequested.load())
//            {
//                m_running.store(false);
//                emit aborted();
//                return;
//            }
//            const double t = static_cast<double>(i) / static_cast<double>(seg4);
//            rl::math::Vector q_interp = params.q_home;              // 其余关节最终 Home
//            q_interp(0) = railEnd;                                  // 地轨保持到位
//            q_interp(J5_INDEX) = (1.0 - t) * j5Start + t * j5End;   // Joint5: 0 → -90°
//            jointTrajectory.push_back(q_interp);
//
//            // 90% → 100%
//            reportProgress(90 + static_cast<int>(10.0 * i / seg4));
//        }
//
//        reportProgress(100);
//
//        const int attempts = successCount + failCount;
//        const double ratio = (attempts > 0) ? (static_cast<double>(successCount) / attempts) : 1.0;
//
//        m_running.store(false);
//        emit finished_return(jointTrajectory, ratio * 100);
//    }
//    catch (const std::exception& e)
//    {
//        // 异常时尝试还原地轨（railMin0 在异常发生前可能未初始化的话就跳过）
//        m_running.store(false);
//        emit failed(QString::fromUtf8(e.what()));
//    }
//    catch (...)
//    {
//        m_running.store(false);
//        emit failed(QStringLiteral("Unknown exception in IK worker (return home)."));
//    }
//}

void IKWorker::doReturnHome(const IKReturnHomeParams & params)
{
    m_running.store(true);
    m_stopRequested.store(false);

    std::shared_ptr<rl::mdl::Kinematic> kinematic;
    {
        QMutexLocker locker(&m_kinMutex);
        kinematic = m_kinematic;
    }

    try
    {
        if (!kinematic)
            throw std::runtime_error(
                "Kinematic not set. Call setKinematic() before doReturnHome().");

        const std::size_t dof = kinematic->getDof();

        if (params.q_current.size() != static_cast<Eigen::Index>(dof))
            throw std::runtime_error("q_current size mismatch with model DOF.");
        if (params.q_home.size() != static_cast<Eigen::Index>(dof))
            throw std::runtime_error("q_home size mismatch with model DOF.");
        if (dof < 1)
            throw std::runtime_error("Model DOF < 1.");
        if (params.jointStepRad <= 0.0 || params.railStepLen <= 0.0)
            throw std::runtime_error("Interpolation step must be positive.");
        if (params.tcpStepBack <= 0.0)
            throw std::runtime_error("tcpStepBack must be positive.");

        emit started();

        const rl::math::Transform T_tcp_to_flange = params.T_flange_to_tcp.inverse();

        // 备份地轨原始限位（结束/异常时还原）
        rl::mdl::Joint* railJoint = (dof >= 1) ? kinematic->getJoint(0) : nullptr;
        rl::math::Vector railMin0, railMax0;
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

        // 地轨锁定在指定值附近（极窄窗口，相当于不可动）
        auto lockRailAt = [&](double railValue) {
            if (!railJoint || railJoint->getDofPosition() != 1) return;
            const double eps = 1e-6;
            const double v = std::min(std::max(railValue, railMin0(0)), railMax0(0));
            rl::math::Vector lo(1), hi(1);
            lo << (v - eps);
            hi << (v + eps);
            railJoint->setMinimum(lo);
            railJoint->setMaximum(hi);
        };

        std::vector<rl::math::Vector> jointTrajectory;
        jointTrajectory.reserve(256);

        int successCount = 0, failCount = 0;
        int lastReportedPercent = -1;

        auto reportProgress = [&](int percent) {
            if (percent < 0) percent = 0;
            if (percent > 100) percent = 100;
            if (percent != lastReportedPercent)
            {
                emit progress(percent);
                lastReportedPercent = percent;
            }
        };

        // 用给定 seed 对一个 TCP 目标位姿求 IK
        auto solveTcpGoal = [&](const rl::math::Transform& T_world_tcp,
            const rl::math::Vector& seed,
            rl::math::Vector& q_out) -> bool
        {
            const rl::math::Transform T_world_flange = T_world_tcp * T_tcp_to_flange;

            rl::mdl::JacobianInverseKinematics ik(kinematic.get());
            ik.setDuration(std::chrono::milliseconds(params.timeoutMs));
            ik.addGoal(T_world_flange, 0);

            kinematic->setPosition(seed);
            const bool ok = ik.solve();
            if (ok)
            {
                q_out = kinematic->getPosition();
                ++successCount;
                return true;
            }
            ++failCount;
            return false;
        };

        // —— 当前 TCP 位姿（法兰 → TCP）——
        kinematic->setPosition(params.q_current);
        kinematic->forwardPosition();
        const rl::math::Transform T_world_flange_now = kinematic->getOperationalPosition(0);
        const rl::math::Transform T_world_tcp_now = T_world_flange_now * params.T_flange_to_tcp;

        jointTrajectory.push_back(params.q_current);

        if (m_stopRequested.load())
        {
            m_running.store(false);
            emit aborted();
            return;
        }

        // ========== 第 1 步：沿 TCP -Z 后退 tcpStepBack → 点 A ==========
        lockRailAt(params.q_current(0));

        const double backDist = std::abs(params.tcpStepBack);
        const int segBack = std::max(1, static_cast<int>(std::ceil(backDist / params.railStepLen)));

        rl::math::Vector q_seed = params.q_current;
        rl::math::Vector q_A = params.q_current; // 点 A 的关节解

        for (int i = 1; i <= segBack; ++i)
        {
            if (m_stopRequested.load())
            {
                restoreRail();
                m_running.store(false);
                emit aborted();
                return;
            }

            const double t = static_cast<double>(i) / static_cast<double>(segBack);
            rl::math::Transform T_back_in_tcp = rl::math::Transform::Identity();
            T_back_in_tcp.translation() = rl::math::Vector3(0.0, 0.0, -t * backDist);
            const rl::math::Transform T_world_tcp_A = T_world_tcp_now * T_back_in_tcp;

            if (!solveTcpGoal(T_world_tcp_A, q_seed, q_A))
            {
                restoreRail();
                m_running.store(false);
                emit failed(QStringLiteral("IK failed: TCP -Z step-back to point A."));
                return;
            }
            // 地轨保持锁定值，避免 IK 窗口 eps 漂移
            q_A(0) = params.q_current(0);
            jointTrajectory.push_back(q_A);
            q_seed = q_A;

            // 0% → 30%
            reportProgress(static_cast<int>(30.0 * i / segBack));
        }

        // 后续为关节空间规划，恢复地轨限位
        restoreRail();

        // Joint5：地轨=0，机械臂 J1..J6 = 1..6 → Joint5 = 5
        const Eigen::Index J5_INDEX = 5;
        if (static_cast<std::size_t>(J5_INDEX) >= dof)
            throw std::runtime_error("J5_INDEX out of range for model DOF.");

        constexpr double kPi = 3.14159265358979323846;
        const double kJoint5FinalRad = -kPi / 2.0; // -90°

        // ========== 第 2 步：点 A → 暂存 Home（地轨保持 A，Joint5 先到 0）==========
        rl::math::Vector q_staging = params.q_home;
        q_staging(0) = q_A(0);
        q_staging(J5_INDEX) = 0.0;

        double maxJointDelta = 0.0;
        for (Eigen::Index j = 1; j < q_A.size(); ++j)
        {
            const double d = std::abs(q_staging(j) - q_A(j));
            if (d > maxJointDelta) maxJointDelta = d;
        }
        const int segArm = (maxJointDelta <= 1e-12)
            ? 0
            : std::max(1, static_cast<int>(std::ceil(maxJointDelta / params.jointStepRad)));

        for (int i = 1; i <= segArm; ++i)
        {
            if (m_stopRequested.load())
            {
                m_running.store(false);
                emit aborted();
                return;
            }
            const double t = static_cast<double>(i) / static_cast<double>(segArm);
            rl::math::Vector q_interp = (1.0 - t) * q_A + t * q_staging;
            jointTrajectory.push_back(q_interp);
            // 30% → 70%
            reportProgress(30 + static_cast<int>(40.0 * i / segArm));
        }
        if (segArm == 0)
            reportProgress(70);

        // ========== 第 3 步：只动地轨到 q_home(0)，手臂保持 staging ==========
        const rl::math::Vector q_rail_start = q_staging;
        const double railStart = q_rail_start(0);
        const double railEnd = params.q_home(0);
        const double railDelta = std::abs(railEnd - railStart);
        const int segRail = (railDelta <= 1e-12)
            ? 0
            : std::max(1, static_cast<int>(std::ceil(railDelta / params.railStepLen)));

        rl::math::Vector q_after_rail = q_rail_start;
        for (int i = 1; i <= segRail; ++i)
        {
            if (m_stopRequested.load())
            {
                m_running.store(false);
                emit aborted();
                return;
            }
            const double t = static_cast<double>(i) / static_cast<double>(segRail);
            q_after_rail = q_rail_start;
            q_after_rail(0) = (1.0 - t) * railStart + t * railEnd;
            q_after_rail(J5_INDEX) = 0.0;
            jointTrajectory.push_back(q_after_rail);
            // 70% → 90%
            reportProgress(70 + static_cast<int>(20.0 * i / segRail));
        }
        if (segRail == 0)
            reportProgress(90);

        // ========== 第 4 步：Joint5 0 → -90°，最后对齐 q_home ==========
        const rl::math::Vector q_j5_start = q_after_rail;
        const double j5Start = q_j5_start(J5_INDEX);
        const double j5End = kJoint5FinalRad;
        const double j5Delta = std::abs(j5End - j5Start);
        const int segJ5 = (j5Delta <= 1e-12)
            ? 0
            : std::max(1, static_cast<int>(std::ceil(j5Delta / params.jointStepRad)));

        for (int i = 1; i <= segJ5; ++i)
        {
            if (m_stopRequested.load())
            {
                m_running.store(false);
                emit aborted();
                return;
            }
            const double t = static_cast<double>(i) / static_cast<double>(segJ5);
            rl::math::Vector q_interp = q_j5_start;
            q_interp(J5_INDEX) = (1.0 - t) * j5Start + t * j5End;
            if (i == segJ5)
                q_interp = params.q_home;
            jointTrajectory.push_back(q_interp);
            // 90% → 100%
            reportProgress(90 + static_cast<int>(10.0 * i / segJ5));
        }
        if (segJ5 == 0 && (jointTrajectory.empty()
            || (jointTrajectory.back() - params.q_home).cwiseAbs().maxCoeff() > 1e-12))
        {
            jointTrajectory.push_back(params.q_home);
        }

        reportProgress(100);

        const int attempts = successCount + failCount;
        const double ratio = (attempts > 0)
            ? (static_cast<double>(successCount) / attempts) : 1.0;

        m_running.store(false);
        emit finished_return(jointTrajectory, ratio * 100);
    }
    catch (const std::exception& e)
    {
        m_running.store(false);
        emit failed(QString::fromUtf8(e.what()));
    }
    catch (...)
    {
        m_running.store(false);
        emit failed(QStringLiteral("Unknown exception in IK worker (return home)."));
    }
}

void IKWorker::doGoToStart(const IKGoToStartParams & params)
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
        if (!kinematic)
            throw std::runtime_error(
                "Kinematic not set. Call setKinematic() before doGoToStart().");

        MainWindow* mw = MainWindow::instance();
        if (!mw || !mw->planner || !mw->model)
            throw std::runtime_error("Planner/model not initialized.");

        const std::size_t dof = kinematic->getDof();
        if (params.q_home.size() != static_cast<Eigen::Index>(dof))
            throw std::runtime_error("q_home size mismatch with model DOF.");
        if (dof < 1)
            throw std::runtime_error("Model DOF < 1.");

        emit started();

        const rl::math::Transform T_tcp_to_flange = params.T_flange_to_tcp.inverse();
        const rl::math::Transform T_world_tcp_start = pointToTransform(params.startPoint);
        const double yWeld = T_world_tcp_start.translation().y();

        rl::mdl::Joint* railJoint = kinematic->getJoint(0);
        rl::math::Vector railMin0, railMax0;
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

        // 规划期间停掉 occtUpdate，场景机械臂保持 qFrozen
        qFrozen = kinematic->getPosition();
        QMetaObject::invokeMethod(mw, "setSceneFlushEnabled",
            Qt::BlockingQueuedConnection, Q_ARG(bool, false));
        sceneFrozen = true;

        auto restoreDisplay = [&]() {
            restoreRail();
            kinematic->setPosition(qFrozen);
            kinematic->forwardPosition();
            QMetaObject::invokeMethod(mw, "setSceneFlushEnabled",
                Qt::QueuedConnection, Q_ARG(bool, true));
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
            restoreDisplay();
            m_running.store(false);
            emit aborted();
            return true;
        };

        // ========== path1：只动地轨 Joint0，对齐焊接起点 Y，其它关节保持 Home ==========
        constexpr double kRailStepMm = 10.0;
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

        // ========== 焊接起点关节解（path2 终点）==========
        if (railJoint && railJoint->getDofPosition() == 1)
        {
            const double eps = 1e-6;
            rl::math::Vector lo(1), hi(1);
            lo << (yTarget - eps);
            hi << (yTarget + eps);
            railJoint->setMinimum(lo);
            railJoint->setMaximum(hi);
        }

        rl::mdl::JacobianInverseKinematics ik(kinematic.get());
        ik.setDuration(std::chrono::milliseconds(params.timeoutMs));
        ik.addGoal(T_world_tcp_start * T_tcp_to_flange, 0);
        kinematic->setPosition(path1.back());
        if (!ik.solve())
        {
            restoreDisplay();
            m_running.store(false);
            emit failed(QStringLiteral("IK failed: weld start configuration."));
            return;
        }
        rl::math::Vector qGoal = kinematic->getPosition();
        qGoal(0) = yTarget;
        restoreRail();

        reportProgress(30);

        // ========== path2：Thread::run 同款 RRT（path1 终点 → 焊接起点）==========
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
                restoreDisplay();
                m_running.store(false);
                emit failed(QStringLiteral("Invalid start or goal configuration."));
                return;
            }

            if (checkStop())
                return;

            const bool solved = mw->planner->solve();
            if (!solved)
            {
                restoreDisplay();
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
            if (checkStop())
                return;
            toolPoints.push_back(toolXyz(q));
        }

        reportProgress(100);

        restoreDisplay();
        m_running.store(false);
        emit finished_start(jointTrajectory, toolPoints, 100.0);
    }
    catch (const std::exception& e)
    {
        if (sceneFrozen && kinematic && qFrozen.size() > 0)
        {
            kinematic->setPosition(qFrozen);
            kinematic->forwardPosition();
        }
        if (MainWindow* mw = MainWindow::instance())
        {
            QMetaObject::invokeMethod(mw, "setSceneFlushEnabled",
                Qt::QueuedConnection, Q_ARG(bool, true));
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
        if (MainWindow* mw = MainWindow::instance())
        {
            QMetaObject::invokeMethod(mw, "setSceneFlushEnabled",
                Qt::QueuedConnection, Q_ARG(bool, true));
        }
        m_running.store(false);
        emit failed(QStringLiteral("Unknown exception in IK worker (go to start)."));
    }
}