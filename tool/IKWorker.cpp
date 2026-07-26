#include "IKWorker.h"

#include <QElapsedTimer>
#include <QMutexLocker>
#include <chrono>
#include <stdexcept>

#include <rl/mdl/Kinematic.h>
#include <rl/mdl/Joint.h>                           
#include <rl/mdl/JacobianInverseKinematics.h>
#include <rl/math/Unit.h>

IKWorker::IKWorker(QObject* parent) : QObject(parent)
{
    qRegisterMetaType<rl::math::Vector>("rl::math::Vector");
    qRegisterMetaType<std::vector<rl::math::Vector>>("std::vector<rl::math::Vector>");
    qRegisterMetaType<IKSolveParams>("IKSolveParams");
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

rl::math::Transform IKWorker::pointToTransform(const DiscretePoint & pt)
{
    rl::math::Transform T = rl::math::Transform::Identity();

    T.translation() = rl::math::Vector3( pt.position.X(), pt.position.Y(), pt.position.Z());

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
            throw std::runtime_error( "Kinematic not set. Call setKinematic() before doSolve().");

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

void IKWorker::doReturnHome(const IKReturnHomeParams& params)
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

        // 输入保持兼容：不再做沿 TCP 后退；tcpBackDistance / T_flange_to_tcp 本函数内不使用

        emit started();

        std::vector<rl::math::Vector> jointTrajectory;
        jointTrajectory.reserve(256);

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

        // 轨迹起点：当前关节（无 TCP 回退）
        const rl::math::Vector q_start = params.q_current;
        jointTrajectory.push_back(q_start);

        if (m_stopRequested.load())
        {
            m_running.store(false);
            emit aborted();
            return;
        }

        // Joint5：地轨=0，机械臂 J1..J6 = 1..6 → Joint5 = 5
        const Eigen::Index J5_INDEX = 5;
        if (static_cast<std::size_t>(J5_INDEX) >= dof)
            throw std::runtime_error("J5_INDEX out of range for model DOF.");

        constexpr double kPi = 3.14159265358979323846;
        const double kJoint5FinalRad = -kPi / 2.0; // -90°

        // —— 第 1 步：q_start → 暂存 Home（地轨保持当前值，Joint5 先到 0）——
        rl::math::Vector q_staging = params.q_home;
        q_staging(0) = q_start(0);
        q_staging(J5_INDEX) = 0.0;

        double maxJointDelta = 0.0;
        for (Eigen::Index j = 1; j < q_start.size(); ++j)
        {
            const double d = std::abs(q_staging(j) - q_start(j));
            if (d > maxJointDelta) maxJointDelta = d;
        }
        const int seg1 = (maxJointDelta <= 1e-12)
            ? 0
            : std::max(1, static_cast<int>(std::ceil(maxJointDelta / params.jointStepRad)));

        for (int i = 1; i <= seg1; ++i)
        {
            if (m_stopRequested.load())
            {
                m_running.store(false);
                emit aborted();
                return;
            }
            const double t = static_cast<double>(i) / static_cast<double>(seg1);
            rl::math::Vector q_interp = (1.0 - t) * q_start + t * q_staging;
            jointTrajectory.push_back(q_interp);
            reportProgress(static_cast<int>(50.0 * i / seg1)); // 0% → 50%
        }
        if (seg1 == 0)
            reportProgress(50);

        // —— 第 2 步：只动地轨到 q_home(0)，手臂保持 staging ——
        const rl::math::Vector q_rail_start = q_staging;
        const double railStart = q_rail_start(0);
        const double railEnd = params.q_home(0);
        const double railDelta = std::abs(railEnd - railStart);
        const int seg2 = (railDelta <= 1e-12)
            ? 0
            : std::max(1, static_cast<int>(std::ceil(railDelta / params.railStepLen)));

        rl::math::Vector q_after_rail = q_rail_start;
        for (int i = 1; i <= seg2; ++i)
        {
            if (m_stopRequested.load())
            {
                m_running.store(false);
                emit aborted();
                return;
            }
            const double t = static_cast<double>(i) / static_cast<double>(seg2);
            q_after_rail = q_rail_start;
            q_after_rail(0) = (1.0 - t) * railStart + t * railEnd;
            q_after_rail(J5_INDEX) = 0.0;
            jointTrajectory.push_back(q_after_rail);
            reportProgress(50 + static_cast<int>(30.0 * i / seg2)); // 50% → 80%
        }
        if (seg2 == 0)
            reportProgress(80);

        // —— 第 3 步：Joint5 0 → -90°，最后对齐 q_home ——
        const rl::math::Vector q_j5_start = q_after_rail;
        const double j5Start = q_j5_start(J5_INDEX);
        const double j5End = kJoint5FinalRad;
        const double j5Delta = std::abs(j5End - j5Start);
        const int seg3 = (j5Delta <= 1e-12)
            ? 0
            : std::max(1, static_cast<int>(std::ceil(j5Delta / params.jointStepRad)));

        for (int i = 1; i <= seg3; ++i)
        {
            if (m_stopRequested.load())
            {
                m_running.store(false);
                emit aborted();
                return;
            }
            const double t = static_cast<double>(i) / static_cast<double>(seg3);
            rl::math::Vector q_interp = q_j5_start;
            q_interp(J5_INDEX) = (1.0 - t) * j5Start + t * j5End;
            if (i == seg3)
                q_interp = params.q_home;
            jointTrajectory.push_back(q_interp);
            reportProgress(80 + static_cast<int>(20.0 * i / seg3)); // 80% → 100%
        }
        if (seg3 == 0 && (jointTrajectory.empty()
            || (jointTrajectory.back() - params.q_home).cwiseAbs().maxCoeff() > 1e-12))
        {
            jointTrajectory.push_back(params.q_home);
        }

        reportProgress(100);
        m_running.store(false);
        emit finished_return(jointTrajectory, 100.0);
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


void IKWorker::doGoToStart(const IKGoToStartParams& params)
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
                "Kinematic not set. Call setKinematic() before doGoToStart().");

        const std::size_t dof = kinematic->getDof();

        if (params.q_home.size() != static_cast<Eigen::Index>(dof))
            throw std::runtime_error("q_home size mismatch with model DOF.");
        if (dof < 1)
            throw std::runtime_error("Model DOF < 1.");
        if (params.jointStepRad <= 0.0 || params.railStepLen <= 0.0 || params.cartStepLen <= 0.0)
            throw std::runtime_error("Interpolation step must be positive.");
        if (params.baseUpDistance == 0.0)
            throw std::runtime_error("baseUpDistance must be non-zero.");

        emit started();

        const rl::math::Transform T_tcp_to_flange = params.T_flange_to_tcp.inverse();

        // 焊接起点 TCP 位姿（由 startPoint 直接换算）；不再依赖预先给出的 q_target_start
        const rl::math::Transform T_world_tcp_start = pointToTransform(params.startPoint);

        // 本模型中 Base +Z 抬升对应世界坐标 Z 减小（与 doReturnHome 一致）
        const double approachUp = -std::abs(params.baseUpDistance);

        // 备份地轨原始限位
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

        // 地轨软窗口：与 doSolve 相同约定，窗口中心取 TCP.Y
        auto softConstrainRailAroundY = [&](double y, double window) {
            if (!railJoint || railJoint->getDofPosition() != 1) return;
            const double w = (window > 0.0) ? window : 100.0;
            const double lo = std::max(railMin0(0), y - w);
            const double hi = std::min(railMax0(0), y + w);
            rl::math::Vector railMin(1), railMax(1);
            railMin << lo;
            railMax << hi;
            railJoint->setMinimum(railMin);
            railJoint->setMaximum(railMax);
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

        // 把 Home 关节作为轨迹首点
        jointTrajectory.push_back(params.q_home);

        const double railHome = params.q_home(0);
        const double yStart = T_world_tcp_start.translation().y();

        // ========== 预备：由 Home + 起点 TCP 求“焊点正上方接近姿态”，同时发现目标地轨 ==========
        // 不再读取 q_target_start；地轨软约束在 TCP.Y 附近，seed 用地轨≈Y 的 Home。
        softConstrainRailAroundY(yStart, params.railWindow);

        rl::math::Transform T_world_tcp_above = T_world_tcp_start;
        T_world_tcp_above.translation().z() += approachUp;

        rl::math::Vector q_seed_above = params.q_home;
        if (railJoint)
        {
            const double railGuess = std::min(std::max(yStart, railMin0(0)), railMax0(0));
            q_seed_above(0) = railGuess;
        }

        rl::math::Vector q_above;
        if (!solveTcpGoal(T_world_tcp_above, q_seed_above, q_above))
        {
            restoreRail();
            m_running.store(false);
            emit failed(QStringLiteral("IK failed: above weld point (approach pose from home)."));
            return;
        }

        const double railTarget = q_above(0);
        q_above(0) = railTarget;

        // 下降阶段锁定地轨，避免笛卡尔插值过程中地轨漂移
        lockRailAt(railTarget);

        // 同一套手臂姿态、但地轨仍在 Home —— 作为“第 1 步”要先摆到的位置
        rl::math::Vector q_above_atHome = q_above;
        q_above_atHome(0) = railHome;

        // ========== 第 1 步：手臂先摆到“接近姿态”（地轨保持 Home，纯关节插值）==========
        double maxJointDelta = 0.0;
        for (Eigen::Index j = 1; j < q_above_atHome.size(); ++j)
        {
            const double d = std::abs(q_above_atHome(j) - params.q_home(j));
            if (d > maxJointDelta) maxJointDelta = d;
        }
        const int seg1 = std::max(1, static_cast<int>(std::ceil(maxJointDelta / params.jointStepRad)));

        for (int i = 1; i <= seg1; ++i)
        {
            if (m_stopRequested.load())
            {
                restoreRail();
                m_running.store(false);
                emit aborted();
                return;
            }
            const double t = static_cast<double>(i) / static_cast<double>(seg1);
            rl::math::Vector q_interp = (1.0 - t) * params.q_home + t * q_above_atHome;
            q_interp(0) = railHome;
            jointTrajectory.push_back(q_interp);

            // 0% → 40%
            reportProgress(static_cast<int>(40.0 * i / seg1));
        }

        // ========== 第 2 步：移动地轨 railHome → railTarget（手臂姿态保持不变）==========
        const double railDelta = std::abs(railTarget - railHome);
        const int seg2 = std::max(1, static_cast<int>(std::ceil(railDelta / params.railStepLen)));

        for (int i = 1; i <= seg2; ++i)
        {
            if (m_stopRequested.load())
            {
                restoreRail();
                m_running.store(false);
                emit aborted();
                return;
            }
            const double t = static_cast<double>(i) / static_cast<double>(seg2);
            rl::math::Vector q = q_above;
            q(0) = (1.0 - t) * railHome + t * railTarget;
            jointTrajectory.push_back(q);

            // 40% → 70%
            reportProgress(40 + static_cast<int>(30.0 * i / seg2));
        }

        // ========== 第 3 步：笛卡尔直线下降到焊接点（地轨锁定在 railTarget）==========
        const int seg3 = std::max(1, static_cast<int>(std::ceil(std::abs(approachUp) / params.cartStepLen)));

        rl::math::Vector q_seed = q_above;
        rl::math::Vector q_step;

        for (int i = 1; i <= seg3; ++i)
        {
            if (m_stopRequested.load())
            {
                restoreRail();
                m_running.store(false);
                emit aborted();
                return;
            }
            const double t = static_cast<double>(i) / static_cast<double>(seg3);

            rl::math::Transform T_target = T_world_tcp_start;
            T_target.translation().z() += (1.0 - t) * approachUp;

            if (!solveTcpGoal(T_target, q_seed, q_step))
            {
                restoreRail();
                m_running.store(false);
                emit failed(QStringLiteral("IK failed: descending to weld point (step 3)."));
                return;
            }
            q_step(0) = railTarget;
            jointTrajectory.push_back(q_step);
            q_seed = q_step;

            // 70% → 100%
            reportProgress(70 + static_cast<int>(30.0 * i / seg3));
        }

        reportProgress(100);

        restoreRail();

        const int attempts = successCount + failCount;
        const double ratio = (attempts > 0)
            ? (static_cast<double>(successCount) / attempts) : 1.0;

        kinematic->setPosition(kinematic->getHomePosition());
        kinematic->forwardPosition();

        m_running.store(false);
        emit finished_start(jointTrajectory, ratio * 100);
    }
    catch (const std::exception& e)
    {
        m_running.store(false);
        emit failed(QString::fromUtf8(e.what()));
    }
    catch (...)
    {
        m_running.store(false);
        emit failed(QStringLiteral("Unknown exception in IK worker (go to start)."));
    }
}