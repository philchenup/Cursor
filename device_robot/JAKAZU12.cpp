/**
 * @file   JAKAZU12.cpp
 * @brief  JAKA Zu12 机械臂驱动层实现
 *
 * JAKA SDK 特殊说明
 * ─────────────────
 *  • login_in + power_on 为阻塞调用（网络连接），放入 QThread::create 异步执行
 *  • enable_robot / disable_robot 为阻塞调用，放入 std::thread detach
 *  • joint_move / linear_move 为非阻塞指令发送，随后在同一线程轮询到位
 *  • get_tcp_position / get_joint_position 为同步读取，由反馈定时器刷新快照
 *  • 坐标单位：mm / deg，与 IRobot 接口一致，无需单位转换
 *  • clearError：JAKA SDK 提供 err_flag_clear() 接口
 *  • 拧螺丝控制逻辑对齐 DobotCR5（tag 状态机 + drillTimer 下压）
 *
 * @version 2.1
 * @date    2026-08-07
 */

#include "JAKAZU12.h"
#include "tool/MathUtils.h"

#include <QThread>
#include <QMetaObject>
#include <QDebug>

#include <thread>
#include <chrono>
#include <cmath>

// ─────────────────────────────────────────────────────────────────────────────
// 构造 / 析构
// ─────────────────────────────────────────────────────────────────────────────
JAKAZU12::JAKAZU12(QObject* parent)
    : IRobot(parent)
    , m_feedbackTimer(new QTimer(this))
    , m_drillTimer(new QTimer(this))
{
    m_feedbackTimer->setInterval(100);
    connect(m_feedbackTimer, &QTimer::timeout,
        this, &JAKAZU12::onFeedbackTimerTimeout);

    // 跨线程定时器控制：后台线程 emit _startFeedbackTimer()，
    // QueuedConnection 保证 slot 在主线程执行
    connect(this, &JAKAZU12::_startFeedbackTimer,
        this, &JAKAZU12::startFeedbackTimer, Qt::QueuedConnection);
    connect(this, &JAKAZU12::_stopFeedbackTimer,
        this, &JAKAZU12::stopFeedbackTimer, Qt::QueuedConnection);

    // 拧螺丝序列：下压步进 + 到位后状态机（对齐 DobotCR5）
    QObject::connect(m_drillTimer, &QTimer::timeout, this, &JAKAZU12::onDrillStep);
    QObject::connect(this, &JAKAZU12::sigMoveFinished, this, &JAKAZU12::moveNextPose);
}

JAKAZU12::~JAKAZU12()
{
    m_feedbackTimer->stop();
    m_drillTimer->stop();

    if (m_connected) {
        m_robot.login_out();
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// IRobot 查询
// ─────────────────────────────────────────────────────────────────────────────
bool JAKAZU12::isConnected() const { return m_connected.load(); }
bool JAKAZU12::isEnabled()   const { return m_enabled.load(); }

// ─────────────────────────────────────────────────────────────────────────────
// connectRobot
//
// JAKA 连接流程（两步）：
//   1. login_in(ip)   — 网络握手，阻塞约 1~3s
//   2. power_on()     — 上电指令
// 两步均阻塞，放入 QThread::create 后台执行
// ─────────────────────────────────────────────────────────────────────────────
void JAKAZU12::connectRobot(const RobotPara& para)
{
    if (m_connected) {
        emit sigStatusMessage(tr("JAKA Zu12 已连接，无需重复连接"));
        return;
    }

    const std::string ip = para.IpAddress;

    QThread* t = QThread::create([=]() {
        connectTask(ip);
        });
    connect(t, &QThread::finished, t, &QThread::deleteLater);
    t->start();
}

void JAKAZU12::connectTask(const std::string& ip)
{
    errno_t ret = m_robot.login_in(ip.c_str());
    if (ret != ERR_SUCC) {
        QMetaObject::invokeMethod(this, [=]() {
            emit sigError(tr("JAKA Zu12 登录失败（IP: %1）")
                .arg(QString::fromStdString(ip)));
            emit sigConnected(false);
            }, Qt::QueuedConnection);
        return;
    }

    ret = m_robot.power_on();
    const bool ok = (ret == ERR_SUCC);

    QMetaObject::invokeMethod(this, [=]() {
        m_connected = ok;
        if (ok) {
            emit _startFeedbackTimer();
            emit sigStatusMessage(tr("JAKA Zu12 连接成功"));
        }
        else {
            emit sigError(tr("JAKA Zu12 上电失败"));
            m_robot.login_out();
        }
        emit sigConnected(ok);
        }, Qt::QueuedConnection);
}

// ─────────────────────────────────────────────────────────────────────────────
// disconnectRobot
// ─────────────────────────────────────────────────────────────────────────────
void JAKAZU12::disconnectRobot()
{
    if (!m_connected) {
        emit sigStatusMessage(tr("JAKA Zu12 未连接"));
        return;
    }

    m_drillTimer->stop();

    QThread* t = QThread::create([=]() {
        errno_t ret = m_robot.login_out();
        const bool ok = (ret == ERR_SUCC);

        QMetaObject::invokeMethod(this, [=]() {
            m_connected = false;
            m_enabled = false;
            emit _stopFeedbackTimer();
            if (ok) {
                emit sigStatusMessage(tr("JAKA Zu12 断开成功"));
            }
            else {
                emit sigError(tr("JAKA Zu12 断开时出现错误，已强制断连"));
            }
            emit sigConnected(false);
            emit sigEnabled(false);
            }, Qt::QueuedConnection);
        });
    connect(t, &QThread::finished, t, &QThread::deleteLater);
    t->start();
}

// ─────────────────────────────────────────────────────────────────────────────
// enableRobot
// ─────────────────────────────────────────────────────────────────────────────
void JAKAZU12::enableRobot(bool enable)
{
    if (!m_connected) {
        emit sigError(tr("JAKA Zu12 未连接，无法使能"));
        return;
    }

    std::thread([=]() {
        errno_t ret = enable ? m_robot.enable_robot()
            : m_robot.disable_robot();
        const bool ok = (ret == ERR_SUCC);

        QMetaObject::invokeMethod(this, [=]() {
            if (ok) {
                m_enabled = enable;
                emit sigEnabled(enable);
                emit sigStatusMessage(enable ? tr("JAKA Zu12 已使能")
                    : tr("JAKA Zu12 已下使能"));
            }
            else {
                emit sigError(enable ? tr("JAKA Zu12 使能失败")
                    : tr("JAKA Zu12 下使能失败"));
            }
            }, Qt::QueuedConnection);
        }).detach();
}

// ─────────────────────────────────────────────────────────────────────────────
// clearError
// JAKA SDK 提供 err_flag_clear()（对齐 DobotCR5::clearError 异步调用风格）
// ─────────────────────────────────────────────────────────────────────────────
void JAKAZU12::clearError()
{
    if (!m_connected) return;

    std::thread([this]() {
        m_robot.err_flag_clear();
        }).detach();
}

// ─────────────────────────────────────────────────────────────────────────────
// setSpeedFactor  (1~100)
// ─────────────────────────────────────────────────────────────────────────────
void JAKAZU12::setSpeedFactor(int percent)
{
    if (percent < 1)   percent = 1;
    if (percent > 100) percent = 100;
    m_globalSpeed = static_cast<double>(percent);
    emit sigStatusMessage(tr("JAKA Zu12 速度已设置为 %1%").arg(percent));
}

// ─────────────────────────────────────────────────────────────────────────────
// moveJoint — 异步关节运动
//
// joint: std::vector<float>(6)，单位 deg（IRobot 规范）
// JAKA SDK joint_move 接受 deg，与 IRobot 接口单位一致，无需转换
// ─────────────────────────────────────────────────────────────────────────────
void JAKAZU12::moveJoint(const std::vector<float>& joint, const QString& tag)
{
    if (!m_connected || joint.size() < 6) {
        emit sigMoveFinished(false, tag);
        return;
    }

    std::vector<float> jointCopy(joint.begin(), joint.begin() + 6);

    std::thread([=]() {
        JointValue jv;
        for (int i = 0; i < 6; ++i) jv.jVal[i] = static_cast<double>(jointCopy[i]);

        errno_t ret = m_robot.joint_move(&jv, ABS, TRUE,
            m_globalSpeed,
            /*acc*/5, /*tol*/1, /*opt*/NULL);
        if (ret != ERR_SUCC) {
            QMetaObject::invokeMethod(this, [=]() {
                emit sigMoveFinished(false, tag);
                emit sigError(QString("JAKA Zu12 关节运动指令发送失败 [%1]").arg(tag));
                }, Qt::QueuedConnection);
            return;
        }

        waitArriveJointAndNotify(jointCopy, tag);
        }).detach();
}

// ─────────────────────────────────────────────────────────────────────────────
// movePose — 异步笛卡尔运动
//
// pose: std::vector<float>(6) [x(mm), y(mm), z(mm), rx(deg), ry(deg), rz(deg)]
// JAKA SDK linear_move 直接使用 mm / deg，与 IRobot 接口一致，无需转换
// ─────────────────────────────────────────────────────────────────────────────
void JAKAZU12::movePose(const std::vector<float>& pose, const QString& tag)
{
    if (!m_connected || pose.size() < 6) {
        emit sigMoveFinished(false, tag);
        return;
    }

    std::vector<float> poseCopy(pose.begin(), pose.begin() + 6);

    std::thread([=]() {
        CartesianPose cp;
        cp.tran.x = poseCopy[0];  cp.tran.y = poseCopy[1];  cp.tran.z = poseCopy[2];
        cp.rpy.rx = poseCopy[3];  cp.rpy.ry = poseCopy[4];  cp.rpy.rz = poseCopy[5];

        errno_t ret = m_robot.linear_move(&cp, MoveMode::ABS,
            TRUE, m_globalSpeed);
        if (ret != ERR_SUCC) {
            QMetaObject::invokeMethod(this, [=]() {
                emit sigMoveFinished(false, tag);
                emit sigError(QString("JAKA Zu12 笛卡尔运动指令发送失败 [%1]").arg(tag));
                }, Qt::QueuedConnection);
            return;
        }

        waitArriveAndNotify(poseCopy, tag);

        // 标定到位后回传当前位姿（对齐 DobotCR5）
        if (tag == "calib") {
            std::vector<float> curPose;
            if (getEndPose(curPose)) {
                QMetaObject::invokeMethod(this, [=]() {
                    emit sigPose(curPose);
                    }, Qt::QueuedConnection);
            }
        }
        }).detach();
}

// ─────────────────────────────────────────────────────────────────────────────
// getEndPose / getCurrentJoint — 同步读快照（IRobot 规范）
// ─────────────────────────────────────────────────────────────────────────────
bool JAKAZU12::getEndPose(std::vector<float>& pose)
{
    std::lock_guard<std::mutex> lk(m_snapMutex);
    if (!m_snap.valid) return false;
    pose = m_snap.endVec;
    return true;
}

bool JAKAZU12::getCurrentJoint(std::vector<float>& j)
{
    std::lock_guard<std::mutex> lk(m_snapMutex);
    if (!m_snap.valid) return false;
    j = m_snap.joint;
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// startScrewSequence — 拧螺丝序列入口（对齐 DobotCR5）
// ─────────────────────────────────────────────────────────────────────────────
void JAKAZU12::startScrewSequence(
    const std::vector<std::pair<std::vector<float>, std::vector<float>>>& pose,
    const Eigen::Affine3f& tcp,
    const QString rotType)
{
    if (!m_connected || isStop) return;
    m_tcp = tcp;
    m_rotType = rotType;
    m_pose = pose;
    recvRet_jaka = "";
    getCurrentJoint(capJoint);

    QTimer::singleShot(500, this, [this]() {
        movePose(m_pose[currentIndex].first, "1_P");
        });
}

// ─────────────────────────────────────────────────────────────────────────────
// onDrillStep — 沿工具 Z 轴步进下压（对齐 DobotCR5）
// ─────────────────────────────────────────────────────────────────────────────
void JAKAZU12::onDrillStep()
{
    if (!m_connected || isStop) return;

    if (recvRet_jaka == "ok" || recvRet_jaka == "ng") {
        recvRet_jaka = "";
        m_drillTimer->stop();
        emit sigMoveFinished(true, "1_B");
        return;
    }

    MathUtils mu;
    std::vector<float> endVec;
    getEndPose(endVec);
    if (endVec.size() != 6) return;

    Eigen::Affine3d trans;
    Eigen::Vector3d euler_angle = Eigen::Vector3d(endVec[3], endVec[4], endVec[5]);
    Eigen::Vector3d euler_rad = mu.deg2radVec(euler_angle);
    trans.linear() = mu.eulerToMatrix(euler_rad, m_rotType.toStdString());
    trans.translation() = Eigen::Vector3d(endVec[0], endVec[1], endVec[2]);

    Eigen::Affine3d off = Eigen::Affine3d::Identity();
    if (downSpeed <= 0.0) downSpeed = 1.0;
    off.translation() = Eigen::Vector3d(0.0, 0.0, -downSpeed);

    Eigen::Affine3d off_dis = trans * m_tcp.cast<double>() * off * m_tcp.inverse().cast<double>();
    std::vector<float> off_vec(6);
    off_vec[0] = static_cast<float>(off_dis.translation().x());
    off_vec[1] = static_cast<float>(off_dis.translation().y());
    off_vec[2] = static_cast<float>(off_dis.translation().z());
    off_vec[3] = endVec[3];
    off_vec[4] = endVec[4];
    off_vec[5] = endVec[5];

    movePose(off_vec, "1_D");
}

void JAKAZU12::robotStop()
{
    isStop = true;
    m_drillTimer->stop();
}

void JAKAZU12::robotStart()
{
    isStop = false;
    m_pose.clear();
    currentIndex = 0;

    QMutexLocker locker(&m_mutex_recv);
    recvRet_jaka = "";
}

void JAKAZU12::getScrewStatus(const std::string& status)
{
    QMutexLocker locker(&m_mutex_recv);
    if (status == "1") {
        recvRet_jaka = "ok";
    }
    else if (status == "2") {
        recvRet_jaka = "ng";
    }
    else {
        recvRet_jaka = "";
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// moveNextPose — 拧螺丝状态机（对齐 DobotCR5）
//
// tag 流程：
//   1_P  → 预备点到位 → 去目标点 1_T
//   1_T  → 目标点到位 → 触发拧螺丝机 + 启动下压定时器
//   1_B  → 下压结束   → 回到预备点 Next
//   Next → 预备点到位 → 下一个螺丝 / 全部完成回 Home
// ─────────────────────────────────────────────────────────────────────────────
void JAKAZU12::moveNextPose(bool ok, const QString& tag)
{
    if (!ok) {
        emit sigError("Not arrive target pose!");
        return;
    }

    if (tag == "1_P") {
        movePose(m_pose[currentIndex].second, "1_T");
    }
    else if (tag == "1_T") {
        emit sigScrewMachine();
        if (downSpeed <= 0) downSpeed = 1.0;
        m_drillTimer->start(static_cast<int>(1000 / downSpeed));
    }
    else if (tag == "1_B") {
        movePose(m_pose[currentIndex].first, "Next");
    }
    else if (tag == "Next") {
        currentIndex++;
        if (currentIndex == static_cast<int>(m_pose.size())) {
            currentIndex = 0;
            m_drillTimer->stop();
            QTimer::singleShot(500, this, [this]() { moveJoint(capJoint, "Home"); });
            return;
        }
        movePose(m_pose[currentIndex].first, "1_P");
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// 反馈定时器（100ms，主线程）
// ─────────────────────────────────────────────────────────────────────────────
void JAKAZU12::startFeedbackTimer() { m_feedbackTimer->start(); }
void JAKAZU12::stopFeedbackTimer() { m_feedbackTimer->stop(); }

void JAKAZU12::onFeedbackTimerTimeout()
{
    if (!m_connected) return;

    updateSnapshot();

    std::vector<float> joint, pose;
    {
        std::lock_guard<std::mutex> lk(m_snapMutex);
        if (!m_snap.valid) return;
        joint = m_snap.joint;
        pose = m_snap.endVec;
    }
    emit sigRobotStatus(joint, pose);
}

void JAKAZU12::updateSnapshot()
{
    std::vector<float> pose, joint;
    if (!readCurrentPose(pose) || !readCurrentJoint(joint)) return;

    FeedbackSnapshot snap;
    snap.valid = true;
    snap.endVec = pose;
    snap.joint = joint;

    std::lock_guard<std::mutex> lk(m_snapMutex);
    m_snap = std::move(snap);
}

// ─────────────────────────────────────────────────────────────────────────────
// SDK 直接读取（供快照刷新使用）
// JAKA SDK 坐标单位：mm / deg，与 IRobot 接口一致，无需转换
// ─────────────────────────────────────────────────────────────────────────────
bool JAKAZU12::readCurrentPose(std::vector<float>& pose)
{
    CartesianPose cp;
    if (m_robot.get_tcp_position(&cp) != ERR_SUCC) return false;

    pose.resize(6);
    pose[0] = static_cast<float>(cp.tran.x);
    pose[1] = static_cast<float>(cp.tran.y);
    pose[2] = static_cast<float>(cp.tran.z);
    pose[3] = static_cast<float>(cp.rpy.rx);
    pose[4] = static_cast<float>(cp.rpy.ry);
    pose[5] = static_cast<float>(cp.rpy.rz);
    return true;
}

bool JAKAZU12::readCurrentJoint(std::vector<float>& joint)
{
    JointValue jv{ 0 };
    if (m_robot.get_joint_position(&jv) != ERR_SUCC) return false;

    joint.resize(6);
    for (int i = 0; i < 6; ++i) {
        joint[i] = static_cast<float>(jv.jVal[i]);
    }
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// waitArriveAndNotify — 笛卡尔到位检测（后台线程阻塞）
//
// 设计说明（对齐 DobotCR5）：
//   • 读取由定时器维护的 m_snap 快照，避免与反馈线程竞争直接访问 SDK
//   • 阈值：位置 1mm，角度 1deg
//   • 超时（默认 15s）或断连时 emit sigMoveFinished(false, tag)
// ─────────────────────────────────────────────────────────────────────────────
void JAKAZU12::waitArriveAndNotify(const std::vector<float>& targetPose,
    const QString& tag,
    int timeoutMs)
{
    const auto deadline = std::chrono::steady_clock::now()
        + std::chrono::milliseconds(timeoutMs);

    auto near = [](float a, float b, float tol = 1.0f) {
        return std::fabs(a - b) < tol;
    };

    while (m_connected) {
        if (std::chrono::steady_clock::now() > deadline) {
            QMetaObject::invokeMethod(this, [=]() {
                emit sigMoveFinished(false, tag);
                emit sigError(QString("JAKA Zu12 笛卡尔运动超时 [%1]").arg(tag));
                }, Qt::QueuedConnection);
            return;
        }

        std::vector<float> cur;
        {
            std::lock_guard<std::mutex> lk(m_snapMutex);
            if (m_snap.valid) cur = m_snap.endVec;
        }

        if (cur.size() == 6) {
            const bool arrived = near(cur[0], targetPose[0])
                && near(cur[1], targetPose[1])
                && near(cur[2], targetPose[2])
                && near(cur[3], targetPose[3])
                && near(cur[4], targetPose[4])
                && near(cur[5], targetPose[5]);
            if (arrived) {
                QMetaObject::invokeMethod(this, [=]() {
                    emit sigMoveFinished(true, tag);
                    }, Qt::QueuedConnection);
                return;
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    // 断连退出
    QMetaObject::invokeMethod(this, [=]() {
        emit sigMoveFinished(false, tag);
        }, Qt::QueuedConnection);
}

// ─────────────────────────────────────────────────────────────────────────────
// waitArriveJointAndNotify — 关节到位检测（后台线程阻塞）
// joint: deg，读取 m_snap 快照（对齐 DobotCR5）
// ─────────────────────────────────────────────────────────────────────────────
void JAKAZU12::waitArriveJointAndNotify(const std::vector<float>& targetJoint,
    const QString& tag,
    int timeoutMs)
{
    const auto deadline = std::chrono::steady_clock::now()
        + std::chrono::milliseconds(timeoutMs);

    auto near = [](float a, float b, float tol = 1.0f) {
        return std::fabs(a - b) < tol;
    };

    while (m_connected) {
        if (std::chrono::steady_clock::now() > deadline) {
            QMetaObject::invokeMethod(this, [=]() {
                emit sigMoveFinished(false, tag);
                emit sigError(QString("JAKA Zu12 关节运动超时 [%1]").arg(tag));
                }, Qt::QueuedConnection);
            return;
        }

        std::vector<float> cur;
        {
            std::lock_guard<std::mutex> lk(m_snapMutex);
            if (m_snap.valid) cur = m_snap.joint;
        }

        if (cur.size() == 6) {
            bool arrived = true;
            for (int i = 0; i < 6; ++i) {
                if (!near(cur[i], targetJoint[i])) { arrived = false; break; }
            }
            if (arrived) {
                QMetaObject::invokeMethod(this, [=]() {
                    emit sigMoveFinished(true, tag);
                    }, Qt::QueuedConnection);
                return;
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    // 断连退出
    QMetaObject::invokeMethod(this, [=]() {
        emit sigMoveFinished(false, tag);
        }, Qt::QueuedConnection);
}
