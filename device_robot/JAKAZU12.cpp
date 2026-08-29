
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
    connect(m_feedbackTimer, &QTimer::timeout, this, &JAKAZU12::onFeedbackTimerTimeout);

    connect(this, &JAKAZU12::_startFeedbackTimer,
        this, &JAKAZU12::startFeedbackTimer, Qt::QueuedConnection);
    connect(this, &JAKAZU12::_stopFeedbackTimer,
        this, &JAKAZU12::stopFeedbackTimer, Qt::QueuedConnection);

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


/*---------------------查询---------------------*/
bool JAKAZU12::isConnected() const { return m_connected.load(); }

bool JAKAZU12::isEnabled()   const { return m_enabled.load(); }

/*---------------------控制---------------------*/
void JAKAZU12::connectRobot(const RobotPara& para)
{
    if (m_connected) {
        emit sigStatusMessage(QStringLiteral("JAKA Zu12 已连接，无需重复连接"));
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
            emit sigError(QStringLiteral("JAKA Zu12 登录失败（IP: %1）")
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
            emit sigStatusMessage(QStringLiteral("JAKA Zu12 连接成功"));
        }
        else {
            emit sigError(QStringLiteral("JAKA Zu12 上电失败"));
            m_robot.login_out();
        }
        emit sigConnected(ok);
        }, Qt::QueuedConnection);
}

void JAKAZU12::disconnectRobot()
{
    if (!m_connected) {
        emit sigStatusMessage(QStringLiteral("JAKA Zu12 未连接"));
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
                emit sigStatusMessage(QStringLiteral("JAKA Zu12 断开成功"));
            }
            else {
                emit sigError(QStringLiteral("JAKA Zu12 断开时出现错误，已强制断连"));
            }
            emit sigConnected(false);
            emit sigEnabled(false);
            }, Qt::QueuedConnection);
        });
    connect(t, &QThread::finished, t, &QThread::deleteLater);
    t->start();
}

void JAKAZU12::enableRobot(bool enable)
{
    if (!m_connected) {
        emit sigError(QStringLiteral("JAKA Zu12 未连接，无法使能"));
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
                emit sigStatusMessage(enable ? QStringLiteral("JAKA Zu12 已使能")
                    : QStringLiteral("JAKA Zu12 已下使能"));
            }
            else {
                emit sigError(enable ? QStringLiteral("JAKA Zu12 使能失败")
                    : QStringLiteral("JAKA Zu12 下使能失败"));
            }
            }, Qt::QueuedConnection);
        }).detach();
}

void JAKAZU12::clearError()
{

}

void JAKAZU12::setSpeedFactor(int percent)
{
    if (percent < 1)   percent = 1;
    if (percent > 20) percent = 20;
    m_globalSpeed = static_cast<double>(percent);
    emit sigStatusMessage(QStringLiteral("JAKA Zu12 速度已设置为 %1%").arg(percent));
}

/*---------------------运动---------------------*/
void JAKAZU12::moveJoint(const std::vector<float>& joint, const QString& tag)
{
    if (!m_connected || joint.size() < 6) {
        emit sigMoveFinished(false, tag);
        return;
    }

    std::vector<float> jointCopy(joint.begin(), joint.begin() + 6);

    std::thread([=]() {
        JointValue jv;
        for (int i = 0; i < 6; ++i) jv.jVal[i] = static_cast<double>(jointCopy[i] / 180.0f * M_PI);

        errno_t ret = m_robot.joint_move(&jv, ABS, FALSE, m_globalSpeed * M_PI / 180, 1, 0.01, NULL);
        if (ret != ERR_SUCC) {
            QMetaObject::invokeMethod(this, [=]() {
                emit sigMoveFinished(false, tag);
                emit sigError(QStringLiteral("JAKA Zu12 关节运动指令发送失败 [%1]").arg(tag));
                }, Qt::QueuedConnection);
            return;
        }

        waitArriveJointAndNotify(jointCopy, tag);
        }).detach();
}

void JAKAZU12::movePose(const std::vector<float>& pose, const QString& tag)
{
    if (!m_connected || pose.size() < 6) {
        emit sigMoveFinished(false, tag);
        return;
    }

    std::vector<float> poseCopy(pose.begin(), pose.begin() + 6);

    std::thread([=]() {
        CartesianPose cart;
        cart.tran.x = poseCopy[0]; cart.tran.y = poseCopy[1]; cart.tran.z = poseCopy[2];
        cart.rpy.rx = poseCopy[3] * M_PI / 180.0f ; cart.rpy.ry = poseCopy[4] * M_PI / 180.0f; cart.rpy.rz = poseCopy[5] * M_PI / 180.0f;

        errno_t ret = m_robot.linear_move(&cart, MoveMode::ABS, FALSE, m_globalSpeed * 10, 200, 0.0, NULL, m_globalSpeed * 5 * M_PI / 180, 0.5);
        if (ret != ERR_SUCC) {
            QMetaObject::invokeMethod(this, [=]() {
                emit sigMoveFinished(false, tag);
                emit sigError(QStringLiteral("JAKA Zu12 笛卡尔运动指令发送失败 [%1]").arg(tag));
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

/*---------------------抓取相关流程---------------------*/
void JAKAZU12::startGraspSequence(const std::vector<std::vector<float>>& path, const std::vector<float>& tgt_joint) {
    if (!m_connected || !m_enabled || isStop) {
        emit sigError("Robot not ready to move!");
        return;
    }
    if (path.size() < 1) {
        emit sigError("Path size data not ready!");
        return;
    }
    if (!loadPlaceStatus) {
        emit sigError("Place config not ready now!");
        return;
    }
    
    m_path.clear();
    m_path = densifyJoints(path);
    m_tgt_joint = std::move(tgt_joint);

    // 1 执行轨迹
    std::vector<float> wait_vec = toFloat(place_config.WaitJoint);
    moveJoint(wait_vec, "start");
}

void JAKAZU12::loadPlaceConfig(const std::string& path) {

    loadPlaceStatus = false;

    std::ifstream file(path);
    if (!file.is_open()) {
        emit sigError("load place config file failed!");
        return;
    }

    json placeConfig;
    try
    {
        file >> placeConfig;
        place_config = placeConfig.get<utils::PlaceConfig>();

        loadPlaceStatus = true;
    }
    catch (const std::exception& e)
    {
        file.close();
        emit sigError(QString("load place config file failed, error : %1").arg(e.what()));
        return;
    }
    file.close();
}

errno_t JAKAZU12::runTrajectoryPoint(const std::vector<std::vector<float>>& m_path)
{
    if (!m_connected) {
        emit sigError(QStringLiteral("JAKA Zu12 未连接，无法执行伺服轨迹"));
        return -1;
    }

    const std::vector<std::vector<float>> pathCopy = m_path;

    std::thread([=]() {
        // NLF 单位是 °/s、°/s²、°/s³。必须高于 densifyJoints 的巡航速度，否则会把轨迹再削慢。
        // 80/180/720 接近 JAKA 文档默认值，让速度由插补梯形决定。
        errno_t ret = m_robot.servo_move_use_joint_NLF(80, 180, 720);
        ret = m_robot.servo_move_enable(TRUE);
        if (ret != ERR_SUCC) {
            QMetaObject::invokeMethod(this, [=]() {
                emit sigError(QStringLiteral("servo mode enable failed!"));
                emit sigMoveFinished(false, QStringLiteral("pre_grasp"));
            }, Qt::QueuedConnection);
            return;
        }

        auto next = std::chrono::steady_clock::now();
        for (const auto& qDeg : pathCopy)
        {
            if (isStop) break;
            JointValue jpos;
            for (int i = 0; i < 6; ++i) jpos.jVal[i] = static_cast<double>(qDeg[i]);
            m_robot.servo_j(&jpos, ABS);
            next += std::chrono::milliseconds(8);
            std::this_thread::sleep_until(next);
        }

        // 等最后几个 8ms 周期被控制器吃完，再退出伺服
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        m_robot.servo_move_enable(FALSE);

        const bool ok = !isStop;
        QMetaObject::invokeMethod(this, [=]() {
            emit sigMoveFinished(ok, QStringLiteral("pre_grasp"));
        }, Qt::QueuedConnection);
    }).detach();

    return 0;
}

/*---------------------螺丝拧紧相关流程---------------------*/
void JAKAZU12::startScrewSequence(
    const std::vector<std::pair<std::vector<float>, std::vector<float>>>& pose,
    const Eigen::Affine3f& tcp,
    const std::string rotType)
{
    if (!m_connected || isStop) return;
    m_tcp = tcp;
    m_rotType = rotType;
    m_screw_pose = pose;
    recvRet_jaka = "";
    currentIndex = 0;
    bool ret = getCurrentJoint(capJoint);

    std::vector<float> currentPose;
    if (!getEndPose(currentPose)) {
        emit sigError("Get robot end pose error!");
        return;
    }
    
    for (auto &pose : m_screw_pose) {
        ret = quat2eulerJaka(pose.first, currentPose);
        pose.first = currentPose;
        ret = quat2eulerJaka(pose.second, currentPose);
        pose.second = currentPose;
    }
    
    QTimer::singleShot(500, this, [this]() {
        movePose(m_screw_pose[currentIndex].first, "grasp_pass");
    });
}

void JAKAZU12::onDrillStep()
{
    if (!m_connected || isStop) return;

    if (recvRet_jaka == "ok" || recvRet_jaka == "ng") {
        recvRet_jaka = "";
        m_drillTimer->stop();
        emit sigMoveFinished(true, "1_B");
        return;
    }
    if (downSpeed <= 0.0) downSpeed = 1.0;
    std::vector<float> off_vec;
    calcOffMatrix(m_tcp, -downSpeed, m_rotType, off_vec);

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
    m_grasp_pose.clear();
    m_screw_pose.clear();
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

/*---------------------运动状态转移---------------------*/
void JAKAZU12::moveNextPose(bool ok, const QString& tag)
{
    if (!ok) return;
    if (isStop) return;

    if (tag == "start") {
        if (m_path.size() < 10) return;
        runTrajectoryPoint(m_path);
        m_path.clear();
    }
    else if (tag == "pre_grasp") {
        QTimer::singleShot(200, this, [=]() { moveJoint(m_tgt_joint, "tgt_grasp"); });
    }
    else if (tag == "tgt_grasp") {
        emit sigGraspMachine(true, "grasp_pass");
    }
    else if (tag == "grasp_pass") {
        std::vector<float> pass_vec = toFloat(place_config.PassJoint);
        assert(pass_vec.size() == 6);
        QTimer::singleShot(500, this, [=]() { moveJoint(pass_vec, "pass");});
    }
    else if (tag == "pass") {
        if (!place_config.useArray) {
            std::vector<float> place_vec = toFloat(place_config.PlaceJoint);
            assert(place_vec.size() == 6);
            moveJoint(place_vec, "grasp_place");
        }
        else {
            std::vector<float> place_pass_vec = toFloat(place_config.PlacePassJoint);
            assert(place_pass_vec.size() == 6);
            moveJoint(place_pass_vec, "grasp_pass_place");
        }
    }
    else if (tag == "grasp_pass_place") {
        std::vector<float> firstplace_vec;
        if (count_grasp > 0) {
            firstplace_vec = last_place_vec;
        }
        else {
            firstplace_vec = toFloat(place_config.PlaceJoint);
        }
        getPlaceJoint(firstplace_vec, place_config.array, count_grasp, place_config.XFirst, last_place_vec);
        moveJoint(last_place_vec, "grasp_place");
    }
    else if (tag == "grasp_place") {
        sigGraspMachine(false, "grasp_wait");
        count_grasp++;
    }
    else if (tag == "grasp_wait") {
        std::vector<float> wait_vec = toFloat(place_config.WaitJoint);
        QTimer::singleShot(500, this, [=]() { moveJoint(wait_vec, "wait"); });
    }

    if (m_screw_pose.size() < 1) return;
    if (tag == "1_P") {
        movePose(m_screw_pose[currentIndex].second, "1_T");
    }
    else if (tag == "1_T") {
        emit sigScrewMachine();
        if (downSpeed <= 0) downSpeed = 1.0;
        m_drillTimer->start(static_cast<int>(1000 / downSpeed));
    }
    else if (tag == "1_B") {
        movePose(m_screw_pose[currentIndex].first, "Next");
    }
    else if (tag == "Next") {
        currentIndex++;
        if (currentIndex == static_cast<int>(m_screw_pose.size())) {
            currentIndex = 0;
            m_drillTimer->stop();
            QTimer::singleShot(500, this, [this]() { moveJoint(capJoint, "Home"); });
            return;
        }
        movePose(m_screw_pose[currentIndex].first, "1_P");
    }
}


/*---------------------状态反馈---------------------*/
void JAKAZU12::startFeedbackTimer() { m_feedbackTimer->start(); }

void JAKAZU12::stopFeedbackTimer() { m_feedbackTimer->stop(); }

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

bool JAKAZU12::readCurrentPose(std::vector<float>& pose)
{
    CartesianPose cp;
    if (m_robot.get_tcp_position(&cp) != ERR_SUCC) return false;

    pose.resize(6);
    pose[0] = static_cast<float>(cp.tran.x);
    pose[1] = static_cast<float>(cp.tran.y);
    pose[2] = static_cast<float>(cp.tran.z);
    pose[3] = static_cast<float>(cp.rpy.rx * 180.0f / M_PI);
    pose[4] = static_cast<float>(cp.rpy.ry * 180.0f / M_PI);
    pose[5] = static_cast<float>(cp.rpy.rz * 180.0f / M_PI);
    return true;
}

bool JAKAZU12::readCurrentJoint(std::vector<float>& joint)
{
    JointValue jv{ 0 };
    if (m_robot.get_joint_position(&jv) != ERR_SUCC) return false;

    joint.resize(6);
    for (int i = 0; i < 6; ++i) {
        joint[i] = static_cast<float>(jv.jVal[i] * 180.0f / M_PI);
    }
    return true;
}

/*---------------------笛卡尔位姿到位检测---------------------*/
void JAKAZU12::waitArriveAndNotify(const std::vector<float>& targetPose,
    const QString& tag,
    int timeoutMs)
{
    const auto deadline = std::chrono::steady_clock::now()
        + std::chrono::milliseconds(timeoutMs);

    auto near = [](float a, float b, float tol = 0.1f) {
        return std::fabs(a - b) < tol;
    };

    while (m_connected) {
        if (std::chrono::steady_clock::now() > deadline) {
            QMetaObject::invokeMethod(this, [=]() {
                emit sigMoveFinished(false, tag);
                emit sigError(QStringLiteral("JAKA Zu12 笛卡尔运动超时 [%1]").arg(tag));
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

/*---------------------关节到位检测---------------------*/
void JAKAZU12::waitArriveJointAndNotify(const std::vector<float>& targetJoint,
    const QString& tag,
    int timeoutMs)
{
    const auto deadline = std::chrono::steady_clock::now()
        + std::chrono::milliseconds(timeoutMs);

    auto near = [](float a, float b, float tol = 0.1f) {
        return std::fabs(a - b) < tol;
    };

    while (m_connected) {
        if (std::chrono::steady_clock::now() > deadline) {
            QMetaObject::invokeMethod(this, [=]() {
                emit sigMoveFinished(false, tag);
                emit sigError(QStringLiteral("JAKA Zu12 关节运动超时 [%1]").arg(tag));
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

/*---------------------辅助函数---------------------*/
void JAKAZU12::calcOffMatrix(const Eigen::Affine3f tcp, double offInter, std::string rotType, std::vector<float>& off_vec) {
    MathUtils mu;
    std::vector<float> endVec;
    getEndPose(endVec);
    if (endVec.size() != 6) return;

    Eigen::Affine3d trans;
    Eigen::Vector3d euler_angle = Eigen::Vector3d(endVec[3], endVec[4], endVec[5]);
    Eigen::Vector3d euler_rad = mu.deg2radVec(euler_angle);
    trans.linear() = mu.eulerToMatrix(euler_rad, rotType);
    trans.translation() = Eigen::Vector3d(endVec[0], endVec[1], endVec[2]);

    Eigen::Affine3d off = Eigen::Affine3d::Identity();
    off.translation() = Eigen::Vector3d(0.0, 0.0, offInter);

    Eigen::Affine3d off_dis = trans * tcp.cast<double>() * off * tcp.inverse().cast<double>();
    off_vec.resize(6);
    off_vec[0] = static_cast<float>(off_dis.translation().x());
    off_vec[1] = static_cast<float>(off_dis.translation().y());
    off_vec[2] = static_cast<float>(off_dis.translation().z());
    off_vec[3] = endVec[3];
    off_vec[4] = endVec[4];
    off_vec[5] = endVec[5];

    return;
}

bool JAKAZU12::quat2eulerJaka(const std::vector<float>& quat_in, std::vector<float>& euler_out) {
    Quaternion quat;
    quat.s = quat_in[3];  quat.x = quat_in[4]; quat.y = quat_in[5];  quat.z = quat_in[6];
    RotMatrix rot_matrix;
    int ret = m_robot.quaternion_to_rot_matrix(&quat, &rot_matrix);
    if (ret != ERR_SUCC) return false;
    Rpy rpy;
    ret = m_robot.rot_matrix_to_rpy(&rot_matrix, &rpy);
    euler_out.resize(6);
    for (int i = 0; i < 3; ++i) {
        euler_out[i] = quat_in[i];
    }
    euler_out[3] = rpy.rx * 180.0f / M_PI;
    euler_out[4] = rpy.ry * 180.0f / M_PI;
    euler_out[5] = rpy.rz * 180.0f / M_PI;
}

std::vector<float> JAKAZU12::toFloat(const utils::Joint& j) {

     std::vector<float> pass_vec{ static_cast<float>(j.j1), static_cast<float>(j.j2), static_cast<float>(j.j3),
            static_cast<float>(j.j4), static_cast<float>(j.j5), static_cast<float>(j.j6) };
     return pass_vec;
 }

std::vector<float> JAKAZU12::toVector(const JointValue& jv) {
     std::vector <float> joint_vec;
     joint_vec.resize(6);
     for (int i = 0; i < 6; ++i) {
         joint_vec[i] = static_cast<float>(jv.jVal[i] * 180.0f / M_PI);
     }
     return joint_vec;
 }

JointValue JAKAZU12::toJoint(const std::vector<float>& joint_vec) {
     JointValue jv;
     for (int i = 0; i < 6; ++i) jv.jVal[i] = static_cast<double>(joint_vec[i] / 180.0f * M_PI);
     return jv;
 }

inline std::vector<std::vector<float>> JAKAZU12::densifyJoints(const std::vector<std::vector<float>>& joints)
 {
     const double dt = 0.008;                          // s
     // 默认 m_globalSpeed=5 → 40 deg/s（原先写死 4 deg/s）。上限 90，避开 180 deg/s 硬限。
     double maxVelDeg = m_globalSpeed * 8.0;
     if (maxVelDeg < 20.0) maxVelDeg = 20.0;
     if (maxVelDeg > 90.0) maxVelDeg = 90.0;
     const double maxVel = maxVelDeg * M_PI / 180.0;   // rad/s
     const double maxAcc = 3.0;       // rad/s^2
     const double hardVel = 2.0;      // rad/s，单步安全（≈114 deg/s）
     const double minDisp = 1e-6;
     std::vector<std::vector<float>> out;
     if (joints.empty())
     {
         return out;
     }
     out.push_back(joints.front());
     auto maxAbsDiff = [](const std::vector<float>& a, const std::vector<float>& b) {
         double m = 0.0;
         const std::size_t n = std::min(a.size(), b.size());
         for (std::size_t d = 0; d < n; ++d)
         {
             m = std::max(m, static_cast<double>(std::fabs(b[d] - a[d])));
         }
         return m;
     };
     for (std::size_t i = 0; i + 1 < joints.size(); ++i)
     {
         const std::vector<float>& q0 = joints[i];
         const std::vector<float>& q1 = joints[i + 1];
         const double s = maxAbsDiff(q0, q1);
         if (s <= minDisp)
         {
             continue;
         }
         const double sCruise = maxVel * maxVel / maxAcc;
         double Tmin = (s >= sCruise) ? (s / maxVel + maxVel / maxAcc)
             : (2.0 * std::sqrt(s / maxAcc));
         int N = std::max(1, static_cast<int>(std::ceil(Tmin / dt)));
         N = std::max(N, static_cast<int>(std::ceil(s / (hardVel * dt))));
         double T = 0.0, v = 0.0, a = maxAcc, Tacc = 0.0, Tflat = 0.0;
         for (int guard = 0; guard < 16; ++guard)
         {
             T = static_cast<double>(N) * dt;
             const double disc = a * a * T * T - 4.0 * a * s;
             if (disc < 0.0)
             {
                 ++N;
                 continue;
             }
             v = 0.5 * (a * T - std::sqrt(disc));
             Tacc = v / a;
             Tflat = T - 2.0 * Tacc;
             if (Tflat < 0.0)
             {
                 Tflat = 0.0;
                 Tacc = T / 2.0;
                 v = a * Tacc;
             }
             if (v > maxVel * 1.000001)
             {
                 ++N;
                 continue;
             }
             break;
         }
         auto pos = [&](double t) {
             if (t <= 0.0) return 0.0;
             if (t >= T) return s;
             if (t < Tacc) return 0.5 * a * t * t;
             if (t < Tacc + Tflat) return 0.5 * a * Tacc * Tacc + v * (t - Tacc);
             const double tDec = t - Tacc - Tflat;
             return 0.5 * a * Tacc * Tacc + v * Tflat + v * tDec - 0.5 * a * tDec * tDec;
             };
         for (int k = 1; k <= N; ++k)
         {
             const double u = std::min(1.0, std::max(0.0, pos(k * dt) / s));
             std::vector<float> q(q0.size());
             for (std::size_t d = 0; d < q0.size(); ++d)
             {
                 const float q1d = d < q1.size() ? q1[d] : q0[d];
                 q[d] = static_cast<float>(q0[d] + u * (q1d - q0[d]));
             }
             out.push_back(q);
         }
     }
     return out;
 }

errno_t JAKAZU12::getPlaceJoint(const std::vector<float>& cur_place_joint, const utils::ArrayConfig& cfg,
     int n, bool x_first, std::vector<float>& new_place_joint)
 {
     if (cur_place_joint.size() != 6 || cfg.layerX <= 0 || cfg.layerY <= 0)
     {
         return -1;
     }

     JointValue cur_jv = toJoint(cur_place_joint);

     const int per_layer = cfg.layerX * cfg.layerY;
     const int idx = n;
     const int iz = idx / per_layer;
     const int idx_in_layer = idx % per_layer;
     int ix = 0;
     int iy = 0;
     if (x_first)
     {
         ix = idx_in_layer % cfg.layerX;
         iy = idx_in_layer / cfg.layerX;
     }
     else
     {
         iy = idx_in_layer % cfg.layerY;
         ix = idx_in_layer / cfg.layerY;
     }
     CartesianPose pose0;
     errno_t ret = m_robot.kine_forward(&cur_jv, &pose0);
     if (ret != ERR_SUCC)
     {
         return ret;
     }
     CartesianPose pose_n = pose0;
     pose_n.tran.x += static_cast<double>(ix) * cfg.obj_length + 10;
     pose_n.tran.y += static_cast<double>(iy) * cfg.obj_width + 10;
     pose_n.tran.z += static_cast<double>(iz) * cfg.obj_height;
     JointValue new_jv;
     ret = m_robot.kine_inverse(&cur_jv, &pose_n, &new_jv);
     new_place_joint = toVector(new_jv); 
     return ret;
 }

void JAKAZU12::quat2Joint(const std::vector<float>& quat_in, std::vector<float>& new_joint) {
    std::vector<float> pose;
    quat2eulerJaka(quat_in, pose);

    std::vector<float> wait = toFloat(place_config.WaitJoint);
    JointValue cur_jv = toJoint(wait);

    CartesianPose pose_n;
    pose_n.tran.x = quat_in[0];
    pose_n.tran.y = quat_in[1];
    pose_n.tran.z = quat_in[2];
    pose_n.rpy.rx = pose[3] / 180.0 * M_PI;
    pose_n.rpy.ry = pose[4] / 180.0 * M_PI;
    pose_n.rpy.rz = pose[5] / 180.0 * M_PI;
    JointValue new_jv;
    m_robot.kine_inverse(&cur_jv, &pose_n, &new_jv);
    new_joint = toVector(new_jv);

    return;
 }