
#include "AuboC5.h"
#include "tool/MathUtils.h"

#include <QThread>
#include <QMetaObject>
#include <QDebug>

#include <thread>
#include <chrono>
#include <cmath>
#include <algorithm>
#include <atomic>
#include <cstddef>

/** @brief RPC 请求超时时间（毫秒） */
constexpr int kRpcTimeoutMs = 1000;

/** @brief 运行时启动超时时间（毫秒） */
constexpr int kRuntimeStartTimeoutMs = 3000;

/** @brief 运行时状态轮询间隔（毫秒） */
constexpr int kRuntimePollIntervalMs = 20;

/** @brief 上电超时时间（毫秒） */
constexpr int kPowerOnTimeoutMs = 20000;

/** @brief 松刹车超时时间（毫秒） */
constexpr int kStartupTimeoutMs = 10000;

/** @brief 机械臂模式轮询间隔（毫秒） */
constexpr int kRobotModePollIntervalMs = 1000;

namespace {
    bool waitForRobotMode(RobotInterfacePtr robot, RobotModeType target_mode,
        int timeout_ms)
    {
        int elapsed_ms = 0;
        while (elapsed_ms < timeout_ms) {
            auto current_mode = robot->getRobotState()->getRobotModeType();
            if (current_mode == target_mode) {
                return true;
            }
            std::this_thread::sleep_for(
                std::chrono::milliseconds(kRobotModePollIntervalMs));
            elapsed_ms += kRobotModePollIntervalMs;
        }
        return false;
    }

    bool waitForRuntimeRunning(RuntimeMachinePtr runtime,
        int timeout_ms = kRuntimeStartTimeoutMs)
    {
        int elapsed_ms = 0;
        while (elapsed_ms < timeout_ms) {
            if (runtime->getRuntimeState() == RuntimeState::Running) {
                return true;
            }
            std::this_thread::sleep_for(
                std::chrono::milliseconds(kRuntimePollIntervalMs));
            elapsed_ms += kRuntimePollIntervalMs;
        }
        return false;
    }

    int waitServoJointComplete(RobotInterfacePtr impl, int timeout_ms = 10000)
    {
        auto start_time = std::chrono::steady_clock::now();

        while (impl->getMotionControl()->getMotionLeftTime(0) != 0) {
            auto current_time = std::chrono::steady_clock::now();
            auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                current_time - start_time)
                .count();

            if (elapsed_ms >= timeout_ms) {
                return 1;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }

        while (!impl->getRobotState()->isSteady()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }

        return 0;
    }

    int waitArrival(RobotInterfacePtr impl, const std::atomic<bool>& abort)
    {
        // 接口调用: 获取当前的运动指令 ID
        int exec_id = impl->getMotionControl()->getExecId();

        int cnt = 0;
        // 在等待机械臂开始运动时，获取exec_id最大的重试次数
        int max_retry_count = 50;

        // 等待机械臂开始运动
        while (exec_id == -1) {
            if (abort.load()) {
                return -1;
            }
            if (cnt++ > max_retry_count) {
                return -1;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            exec_id = impl->getMotionControl()->getExecId();
        }

        // 等待机械臂动作完成
        while (exec_id != -1) {
            if (abort.load()) {
                return -1;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            exec_id = impl->getMotionControl()->getExecId();
        }

        return abort.load() ? -1 : 0;
    }

    int isBufferValid(RobotInterfacePtr impl, const std::atomic<bool>& abort)
    {
        // 调用pathBufferValid最大的重试次数
        int max_retry_count = 5;
        // 调用pathBufferValid的次数
        int cnt = 0;
        bool isValid = impl->getMotionControl()->pathBufferValid("rec");

        while (!isValid) {
            if (abort.load()) {
                return -1;
            }
            if (cnt++ > max_retry_count) {
                return -1;
            }
            isValid = impl->getMotionControl()->pathBufferValid("rec");
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        return 0;
    }
}

AUBOC5::AUBOC5(QObject* parent)
    : IRobot(parent)
    , m_feedbackTimer(new QTimer(this))
    , m_drillTimer(new QTimer(this))
{
    m_feedbackTimer->setInterval(100);
    connect(m_feedbackTimer, &QTimer::timeout, this, &AUBOC5::onFeedbackTimerTimeout);

    connect(this, &AUBOC5::_startFeedbackTimer,
        this, &AUBOC5::startFeedbackTimer, Qt::QueuedConnection);
    connect(this, &AUBOC5::_stopFeedbackTimer,
        this, &AUBOC5::stopFeedbackTimer, Qt::QueuedConnection);

    QObject::connect(m_drillTimer, &QTimer::timeout, this, &AUBOC5::onDrillStep);

    QObject::connect(this, &AUBOC5::sigMoveFinished, this, &AUBOC5::moveNextPose);

	m_connected.store(false);
	m_enabled.store(false);
}

AUBOC5::~AUBOC5()
{
    m_trajAbort.store(true);
    isStop = true;
    m_feedbackTimer->stop();
    m_drillTimer->stop();

    if (m_connected && m_rpcClient) {
        if (m_trajRunning.load()) {
            m_rpcClient->getRuntimeMachine()->stop();
            if (task_id.load() >= 0) {
                m_rpcClient->getRuntimeMachine()->deleteTask(task_id.load());
                task_id.store(-1);
            }
        }
        m_rpcClient->logout();
        m_rpcClient->disconnect();
    }
}

/*---------------------查询---------------------*/
bool AUBOC5::isConnected() const { return m_connected.load(); }

bool AUBOC5::isEnabled()   const { return m_enabled.load(); }

/*---------------------控制---------------------*/
void AUBOC5::connectRobot(const RobotPara& para)
{
    if (isConnected()) {
        emit sigStatusMessage(QStringLiteral("Aubo C5 已连接，无需重复连接"));
        return;
    }

    const std::string ip = para.IpAddress;

    m_rpcClient = std::make_shared<RpcClient>();
    // 接口调用: 设置 RPC 超时
    m_rpcClient->setRequestTimeout(1000);
    
    if (m_rpcClient->connect(ip, 30004) != AUBO_OK) {
        emit sigError(QStringLiteral("Aubo C5 连接失败（IP: %1）")
            .arg(QString::fromStdString(ip)));
        return ;
    }

    if (m_rpcClient->login("aubo", "123456") != AUBO_OK) {
        emit sigError(QStringLiteral("Aubo C5 登录失败!"));
        m_rpcClient->disconnect();
        return;
    }

    auto robot_names = m_rpcClient->getRobotNames();
    if (robot_names.empty()) {
        emit sigError(QStringLiteral("未找到机器人!"));
        m_rpcClient->logout();
        m_rpcClient->disconnect();
        return;
    }
    m_robot = m_rpcClient->getRobotInterface(robot_names.front());
    m_connected.store(true);

    emit _startFeedbackTimer();
	emit sigConnected(m_connected.load());
    emit sigStatusMessage(QStringLiteral("Aubo C5 连接成功（IP: %1）").arg(QString::fromStdString(ip)));
    return;
}

void AUBOC5::disconnectRobot()
{
    if (!isConnected()) {
        emit sigStatusMessage(QStringLiteral("Aubo C5 未连接"));
        return;
    }

    if (isEnabled()) {
        emit sigStatusMessage(QStringLiteral("请先进行机械臂下电!"));
        return;
    }

    m_drillTimer->stop();
    m_trajAbort.store(true);

    QThread* t = QThread::create([=]() {
        int ret = m_rpcClient->logout();
        ret = m_rpcClient->disconnect();
        const bool ok = (ret == 0);

        QMetaObject::invokeMethod(this, [=]() {
            m_connected.store(false);
            emit _stopFeedbackTimer();
            if (ok) {
                emit sigStatusMessage(QStringLiteral("Aubo C5 断开成功"));
                emit sigConnected(m_connected.load());
            }
            else {
                emit sigError(QStringLiteral("Aubo C5 断开时出现错误，已强制断连"));
                return;
            }
            }, Qt::QueuedConnection);
        });
    connect(t, &QThread::finished, t, &QThread::deleteLater);
    t->start();
}

void AUBOC5::enableRobot(bool enable)
{
    if (!isConnected()) {
        emit sigStatusMessage(QStringLiteral("Aubo C5 未连接"));
        return;
    }

    if (enable) {

        double mass = 2.0;
        std::vector<double> cog(0.0, 0.0);
        std::vector<double> aom(0.0, 0.0);
        std::vector<double> inertia(0.0, 0.0);
        m_robot->getRobotConfig()->setPayload(mass, cog, aom, inertia);

        auto robot_mode = m_robot->getRobotState()->getRobotModeType();

        if (robot_mode != RobotModeType::Idle && robot_mode != RobotModeType::Running) {
            if (m_robot->getRobotManage()->poweron() != AUBO_OK) {
                emit sigError(QStringLiteral("Aubo C5上电失败!"));
                m_rpcClient->logout();
                m_rpcClient->disconnect();
                m_connected.store(false);
                emit sigConnected(m_connected.load());
                return;
            }

            if (!waitForRobotMode(m_robot, RobotModeType::Idle, kPowerOnTimeoutMs)) {
                emit sigError(QStringLiteral("Aubo C5等待上电超时!"));
                m_rpcClient->logout();
                m_rpcClient->disconnect();
                m_connected.store(false);
                emit sigConnected(m_connected.load());
                return;
            }
            robot_mode = RobotModeType::Idle;

            if (robot_mode == RobotModeType::Idle) {
                if (m_robot->getRobotManage()->startup() != AUBO_OK) {
                    emit sigError(QStringLiteral("Aubo C5松刹车请求失败!"));
                    m_rpcClient->logout();
                    m_rpcClient->disconnect();
                    m_connected.store(false);
                    emit sigConnected(m_connected.load());
                    return;
                }

                if (!waitForRobotMode(m_robot, RobotModeType::Running, kStartupTimeoutMs)) {
                    emit sigError(QStringLiteral("Aubo C5等待松刹车超时!"));
                    m_rpcClient->logout();
                    m_rpcClient->disconnect();
                    m_connected.store(false);
                    emit sigConnected(m_connected.load());
                    return;
                }
            }
        }
        m_enabled.store(true);

        emit sigEnabled(m_enabled.load());

        emit sigStatusMessage(QStringLiteral("Aubo C5上电成功!"));
    }
    else {
        if (m_robot->getRobotManage()->poweroff() != AUBO_OK) {
            emit sigError(QStringLiteral("Aubo C5断电失败!"));
            return;
        }

        if (!waitForRobotMode(m_robot, RobotModeType::PowerOff, kStartupTimeoutMs)) {
            emit sigError(QStringLiteral("Aubo C5等待断电超时!"));
            return;
        }

        m_enabled.store(false);

        emit sigEnabled(m_enabled.load());

        emit sigStatusMessage(QStringLiteral("Aubo C5断电成功!"));
    }
}

void AUBOC5::clearError()
{

}

void AUBOC5::setSpeedFactor(int percent)
{
    if (percent < 1)   percent = 1;
    if (percent > 100) percent = 100;
    m_globalSpeed = static_cast<double>(percent);
    emit sigStatusMessage(QStringLiteral("Aubo C5 速度已设置为 %1%").arg(percent));
}

void AUBOC5::resetPlace()
{
	count_grasp = 0;
	emit sigStatusMessage(QStringLiteral("Aubo C5 抓取计数已重置"));
}

/*---------------------运动---------------------*/
void AUBOC5::moveJoint(const std::vector<double>& joint, const QString& tag)
{
    if (!m_connected || joint.size() < 6) {
        emit sigMoveFinished(false, tag);
        return;
    }

    m_robot->getMotionControl()->setSpeedFraction(m_globalSpeed / 100.0);

    std::vector<double> jointCopy(6);
    for(int i = 0;i < 6; ++i) {
        jointCopy[i] = joint[i] * (M_PI / 180.0); // deg -> rad
	}

    std::thread([=]() {

        int ret = m_robot->getMotionControl()->moveJoint(
            jointCopy, m_globalSpeed * (M_PI / 180), m_globalSpeed * (M_PI / 180) * 0.5, 0, 0);

        if (ret != 0) {
            QMetaObject::invokeMethod(this, [=]() {
                emit sigMoveFinished(false, tag);
                emit sigError(QStringLiteral("AuboC5 关节运动指令发送失败 [%1]").arg(tag));
                }, Qt::QueuedConnection);
            return;
        }

        waitArriveJointAndNotify(joint, tag);
        }).detach();
}

void AUBOC5::movePose(const std::vector<double>& pose, const QString& tag)
{
    if (!m_connected || pose.size() < 6) {
        emit sigMoveFinished(false, tag);
        return;
    }

    m_robot->getMotionControl()->setSpeedFraction(m_globalSpeed / 100.0);

    std::vector<double> tgtPose;
	tgtPose.resize(6);
    for (int i = 0; i < 3; ++i) {
        tgtPose[i] = pose[i] * 0.001; // mm -> 
        tgtPose[i + 3] = pose[i + 3] * M_PI / 180.0;
    }

    std::thread([=]() {
        int ret = m_robot->getMotionControl()->moveLine(tgtPose, 1, 0.25, 0.025, 0);
        if (ret != 0) {
            QMetaObject::invokeMethod(this, [=]() {
                emit sigMoveFinished(false, tag);
                emit sigError(QStringLiteral("Aubo C5 笛卡尔运动指令发送失败 [%1]").arg(tag));
                }, Qt::QueuedConnection);
            return;
        }

        waitArriveAndNotify(pose, tag);

        if (tag == "calib") emit triggerCam();
        }).detach();
}

/*---------------------抓取相关流程---------------------*/
void AUBOC5::startGraspSequence(const std::vector<std::vector<double>>& path, const Eigen::Affine3f& tcp) {
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

    m_tcp = tcp;

    m_path.clear();
    m_path = densifyJoints(path);

    // 1 执行轨迹
    std::vector<double> wait_vec = toDouble(place_config.WaitJoint);
    moveJoint(wait_vec, "start");
}

void AUBOC5::loadPlaceConfig(const std::string& path) {

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

int AUBOC5::runTrajectoryPoint(const std::vector<std::vector<double>>& path)
{
    if (!m_connected || !m_enabled || isStop) {
        emit sigError(QStringLiteral("Aubo C5 未就绪，无法执行轨迹!"));
        QMetaObject::invokeMethod(this, [this]() {
            emit sigMoveFinished(false, QStringLiteral("traj"));
        }, Qt::QueuedConnection);
        return -1;
    }
    if (path.size() < 2) {
        emit sigError(QStringLiteral("Aubo C5 轨迹路点数量不足!"));
        QMetaObject::invokeMethod(this, [this]() {
            emit sigMoveFinished(false, QStringLiteral("traj"));
        }, Qt::QueuedConnection);
        return -1;
    }
    if (m_trajRunning.exchange(true)) {
        emit sigError(QStringLiteral("Aubo C5 轨迹正在运行!"));
        return -1;
    }

    m_trajAbort.store(false);
    const auto pathCopy = path;

    emit sigStatusMessage(QStringLiteral("Aubo C5开始执行轨迹运行!"));

    QThread* t = QThread::create([this, pathCopy]() {
        const int ret = executeTrajectory(pathCopy);
        QMetaObject::invokeMethod(this, [this, ret]() {
            m_trajRunning.store(false);
            if (ret == 0) {
                emit sigStatusMessage(QStringLiteral("Aubo C5轨迹运动结束!"));
                emit sigMoveFinished(true, QStringLiteral("traj"));
            }
            else if (m_trajAbort.load() || isStop) {
                emit sigStatusMessage(QStringLiteral("Aubo C5轨迹运动已中断"));
                emit sigMoveFinished(false, QStringLiteral("traj"));
            }
            else {
                emit sigError(QStringLiteral("Aubo C5轨迹运动失败!"));
                emit sigMoveFinished(false, QStringLiteral("traj"));
            }
        }, Qt::QueuedConnection);
    });
    connect(t, &QThread::finished, t, &QThread::deleteLater);
    t->start();
    return 0;
}

int AUBOC5::executeTrajectory(const std::vector<std::vector<double>>& path)
{
    auto notifyStatus = [this](const QString& msg) {
        QMetaObject::invokeMethod(this, [this, msg]() {
            emit sigStatusMessage(msg);
        }, Qt::QueuedConnection);
    };
    auto notifyError = [this](const QString& msg) {
        QMetaObject::invokeMethod(this, [this, msg]() {
            emit sigError(msg);
        }, Qt::QueuedConnection);
    };

    auto runtime = m_rpcClient->getRuntimeMachine();
    task_id.store(-1);

    auto cleanup = [this, runtime]() {
        if (runtime) {
            runtime->stop();
            const int id = task_id.exchange(-1);
            if (id >= 0) {
                runtime->deleteTask(id);
            }
        }
        if (m_robot) {
            m_robot->getMotionControl()->pathBufferFree("rec");
        }
    };

    if (m_trajAbort.load() || isStop) {
        return -1;
    }

    m_robot->getMotionControl()->setSpeedFraction(m_globalSpeed / 100.0);
    runtime->start();
    if (!waitForRuntimeRunning(runtime)) {
        notifyError(QStringLiteral("Aubo C5规划器启动超时!"));
        cleanup();
        return -1;
    }

    auto cur_plan_context = runtime->getPlanContext();
    const int new_task_id = runtime->newTask();
    task_id.store(new_task_id);
    runtime->setPlanContext(
        new_task_id, std::get<1>(cur_plan_context), std::get<2>(cur_plan_context));

    int ret = m_robot->getMotionControl()->moveJoint(
        path[0], 30 * (M_PI / 180), 30 * (M_PI / 180), 0., 0.);
    if (ret != 0) {
        notifyError(QStringLiteral("Aubo C5关节运动到轨迹首点指令发送失败!"));
        cleanup();
        return -1;
    }

    ret = waitArrival(m_robot, m_trajAbort);
    if (ret != 0) {
        if (!m_trajAbort.load() && !isStop) {
            notifyError(QStringLiteral("Aubo C5关节运动到轨迹文件中的第一个路点失败!"));
        }
        cleanup();
        return -1;
    }
    notifyStatus(QStringLiteral("Aubo C5关节运动到轨迹文件中的第一个路点成功!"));

    if (m_trajAbort.load() || isStop) {
        cleanup();
        return -1;
    }

    m_robot->getMotionControl()->pathBufferFree("rec");
    m_robot->getMotionControl()->pathBufferAlloc("rec", 2, static_cast<int>(path.size()));

    const size_t chunk = 10;
    for (size_t i = 0; i < path.size();) {
        if (m_trajAbort.load() || isStop) {
            cleanup();
            return -1;
        }
        const size_t n = std::min(chunk, path.size() - i);
        m_robot->getMotionControl()->pathBufferAppend(
            "rec",
            std::vector<std::vector<double>>(path.begin() + static_cast<std::ptrdiff_t>(i),
                path.begin() + static_cast<std::ptrdiff_t>(i + n)));
        i += n;
    }

    const double interval = 0.005;
    m_robot->getMotionControl()->pathBufferEval(
        "rec", { 1, 1, 1, 1, 1, 1 }, { 1, 1, 1, 1, 1, 1 }, interval);

    if (isBufferValid(m_robot, m_trajAbort) == -1) {
        if (!m_trajAbort.load() && !isStop) {
            notifyError(QStringLiteral("Aubo C5路径缓存无效，无法进行轨迹运动!"));
        }
        cleanup();
        return -1;
    }

    if (m_trajAbort.load() || isStop) {
        cleanup();
        return -1;
    }

    m_robot->getMotionControl()->movePathBuffer("rec");
    ret = waitArrival(m_robot, m_trajAbort);
    cleanup();
    return ret == 0 ? 0 : -1;
}

/*---------------------螺丝拧紧相关流程---------------------*/
void AUBOC5::startScrewSequence(
    const std::vector<std::pair<std::vector<double>, std::vector<double>>>& pose,
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

    std::vector<double> currentPose;
    if (!getEndPose(currentPose)) {
        emit sigError("Get robot end pose error!");
        return;
    }

    QTimer::singleShot(500, this, [this]() {
        movePose(m_screw_pose[currentIndex].first, "grasp_pass");
        });
}

void AUBOC5::onDrillStep()
{
    if (!m_connected || isStop) return;

    if (recvRet_jaka == "ok" || recvRet_jaka == "ng") {
        recvRet_jaka = "";
        m_drillTimer->stop();
        emit sigMoveFinished(true, "1_B");
        return;
    }
    if (downSpeed <= 0.0) downSpeed = 1.0;
    std::vector<double> off_vec;
    calcOffMatrix(m_tcp, -downSpeed, m_rotType, off_vec);

    movePose(off_vec, "1_D");
}

void AUBOC5::robotStop()
{
    isStop = true;
    m_trajAbort.store(true);
    m_drillTimer->stop();

    // 槽函数中断：置位后停规划器，工作线程从 waitArrival 退出并 deleteTask
    if (m_connected && m_rpcClient && m_trajRunning.load()) {
        QThread* t = QThread::create([this]() {
            if (m_rpcClient) {
                m_rpcClient->getRuntimeMachine()->stop();
            }
        });
        connect(t, &QThread::finished, t, &QThread::deleteLater);
        t->start();
    }
}

void AUBOC5::robotStart()
{
    isStop = false;
    if (!m_trajRunning.load()) {
        m_trajAbort.store(false);
    }
    m_grasp_pose.clear();
    m_screw_pose.clear();
    currentIndex = 0;

    QMutexLocker locker(&m_mutex_recv);
    recvRet_jaka = "";
}

void AUBOC5::getScrewStatus(const std::string& status)
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
void AUBOC5::moveNextPose(bool ok, const QString& tag)
{
    if (!ok) return;
    if (isStop) return;

    if (tag == "start") {
        if (m_path.size() < 10) return;
        runTrajectoryPoint(m_path);
    }
    else if (tag == "traj") {
        m_path.clear();
        emit sigSimulation(place_config.wrlModelPath, m_tcp, (count_grasp / (place_config.array.layerX * place_config.array.layerY)) * place_config.array.obj_height);
    }
    else if (tag == "tgt_grasp") {
        QTimer::singleShot(100, this, [=]() { emit sigGraspMachine(true, "grasp_pass"); });
    }
    else if (tag == "grasp_pass") {
        std::vector<double> pass_vec = toDouble(place_config.PassJoint);
        assert(pass_vec.size() == 6);
        QTimer::singleShot(200, this, [=]() { moveJoint(pass_vec, "pass"); });
    }
    else if (tag == "pass") {
        if (!place_config.useArray) {
            std::vector<double> place_vec = toDouble(place_config.PlaceJoint);
            assert(place_vec.size() == 6);
            moveJoint(place_vec, "grasp_place");
        }
        else {
            std::vector<double> place_pass_vec = toDouble(place_config.PlacePassJoint);
            assert(place_pass_vec.size() == 6);
            moveJoint(place_pass_vec, "grasp_pass_place");
        }
    }
    else if (tag == "grasp_pass_place") {
        std::vector<double> place_vec;
        getPlaceJoint(toDouble(place_config.PlaceJoint), place_config.array, count_grasp, place_config.XFirst, place_vec);
        moveJoint(place_vec, "grasp_place");
    }
    else if (tag == "grasp_place") {
        sigGraspMachine(false, "grasp_wait");
        count_grasp++;
    }
    else if (tag == "grasp_wait") {
        std::vector<double> wait_vec = toDouble(place_config.WaitJoint);
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
void AUBOC5::startFeedbackTimer() { m_feedbackTimer->start(); }

void AUBOC5::stopFeedbackTimer() { m_feedbackTimer->stop(); }

bool AUBOC5::getEndPose(std::vector<double>& pose)
{
    std::lock_guard<std::mutex> lk(m_snapMutex);
    if (!m_snap.valid) return false;
    pose = m_snap.endVec;
    return true;
}

bool AUBOC5::getCurrentJoint(std::vector<double>& j)
{
    std::lock_guard<std::mutex> lk(m_snapMutex);
    if (!m_snap.valid) return false;
    j = m_snap.joint;
    return true;
}

void AUBOC5::onFeedbackTimerTimeout()
{
    if (!m_connected) return;

    updateSnapshot();

    std::vector<double> joint, pose;
    {
        std::lock_guard<std::mutex> lk(m_snapMutex);
        if (!m_snap.valid) return;
        joint = m_snap.joint;
        pose = m_snap.endVec;
    }
    emit sigRobotStatus(joint, pose);
}

void AUBOC5::updateSnapshot()
{
    std::vector<double> pose, joint;
    if (!readCurrentPose(pose) || !readCurrentJoint(joint)) return;

    FeedbackSnapshot snap;
    snap.valid = true;
    snap.endVec = pose;
    snap.joint = joint;

    std::lock_guard<std::mutex> lk(m_snapMutex);
    m_snap = std::move(snap);
}

bool AUBOC5::readCurrentPose(std::vector<double>& pose)
{
    std::vector<double> actual_flange_pose;
    actual_flange_pose = m_robot->getRobotState()->getToolPose();

	if (actual_flange_pose.size() != 6) return false;

    pose.resize(6);

    for(int i = 0; i < 3; ++i) {
        pose[i] = actual_flange_pose[i] * 1000.0; // m -> mm
		pose[i + 3] = actual_flange_pose[i + 3] * 180.0 / M_PI; // rad -> deg
	}
    return true;
}

bool AUBOC5::readCurrentJoint(std::vector<double>& joint)
{
    std::vector<double> joint_positions;
    joint_positions = m_robot->getRobotState()->getJointPositions();

    if (joint_positions.size() != 6) return false;

    joint.resize(6);
    for (int i = 0; i < 6; ++i) {
        joint[i] = joint_positions[i] * 180.0f / M_PI;
    }
    return true;
}

void AUBOC5::sigUpdatePose() {
    std::vector<double> curPose;
    if (readCurrentPose(curPose)) emit sigPose(curPose);
    return;
}

/*---------------------笛卡尔位姿到位检测---------------------*/
void AUBOC5::waitArriveAndNotify(const std::vector<double>& targetPose,
    const QString& tag,
    int timeoutMs)
{
    const auto deadline = std::chrono::steady_clock::now()
        + std::chrono::milliseconds(timeoutMs);

    auto near_ = [](double a, double b, double tol = 0.1f) {
        return std::abs(a - b) < tol;
        };

    while (m_connected) {
        if (std::chrono::steady_clock::now() > deadline) {
            QMetaObject::invokeMethod(this, [=]() {
                emit sigMoveFinished(false, tag);
                emit sigError(QStringLiteral("Aubo C5 笛卡尔运动超时 [%1]").arg(tag));
                }, Qt::QueuedConnection);
            return;
        }

        std::vector<double> cur;
        {
            std::lock_guard<std::mutex> lk(m_snapMutex);
            if (m_snap.valid) cur = m_snap.endVec;
        }

        if (cur.size() == 6) {
            const bool arrived = near_(cur[0], targetPose[0])
                && near_(cur[1], targetPose[1])
                && near_(cur[2], targetPose[2])
                && near_(cur[3], targetPose[3])
                && near_(cur[4], targetPose[4])
                && near_(cur[5], targetPose[5]);
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
void AUBOC5::waitArriveJointAndNotify(const std::vector<double>& targetJoint,
    const QString& tag,
    int timeoutMs)
{
    const auto deadline = std::chrono::steady_clock::now()
        + std::chrono::milliseconds(timeoutMs);

    auto near_ = [](double a, double b, double tol = 0.1f) {
        return std::fabs(a - b) < tol;
        };

    while (m_connected) {
        if (std::chrono::steady_clock::now() > deadline) {
            QMetaObject::invokeMethod(this, [=]() {
                emit sigMoveFinished(false, tag);
                emit sigError(QStringLiteral("Aubo C5 关节运动超时 [%1]").arg(tag));
                }, Qt::QueuedConnection);
            return;
        }

        std::vector<double> cur;
        {
            std::lock_guard<std::mutex> lk(m_snapMutex);
            if (m_snap.valid) cur = m_snap.joint;
        }

        if (cur.size() == 6) {
            bool arrived = true;
            for (int i = 0; i < 6; ++i) {
                if (!near_(cur[i], targetJoint[i])) { arrived = false; break; }
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
void AUBOC5::calcOffMatrix(const Eigen::Affine3f tcp, double offInter, std::string rotType, std::vector<double>& off_vec) {
    MathUtils mu;
    std::vector<double> endVec;
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

std::vector<double> AUBOC5::toDouble(const utils::Joint& j) {
    std::vector<double> pass_vec{ j.j1, j.j2, j.j3, j.j4, j.j5, j.j6 };
    return pass_vec;
}

inline std::vector<std::vector<double>> AUBOC5::densifyJoints(const std::vector<std::vector<double>>& joints)
{
    const double dt = 0.005;                          // s
    const double maxVel = 20.0 * M_PI / 180.0;        // rad/s（低于 180 deg/s 硬限）
    const double maxAcc = 1.0;       // rad/s^2
    const double hardVel = 1.0;      // rad/s
    const double minDisp = 1e-6;
    std::vector<std::vector<double>> out;
    if (joints.empty())
    {
        return out;
    }
    out.push_back(joints.front());
    auto maxAbsDiff = [](const std::vector<double>& a, const std::vector<double>& b) {
        double m = 0.0;
        const std::size_t n = (((a.size()) < (b.size())) ? (a.size()) : (b.size()));
        for (std::size_t d = 0; d < n; ++d)
        {
            m = std::max(m, static_cast<double>(std::fabs(b[d] - a[d])));
        }
        return m;
        };
    for (std::size_t i = 0; i + 1 < joints.size(); ++i)
    {
        const std::vector<double>& q0 = joints[i];
        const std::vector<double>& q1 = joints[i + 1];
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
            const double u = (((1.0) < (std::max(0.0, pos(k * dt) / s))) ? (1.0) : (std::max(0.0, pos(k * dt) / s)));
            std::vector<double> q(q0.size());
            for (std::size_t d = 0; d < q0.size(); ++d)
            {
                const double q1d = d < q1.size() ? q1[d] : q0[d];
                q[d] = static_cast<double>(q0[d] + u * (q1d - q0[d]));
            }
            out.push_back(q);
        }
    }
    return out;
}

void AUBOC5::getPlaceJoint(const std::vector<double>& cur_place_joint, const utils::ArrayConfig& cfg,
    int n, bool x_first, std::vector<double>& new_place_joint)
{
    if (cur_place_joint.size() != 6 || cfg.layerX <= 0 || cfg.layerY <= 0)
    {
        return;
    }
    auto algorithm = m_robot->getRobotAlgorithm();
    auto reference_q = m_robot->getRobotState()->getJointPositions();
    for(int i = 0; i < 6; ++i) {
		reference_q[i] = cur_place_joint[i] * M_PI / 180.0; // deg -> rad
	}
    auto reference_config_result = algorithm->getRobotConfiguration(reference_q);
    auto reference_config = std::get<0>(reference_config_result);
    auto reference_config_ret = std::get<1>(reference_config_result);

    if (reference_config_ret != 0) {
        emit sigError(QStringLiteral("获取参考点构型失败，返回值[%1]").arg(reference_config_ret));
        return;
    }

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

    std::vector<double> current_tcp_pose;
    algorithm->forwardKinematics1(reference_q, current_tcp_pose);

    current_tcp_pose[0] += static_cast<double>(ix) * cfg.obj_length * 0.001;
    current_tcp_pose[1] += static_cast<double>(iy) * cfg.obj_width * 0.001;
    current_tcp_pose[2] += static_cast<double>(iz) * cfg.obj_height * 0.001;

    auto ik_result = algorithm->inverseKinematics2(reference_q, current_tcp_pose);
    auto q = std::get<0>(ik_result);
    auto ik_ret = std::get<1>(ik_result);
    if (ik_ret != 0 || q.size() != reference_q.size()) {
        emit sigError(QStringLiteral("逆解失败，返回值[%1]").arg(ik_ret));
        return;
    }

	new_place_joint.resize(6);
    for (size_t i = 0; i < q.size(); i++) {
		new_place_joint[i] = q.at(i); // rad -> deg
    }
    return;
}
