
#ifndef AUBOC5_H
#define AUBOC5_H

#include "device_robot/IRobot.h"
#include "aubo_sdk/rpc.h"
#ifdef WIN32
#include <windows.h>
#endif
#include "utils/utils.h"
#include <QTimer>
#include <QMutex>
#include <atomic>
#include <mutex>
#include <vector>
#include <string>
#include <utility>

using namespace arcs::common_interface;
using namespace arcs::aubo_sdk;

class AUBOC5 : public IRobot
{
    Q_OBJECT

public:
    explicit AUBOC5(QObject* parent = nullptr);
    ~AUBOC5() override;

    // ── IRobot 查询 ─────
    bool isConnected() const override;
    bool isEnabled()   const override;

    void calcOffMatrix(const Eigen::Affine3f tcp, double offInter, std::string rotType, std::vector<double>& off_vec);

    std::vector<double> toDouble(const utils::Joint& j);

    // 异步启动 pathBuffer 轨迹；完成后 emit sigMoveFinished(ok, "traj")
    int runTrajectoryPoint(const std::vector<std::vector<double>>& path);

    void getPlaceJoint(const std::vector<double>& cur_place_joint, const utils::ArrayConfig& cfg,
        int n, bool x_first, std::vector<double>& new_place_joint);

    inline std::vector<std::vector<double>> densifyJoints(const std::vector<std::vector<double>>& joints);

public slots:
    // ── IRobot 连接管理 ──────────────────────────────────────────────────────
    void connectRobot(const RobotPara& para) override;
    void disconnectRobot()                      override;
    void enableRobot(bool enable)           override;
    void clearError()                      override;

    void resetPlace()   override;
    // ── IRobot 运动指令（异步，到位后 emit sigMoveFinished(ok, tag)）─────────
    void moveJoint(const std::vector<double>& joint,
        const QString& tag = QString()) override;

    void movePose(const std::vector<double>& pose,
        const QString& tag = QString()) override;

    void sigUpdatePose() override;

    // ── IRobot 参数设置 ──────────────────────────────────────────────────────
    void setSpeedFactor(int percent) override;   // 1~100，对应 globalSpeed

    // ── IRobot 数据查询（同步读快照）────────────────────────────────────────
    bool getEndPose(std::vector<double>& pose) override;
    bool getCurrentJoint(std::vector<double>& j)    override;

    void startGraspSequence(const std::vector<std::vector<double>>& path, const Eigen::Affine3f& tcp) override;

    // ── 拧螺丝序列（控制逻辑对齐 DobotCR5）──────────────────────────────────
    void startScrewSequence(const std::vector<std::pair<std::vector<double>, std::vector<double>>>& pose,
        const Eigen::Affine3f& tcp, const std::string rotType) override;

    void onDrillStep() override;

    void robotStop() override;
    void robotStart() override;

    void setDownSpeed(const double speed) override { downSpeed = speed; }

    void getScrewStatus(const std::string& status) override;

    void moveNextPose(bool ok, const QString& tag);

    void loadPlaceConfig(const std::string& path) override;

private slots:
    // ── 反馈定时器（100ms，主线程）───
    void onFeedbackTimerTimeout();

    // ── 定时器跨线程启停 ───
    void startFeedbackTimer();
    void stopFeedbackTimer();

signals:
    // 内部跨线程定时器控制（禁止外部 emit）
    void _startFeedbackTimer();
    void _stopFeedbackTimer();

private:

    // ── 等待到位并通知（在 std::thread 中阻塞执行）──────────────────────────
    void waitArriveAndNotify(const std::vector<double>& targetPose,
        const QString& tag,
        int timeoutMs = 20000);
    void waitArriveJointAndNotify(const std::vector<double>& targetJoint,
        const QString& tag,
        int timeoutMs = 20000);

    // 在工作线程中执行 pathBuffer 轨迹（由 runTrajectoryPoint 启动）
    int executeTrajectory(const std::vector<std::vector<double>>& path);

    // ── 螺丝拧紧过程变量（对齐 DobotCR5）────────────────────────────────────
    QMutex m_mutex_recv;
    std::string recvRet_jaka = "";
    Eigen::Affine3f m_tcp;
    double m_offInter = 0.0;
    std::string m_rotType;
    std::vector<std::pair<std::vector<double>, std::vector<double>>> m_grasp_pose;
    std::vector<std::pair<std::vector<double>, std::vector<double>>> m_screw_pose;
    int currentIndex = 0;
    bool isStop = false;
    std::vector<double> capJoint;
    double downSpeed = 0.0;

    bool loadPlaceStatus = false;
    utils::PlaceConfig place_config;
    std::vector<std::vector<double>> m_path;
    int count_grasp = 0;
    std::atomic<int> task_id{ -1 };
    std::atomic<bool> m_trajRunning{ false };
    std::atomic<bool> m_trajAbort{ false };

    // ── 快照刷新（仅在 onFeedbackTimerTimeout 中调用）──────────────────────
    void updateSnapshot();

    // ── SDK 直接读取（供快照刷新使用）───────────────────────────────────────
    bool readCurrentPose(std::vector<double>& pose);    // mm / deg
    bool readCurrentJoint(std::vector<double>& joint);   // deg

    // ── Feedback 数据快照（定时器刷新，互斥锁保护）──────────────────────────
    struct FeedbackSnapshot {
        std::vector<double> endVec{ 0,0,0,0,0,0 };   // mm / deg
        std::vector<double> joint{ 0,0,0,0,0,0 };   // deg
        bool valid = false;
    };
    mutable std::mutex m_snapMutex;
    FeedbackSnapshot   m_snap;

    // ── JAKA SDK 对象 ────────────────────────────────────────────────────────
    RobotInterfacePtr        m_robot;
	RpcClientPtr			 m_rpcClient;

    // ── 状态标志 ─────────────────────────────────────────────────────────────
    std::atomic<bool>  m_connected{ false };
    std::atomic<bool>  m_enabled{ false };

    // ── 运动参数 ─────────────────────────────────────────────────────────────
    double             m_globalSpeed = 5.0;   // JAKA 速度百分比（1~100）

    // ── 定时器（只在主线程访问）─────────────────────────────────────────────
    QTimer* m_feedbackTimer{ nullptr };
    QTimer* m_drillTimer{ nullptr };
};

#endif // AUBOC5
