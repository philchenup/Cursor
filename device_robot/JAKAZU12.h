
#ifndef JAKAZU12_H
#define JAKAZU12_H

#include "device_robot/IRobot.h"
#include "JAKAZuRobot.h"
#include "utils/utils.h"
#include <QTimer>
#include <QMutex>
#include <atomic>
#include <mutex>
#include <vector>
#include <string>
#include <utility>

// ─────────────────────────────────────────────────────────────────────────────
// JAKAZU12
// ─────────────────────────────────────────────────────────────────────────────
class JAKAZU12 : public IRobot
{
    Q_OBJECT

public:
    explicit JAKAZU12(QObject* parent = nullptr);
    ~JAKAZU12() override;

    // ── IRobot 查询 ──────────────────────────────────────────────────────────
    bool isConnected() const override;
    bool isEnabled()   const override;

    void calcOffMatrix(const Eigen::Affine3f tcp, double offInter, std::string rotType, std::vector<float>& off_vec);

    bool quat2eulerJaka(const std::vector<float>& quat_in, std::vector<float>& euler_out);

    std::vector<float> toFloat(const utils::Joint& j);

    inline std::vector<std::vector<float>> densifyJoints( const std::vector<std::vector<float>>& joints);

    // 异步：在独立线程中 servo_j，结束后 emit sigMoveFinished
    errno_t runTrajectoryPoint(const std::vector<std::vector<float>>& m_path);

    errno_t getPlaceJoint(const std::vector<float>& cur_place_joint, const utils::ArrayConfig& cfg,
        int n, bool x_first, std::vector<float>& new_place_joint);

    std::vector<float> toVector(const JointValue& jv);

    JointValue toJoint(const std::vector<float>& joint_vec);

    void quat2Joint(const std::vector<float>& quat_in, std::vector<float>& new_joint) override;

public slots:
    // ── IRobot 连接管理 ──────────────────────────────────────────────────────
    void connectRobot(const RobotPara& para) override;
    void disconnectRobot()                      override;
    void enableRobot(bool enable)           override;
    void clearError()                      override;

    // ── IRobot 运动指令（异步，到位后 emit sigMoveFinished(ok, tag)）─────────
    void moveJoint(const std::vector<float>& joint,
        const QString& tag = QString()) override;

    void movePose(const std::vector<float>& pose,
        const QString& tag = QString()) override;

    // ── IRobot 参数设置 ──────────────────────────────────────────────────────
    void setSpeedFactor(int percent) override;   // 1~100，对应 globalSpeed

    // ── IRobot 数据查询（同步读快照）────────────────────────────────────────
    bool getEndPose(std::vector<float>& pose) override;
    bool getCurrentJoint(std::vector<float>& j)    override;

    void startGraspSequence(const std::vector<std::vector<float>>& path, const std::vector<float>& tgt_joint) override;

    
    
    // ── 拧螺丝序列（控制逻辑对齐 DobotCR5）──────────────────────────────────
    void startScrewSequence(const std::vector<std::pair<std::vector<float>, std::vector<float>>>& pose,
        const Eigen::Affine3f& tcp, const std::string rotType) override;

    void onDrillStep() override;

    void robotStop() override;
    void robotStart() override;

    void setDownSpeed(const double speed) { downSpeed = speed; }

    void getScrewStatus(const std::string& status) override;

    void moveNextPose(bool ok, const QString& tag);

    void loadPlaceConfig(const std::string& path) override;

private slots:
    // ── 反馈定时器（100ms，主线程）─────────────────────────────────────────
    void onFeedbackTimerTimeout();

    // ── 定时器跨线程启停 ────────────────────────────────────────────────────
    void startFeedbackTimer();
    void stopFeedbackTimer();

signals:
    // 内部跨线程定时器控制（禁止外部 emit）
    void _startFeedbackTimer();
    void _stopFeedbackTimer();

private:
    // ── 连接任务（在 QThread 中执行）────────────────────────────────────────
    void connectTask(const std::string& ip);

    // ── 等待到位并通知（在 std::thread 中阻塞执行）──────────────────────────
    void waitArriveAndNotify(const std::vector<float>& targetPose,
        const QString& tag,
        int timeoutMs = 30000);
    void waitArriveJointAndNotify(const std::vector<float>& targetJoint,
        const QString& tag,
        int timeoutMs = 30000);

    // ── 螺丝拧紧过程变量（对齐 DobotCR5）────────────────────────────────────
    QMutex m_mutex_recv;
    std::string recvRet_jaka = "";
    Eigen::Affine3f m_tcp;
    float m_offInter = 0.0;
    std::string m_rotType;
    std::vector<std::pair<std::vector<float>, std::vector<float>>> m_grasp_pose;
    std::vector<std::pair<std::vector<float>, std::vector<float>>> m_screw_pose;
    int currentIndex = 0;
    bool isStop = false;
    std::vector<float> capJoint;
    double downSpeed = 0.0;

    bool loadPlaceStatus = false;
    utils::PlaceConfig place_config;
    std::vector<float> last_place_vec;
    std::vector<std::vector<float>> m_path;
    int count_grasp = 0;
    std::vector<float> m_tgt_joint;
    // ── 快照刷新（仅在 onFeedbackTimerTimeout 中调用）──────────────────────
    void updateSnapshot();

    // ── SDK 直接读取（供快照刷新使用）───────────────────────────────────────
    bool readCurrentPose(std::vector<float>& pose);    // mm / deg
    bool readCurrentJoint(std::vector<float>& joint);   // deg

    // ── Feedback 数据快照（定时器刷新，互斥锁保护）──────────────────────────
    struct FeedbackSnapshot {
        std::vector<float> endVec{ 0,0,0,0,0,0 };   // mm / deg
        std::vector<float> joint{ 0,0,0,0,0,0 };   // deg
        bool valid = false;
    };
    mutable std::mutex m_snapMutex;
    FeedbackSnapshot   m_snap;

    // ── JAKA SDK 对象 ────────────────────────────────────────────────────────
    JAKAZuRobot        m_robot;

    // ── 状态标志 ─────────────────────────────────────────────────────────────
    std::atomic<bool>  m_connected{ false };
    std::atomic<bool>  m_enabled{ false };

    // ── 运动参数 ─────────────────────────────────────────────────────────────
    double             m_globalSpeed = 5.0;   // JAKA 速度百分比（1~100）

    // ── 定时器（只在主线程访问）─────────────────────────────────────────────
    QTimer* m_feedbackTimer{ nullptr };
    QTimer* m_drillTimer{ nullptr };
};

#endif // JAKAZU12_H
