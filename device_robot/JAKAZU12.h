/**
 * @file   JAKAZU12.h
 * @brief  JAKA Zu12 机械臂驱动层 —— 实现 IRobot 硬件抽象接口
 *
 * 设计原则（与 DobotCRA10 / DobotCR5 / AuboC5 保持一致）
 * ──────────────────────────────────────────────────────────
 *  • 只继承 IRobot（IRobot 已继承 QObject，不可重复继承）
 *  • 只暴露 IRobot 定义的能力：连接 / 使能 / 运动 / 反馈 / 拧螺丝序列
 *  • 所有运动指令异步执行，到位后 emit sigMoveFinished(ok, tag)
 *  • 实时状态由 100ms QTimer 驱动 emit sigRobotStatus
 *  • 拧螺丝控制逻辑对齐 DobotCR5：startScrewSequence / onDrillStep /
 *    moveNextPose / robotStart / robotStop / getScrewStatus
 *
 * JAKA SDK 特殊说明
 * ──────────────────
 *  • JAKAZuRobot 所有接口均为同步阻塞调用
 *  • 位姿单位：mm / deg（与 IRobot 接口一致，无需转换）
 *  • 到位检测：读取定时器维护的 Feedback 快照（与 DobotCR5 一致）
 *  • 反馈定时器跨线程启停：私有信号 _startFeedbackTimer / _stopFeedbackTimer
 *
 * @version 2.1
 * @date    2026-08-07
 */

#ifndef JAKAZU12_H
#define JAKAZU12_H

#include "device_robot/IRobot.h"
#include "JAKAZU12/include/JAKAZuRobot.h"

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

    // ── 拧螺丝序列（控制逻辑对齐 DobotCR5）──────────────────────────────────
    void startScrewSequence(const std::vector<std::pair<std::vector<float>, std::vector<float>>>& pose,
        const Eigen::Affine3f& tcp, const QString rotType) override;
    void onDrillStep() override;

    void robotStop() override;
    void robotStart() override;

    void setDownSpeed(const double speed) { downSpeed = speed; }

    void getScrewStatus(const std::string& status) override;

    void moveNextPose(bool ok, const QString& tag);

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
        int timeoutMs = 15000);
    void waitArriveJointAndNotify(const std::vector<float>& targetJoint,
        const QString& tag,
        int timeoutMs = 15000);

    // ── 螺丝拧紧过程变量（对齐 DobotCR5）────────────────────────────────────
    QMutex m_mutex_recv;
    std::string recvRet_jaka = "";
    Eigen::Affine3f m_tcp;
    QString m_rotType;
    std::vector<std::pair<std::vector<float>, std::vector<float>>> m_pose;
    int currentIndex = 0;
    bool isStop = false;
    std::vector<float> capJoint;
    double downSpeed = 0.0;

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
    double             m_globalSpeed{ 30.0 };   // JAKA 速度百分比（1~100）

    // ── 定时器（只在主线程访问）─────────────────────────────────────────────
    QTimer* m_feedbackTimer{ nullptr };
    QTimer* m_drillTimer{ nullptr };
};

#endif // JAKAZU12_H
