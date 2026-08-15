#ifndef IK_WORKER_H
#define IK_WORKER_H

#include <QObject>
#include <QMutex>
#include <atomic>
#include <memory>
#include <vector>

#include "GlobalDefs.h"

Q_DECLARE_METATYPE(rl::math::Vector)
Q_DECLARE_METATYPE(std::vector<rl::math::Vector>)
Q_DECLARE_METATYPE(IKSolveParams)
Q_DECLARE_METATYPE(IKReturnHomeParams)
Q_DECLARE_METATYPE(DiscretePoint)
Q_DECLARE_METATYPE(IKGoToStartParams)
Q_DECLARE_METATYPE(std::vector<rl::math::Vector3>)

class IKWorker : public QObject
{
    Q_OBJECT
public:
    explicit IKWorker(QObject* parent = nullptr);
    ~IKWorker() override;

    /**
     * 注入外部 Kinematic（shared_ptr 共享所有权）。
     *
     * 适用模式 A：主线程与 Worker 共享同一个模型对象。
     * 求解期间主线程禁止读写该模型。
     */
    void setKinematic(const std::shared_ptr<rl::mdl::Dynamic>& mdl);

    void requestStop();
    bool isRunning() const { return m_running.load(); }

public slots:
    void doSolve(const IKSolveParams& params);

    void doReturnHome(const IKReturnHomeParams& params);

    void doGoToStart(const IKGoToStartParams& params);
signals:
    void started();
    void progress(int percent);
    void finished(const std::vector<rl::math::Vector>& jointTrajectory,
        const double& ratio, const DiscretePoint& start);
    void finished_return(const std::vector<rl::math::Vector>& jointTrajectory,
        const double& ratio);
    void finished_start(const std::vector<rl::math::Vector>& jointTrajectory,
        const std::vector<rl::math::Vector3>& toolPoints,
        const double& ratio);
    void failed(const QString& errorMessage); 
    void aborted();
private:
    rl::math::Transform pointToTransform(const DiscretePoint& pt);

private:
    std::shared_ptr<rl::mdl::Kinematic> m_kinematic;   // 共享所有权
    mutable QMutex m_kinMutex;
    std::atomic<bool> m_stopRequested{ false };
    std::atomic<bool> m_running{ false };
};

#endif