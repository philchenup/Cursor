#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTranslator>
#include <QTcpServer>
#include <QTcpSocket>
#include "ui_mainwindow.h"

#include "GlobalDef.h"

#include "base/customdock.h"
#include "base/customdialog.h"

#include "device_cam/ICamera_factory.h"
#include "device_robot/IRobot_factory.h"
#include "device_tool/IGripper_factory.h"

#include "device_comm/SocketComm.h"

class GripperFactory;
class ProcessWorker;
class ScrewComm;
class RobotFactory;
class SocketWorker;
class Thread;
class Viewer;
class MovableWrlManager;
class MovableWrl;

QT_BEGIN_NAMESPACE
namespace Ui
{
    class MainWindow;
}
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT
public:
    MainWindow(QWidget* parent = nullptr);
    ~MainWindow();

    void changeTheme(int);
    void changeLanguage(int);
    void saveScreenshot();

    template <class T>
    void createLeftDock(const QString& label)
    {
        ct::createDock<T>(this, label, ui->cloudview, ui->cloudtree, ui->console,
                          Qt::LeftDockWidgetArea, ui->PropertiesDock);
    }

    template <class T>
    void createRightDock(const QString& label)
    {
        ct::createDock<T>(this, label, ui->cloudview, ui->cloudtree, ui->console,
                          Qt::RightDockWidgetArea);
    }

    template <class T>
    void createDialog(const QString& label, const bool& center)
    {
        ct::createDialog<T>(this, label, ui->cloudview, ui->cloudtree, ui->console, center);
    }

public:
    void load(const QString& filename);

    rl::sg::Model* environmentModel(bool createIfMissing = true);

    void syncObstaclesToPlanner();

    void safeImwrite(const QString& savePath, const cv::Mat& Depth, const cv::Mat& Color);

    void view_connect(const QObject* sender, const QObject* receiver);

    void view_disconnect(const QObject* sender, const QObject* receiver);
public:

    static MainWindow* instance();

    std::vector<std::shared_ptr<rl::math::Vector3>> explorerGoals;

    std::vector<std::shared_ptr<rl::plan::WorkspaceSphereExplorer>> explorers;

    std::vector<std::shared_ptr<rl::math::Vector3>> explorerStarts;

    std::shared_ptr<rl::math::Vector> goal;

    std::shared_ptr<rl::kin::Kinematics> kin;

    std::shared_ptr<rl::kin::Kinematics> kin2;

    std::shared_ptr<rl::mdl::Dynamic> mdl;

    std::shared_ptr<rl::mdl::Dynamic> mdl2;

    std::shared_ptr<rl::plan::SimpleModel> model;

    std::shared_ptr<rl::plan::Model> model2;

    QMutex mutex;

    std::vector<std::shared_ptr<rl::plan::NearestNeighbors>> nearestNeighbors;

    std::shared_ptr<rl::plan::Optimizer> optimizer;

    std::shared_ptr<rl::plan::Planner> planner;

    std::shared_ptr<rl::math::Vector> q;

    std::shared_ptr<rl::plan::Sampler> sampler;

    std::shared_ptr<rl::plan::Sampler> sampler2;

    std::shared_ptr<rl::math::Vector> sigma;

    std::shared_ptr<rl::sg::Scene> scene;

    std::shared_ptr<rl::sg::so::Scene> scene2;

    rl::sg::Model* sceneModel;

    rl::sg::so::Model* sceneModel2;

    std::shared_ptr<rl::math::Vector> start;

    Thread* thread;

    std::shared_ptr<rl::plan::Verifier> verifier;

    std::shared_ptr<rl::plan::Verifier> verifier2;

    Viewer* viewer;

    MovableWrl* obstacle;

    SoVRMLTransform* createWorldFrame(float length = 0.25f);

    std::vector<SoVRMLTransform*> coor_node;

private:
    static MainWindow* singleton;

signals:
   
    void triggerVisionPro();

    void visionConfig(const QString&);

    void toolkitnConfig(const QString&);

    void screwConnect(const QString& ip, const QString& port);

    void screwClose();

    void screwStart();

    void screwReset();

    void screwBacklash();

private slots:
    void connect_init();

    void recOffMatrix(const std::vector<Eigen::Affine3f>& pre, 
        const std::vector<Eigen::Affine3f>& target, const Eigen::Affine3f& sceneTrans);

    void InitializeGripper();

    void InitializeCamera();

    void InitializeScrew();

    void InitializeRobot();

    void InitializeComm();

    void setConnected(bool connected);

    void GetCamData(const CamData&, const QString&);

    void on_loadToolBtn_clicked();

    void on_loadVisionConfBtn_clicked();

    void on_actionRunonce_triggered();

    void on_actionCycle_triggered();

    void on_loadAutoCalibBtn_clicked();

    void loadSceneWrl();

    void saveSceneWrl();

    void removeSceneWrl();

    void showSceneCloud(const ct::Cloud::Ptr& scene_cloud);

    void receiveGraspPose(const std::vector<std::pair<Eigen::Affine3f, Eigen::Affine3f>>& target_pose,
        const Eigen::Affine3f& tcp, const std::string& rotType);

    void startAutoCalib();
protected:
    void closeEvent(QCloseEvent* event) override;

    void moveEvent(QMoveEvent* event);
private:
    Ui::MainWindow* ui;
    QTranslator* translator;
    CameraFactory::CameraType camType;
    std::unique_ptr<ICamera> camera;

    std::unique_ptr<ProcessWorker> proworker;
    QThread* m_thread_prowork;

    bool camConnected = false;

    std::unique_ptr<ScrewComm> sc;
    QThread* m_thread_scwork;

    GripperFactory::GripperType gripperType;
    std::unique_ptr<IGripper> gripper;
    QThread* m_thread_gripperwork;
    bool gripperEnabled = false;
    bool gripperConnected = false;

    RobotFactory::RobotType robotType;
    std::unique_ptr<IRobot> robot;
    QThread* m_thread_rfwork;
    bool robotConnected = false;
    bool robotEnabled = false;

    bool screwRunning = false;
    bool screwConnected = false;

    QThread* m_thread_comm;
    SocketWorker* m_worker_comm = nullptr;
    bool m_comm_connected = false;
    rl::hal::Socket* m_udpSocket{ nullptr };
    rl::hal::Socket::Address         m_udpAddress;
    bool                             m_udpMode{ false };

    QString current_cloud_id;

    bool isRunOnce = false;
    bool isAutoCalib = false;

    int calibImageCount = 0;

    std::vector<std::vector<float>> calib_off;
    std::vector<float> current_endvec;

    int m_jointIndex = 0;   // joint ��¼���
    int m_poseIndex = 0;   // pose ��¼���
    bool m_isJointMode = true;

    QTcpServer* m_tcpServer = nullptr;   // TCP ����˶���
    QTcpSocket* m_tcpSocket = nullptr;   // ��ǰ���ӵĿͻ��� Socket
};
#endif // MAINWINDOW_H
