#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTranslator>
#include <QMessageBox>
#include "ui_mainwindow.h"

#include "GlobalDefs.h"

#include "device_cam/ICamera_factory.h"
#include "device_laser/ILaser_factory.h"
#include "device_robot/IRobot_factory.h"

#include "base/customdock.h"
#include "base/customdialog.h"

#include "device_robot/kukacommunicator.h"

class ConfigurationDelegate;
class ConfigurationModel;
class OperationalDelegate;
class OperationalModel;
class ReadModel;
class CustomDialog;
class CustomDock;
class OccView;
class IKWorker;
class KukaCommunicator;
class ProcessWorker;

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
    static MainWindow* instance();

    explicit MainWindow(QWidget* parent = nullptr);
    ~MainWindow();

    void changeTheme(int);

    void changeLanguage(int);

    void changeRobotCoorBtn(bool);

    void safeImwrite(const QString& savePath, const cv::Mat& image);

   template <class T>
    void createLeftDock(const QString& label)
    {
        ct::createDock<T>(this, label, ui->cloudview, ui->cloudtree, ui->console,
            Qt::LeftDockWidgetArea, ui->pointCloudDock);
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

    void loadOcctModel(std::string);

    void connect_init();

    void showWeldTrajPt(const std::vector<DiscretePoint>& trajPt);

    void initSimulation();

signals:
    void triggerVisionPro();

    void sendTrajectory(const rl::math::Vector&);

private:
    static MainWindow* singleton;

    QVector<TopoDS_Shape> links;

    QVector<AIS_Shape*> displinks;

    bool isTheFirstDraw = true;

protected:
    void closeEvent(QCloseEvent* event) override;

private:
    /** Dual-write operation log: UI console + glog (by severity). */
    void logInfo(const QString& msg);
    void logWarning(const QString& msg);
    void logError(const QString& msg);

    Ui::MainWindow* ui;
    QTranslator* translator;

    // ���ӻ�����
    QMutex mapMutex;
    OccView* myOccView;
    
    int IsJointSpace = true;

    QTimer* flushSceneTimer;

    QTimer* flushTrajTimer;

    QTimer* flushHomeTimer;
public:
    QString engine;

    std::vector<std::shared_ptr<rl::plan::WorkspaceSphereExplorer>> explorers;

    std::shared_ptr<rl::math::Vector> goal;

    std::shared_ptr<rl::mdl::Dynamic> mdl;

    std::shared_ptr<rl::plan::SimpleModel> model;

    QMutex mutex;

    std::vector<std::shared_ptr<rl::plan::NearestNeighbors>> nearestNeighbors;

    std::shared_ptr<rl::plan::Optimizer> optimizer;

    std::shared_ptr<rl::plan::Planner> planner;

    std::shared_ptr<rl::math::Vector> q;

    std::shared_ptr<rl::plan::Sampler> sampler;

    std::shared_ptr<rl::math::Vector> sigma;

    std::shared_ptr<rl::sg::Scene> scene;

    rl::sg::Model* sceneModel; // �˶�ѧ

    rl::sg::Body* sceneBody;

    std::shared_ptr<rl::math::Vector> start;

    std::shared_ptr<rl::plan::Verifier> verifier;

    bool isConnectToRobot = false;

    std::vector<rl::sg::Shape*> sgShapes;

    rl::math::Transform tcp_transform;

    ConfigurationDelegate* configurationDelegates;
    OperationalDelegate* operationalDelegates;

    OperationalModel* operationalModel;
    ConfigurationModel* configurationModel;

private slots:
    void on_InitializeLaser_clicked();

    void on_InitializeCamera_clicked();

    void on_InitializeRobot_clicked();

    void occtUpdate();

    void load3DModel();
    void saveModel();
    void removeModel();

    void on_actionGoOrigin_triggered();

    void on_loadVisionConfigBtn_clicked();

    void GetCamData(const CamData&, const QString&);

    void actionStart_triggered();

    void flushWeldList(const TopoDS_Shape& selectShape, const std::vector<TopoDS_Edge>& weldEdges);
    void onWeldSelected(const QItemSelection& selected, const QItemSelection& deselected);

    void robotGoHome();
    
    void execSimulation(int&);

    void execReturnHome(int&);

    void Trajectory();
private:
    IKWorker* ikwork;
    QThread* m_thread_ikwork = nullptr;

    std::unique_ptr<ProcessWorker> proworker;
    QThread* m_thread_prowork = nullptr;

    RobotFactory::RobotType robotType;
    std::unique_ptr<IRobot> robot;
    QThread* m_thread_rworker = nullptr;

    std::unique_ptr<KukaCommunicator> m_comm;

    CameraFactory::CameraType camType;
    std::unique_ptr<ICamera> camera;
    int robot_heart_beat_num = 0;
    int robot_heart_beat_num_last = 0;
    int heart_beat_different_times = 0;
    QTimer* heartMonitorTimer;
    bool robotConnected = false;

    LaserFactory::LaserType laserType;
    std::unique_ptr<ILaser> laser;
    QThread* m_thread_laserwork = nullptr;
    int laserIndex = 0;
    
    void reset();

    void robotWidgetInit();

    void loadRobotWidget(const QString& filename);

    void loadWrlModel(const std::string& wrlName);

    bool camConnected = false;
    bool laserConnected = false;
    
    bool camCaptureRunning = false;

    bool isRunOnce = false;
    bool isRealRunning = false;
    bool isSimulation = false;
    int m_counter = 0;
    int m_counter_home = 0;
    std::vector<rl::math::Vector> m_jointTrajectory_home;
    std::vector<rl::math::Vector> wholeTrajectory;
    TopoDS_Shape m_selectShape;
    // std::vector<TopoDS_Edge> m_weldEdges;
    std::vector<EdgeCluster> sortedClusters;
    std::vector<DiscretePoint> mergedTraj;

    int calibNumCount = 0;
    QString current_cloud_id = "";
    AIS_Shape* loadShape;
    QMutex g_robotMutex;
    RobotData g_robotData;
    bool m_IsCartMove = false;
};
#endif // MAINWINDOW_H
