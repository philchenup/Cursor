#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTranslator>

#include "ui_mainwindow.h"

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
    void setConnected(bool connected);

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

protected:
    void moveEvent(QMoveEvent* event);

signals:
   
    void triggerVisionPro();

    void visionConfig(const QString&);

    void toolkitnConfig(const QString&);

    void screwConnect(const QString& ip, const QString& port);

    void screwClose();

    void screwStart();

    void screwReset();

    void screwBacklash();

    void triggerCalibPro(const std::vector<float>&, bool isCalib, const int);

private slots:

    void recOffMatrix(const std::vector<Eigen::Affine3f>& pre, 
        const std::vector<Eigen::Affine3f>& target, const Eigen::Affine3f& sceneTrans);

    void InitializeComm();

    void InitializeCamera();

    void InitializeRobot();

    void GetCamModel(const QVector<QString>&);
    void GetCamInfo(const QString&);

    void GetCamStatus(const bool&);
    void GetCamData(const CamData&, const QString&);

    void on_loadToolBtn_clicked();

    void on_loadVisionConfBtn_clicked();

    void on_runOnceBtn_clicked();

    void on_runCycleBtn_clicked();

    /*void on_screwConnectBtn_clicked();
    
    void on_screwStartBtn_clicked();*/

    void on_loadAutoCalibBtn_clicked();
protected:
    void closeEvent(QCloseEvent* event) override;

private:
    Ui::MainWindow* ui;
    QTranslator* translator;
    CameraFactory::CameraType camType;
    std::unique_ptr<ICamera> camera;

    std::unique_ptr<ProcessWorker> proworker;
    QThread* m_thread_prowork;

    bool camConnected = false;

    QThread* m_thread_comm;
    SocketWorker* m_worker_comm;
    bool m_comm_connected = false;
    rl::hal::Socket* m_udpSocket{ nullptr };
    rl::hal::Socket::Address         m_udpAddress;
    bool                             m_udpMode{ false };

    RobotFactory::RobotType robotType;
    std::unique_ptr<IRobot> robot;
    QThread* m_thread_rfwork;
    bool robotConnected = false;
    bool robotEnabled = false;

    bool screwRunning = false;
    bool screwConnected = false;

    QString current_cloud_id;

    bool isRunOnce = false;
    bool isAutoCalib = false;

    int calibImageCount = 0;

    std::vector<std::vector<float>> calib_off;
    std::vector<float> current_endvec;

    int m_jointIndex = 0;   // joint ��¼���
    int m_poseIndex = 0;   // pose ��¼���
    bool m_isJointMode = true;
};
#endif // MAINWINDOW_H
