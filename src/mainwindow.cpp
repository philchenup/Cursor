#include "mainwindow.h"

#include "edit/color.h"
#include "edit/coordinate.h"
#include "edit/normals.h"
#include "edit/scale.h"
#include "edit/transformation.h"
#include "edit/filters.h"
#include "edit/segmentation.h"
#include "edit/cutting.h"
#include "edit/sampling.h"

#include "help/about.h"
#include "help/shortcutkey.h"

#include "tool/TcpCalib.h"
#include "tool/HandEyeCalib.h"
#include "tool/MakeTool.h"
#include "tool/VisionConfig.h"

#include "utils/processWorker.h"

#include "device_tool/screwComm.h"

#include <QDesktopWidget>
#include <QDir>
#include <QFile>
#include <QDateTime>
#include <QFileDialog>
#include <QTimer>
#include <QMessageBox>

#include <glog/logging.h>

void MainWindow::logInfo(const QString& msg)
{
    ui->console->print(ct::LOG_INFO, msg);
    LOG(INFO) << msg.toStdString();
}

void MainWindow::logWarning(const QString& msg)
{
    ui->console->print(ct::LOG_WARNING, msg);
    LOG(WARNING) << msg.toStdString();
}

void MainWindow::logError(const QString& msg)
{
    ui->console->print(ct::LOG_ERROR, msg);
    LOG(ERROR) << msg.toStdString();
}


MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent),
    ui(new Ui::MainWindow),
    translator(nullptr),
    m_thread_prowork(new QThread(this)),
    m_thread_comm(new QThread(this)),
    m_thread_rfwork(new QThread(this))
{
    ui->setupUi(this);

    setEnabled(false);
    ui->cloudview->setBackgroundColor({ 127,127,127 });
    // resize
    this->resize(1600, 1200);

    // init qrc
    Q_INIT_RESOURCE(res);

    // connect pointer
    ui->cloudtree->setCloudView(ui->cloudview);
    ui->cloudtree->setConsole(ui->console);
    ui->cloudtree->setPropertiesTable(ui->cloudtable);
    ui->cloudtree->setProgressBar(ui->progress_bar);
    ui->cloudtree->setParentIcon(QIcon(":/res/icon/document-open.svg"));
    ui->cloudtree->setChildIcon(QIcon(":/res/icon/view-calendar.svg"));

    // toolbar
    // file
    connect(ui->actionOpen, &QAction::triggered, [this]() {
        logInfo("Action triggered: Open");
        ui->cloudtree->addCloud();
    });
    connect(ui->actionSave, &QAction::triggered, [this]() {
        logInfo("Action triggered: Save");
        ui->cloudtree->saveSelectedClouds();
    });
    connect(ui->actionClose, &QAction::triggered, [this]() {
        logInfo("Action triggered: Close");
        ui->cloudtree->removeSelectedClouds();
    });
    connect(ui->actionCloseAll, &QAction::triggered, [this]() {
        logInfo("Action triggered: CloseAll");
        ui->cloudtree->removeAllClouds();
    });
    connect(ui->actionMerge, &QAction::triggered, [this]() {
        logInfo("Action triggered: Merge");
        ui->cloudtree->mergeSelectedClouds();
    });
    connect(ui->actionClone, &QAction::triggered, [this]() {
        logInfo("Action triggered: Clone");
        ui->cloudtree->cloneSelectedClouds();
    });
    connect(ui->actionQuit, &QAction::triggered, [this]() {
        logInfo("Action triggered: Quit");
        this->close();
    });

    // edit
    connect(ui->actionColor, &QAction::triggered, [=] {
        logInfo("Action triggered: Color");
        this->createLeftDock<Color>("Color");
    });
    connect(ui->actionCoordinate, &QAction::triggered, [=] {
        logInfo("Action triggered: Coordinate");
        this->createDialog<Coordinate>("Coordinate", false);
    });
    connect(ui->actionNormals, &QAction::triggered, [=] {
        logInfo("Action triggered: Normals");
        this->createLeftDock<Normals>("Normals");
    });
    connect(ui->actionScale, &QAction::triggered, [=] {
        logInfo("Action triggered: Scale");
        this->createDialog<Scale>("Scale", false);
    });
    connect(ui->actionTransformation, &QAction::triggered, [=] {
        logInfo("Action triggered: Transformation");
        this->createLeftDock<Transformation>("Transformation");
    });
    connect(ui->actionFilters, &QAction::triggered, [=] {
        logInfo("Action triggered: Filters");
        this->createLeftDock<Filters>("Filters");
    });
    connect(ui->actionSegmentation, &QAction::triggered, [=] {
        logInfo("Action triggered: Segmentation");
        this->createLeftDock<Segmentation>("Segmentation");
    });
    connect(ui->actionCutting, &QAction::triggered, [=] {
        logInfo("Action triggered: Cutting");
        this->createDialog<Cutting>("Cutting", false);
    });
    connect(ui->actionSampling, &QAction::triggered, [=] {
        logInfo("Action triggered: Sampling");
        this->createDialog<Sampling>("Sampling", false);
    });

    // view
    connect(ui->actionTopView, &QAction::triggered, [this]() {
        logInfo("Action triggered: TopView");
        ui->cloudview->setTopView();
    });
    connect(ui->actionFrontView, &QAction::triggered, [this]() {
        logInfo("Action triggered: FrontView");
        ui->cloudview->setFrontView();
    });
    connect(ui->actionLeftSideView, &QAction::triggered, [this]() {
        logInfo("Action triggered: LeftSideView");
        ui->cloudview->setLeftSideView();
    });
    connect(ui->actionShowAxes, &QAction::triggered, [this](bool checked) {
        logInfo(QString("Action triggered: ShowAxes checked=%1").arg(checked));
        ui->cloudview->setShowAxes(checked);
    });
    connect(ui->actionShowFPS, &QAction::triggered, [this](bool checked) {
        logInfo(QString("Action triggered: ShowFPS checked=%1").arg(checked));
        ui->cloudview->setShowFPS(checked);
    });
    connect(ui->actionShowId, &QAction::triggered, [this](bool checked) {
        logInfo(QString("Action triggered: ShowId checked=%1").arg(checked));
        ui->cloudview->setShowId(checked);
    });

    // tools
    connect(ui->actionTCPCalib, &QAction::triggered, [=] {
        logInfo("Action triggered: TCPCalib");
        this->createDialog<TcpCalib>("TcpCalib", true);
    });
    connect(ui->actionCalib, &QAction::triggered, [=] {
        logInfo("Action triggered: HandEyeCalib");
        this->createDialog<HandEyeCalib>("HandEyeCalib", true);
    });
    connect(ui->actionMakeTool, &QAction::triggered, [=] {
        logInfo("Action triggered: MakeTool");
        this->createDialog<MakeTool>("MakeTool", true);
    });
    connect(ui->actionVision, &QAction::triggered, [=] {
        logInfo("Action triggered: VisionConfig");
        this->createDialog<VisionConfig>("VisionConfig", true);
    });
    
    // options
    connect(ui->actionLight, &QAction::triggered, [=] {
        logInfo("Action triggered: Theme Light");
        changeTheme(0);
    });
    connect(ui->actionUbuntu, &QAction::triggered, [=] {
        logInfo("Action triggered: Theme ElegantDark");
        changeTheme(1);
    });
    connect(ui->actionDark, &QAction::triggered, [=] {
        logInfo("Action triggered: Theme Dark");
        changeTheme(2);
    });

    connect(ui->actionEnglish, &QAction::triggered, [=] {
        logInfo("Action triggered: Language English");
        changeLanguage(0);
    });
    connect(ui->actionChinese, &QAction::triggered, [=] {
        logInfo("Action triggered: Language Chinese");
        changeLanguage(1);
    });

    // help
    About* about = new About(this);
    ShortcutKey* shortcutKey = new ShortcutKey(this);
    connect(ui->actionAbout, &QAction::triggered, [this, about]() {
        logInfo("Action triggered: About");
        about->show();
    });
    connect(ui->actionShortcutKey, &QAction::triggered, [this, shortcutKey]() {
        logInfo("Action triggered: ShortcutKey");
        shortcutKey->show();
    });
    connect(ui->actionScreenShot, &QAction::triggered, [this]() {
        logInfo("Action triggered: ScreenShot");
        this->saveScreenshot();
    });

    connect(ui->getCamBtn, &QPushButton::clicked, [this]() {
        logInfo("Button clicked: InitializeCamera");
        this->InitializeCamera();
    });
    ui->camComb->setCurrentIndex(-1);
    ui->camControlWidget->setCurrentIndex(0);
    connect(ui->camComb, QOverload<int>::of(&QComboBox::currentIndexChanged),
        [this](int index) {
            if(index < 3 && index >= 0)
                ui->camControlWidget->setCurrentIndex(0);
            else if(index == 3)
                ui->camControlWidget->setCurrentIndex(1);
        });

    ui->camControlWidget->setContentsMargins(2, 0, 2, 0);
    
    //connect(ui->GripperInitBtn, &QPushButton::clicked, this, &MainWindow::InitializeGripper);
    
    // robot
    ui->robotComb->setCurrentIndex(-1);
    connect(ui->robotInitBtn, &QPushButton::clicked, [this]() {
        logInfo("Button clicked: InitializeRobot");
        this->InitializeRobot();
    });

    connect(ui->commInitializeBtn, &QPushButton::clicked, [this]() {
        logInfo("Button clicked: InitializeComm");
        this->InitializeComm();
    });

    ui->jointCheckBtn->setChecked(true);
    ui->recordRobotBtn->setText("joint");

    // vision process
    proworker = std::make_unique<ProcessWorker>();
    proworker->moveToThread(m_thread_prowork);
    connect(m_thread_prowork, &QThread::finished, this, &MainWindow::deleteLater);
    connect(this, &MainWindow::triggerVisionPro, proworker.get(), &ProcessWorker::triggerVisionPro);
    connect(proworker.get(), &ProcessWorker::errorOccurred, [this](const QString& errorMsg) {
        logError(errorMsg);
    });
    connect(proworker.get(), &ProcessWorker::statusWarning, [this](const QString& errorMsg) {
        logWarning(errorMsg);
    });
    connect(proworker.get(), &ProcessWorker::statusChanged, [this](const QString& errorMsg) {
        logInfo(errorMsg);
    });
    connect(proworker.get(), &ProcessWorker::sendOffMatrix, this, &MainWindow::recOffMatrix);
    m_thread_prowork->start();

    this->changeTheme(0);
    ui->progress_bar->close();
    logInfo("HSmartVision started.");

    // auto calibration
    connect(ui->autoCalibBtn, &QPushButton::clicked, [this]() {
        logInfo("Button clicked: AutoCalib");
        QMessageBox::StandardButton reply;
        reply = QMessageBox::question(this,
            QStringLiteral("�Զ��궨"),
            QStringLiteral("��ȷ��������ӣ���е�����ӣ��궨������������Ұ����!"),
            QMessageBox::Yes | QMessageBox::No,
            QMessageBox::No); // Ĭ�ϰ�ť
        if (reply == QMessageBox::Yes) {
            logInfo("AutoCalib confirmed; starting capture sequence.");
            isAutoCalib = true;
            calibImageCount = 0;
            this->camera->captureDevice();
        }
        else {
            logInfo("AutoCalib cancelled by user.");
            return;
        }
    });

    QTimer::singleShot(3000, this, [this]() {
        setEnabled(true);
        statusBar()->showMessage("Software Initialize Done!", 2000);
        logInfo("Software initialization completed; UI enabled.");
    });
}

MainWindow::~MainWindow() {
    logInfo("MainWindow destroying; shutting down worker threads.");

    if (m_thread_prowork)
    {
        m_thread_prowork->quit();
        m_thread_prowork->wait(3000);
        m_thread_prowork->deleteLater();
        m_thread_prowork = nullptr;
        m_thread_prowork = nullptr; // deleted via finished �� deleteLater
    }

    if (m_thread_rfwork) {
        m_thread_rfwork->quit();
        m_thread_rfwork->wait(3000);
        m_thread_rfwork->deleteLater();
        m_thread_rfwork = nullptr;
        m_thread_rfwork = nullptr; // deleted via finished �� deleteLater
    }

    if (m_thread_comm)
    {
        m_thread_comm->quit();
        m_thread_comm->wait(3000);
        m_thread_comm->deleteLater();
        m_thread_comm = nullptr;
        m_worker_comm = nullptr; // deleted via finished �� deleteLater
    }

    delete ui; 
}

void MainWindow::changeTheme(int index)
{
    QFile qss;
    switch (index)
    {
    case 0:
        qss.setFileName(":/res/theme/light.qss");
        qss.open(QFile::ReadOnly);
        qApp->setStyleSheet(qss.readAll());
        qss.close();
        ui->statusBar->showMessage(tr("light Theme"), 2000);
        logInfo("Theme changed to Light.");
        break;
    case 1:
        qss.setFileName(":/res/theme/ElegantDark.qss");
        qss.open(QFile::ReadOnly);
        qApp->setStyleSheet(qss.readAll());
        qss.close();
        ui->statusBar->showMessage(tr("ElegantDark Theme"), 2000);
        logInfo("Theme changed to ElegantDark.");
        break;
    case 2:
        qss.setFileName(":/res/theme/dark.qss");
        qss.open(QFile::ReadOnly);
        qApp->setStyleSheet(qss.readAll());
        qss.close();
        ui->statusBar->showMessage(tr("Dark Theme"), 2000);
        logInfo("Theme changed to Dark.");
        break;
    default:
        break;
    }
}

void MainWindow::changeLanguage(int index)
{
    switch (index)
    {
    case 0:
        if (translator != nullptr)
        {
            qApp->removeTranslator(translator);
            ui->retranslateUi(this);
            logInfo("Language switched to English.");
        }
        break;
    case 1:
        if (translator == nullptr)
        {
            translator = new QTranslator;
            bool ret = translator->load(":/res/trans/zh_CN.qm");
            if (!ret) {
                logError(tr("Failed to load language file."));
                return;
            }
        }
        qApp->installTranslator(translator);
        ui->retranslateUi(this);
        logInfo("Language switched to Chinese.");
        break;
    }
}

void MainWindow::saveScreenshot()
{
    QString filename = "screenshot" + QDateTime::currentDateTime().toString("-hh-mm-ss");
    QString filepath = QFileDialog::getSaveFileName(this, tr("Save Screenshot"), filename, "PNG(*.png)");
    if (filepath.isEmpty()) {
        logInfo("Screenshot cancelled: no file path selected.");
        return;
    }
    if (filepath.endsWith(".png", Qt::CaseInsensitive))
        ui->cloudview->saveScreenshot(filepath.toLocal8Bit());
    else
        ui->cloudview->saveScreenshot(filepath.append(".png").toLocal8Bit());
    logInfo(tr("Screenshot saved successfully: %1").arg(filepath));
}

void MainWindow::moveEvent(QMoveEvent* event)
{
    QPoint pos = ui->cloudview->mapToGlobal(QPoint(0, 0));
    emit ui->cloudview->posChanged(pos);
    return QMainWindow::moveEvent(event);
}

void MainWindow::InitializeCamera() {
    logInfo(QString("InitializeCamera started, camIndex=%1").arg(ui->camComb->currentIndex()));

    if (camera != nullptr) {
        camera->reset();
    }
    int camIndex = ui->camComb->currentIndex();
    if (camIndex < 0) {
        logError(tr("Please select a camera type first."));
        return;
    }
    
    switch (camIndex)
    {
    case 0:
        camType = CameraFactory::CameraType::MechMind;
        break;
    case 1:
        camType = CameraFactory::CameraType::HKRgbd;
        break;
    case 2:
        camType = CameraFactory::CameraType::TuYang;
        break;
    default:
        camType = CameraFactory::CameraType::UNKOWN;
        break;
    }

    if (camType == CameraFactory::CameraType::UNKOWN) {
        logError("InitializeCamera aborted: unknown camera type.");
        return;
    }
    camera = CameraFactory::createCamera(camType);

    connect(this->camera.get(), &ICamera::sendCamModel, this, &MainWindow::GetCamModel);
    connect(this->camera.get(), &ICamera::sendCamInfo, this, &MainWindow::GetCamInfo);
    connect(this->camera.get(), &ICamera::sendCamConnectStatus, this, &MainWindow::GetCamStatus);
    connect(this->camera.get(), &ICamera::sendCamCloud, this, &MainWindow::GetCamData);
    connect(ui->cam_btn_search, &QPushButton::clicked, [this]() {
        logInfo("Button clicked: Camera Search");
        if (this->camera) this->camera->searchDevice();
    });
    connect(ui->cam_btn_connect, &QPushButton::clicked, this, [&]() {
        if (this->camera) {
            int index = ui->cam_cbox_device->currentIndex();
            logInfo(QString("Button clicked: Camera Connect, deviceIndex=%1").arg(index));
            this->camera->connectDevice(index);
        }
    }, Qt::UniqueConnection);

    connect(ui->cam_btn_capture, &QPushButton::clicked, [this]() {
        logInfo("Button clicked: Camera Capture");
        if (this->camera) this->camera->captureDevice();
    });
    connect(ui->cam_btn_add, &QPushButton::clicked, [this]() {
        logInfo("Button clicked: Camera Add");
        if (this->camera) this->camera->add();
    });
    connect(ui->cam_btn_reset, &QPushButton::clicked, [this]() {
        logInfo("Button clicked: Camera Reset");
        if (this->camera) this->camera->reset();
    });

    camera->setCloudTree(ui->cloudtree);
    camera->setCloudView(ui->cloudview);
    camera->setConsole(ui->console);

    ui->cam_cbox_device->clear();
    ui->cam_txt_info->clear();

    logInfo(tr("Camera initialized successfully."));
}

void MainWindow::GetCamModel(const QVector<QString>& camModel) {
    if (camModel.size() == 0) {
        logError(tr("Camera model list is empty."));
        return;
    }
    for (auto model : camModel) {
        ui->cam_cbox_device->addItem(model);
    }
    ui->cam_cbox_device->setCurrentIndex(0);
    logInfo(QString("Camera feedback: received %1 device model(s).").arg(camModel.size()));
}

void MainWindow::GetCamInfo(const QString& info) {
    if (info.isEmpty()) {
        logError(tr("Camera info is empty."));
        return;
    }
    ui->cam_txt_info->append(info);
}

void MainWindow::GetCamStatus(const bool& status) {
    if (status == false) {
        ui->cam_btn_connect->setIcon(QIcon(":/res/icon/connect.svg"));
        ui->cam_btn_connect->setText("connect");
        ui->camComb->setEnabled(true);
        ui->cam_btn_search->setEnabled(true);
        ui->getCamBtn->setEnabled(true);
        camConnected = false;
        logInfo("Camera feedback: disconnected.");
    }
    else {
        ui->cam_btn_connect->setIcon(QIcon(":/res/icon/disconnect.svg"));
        ui->cam_btn_connect->setText("discon");
        ui->camComb->setEnabled(false);
        ui->cam_btn_search->setEnabled(false);
        ui->getCamBtn->setEnabled(false);
        camConnected = true;
        logInfo("Camera feedback: connected.");
    }
}

void MainWindow::GetCamData(const CamData& data, const QString& id) {
    if (isRunOnce) {
        logInfo(QString("Camera feedback: RunOnce capture received, cloudId=%1").arg(id));
        proworker->setInputSceneData(data.cloud, data.color_Image, data.depth_Image);
        current_cloud_id = id;
        emit triggerVisionPro();
        logInfo("Vision process triggered from camera capture.");
        isRunOnce = false;
    }

    if (isAutoCalib) {
        std::string savePath;
        if (calibImageCount < 10) {
            savePath = "./config/Calibration/CalibImage/color_0" + std::to_string(calibImageCount) + ".bmp";
        }
        else {
            savePath = "./config/Calibration/CalibImage/color_" + std::to_string(calibImageCount) + ".bmp";
        }
        cv::imwrite(savePath, data.color_Image);
        if (calibImageCount == 0) {
            current_endvec.resize(6);
            current_endvec[0] = ui->robotX->value();
            current_endvec[1] = ui->robotY->value();
            current_endvec[2] = ui->robotZ->value();
            current_endvec[3] = ui->robotRx->value();
            current_endvec[4] = ui->robotRy->value();
            current_endvec[5] = ui->robotRz->value();
        }
        if (calibImageCount < calib_off.size()) {
            std::vector<float> current_offset(6);
            for (int i = 0; i < 6; ++i) {
                current_offset[i] = current_endvec[i] + calib_off[calibImageCount][i];
            }
            emit triggerCalibPro(current_offset, true, calib_off.size());
            calibImageCount++;
            if (calibImageCount == calib_off.size()) {
                calibImageCount = 0;
                isAutoCalib = false;
                logInfo(tr("Auto calibration flow completed."));
}
        }
    }
    
    return;
}

void MainWindow::on_runOnceBtn_clicked() {
    logInfo("Button clicked: RunOnce");
    isRunOnce = true;
    if (camConnected) {
        logInfo("RunOnce: capturing from connected camera.");
        ui->cam_btn_capture->click();
    }
    else if (ui->cloudtree->getSelectedClouds().size() != 0) {
        logInfo(tr("Using selected cloud-tree data for vision test flow."));
        std::vector<ct::Cloud::Ptr> selectPcd = ui->cloudtree->getSelectedClouds();
        proworker->setInputSceneData(selectPcd[0], cv::Mat(), cv::Mat());
        emit triggerVisionPro();
    }
    else {
        logInfo(tr("Using local file data for vision test flow."));
        QString pcd_path = QFileDialog::getOpenFileName(this, tr("Open pcd File"), "", tr("pcd File (*.ply);�����ļ� (*)"));
        ct::Cloud::Ptr cloud_in(new ct::Cloud);
        pcl::io::loadPLYFile(pcd_path.toStdString(), *cloud_in);
        proworker->setInputSceneData(cloud_in, cv::Mat(), cv::Mat());
        emit triggerVisionPro();
    }
    
}

void MainWindow::on_runCycleBtn_clicked() {
    logInfo("Button clicked: RunCycle (not implemented).");
    logWarning("RunCycle is not implemented yet.");
}

void MainWindow::on_loadToolBtn_clicked() {
    logInfo("Button clicked: LoadToolConfig");
    QString filePath = QFileDialog::getOpenFileName( this,tr("Open Json File"), "./config/toolkit/", tr("Json File (*.Json );�����ļ� (*)"));
    if (!filePath.isEmpty()) {
        ui->lineToolPath->setText(filePath);
    }
    else {
        logWarning(tr("No valid JSON file selected."));
        return;
    }
    logInfo(QString("Toolkit config selected: %1").arg(filePath));
    proworker->loadToolkitConfig(filePath.toStdString());
    logInfo("Toolkit config load requested.");
}

void MainWindow::on_loadVisionConfBtn_clicked() {
    logInfo("Button clicked: LoadVisionConfig");
    QString filePath = QFileDialog::getOpenFileName(this, tr("Open Json File"), "./config/visionConfig/", tr("Json File (*.Json );�����ļ� (*)"));
    if (!filePath.isEmpty()) {
        ui->visionEdit->setText(filePath);
    }
    else {
        logWarning(tr("No valid JSON file selected."));
        return;
    }
    logInfo(QString("Vision config selected: %1").arg(filePath));
    proworker->loadVisionConfig(filePath.toStdString());
    logInfo("Vision config load requested.");
}

//void MainWindow::on_screwConnectBtn_clicked() {
//    if (!screwConnected) {
//        emit screwConnect(ui->ScrewCommIp->text(), ui->ScrewCommPort->text());
//    }
//    else {
//        emit screwClose();
//    }    
//}
//
//void MainWindow::on_screwStartBtn_clicked() {
//    if (!screwConnected) {
//        return;
//    }
//    emit screwStart();
//}

void MainWindow::on_loadAutoCalibBtn_clicked() {
    logInfo("Button clicked: LoadAutoCalib");
    QString filePath = QFileDialog::getOpenFileName(this, tr("Open Json File"), "./config/robot/", tr("Json File (*.Json );�����ļ� (*)"));
    if (!filePath.isEmpty()) {
        ui->autocalibFile->setText(filePath);
    }
    else {
        logWarning(tr("No valid JSON file selected."));
        return;
    }
    std::ifstream file(filePath.toStdString());
    if (!file.is_open()) {
        logError(tr("Failed to open robot auto-calibration JSON file."));
        return;
    }
    json config;
    file >> config;
    file.close();
    calib_off = config.get<std::vector<std::vector<float>>>();
    logInfo(QString("Loaded robot calibration offset poses, count=%1.").arg(calib_off.size()));
}

//void MainWindow::InitializeScrew() {
//    sc = std::make_unique<ScrewComm>();
//    sc->moveToThread(&m_thread_scwork);
//    connect(&m_thread_scwork, &QThread::finished, this, &MainWindow::deleteLater);
//    connect(this, &MainWindow::screwConnect, this->sc.get(), &ScrewComm::init);
//    connect(this, &MainWindow::screwClose, this->sc.get(), &ScrewComm::close);
//    connect(this, &MainWindow::screwStart, this->sc.get(), &ScrewComm::runStatus);
//    connect(ui->screwResetBtn, &QPushButton::clicked, this->sc.get(), &ScrewComm::reset_stop);
//    connect(ui->screwBackBtn, &QPushButton::clicked, this->sc.get(), &ScrewComm::backlash);
//    connect(this->sc.get(), &ScrewComm::screwDone, this->robot.get(), &IRobot::getScrewStatus);
//    connect(this->robot.get(), &IRobot::screwStart, this->sc.get(), &ScrewComm::start);
//    connect(this->robot.get(), &IRobot::screwBack, this->sc.get(), &ScrewComm::backlash);
//    connect(this->sc.get(), &ScrewComm::sendScrewStatus, [this](bool status) {
//        if (status == false) {
//            ui->screwConnectBtn->setIcon(QIcon(":/res/icon/connect.svg"));
//        }
//        else {
//            ui->screwConnectBtn->setIcon(QIcon(":/res/icon/disconnect.svg"));
//        }
//        screwConnected = status;
//        });
//    connect(this->sc.get(), &ScrewComm::sendRunningStatus, [this](bool status) {
//        if (status == false) {
//            ui->screwStartBtn->setIcon(QIcon(":/res/icon/start_.svg"));
//        }
//        else {
//            ui->screwStartBtn->setIcon(QIcon(":/res/icon/stop_.svg"));
//        }
//        screwRunning = status;
//        });
//    m_thread_scwork.start();
//
//    logInfo(tr("The ScrewMachine Initialized Done!"));
//}

void MainWindow::InitializeRobot() {
    logInfo(QString("InitializeRobot started, robotIndex=%1").arg(ui->robotComb->currentIndex()));

    if (robot != nullptr) {
        robot->reset();
    }
    int robotIndex = ui->robotComb->currentIndex();
    if (robotIndex < 0) {
        logError(tr("Please select a robot type first."));
        return;
    }

    switch (robotIndex)
    {
    case 0:
        robotType = RobotFactory::RobotType::DOBOTCR5;
        break;
    case 1:
        robotType = RobotFactory::RobotType::DOBOTCRA10;
        break;
    case 2:
        robotType = RobotFactory::RobotType::JAKAZU12;
        break;
    case 3:
        robotType = RobotFactory::RobotType::RokaeCR12;
        break;
    default:
        robotType = RobotFactory::RobotType::UNKOWN;
        break;
    }

    if (robotType == RobotFactory::RobotType::UNKOWN) {
        logError("InitializeRobot aborted: unknown robot type.");
        return;
    }
    robot = RobotFactory::createRobot(robotType);
    robot->moveToThread(m_thread_rfwork);
    connect(m_thread_rfwork, &QThread::finished, this, &MainWindow::deleteLater);

    connect(this->proworker.get(), &ProcessWorker::sendRegTransform, this, [=](const std::vector<std::pair<std::vector<float>, std::vector<float>> >& t1, const Eigen::Affine3f& t2, const QString t3) {
        if(t1.size() == 0) return;
        const auto& pair = t1[0];

        // ��ʽ����һ�� float ���� (6��)��������λС��
        QString vec1Str;
        for (size_t j = 0; j < pair.first.size(); ++j) {
            vec1Str += QString::number(pair.first[j], 'f', 3); // ��Ϊ����3λС��
            if (j != pair.first.size() - 1) vec1Str += ", ";   // ���ŷָ�
        }

        // ��ʽ���ڶ��� float ���� (6��)��������λС��
        QString vec2Str;
        for (size_t k = 0; k < pair.second.size(); ++k) {
            vec2Str += QString::number(pair.second[k], 'f', 3); // ��Ϊ����3λС��
            if (k != pair.second.size() - 1) vec2Str += ", ";
        }

        // ��ÿһ��������ϳ�һ�� HTML �ı�
        QString htmlContent = QString("<font color='blue'>[%1]</font> Vec1: [%2] | Vec2: [%3]<br>")
            .arg(0)
            .arg(vec1Str)
            .arg(vec2Str);

        logInfo(htmlContent);
        this->robot.get()->startScrewSequence(t1, t2, t3);
        }, Qt::UniqueConnection);
    
    connect(ui->stopFlowBtn, &QPushButton::clicked, [this]() {
        if (ui->stopFlowBtn->text() == "start") {
            logInfo("Button clicked: StopFlow -> robotStop");
            this->robot->robotStop();
            ui->stopFlowBtn->setIcon(QIcon(":/res/icon/stop_.svg"));
            ui->stopFlowBtn->setText("stop");
            logInfo("Robot flow stopped.");
        }
        else if(ui->stopFlowBtn->text() == "stop"){
            logInfo("Button clicked: StopFlow -> robotStart");
            this->robot->robotStart();
            ui->stopFlowBtn->setIcon(QIcon(":/res/icon/start_.svg"));
            ui->stopFlowBtn->setText("start");
            logInfo("Robot flow started.");
        }
        });

    connect(this, &MainWindow::triggerCalibPro, this->robot.get(), &IRobot::RobotMoveJ);
    
    connect(this->robot.get(), &IRobot::capNextPose, this->camera.get(), &ICamera::captureDevice);
    connect(this->robot.get(), &IRobot::statusError, [this](const QString& msq) {
        logError(msq);
    });

    connect(ui->goCapBtn, &QPushButton::clicked, [this]() {
        logInfo("Button clicked: GoCap");
        if (!robotEnabled || !robotConnected) {
            logWarning("GoCap ignored: robot not connected or not enabled.");
            return;
        }
        std::ifstream file("./robot_config.json");
        if (!file.is_open()) {
            logError(tr("Failed to open robot_config.json."));
            return;
        }
        json config;
        file >> config;
        file.close();
        std::vector<float> capVector = config["Cap"].get<std::vector<float>>();
        if (capVector.size() != 6) {
            logError("GoCap failed: Cap pose in robot_config.json must contain 6 values.");
            return;
        }
        this->robot->RobotMoveJ(capVector, false, 0);
        logInfo("GoCap: RobotMoveJ to Cap pose requested.");
        });

    connect(ui->goHomeBtn, &QPushButton::clicked, [this]() {
        logInfo("Button clicked: GoHome");
        if (!robotEnabled || !robotConnected) {
            logWarning("GoHome ignored: robot not connected or not enabled.");
            return;
        }
        std::ifstream file("./robot_config.json");
        if (!file.is_open()) {
            logError(tr("Failed to open robot_config.json."));
            return;
        }
        json config;
        file >> config;
        file.close();
        std::vector<float> homeVector = config["Home"].get<std::vector<float>>();
        if (homeVector.size() != 6) {
            logError("GoHome failed: Home joints in robot_config.json must contain 6 values.");
            return;
        }
        this->robot->RobotJointMove(homeVector, false);
        logInfo("GoHome: RobotJointMove to Home joints requested.");
        });

    connect(ui->setCapBtn, &QPushButton::clicked, [this]() {
        logInfo("Button clicked: SetCap");
        if (!robotEnabled || !robotConnected) {
            logWarning("SetCap ignored: robot not connected or not enabled.");
            return;
        }
        json config;

        std::ifstream inFile("robot_config.json");
        if (inFile.is_open()) {
            try {
                inFile >> config;
            }
            catch (json::parse_error& e) {
                logError(tr("Failed to parse robot_config.json; file will be overwritten."));
                config = json::object();
            }
            inFile.close();
        }

        config["Cap"] = { ui->robotX->value(), ui->robotY->value(), ui->robotZ->value(), ui->robotRx->value(), ui->robotRy->value(), ui->robotRz->value() };
        std::ofstream outFile("robot_config.json");
        if (!outFile.is_open()) {
            logError(tr("Failed to open robot_config.json."));
            return;
        }
        outFile << std::setw(4) << config << std::endl;
        outFile.close();
        logInfo(tr("Cap position saved to robot_config.json.")); });

    connect(ui->setHomeBtn, &QPushButton::clicked, [this]() {
        logInfo("Button clicked: SetHome");
        if (!robotEnabled || !robotConnected) {
            logWarning("SetHome ignored: robot not connected or not enabled.");
            return;
        }
        json config;

        std::ifstream inFile("robot_config.json");
        if (inFile.is_open()) {
            try {
                inFile >> config;
            }
            catch (json::parse_error& e) {
                logError(tr("Failed to parse robot_config.json; file will be overwritten."));
                config = json::object();
            }
            inFile.close();
        }

        config["Home"] = { ui->j1->value(), ui->j2->value(), ui->j3->value(), ui->j4->value(), ui->j5->value(), ui->j6->value()};
        std::ofstream outFile("robot_config.json");
        if (!outFile.is_open()) {
            logError(tr("Failed to open robot_config.json."));
            return;
        }
        outFile << std::setw(4) << config << std::endl;
        outFile.close();
        logInfo(tr("Home position saved to robot_config.json.")); });

    connect(this->robot.get(), &IRobot::sendRobotStatus, [this](const std::vector<float>& Joint, const std::vector<float>& Pose) {
        if (Joint.size() != 6 || Pose.size() != 6) {
            logError(tr("Invalid robot status data received (expected 6 joints and 6 pose values)."));
            return;
        }
        ui->j1->setValue(Joint[0]); ui->j2->setValue(Joint[1]);
        ui->j3->setValue(Joint[2]); ui->j4->setValue(Joint[3]);
        ui->j5->setValue(Joint[4]); ui->j6->setValue(Joint[5]);
        
        ui->robotX->setValue(Pose[0]); ui->robotY->setValue(Pose[1]);
        ui->robotZ->setValue(Pose[2]); ui->robotRx->setValue(Pose[3]);
        ui->robotRy->setValue(Pose[4]); ui->robotRz->setValue(Pose[5]);
        });
    connect(this->robot.get(), &IRobot::statusEnable, [this](bool status) {
        if (status) {
            ui->robotEnableBtn->setIcon(QIcon(":/res/icon/disable.svg"));
        }
        else {
            ui->robotEnableBtn->setIcon(QIcon(":/res/icon/enable.svg"));
        }
        this->robotEnabled = status;
        logInfo(QString("Robot feedback: enabled=%1").arg(status ? "true" : "false"));
        });
    connect(this->robot.get(), &IRobot::statusConnected, [this](bool status) {
        if (status) {
            ui->robotConBtn->setIcon(QIcon(":/res/icon/disconnect.svg"));
            logInfo(tr("Robot connected successfully."));
}
        else {
            ui->robotConBtn->setIcon(QIcon(":/res/icon/connect.svg"));
            logInfo(tr("Robot disconnected successfully."));
}
        this->robotConnected = status;
        });

    connect(ui->robotConBtn, &QPushButton::clicked, [this]() {
        RobotPara rp;
        rp.IpAddress = ui->RobotCommIp->text().toStdString();
        rp.iPortDashboard = ui->RobotCommPort->text().toShort();
        if (!robotConnected) {
            logInfo(QString("Button clicked: RobotConnect ip=%1 port=%2")
                .arg(ui->RobotCommIp->text())
                .arg(ui->RobotCommPort->text()));
            this->robot->connectRobot(rp);
        }
        else {
            logInfo("Button clicked: RobotDisconnect");
            if (robotEnabled) {
                logWarning(tr("Please disable the robot before disconnecting."));
                return;
            }
            this->robot->DisconnectRobot();
            
        }
        });
    connect(ui->robotEnableBtn, &QPushButton::clicked, [this]() {
        logInfo(QString("Button clicked: RobotEnable targetEnabled=%1").arg(!robotEnabled));
        if (!robotConnected) {
            logWarning("RobotEnable ignored: robot is not connected.");
            return;
        }
        this->robot->EnableRobot(!robotEnabled);
        logInfo(!robotEnabled ? tr("Robot enabled successfully.") : tr("Robot disabled successfully.")); });
    connect(ui->robotSpeedBtn, &QPushButton::clicked, [this]() {
        logInfo(QString("Button clicked: RobotSpeed value=%1").arg(ui->speedSpin->value()));
        if (!robotEnabled || !robotConnected) {
            logWarning("RobotSpeed ignored: robot not connected or not enabled.");
            return;
        }
        this->robot->setParams(ui->speedSpin->value());
        logInfo("Robot speed parameter updated.");
        });

    connect(ui->jointCheckBtn, &QRadioButton::toggled, this, [this](bool checked) {
        if (checked) {
            m_isJointMode = true;
            ui->recordRobotBtn->setText("joint"); }
        });

    connect(ui->poseCheckBtn, &QRadioButton::toggled, this, [this](bool checked) {
        if (checked) {
            m_isJointMode = false;
            ui->recordRobotBtn->setText("pose"); }
        });

    connect(ui->recordRobotBtn, &QPushButton::clicked, this, [this]() {
        logInfo(QString("Button clicked: RecordRobot mode=%1").arg(m_isJointMode ? "joint" : "pose"));
        // ���ݵ�ǰģʽȷ���ļ�����������
        const std::string fileName = m_isJointMode ? "./config/robot/calib_joint.json" : "./config/robot/calib_pose.json";
        int& index = m_isJointMode ? m_jointIndex : m_poseIndex;

        json root;
        {
            std::ifstream fin(fileName);
            if (fin.is_open()) {
                try {
                    fin >> root;
                }
                catch (...) {
                    root = json::object();  // �ļ��𻵻�Ϊ��ʱ����
                }
            }
        }
        if (!root.is_object()) {
            root = json::object();
        }

        // 2. ��װ��������
        json record;
        if (m_isJointMode) {
            // ��¼ 6 ���ؽڽ�
            record = {
                ui->j1->value(),
                ui->j2->value(),
                ui->j3->value(),
                ui->j4->value(),
                ui->j5->value(),
                ui->j6->value()
            };
        }
        else {
            // ��¼����λ��
            record = {
                ui->robotX->value(),
                ui->robotY->value(),
                ui->robotZ->value(),
                ui->robotRx->value(),
                ui->robotRy->value(),
                ui->robotRz->value()
            };
        }

        QMessageBox::StandardButton reply;
        reply = QMessageBox::question(this,
            tr(u8"ȷ��"),
            tr(u8"�Ƿ�ȷ�ϼ�¼��λ�û�����״̬��"),
            QMessageBox::Yes | QMessageBox::No,
            QMessageBox::No); // Ĭ�ϰ�ť
        if (reply != QMessageBox::Yes) {
            logInfo("RecordRobot cancelled by user.");
            return;
        }

        // 3. ����ۼӣ��Ա����Ϊ��д��
        ++index;
        root[std::to_string(index)] = record;

        // 4. д���ļ�
        std::ofstream fout(fileName);
        if (fout.is_open()) {
            fout << root.dump(4);
            fout.close();
            logInfo(QString("RecordRobot success: wrote entry index=%1 to %2")
                .arg(index)
                .arg(QString::fromStdString(fileName)));
        } else {
            logError(QString("RecordRobot failed: cannot open file %1")
                .arg(QString::fromStdString(fileName)));
        }
        });

    connect(ui->clearRecordBtn, &QPushButton::clicked, this, [this]() {
        logInfo(QString("Button clicked: ClearRecord mode=%1").arg(m_isJointMode ? "joint" : "pose"));
        const std::string fileName = m_isJointMode ? "./config/robot/calib_joint.json" : "./config/robot/calib_pose.json";
        int& index = m_isJointMode ? m_jointIndex : m_poseIndex;

        QMessageBox::StandardButton reply;
        reply = QMessageBox::question(this,
            tr(u8"ȷ��"),
            tr(u8"�Ƿ�ȷ�����ѡ���ļ����ݣ�"),
            QMessageBox::Yes | QMessageBox::No,
            QMessageBox::No); // Ĭ�ϰ�ť
        if (reply != QMessageBox::Yes) {
            logInfo("ClearRecord cancelled by user.");
            return;
        }

        // д��յ� json �����������
        std::ofstream fout(fileName, std::ios::trunc);
        if (fout.is_open()) {
            fout << nlohmann::json::object().dump(4);
            fout.close();
            logInfo(QString("ClearRecord success: cleared %1")
                .arg(QString::fromStdString(fileName)));
        } else {
            logError(QString("ClearRecord failed: cannot open file %1")
                .arg(QString::fromStdString(fileName)));
        }

        index = 0;
        });

    m_thread_rfwork->start();

    logInfo(tr("Robot initialized successfully."));
}

void MainWindow::recOffMatrix(const std::vector<Eigen::Affine3f>& before, 
    const std::vector<Eigen::Affine3f>& after, const Eigen::Affine3f& sceneTrans) {
    if (before.size() != after.size()) {
        logError(tr("Pose output size mismatch between before and after transforms."));
        return;
    }
    
    ui->cloudview->removeAllCoordinateSystems();

    std::vector<ct::Cloud::Ptr> cloud_cluster = ui->cloudtree->getSelectedClouds();
    if (cloud_cluster.size() != 0) {
        current_cloud_id = cloud_cluster[0]->id();
    }
    else {
        if (current_cloud_id == "") {
            logError(tr("Current cloud ID is empty; cannot update scene pose."));
            return;
        }
    }

    ui->cloudview->updateCloudPose(current_cloud_id, sceneTrans);

    for (int i = 0; i < before.size(); ++i) {
        std::string id = "coor_" + std::to_string(i);
        ct::Coord Coord(QString::fromStdString(id), 30, before[i]);
        ui->cloudview->addCoordinateSystem(Coord);

        id = "coor_a_" + std::to_string(i);
        ct::Coord Coord_(QString::fromStdString(id), 50, after[i]);
        ui->cloudview->addCoordinateSystem(Coord_);

        ct::PointXYZRGBN txtPos;
        txtPos.x = after[i].translation().x();
        txtPos.y = after[i].translation().y();
        txtPos.z = after[i].translation().z() + 5;
        ui->cloudview->addText3D(QString::number(i), txtPos, "text" + QString::number(i));
    }
    current_cloud_id = "";
    logInfo(QString("Vision feedback: rendered %1 coordinate frame pair(s).").arg(static_cast<int>(before.size())));
}

void MainWindow::closeEvent(QCloseEvent* event)
{
    logInfo("Close requested: Exit confirmation dialog shown.");
    QMessageBox::StandardButton reply;
    reply = QMessageBox::question(this,
        tr("Exit"),
        tr("Are you sure you want to exit HSmartVision?"),
        QMessageBox::Yes | QMessageBox::No,
        QMessageBox::No);
    if (reply == QMessageBox::Yes) {
        logInfo("Exit confirmed; closing application.");
        event->accept();
    }
    else {
        logInfo("Exit cancelled by user.");
        event->ignore();
    }
}

//void MainWindow::InitializeComm() {
//
//    connect(ui->connectBtn, &QPushButton::clicked, this, [=]() {
//        if (m_comm_connected)
//        {
//            if (m_worker_comm) m_worker_comm->stopReceiving();
//
//            if (m_thread_comm)
//            {
//                m_thread_comm->quit();
//                m_thread_comm->wait(3000);
//                delete m_thread_comm;            // wait() ��ɺ����ֱ�� delete
//                m_thread_comm = nullptr;
//                m_worker_comm = nullptr;
//            }
//
//            if (m_udpSocket)
//            {
//                try { m_udpSocket->close(); }
//                catch (...) {}
//                delete m_udpSocket;
//                m_udpSocket = nullptr;
//            }
//
//            setConnected(false);
//            return;
//        }
//
//        // ���� ���� ����
//        const QString ip = ui->ipaddressEdit->text().trimmed();
//        const QString portStr = ui->commPortEdit->text().trimmed();
//
//        if (ip.isEmpty() || portStr.isEmpty())
//        {
//            logWarning("Invalid input: IP address and port are required.");
//            return;
//        }
//
//        m_udpMode = (ui->scoketComb->currentText() == "UDP");
//
//        try
//        {
//            if (m_udpMode)
//            {
//                // ���� UDP ����������������������������������������������������������������������������������������������������
//                m_udpSocket = new rl::hal::Socket(rl::hal::Socket::Udp(m_udpAddress));
//                m_udpSocket->open();
//                setConnected(true);
//            }
//            else
//            {
//                // ���� TCP ����������������������������������������������������������������������������������������������������
//                rl::hal::Socket::Address address = rl::hal::Socket::Address::Ipv4(
//                    ip.toStdString(), portStr.toStdString());
//
//                auto sock = std::make_unique<rl::hal::Socket>(
//                    rl::hal::Socket::Tcp(address));
//                sock->open();
//                sock->connect();
//
//                // ���� Worker + �߳�
//                m_thread_comm = new QThread(this);
//                m_worker_comm = new SocketWorker();   // ���� parent�����̹߳���
//                m_worker_comm->setSocket(std::move(sock));
//                m_worker_comm->moveToThread(m_thread_comm);
//
//                // Worker �ź� �� Panel slot�����̣߳�Qt::QueuedConnection �Զ���
//                connect(m_worker_comm, &SocketWorker::dataReceived, this, [=](const QString& data) {
//                    ui->receiveEdit->setText(data);
//                    });
//
//                connect(m_worker_comm, &SocketWorker::errorOccurred, this, [=](const QString& msg) {
//                    logError(msg);
//                    });
//
//                connect(m_worker_comm, &SocketWorker::disconnected, this, [=]() {
//                    if (m_thread_comm)
//                    {
//                        m_thread_comm->quit();
//                        m_thread_comm->wait(2000);
//                        m_thread_comm->deleteLater();
//                        m_thread_comm = nullptr;
//                        m_worker_comm = nullptr;
//                    }
//                    setConnected(false);
//                    });
//
//                // �߳�������ʼ����
//                connect(m_thread_comm, &QThread::started, m_worker_comm, &SocketWorker::startReceiving);
//
//                connect(m_thread_comm, &QThread::finished, m_worker_comm, &QObject::deleteLater);
//
//                m_thread_comm->start();
//                setConnected(true);
//            }
//        }
//        catch (const std::exception& e)
//        {
//            QMessageBox::critical(this, "Connection Failed", QString::fromStdString(e.what()));
//        }
//        });
//
//    connect(ui->commSendBtn, &QPushButton::clicked, this, [=]() {
//        if (!m_comm_connected) return;
//
//        const QString msg = ui->sendEdit->text();
//        if (msg.isEmpty()) return;
//
//        if (m_udpMode)
//        {
//            try
//            {
//                QByteArray bytes = msg.toUtf8();
//                m_udpSocket->sendto(bytes.constData(), static_cast<std::size_t>(bytes.size()), m_udpAddress);
//            }
//            catch (const std::exception& e)
//            {
//                QMessageBox::warning(this, "Send Error", QString::fromStdString(e.what()));
//            }
//        }
//        else
//        {
//            if (m_worker_comm) m_worker_comm->sendMsg(msg);
//        }
//        });
//
//    connect(ui->sendEdit, &QLineEdit::returnPressed, this, [=]() {
//        if (!m_comm_connected) return;
//
//        const QString msg = ui->sendEdit->text();
//        if (msg.isEmpty()) return;
//
//        if (m_udpMode)
//        {
//            try
//            {
//                QByteArray bytes = msg.toUtf8();
//                m_udpSocket->sendto(bytes.constData(), static_cast<std::size_t>(bytes.size()), m_udpAddress);
//            }
//            catch (const std::exception& e)
//            {
//                QMessageBox::warning(this, "Send Error", QString::fromStdString(e.what()));
//            }
//        }
//        else
//        {
//            if (m_worker_comm) m_worker_comm->sendMsg(msg);
//        }
//        });
//
//    logInfo("Socket communication is ready.");
//}

void MainWindow::InitializeComm()
{
    logInfo("InitializeComm started; rewiring communication UI handlers.");
    // ���� Re-init: disconnect previous UI hooks + tear down active session ������
    disconnect(ui->connectBtn, &QPushButton::clicked, this, nullptr);
    disconnect(ui->commSendBtn, &QPushButton::clicked, this, nullptr);
    disconnect(ui->sendEdit, &QLineEdit::returnPressed, this, nullptr);

    if (m_worker_comm)
    {
        disconnect(m_worker_comm, nullptr, this, nullptr);
        m_worker_comm->stopReceiving();
    }

    if (m_thread_comm)
    {
        m_thread_comm->quit();
        m_thread_comm->wait(3000);
        m_thread_comm->deleteLater();
        m_thread_comm = nullptr;
        m_worker_comm = nullptr; // deleted via finished �� deleteLater
    }

    if (m_comm_connected)
        setConnected(false);

    // ���� Helpers ������������������������������������������������������������������������������������������������������������������������
    auto cleanupComm = [this]() {
        if (m_worker_comm)
        {
            disconnect(m_worker_comm, nullptr, this, nullptr);
            m_worker_comm->stopReceiving();
        }

        if (m_thread_comm)
        {
            m_thread_comm->quit();
            m_thread_comm->wait(3000);
            m_thread_comm->deleteLater();
            m_thread_comm = nullptr;
            m_worker_comm = nullptr;
        }

        setConnected(false);
    };

    connect(ui->connectBtn, &QPushButton::clicked, this, [=]() {
        if (m_comm_connected)
        {
            logInfo("Button clicked: CommDisconnect");
            cleanupComm();
            logInfo("Communication session disconnected.");
            return;
        }
        logInfo(QString("Button clicked: CommConnect mode=%1 ip=%2 port=%3")
            .arg(ui->scoketComb->currentText().trimmed())
            .arg(ui->ipaddressEdit->text().trimmed())
            .arg(ui->commPortEdit->text().trimmed()));

        // ���� connect ����
        const QString modeText = ui->scoketComb->currentText().trimmed();
        const QString ip = ui->ipaddressEdit->text().trimmed();
        const QString portStr = ui->commPortEdit->text().trimmed();

        const bool isTcpServer = (modeText == QStringLiteral("TCP Server"));
        const bool isUdp = (modeText == QStringLiteral("UDP"));
        // anything else (including "TCP Client") �� TCP Client

        if (portStr.isEmpty())
        {
            logWarning("Invalid input: port is required.");
            return;
        }

        if (!isTcpServer && ip.isEmpty())
        {
            logWarning("Invalid input: IP address and port are required.");
            return;
        }

        try
        {
            // Clear any leftover objects from a previous failed attempt
            if (m_worker_comm)
            {
                disconnect(m_worker_comm, nullptr, this, nullptr);
                m_worker_comm->stopReceiving();
                m_worker_comm->deleteLater();
                m_worker_comm = nullptr;
            }
            if (m_thread_comm)
            {
                m_thread_comm->quit();
                m_thread_comm->wait(1000);
                m_thread_comm->deleteLater();
                m_thread_comm = nullptr;
            }

            m_thread_comm = new QThread(this);
            m_worker_comm = new SocketWorker(); // no parent; owned by thread via deleteLater

            if (isUdp)
            {
                const auto localAddr = rl::hal::Socket::Address::Ipv4(
                    std::string("0.0.0.0"), portStr.toStdString());
                const auto peerAddr = rl::hal::Socket::Address::Ipv4(
                    ip.toStdString(), portStr.toStdString());

                auto sock = std::make_unique<rl::hal::Socket>(
                    rl::hal::Socket::Udp(localAddr));
                sock->open();
                sock->bind();
                m_worker_comm->setUdpSocket(std::move(sock), peerAddr);
            }
            else if (isTcpServer)
            {
                const QString bindIp = ip.isEmpty() ? QStringLiteral("0.0.0.0") : ip;
                const auto address = rl::hal::Socket::Address::Ipv4(
                    bindIp.toStdString(), portStr.toStdString());

                auto sock = std::make_unique<rl::hal::Socket>(
                    rl::hal::Socket::Tcp(address));
                sock->open();
                sock->bind();
                sock->listen();
                m_worker_comm->setListenSocket(std::move(sock));
            }
            else
            {
                // TCP Client
                const auto address = rl::hal::Socket::Address::Ipv4(
                    ip.toStdString(), portStr.toStdString());

                auto sock = std::make_unique<rl::hal::Socket>(
                    rl::hal::Socket::Tcp(address));
                sock->open();
                sock->connect();
                m_worker_comm->setSocket(std::move(sock));
            }

            m_worker_comm->moveToThread(m_thread_comm);

            connect(m_worker_comm, &SocketWorker::dataReceived, this, [=](const QString& data) {
                ui->receiveEdit->setText(data);
            });

            connect(m_worker_comm, &SocketWorker::errorOccurred, this, [=](const QString& msg) {
                logError(msg); });

            connect(m_worker_comm, &SocketWorker::statusMessage, this, [=](const QString& msg) {
                logInfo(msg); });

            connect(m_worker_comm, &SocketWorker::clientAccepted, this, [=](const QString& peer) {
                logInfo(QStringLiteral("TCP server accepted client: %1").arg(peer));
            });

            connect(m_worker_comm, &SocketWorker::disconnected, this, [=]() {
                if (m_worker_comm)
                {
                    disconnect(m_worker_comm, nullptr, this, nullptr);
                    m_worker_comm->stopReceiving();
                }

                if (m_thread_comm)
                {
                    m_thread_comm->quit();
                    m_thread_comm->wait(2000);
                    m_thread_comm->deleteLater();
                    m_thread_comm = nullptr;
                    m_worker_comm = nullptr;
                }
                setConnected(false);
                });

            connect(m_thread_comm, &QThread::started, m_worker_comm, &SocketWorker::startReceiving);
            connect(m_thread_comm, &QThread::finished, m_worker_comm, &QObject::deleteLater);

            m_thread_comm->start();
            setConnected(true);

            const char* readyMsg =
                isUdp ? "UDP connected."
                : (isTcpServer ? "TCP Server listening."
                    : "TCP Client connected.");
            logInfo(readyMsg);
}
        catch (const std::exception& e)
        {
            if (m_worker_comm)
            {
                disconnect(m_worker_comm, nullptr, this, nullptr);
                delete m_worker_comm;
                m_worker_comm = nullptr;
            }
            if (m_thread_comm)
            {
                delete m_thread_comm;
                m_thread_comm = nullptr;
            }
            logError(QString("Communication connection failed: %1").arg(QString::fromStdString(e.what())));
            QMessageBox::critical(this, "Connection Failed", QString::fromStdString(e.what()));
        }
        });

    auto sendCurrent = [this]() {
        if (!m_comm_connected || !m_worker_comm) {
            logWarning("CommSend ignored: communication is not connected.");
            return;
        }

        const QString msg = ui->sendEdit->text();
        if (msg.isEmpty()) {
            logWarning("CommSend ignored: message is empty.");
            return;
        }

        logInfo(QString("Button clicked: CommSend, bytes=%1").arg(msg.toUtf8().size()));
        m_worker_comm->sendMsg(msg);
        logInfo("CommSend: message dispatched.");
    };

    connect(ui->commSendBtn, &QPushButton::clicked, this, [=]() { sendCurrent(); });
    connect(ui->sendEdit, &QLineEdit::returnPressed, this, [=]() { sendCurrent(); });

    logInfo("Socket communication is ready.");
}

void MainWindow::setConnected(bool connected)
{
    m_comm_connected = connected;
    logInfo(QString("Communication feedback: connected=%1").arg(connected ? "true" : "false"));

    ui->connectBtn->setText(connected ? "Disconnect" : "Connect");
    ui->connectBtn->setProperty("connected", connected);
    ui->connectBtn->setIcon(connected ? QIcon(":/res/icon/disconnect.svg") : QIcon(":/res/icon/connect.svg"));

    ui->commSendBtn->setEnabled(connected);
    ui->ipaddressEdit->setEnabled(!connected);
    ui->commPortEdit->setEnabled(!connected);
    ui->scoketComb->setEnabled(!connected);

    if (!connected) {
        ui->receiveEdit->clear();
        ui->sendEdit->clear();
    }
}

//void MainWindow::InitializeGripper() {
//
//    int gripperIndex = ui->gripperComb->currentIndex();
//    if (gripperIndex < 0) {
//        logError(tr("Please ensure the gripper type frist!"));
//        return;
//    }
//
//    switch (gripperIndex)
//    {
//    case 0:
//        gripperType = GripperFactory::GripperType::Jodell_RG;
//        break;
//    /*case 1:
//        camType = CameraFactory::CameraType::HKRgbd;
//        break;
//    case 2:
//        camType = CameraFactory::CameraType::TuYang;
//        break;
//    case 3:
//        camType = CameraFactory::CameraType::KWSeries;
//        break;*/
//    default:
//        gripperType = GripperFactory::GripperType::UNKOWN;
//        break;
//    }
//
//    if (gripperType == GripperFactory::GripperType::UNKOWN) return;
//    gripper = GripperFactory::createGripper(gripperType);
//
//    gripper->moveToThread(&m_thread_gripperwork);
//    connect(&m_thread_gripperwork, &QThread::finished, this, &MainWindow::deleteLater);
//
//    connect(ui->connectGripperBtn, &QPushButton::clicked, this, [=]() {
//        if (gripperConnected)
//            this->gripper.get()->disconnect();
//        else
//            this->gripper.get()->connect();
//        });
//
//    connect(ui->enableGripperBtn, &QPushButton::clicked, this, [=]() {
//        if (gripperEnabled)
//            this->gripper.get()->disenable();
//        else
//            this->gripper.get()->enable();
//        });
//
//    connect(ui->searchGripperBtn, &QPushButton::clicked, this->gripper.get(), &IGripper::search);
//
//    connect(this->gripper.get(), &IGripper::sendSearchCom, [this](int com) {
//        ui->ComPortBox->setCurrentIndex(com - 1);
//        });
//
//    connect(ui->openGripperBtn, &QPushButton::clicked, this->gripper.get(), &IGripper::open_gripper);
//    connect(ui->closeGripperBtn, &QPushButton::clicked, this->gripper.get(), &IGripper::close_gripper);
//
//    connect(this->gripper.get(), &IGripper::sendConnectStatus, [this](bool status) {
//        if (status == false) {
//            ui->connectGripperBtn->setIcon(QIcon(":/res/icon/connect.svg"));
//        }
//        else {
//            ui->connectGripperBtn->setIcon(QIcon(":/res/icon/disconnect.svg"));
//        }
//        gripperConnected = status;
//        });
//    connect(this->gripper.get(), &IGripper::sendEnableStatus, [this](bool status) {
//        if (status == false) {
//            ui->enableGripperBtn->setIcon(QIcon(":/res/icon/disenable.svg"));
//        }
//        else {
//            ui->enableGripperBtn->setIcon(QIcon(":/res/icon/enable_on.svg"));
//        }
//        gripperEnabled = status;
//        });
//    m_thread_gripperwork.start();
//
//    logInfo(tr("The Gripper Initialized Done!"));
//}
