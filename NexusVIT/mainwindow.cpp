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
#include "tool/PlaceSet.h"
#include "tool/MathUtils.h"

#include "utils/processWorker.h"
#include "utils/MovableWrl.h"

#include "device_tool/screwComm.h"

#include "device_sim/Thread.h"
#include "device_sim/Viewer.h"
#include "device_sim/SoGradientBackground.h"

#include <QDesktopWidget>
#include <QDir>
#include <QFile>
#include <QDateTime>
#include <QFileDialog>
#include <QTimer>
#include <QMessageBox>

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent),
    ui(new Ui::MainWindow),
    translator(nullptr),
    m_thread_prowork(new QThread),
    m_thread_scwork(new QThread),
    m_thread_rfwork(new QThread),
    m_thread_comm(new QThread),
    m_thread_gripperwork(new QThread),
    thread(new Thread)
{
    ui->setupUi(this);

    MainWindow::singleton = this;

    SoQt::init(this);
    SoDB::init();

    this->viewer = new Viewer(ui->tabWidget->widget(0));
    ui->robotWidget->addWidget(this->viewer);

    setEnabled(false);
    ui->cloudview->setBackgroundColor(ct::Color::Gray);
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

    connect_init();

    this->load(QDir::currentPath() + "/config/env/jaka/jakazu12.xml");

    QTimer::singleShot(3000, this, [this]() {
        setEnabled(true);
        statusBar()->showMessage("Software Initialize Done!", 2000);
    });
}

MainWindow::~MainWindow() {

    std::chrono::steady_clock::duration duration = this->planner->duration;

    this->thread->blockSignals(true);
    QCoreApplication::processEvents();
    this->planner->duration = std::chrono::steady_clock::duration::zero();
    this->thread->stop();
    this->planner->duration = duration;
    this->thread->blockSignals(false);

    this->planner->reset();
    this->model->reset();
    this->viewer->reset();

    if (camera) {
        camera = nullptr;
    }

    if (m_thread_prowork) {
        m_thread_prowork->quit();
        m_thread_prowork->wait(3000);
        delete m_thread_prowork;
        m_thread_prowork = nullptr;
    }

    if (m_thread_scwork) {
        m_thread_scwork->quit();
        m_thread_scwork->wait(3000);
        delete m_thread_scwork;
        m_thread_scwork = nullptr;
    }

    if (m_thread_comm) {
        m_thread_comm->quit();
        m_thread_comm->wait(3000);
        delete m_thread_comm;
        m_thread_comm = nullptr;
    }

    if (m_thread_rfwork) {
        m_thread_rfwork->quit();
        m_thread_rfwork->wait(3000);
        delete m_thread_rfwork;
        m_thread_rfwork = nullptr;
    }

    if (m_thread_gripperwork) {
        m_thread_gripperwork->quit();
        m_thread_gripperwork->wait(3000);
        delete m_thread_gripperwork;
        m_thread_gripperwork = nullptr;
    }

    delete ui; 
}

MainWindow* MainWindow::instance()
{
    if (nullptr == MainWindow::singleton)
    {
        new MainWindow();
    }

    return MainWindow::singleton;
}

void MainWindow::connect_init() {
    // toolbar
    // file
    connect(ui->actionOpen, &QAction::triggered, this, [this]() {
        if (ui->tabWidget->currentIndex() == 0)
            loadSceneWrl();
        else if (ui->tabWidget->currentIndex() == 1)
            ui->cloudtree->addCloud();
        });

    connect(ui->actionSave, &QAction::triggered, this, [this]() {
        if (ui->tabWidget->currentIndex() == 0)
            saveSceneWrl();
        else if (ui->tabWidget->currentIndex() == 1)
            ui->cloudtree->saveSelectedClouds();
        });
    connect(ui->actionClose, &QAction::triggered, this, [this]() {
        if (ui->tabWidget->currentIndex() == 0)
            removeSceneWrl();
        else if (ui->tabWidget->currentIndex() == 1)
            ui->cloudtree->removeSelectedClouds();
        });

    /*connect(ui->actionSave, &QAction::triggered, ui->cloudtree, &ct::CloudTree::saveSelectedClouds);
    connect(ui->actionClose, &QAction::triggered, ui->cloudtree, &ct::CloudTree::removeSelectedClouds);*/
    connect(ui->actionCloseAll, &QAction::triggered, ui->cloudtree, &ct::CloudTree::removeAllClouds);
    connect(ui->actionMerge, &QAction::triggered, ui->cloudtree, &ct::CloudTree::mergeSelectedClouds);
    connect(ui->actionClone, &QAction::triggered, ui->cloudtree, &ct::CloudTree::cloneSelectedClouds);
    connect(ui->actionQuit, &QAction::triggered, this, &MainWindow::close);

    // edit
    connect(ui->actionColor, &QAction::triggered, [=] { this->createLeftDock<Color>("Color"); });
    connect(ui->actionCoordinate, &QAction::triggered, [=] { this->createDialog<Coordinate>("Coordinate", false); });
    connect(ui->actionNormals, &QAction::triggered, [=] { this->createLeftDock<Normals>("Normals"); });
    connect(ui->actionScale, &QAction::triggered, [=] { this->createDialog<Scale>("Scale", false); });
    connect(ui->actionTransformation, &QAction::triggered, [=] { this->createLeftDock<Transformation>("Transformation"); });
    connect(ui->actionFilters, &QAction::triggered, [=] { this->createLeftDock<Filters>("Filters"); });
    connect(ui->actionSegmentation, &QAction::triggered, [=] { this->createLeftDock<Segmentation>("Segmentation"); });
    connect(ui->actionCutting, &QAction::triggered, [=] { this->createDialog<Cutting>("Cutting", false); });
    connect(ui->actionSampling, &QAction::triggered, [=] { this->createDialog<Sampling>("Sampling", false); });

    // view
    connect(ui->actionTopView, &QAction::triggered, ui->cloudview, &ct::CloudView::setTopView);
    connect(ui->actionFrontView, &QAction::triggered, ui->cloudview, &ct::CloudView::setFrontView);
    connect(ui->actionLeftSideView, &QAction::triggered, ui->cloudview, &ct::CloudView::setLeftSideView);
    connect(ui->actionShowAxes, &QAction::triggered, ui->cloudview, &ct::CloudView::setShowAxes);
    connect(ui->actionShowFPS, &QAction::triggered, ui->cloudview, &ct::CloudView::setShowFPS);
    connect(ui->actionShowId, &QAction::triggered, ui->cloudview, &ct::CloudView::setShowId);

    // tools
    connect(ui->actionTCPCalib, &QAction::triggered, [=] { this->createDialog<TcpCalib>("TcpCalib", true); });
    connect(ui->actionCalib, &QAction::triggered, [=] { this->createDialog<HandEyeCalib>("HandEyeCalib", true); });
    connect(ui->actionMakeTool, &QAction::triggered, [=] { this->createDialog<MakeTool>("MakeTool", true); });
    connect(ui->actionVision, &QAction::triggered, [=] { this->createDialog<VisionConfig>("VisionConfig", true); });
    connect(ui->placeSetAction, &QAction::triggered, [=] { this->createDialog<PlaceSet>("PlaceSet", true); });
    
    // options
    connect(ui->actionDark, &QAction::triggered, [=] { changeTheme(0); });
    connect(ui->actionLight, &QAction::triggered, [=] { changeTheme(1); });
    connect(ui->actionUbuntu, &QAction::triggered, [=] { changeTheme(2); });
    connect(ui->actionMacOS, &QAction::triggered, [=] { changeTheme(3); });

    connect(ui->actionEnglish, &QAction::triggered, [=] { changeLanguage(0); });
    connect(ui->actionChinese, &QAction::triggered, [=] { changeLanguage(1); });

    connect(ui->dataShowAction, &QAction::triggered, this, [this]() {
        if (!ui->DataDock->isVisible()) {
            ui->dataShowAction->setIcon(QIcon(":/res/icon/check.svg"));
        }
        else {
            ui->dataShowAction->setIcon(QIcon(""));
        }
        ui->DataDock->setVisible(!ui->DataDock->isVisible());
        });
    connect(ui->proShowAction, &QAction::triggered, this, [this]() {
        if (!ui->PropertiesDock->isVisible()) {
            ui->proShowAction->setIcon(QIcon(":/res/icon/check.svg"));
        }
        else {
            ui->proShowAction->setIcon(QIcon(""));
        }
        ui->PropertiesDock->setVisible(!ui->PropertiesDock->isVisible());
        });
    connect(ui->toolPanelShow, &QAction::triggered, this, [this]() {
        if (!ui->toolDockWidget->isVisible()) {
            ui->toolPanelShow->setIcon(QIcon(":/res/icon/check.svg"));
        }
        else {
            ui->toolPanelShow->setIcon(QIcon(""));
        }
        ui->toolDockWidget->setVisible(!ui->toolDockWidget->isVisible());
        });
    connect(ui->commShowAction, &QAction::triggered, this, [this]() {
        if (!ui->CommunicationDock->isVisible()) {
            ui->commShowAction->setIcon(QIcon(":/res/icon/check.svg"));
        }
        else {
            ui->commShowAction->setIcon(QIcon(""));
        }
        ui->CommunicationDock->setVisible(!ui->CommunicationDock->isVisible());
        });
    connect(ui->visionShowAction, &QAction::triggered, this, [this]() {
        if (!ui->taskDock->isVisible()) {
            ui->visionShowAction->setIcon(QIcon(":/res/icon/check.svg"));
        }
        else {
            ui->visionShowAction->setIcon(QIcon(""));
        }
        ui->taskDock->setVisible(!ui->taskDock->isVisible());
        });
    connect(ui->robotShowAction, &QAction::triggered, this, [this]() {
        if (!ui->robotControlWidget->isVisible()) {
            ui->robotShowAction->setIcon(QIcon(":/res/icon/check.svg"));
        }
        else {
            ui->robotShowAction->setIcon(QIcon(""));
        }
        ui->robotControlWidget->setVisible(!ui->robotControlWidget->isVisible());
        });

    connect(ui->actionReset, &QAction::triggered, this, [this]() {
        this->thread->stop();
        this->thread->reset();
        if (this->start!= nullptr && this->viewer != nullptr) this->viewer->drawConfiguration(*this->start);
    }, Qt::QueuedConnection);

    // help
    About* about = new About(this);
    ShortcutKey* shortcutKey = new ShortcutKey(this);
    connect(ui->actionAbout, &QAction::triggered, about, &QDialog::show);
    connect(ui->actionShortcutKey, &QAction::triggered, shortcutKey, &QDialog::show);
    connect(ui->actionScreenShot, &QAction::triggered, this, &MainWindow::saveScreenshot);

    connect(ui->getCamBtn, &QPushButton::clicked, this, &MainWindow::InitializeCamera, Qt::QueuedConnection);
    ui->camComb->setCurrentIndex(-1);
    ui->camControlWidget->setCurrentIndex(0);
    connect(ui->camComb, QOverload<int>::of(&QComboBox::currentIndexChanged),
        [this](int index) {
            if (index < 3 && index >= 0)
                ui->camControlWidget->setCurrentIndex(0);
            else if (index == 3)
                ui->camControlWidget->setCurrentIndex(1);
        });
    connect(ui->loadcamConfigBtn, &QPushButton::clicked, this, [&]() {
        QString PoseFilePath = QFileDialog::getOpenFileName(this, tr("Open Json File"), "./config/cam/", tr("Json File (*.Json)"));
        if (PoseFilePath.isEmpty()) return;
        ui->kwCamConfigEdit->setText(PoseFilePath);
        } , Qt::QueuedConnection);

    ui->camControlWidget->setContentsMargins(0, 0, 0, 0);
    ui->stackedWidget_2->setContentsMargins(0, 0, 0, 0);
    ui->toolStackWidget->setContentsMargins(0, 0, 0, 0);

    ui->toolStackWidget->setCurrentIndex(0);
    connect(ui->toolComboBox, QOverload<int>::of(&QComboBox::currentIndexChanged),
        [this](int index) {
            switch (index)
            {
            case 0:
                ui->toolStackWidget->setCurrentIndex(0);
                break;
            case 1:
                ui->toolStackWidget->setCurrentIndex(1);
                break;
            default:
                break;
            }
        });

    connect(ui->screwStartBtn, &QPushButton::clicked, this, [this]() {
        if (!screwConnected) return;
        emit screwStart();
        }, Qt::QueuedConnection);

    connect(ui->screwConnectBtn, &QPushButton::clicked, this, [this]() {
        if (!screwConnected) {
            emit screwConnect(ui->ScrewCommIp->text(), ui->ScrewCommPort->text());
        }
        else {
            emit screwClose();
        }
    }, Qt::QueuedConnection);

    // screw
    ui->screwComb->setCurrentIndex(-1);
    connect(ui->ScrewInitBtn, &QPushButton::clicked, this, &MainWindow::InitializeScrew, Qt::QueuedConnection);

    // gripper
    connect(ui->GripperInitBtn, &QPushButton::clicked, this, &MainWindow::InitializeGripper, Qt::QueuedConnection);

    // robot
    ui->robotComb->setCurrentIndex(-1);
    connect(ui->robotInitBtn, &QPushButton::clicked, this, &MainWindow::InitializeRobot, Qt::QueuedConnection);

    connect(ui->commInitializeBtn, &QPushButton::clicked, this, &MainWindow::InitializeComm, Qt::QueuedConnection);

    // trajectory
    connect(this->thread, &Thread::sendInfoMessage, this, [this](const std::string& msg) {
        ui->console->print(ct::LOG_INFO, QString::fromStdString(msg));
        }, Qt::QueuedConnection);
    connect(this->thread, &Thread::sendErrorMessage, this, [this](const std::string& msg) {
        ui->console->print(ct::LOG_ERROR, QString::fromStdString(msg));
        }, Qt::QueuedConnection);
    connect(this->thread, &Thread::planningFinished, this, [this](const std::vector<std::vector<float>>& path, double plannerMs) {
        ui->console->print(ct::LOG_INFO, QString("Planning finished, use time %1 ms!").arg(plannerMs));
        if (this->robot != nullptr && path.size() > 2 && this->robotConnected && this->robotEnabled) this->robot->startGraspSequence(path);
    }, Qt::QueuedConnection);

    // vision process
    proworker = std::make_unique<ProcessWorker>();
    proworker->moveToThread(m_thread_prowork);
    connect(m_thread_prowork, &QThread::finished, this, &MainWindow::deleteLater);
    connect(this, &MainWindow::triggerVisionPro, proworker.get(), &ProcessWorker::triggerVisionPro, Qt::QueuedConnection);
    connect(proworker.get(), &ProcessWorker::errorOccurred, [this](const QString& errorMsg) {
        ui->console->print(ct::LOG_ERROR, errorMsg); });
    connect(proworker.get(), &ProcessWorker::statusWarning, [this](const QString& errorMsg) {
        ui->console->print(ct::LOG_WARNING, errorMsg); });
    connect(proworker.get(), &ProcessWorker::statusChanged, [this](const QString& errorMsg) {
        ui->console->print(ct::LOG_INFO, errorMsg); });
    connect(proworker.get(), &ProcessWorker::sendOffMatrix, this, &MainWindow::recOffMatrix, Qt::QueuedConnection);
    connect(proworker.get(), &ProcessWorker::sendSceneCloud, this, &MainWindow::showSceneCloud, Qt::QueuedConnection);
    connect(proworker.get(), &ProcessWorker::sendTargetPose, this, &MainWindow::receiveGraspPose, Qt::QueuedConnection);
    m_thread_prowork->start();

    connect(ui->autoCalibBtn, &QPushButton::clicked, this, &MainWindow::startAutoCalib, Qt::QueuedConnection);

    this->changeTheme(0);
    ui->progress_bar->close();
    ui->console->print(ct::LOG_INFO, "NexusVIT started!");
}

void MainWindow::receiveGraspPose(const std::vector<std::pair<Eigen::Affine3f, Eigen::Affine3f>>& target_pose,
    const Eigen::Affine3f& tcp, const std::string& rotType) {
    this->thread->reset();

    if (coor_node.size() > 0) {
        for (int i = 0; i < coor_node.size(); i++) {
            this->viewer->sceneGroup->removeChild(coor_node[i]);
        }
        coor_node.clear();
    }

    std::vector<std::vector<float>> joint_array;
    for (size_t i = 0; i < target_pose.size(); i++)
    {
        Eigen::Affine3f trans = target_pose[i].second;
        trans.translation() *= 0.001;
        SoVRMLTransform* node = createWorldFrame(0.1f);
        SbMatrix matrix;
        for (int i = 0; i < 4; ++i)
        {
            for (int j = 0; j < 4; ++j)
            {
                matrix[i][j] = static_cast<float>(trans(j, i));
            }
        }
        node->setMatrix(matrix);
        this->viewer->sceneGroup->addChild(node);
        coor_node.push_back(node);
    }
    // this->model->reset();
    this->thread->animate = true;
    this->thread->setIkTimeoutMs(1000);
    this->thread->sortPose(target_pose);
}

void MainWindow::view_connect(const QObject* sender, const QObject* receiver)
{
    QObject::connect(
        sender,
        SIGNAL(configurationRequested(const rl::math::Vector&)),
        receiver,
        SLOT(drawConfiguration(const rl::math::Vector&))
    );

    QObject::connect(
        sender,
        SIGNAL(configurationEdgeRequested(const rl::math::Vector&, const rl::math::Vector&, const bool&)),
        receiver,
        SLOT(drawConfigurationEdge(const rl::math::Vector&, const rl::math::Vector&, const bool&))
    );

    QObject::connect(
        sender,
        SIGNAL(configurationPathRequested(const rl::plan::VectorList&)),
        receiver,
        SLOT(drawConfigurationPath(const rl::plan::VectorList&))
    );

    QObject::connect(
        sender,
        SIGNAL(configurationVertexRequested(const rl::math::Vector&, const bool&)),
        receiver,
        SLOT(drawConfigurationVertex(const rl::math::Vector&, const bool&))
    );

    QObject::connect(
        sender,
        SIGNAL(edgeResetRequested()),
        receiver,
        SLOT(resetEdges())
    );

    QObject::connect(
        sender,
        SIGNAL(lineRequested(const rl::math::Vector&, const rl::math::Vector&)),
        receiver,
        SLOT(drawLine(const rl::math::Vector&, const rl::math::Vector&))
    );

    QObject::connect(
        sender,
        SIGNAL(lineResetRequested()),
        receiver,
        SLOT(resetLines())
    );

    QObject::connect(
        sender,
        SIGNAL(messageRequested(const std::string&)),
        receiver,
        SLOT(showMessage(const std::string&))
    );

    QObject::connect(
        sender,
        SIGNAL(pointRequested(const rl::math::Vector&)),
        receiver,
        SLOT(drawPoint(const rl::math::Vector&))
    );

    QObject::connect(
        sender,
        SIGNAL(pointResetRequested()),
        receiver,
        SLOT(resetPoints())
    );

    QObject::connect(
        sender,
        SIGNAL(resetRequested()),
        receiver,
        SLOT(reset())
    );

    QObject::connect(
        sender,
        SIGNAL(sphereRequested(const rl::math::Vector&, const rl::math::Real&)),
        receiver,
        SLOT(drawSphere(const rl::math::Vector&, const rl::math::Real&))
    );

    QObject::connect(
        sender,
        SIGNAL(sphereResetRequested()),
        receiver,
        SLOT(resetSpheres())
    );

    QObject::connect(
        sender,
        SIGNAL(sweptVolumeRequested(const rl::plan::VectorList&)),
        receiver,
        SLOT(drawSweptVolume(const rl::plan::VectorList&))
    );

    QObject::connect(
        sender,
        SIGNAL(vertexResetRequested()),
        receiver,
        SLOT(resetVertices())
    );

    QObject::connect(
        sender,
        SIGNAL(workRequested(const rl::math::Transform&)),
        receiver,
        SLOT(drawWork(const rl::math::Transform&))
    );

    QObject::connect(
        sender,
        SIGNAL(workEdgeRequested(const rl::math::Vector&, const rl::math::Vector&)),
        receiver,
        SLOT(drawWorkEdge(const rl::math::Vector&, const rl::math::Vector&))
    );

    QObject::connect(
        sender,
        SIGNAL(workPathRequested(const rl::plan::VectorList&)),
        receiver,
        SLOT(drawWorkPath(const rl::plan::VectorList&))
    );
}

void MainWindow::view_disconnect(const QObject* sender, const QObject* receiver)
{
    QObject::disconnect(sender, nullptr, receiver, nullptr);
}

void MainWindow::changeTheme(int index)
{
    QFile qss;
    switch (index)
    {
    case 0:
        qss.setFileName(":/res/theme/dark.qss");
        qss.open(QFile::ReadOnly);
        qApp->setStyleSheet(qss.readAll());
        qss.close();
        ui->statusBar->showMessage(tr("Dark Theme"), 2000);
        break;
    case 1:
        qss.setFileName(":/res/theme/light.qss");
        qss.open(QFile::ReadOnly);
        qApp->setStyleSheet(qss.readAll());
        qss.close();
        ui->statusBar->showMessage(tr("light Theme"), 2000);
        break;
    case 2:
        qss.setFileName(":/res/theme/ElegantDark.qss");
        qss.open(QFile::ReadOnly);
        qApp->setStyleSheet(qss.readAll());
        qss.close();
        ui->statusBar->showMessage(tr("ElegantDark Theme"), 2000);
        break;
    case 3:
        qss.setFileName(":/res/theme/MaterialDark.qss");
        qss.open(QFile::ReadOnly);
        qApp->setStyleSheet(qss.readAll());
        qss.close();
        ui->statusBar->showMessage(tr("MaterialDark Theme"), 2000);
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
        }
        break;
    case 1:
        if (translator == nullptr)
        {
            translator = new QTranslator;
            bool ret = translator->load(":/res/trans/zh_CN.qm");
            if (!ret) {
                ui->console->print(ct::LOG_ERROR, tr("The language file load failed!"));
                return;
            }
        }
        qApp->installTranslator(translator);
        ui->retranslateUi(this);
        break;
    }
}

void MainWindow::saveScreenshot()
{
    QString filename = "screenshot" + QDateTime::currentDateTime().toString("-hh-mm-ss");
    QString filepath = QFileDialog::getSaveFileName(this, tr("Save Screenshot"), filename, "PNG(*.png)");
    if (filepath.isEmpty()) return;
    if (filepath.endsWith(".png", Qt::CaseInsensitive))
        ui->cloudview->saveScreenshot(filepath.toLocal8Bit());
    else
        ui->cloudview->saveScreenshot(filepath.append(".png").toLocal8Bit());
    ui->console->print(ct::LOG_INFO, tr("The screenshot(%1) save done!").arg(filepath));
}

void MainWindow::moveEvent(QMoveEvent* event)
{
    QPoint pos = ui->cloudview->mapToGlobal(QPoint(0, 0));
    emit ui->cloudview->posChanged(pos);
    return QMainWindow::moveEvent(event);
}

void MainWindow::InitializeCamera() {

    if (camera != nullptr) {
        disconnect(camera.get(), nullptr, this, nullptr);
        camera->reset();
    }

    int camIndex = ui->camComb->currentIndex();
    if (camIndex < 0) {
        ui->console->print(ct::LOG_ERROR, tr("Please ensure the cam type frist!"));
        return;
    }

    switch (camIndex) {
    case 0: camType = CameraFactory::CameraType::MechMind; break;
    case 1: camType = CameraFactory::CameraType::HKRgbd; break;
    case 2: camType = CameraFactory::CameraType::TuYang; break;
    case 3: camType = CameraFactory::CameraType::KWSeries; break;
    default: camType = CameraFactory::CameraType::UNKOWN; break;
    }

    if (camType == CameraFactory::CameraType::UNKOWN) return;
    camera = CameraFactory::createCamera(camType);

    // 相机信号绑定（因为上面已经断开了旧相机的连接，这里直接绑定新相机即可，不会重复）
    connect(this->camera.get(), &ICamera::sendCamModel, this, [=](const QVector<QString>& camModel) {
        if (camModel.size() == 0) {
            ui->console->print(ct::LOG_ERROR, tr("The Cam mode is empty!"));
            return;
        }
        for (auto model : camModel) {
            ui->cam_cbox_device->addItem(model);
        }
        ui->cam_cbox_device->setCurrentIndex(0);
        }, Qt::QueuedConnection);

    connect(this->camera.get(), &ICamera::sendCamInfo, this, [=](const QString& info) {
        if (info.isEmpty()) {
            ui->console->print(ct::LOG_ERROR, tr("The Cam info is empty!"));
            return;
        }
        if (camType == CameraFactory::CameraType::KWSeries) {
            ui->kwcam_txt_info->append(info);
        }
        else {
            ui->cam_txt_info->append(info);
        }
        }, Qt::QueuedConnection);

    connect(this->camera.get(), &ICamera::sendCamConnectStatus, this, [=](const bool& status) {
        if (status == false) {
            if (camType != CameraFactory::CameraType::KWSeries) {
                ui->cam_btn_connect->setIcon(QIcon(":/res/icon/connect.svg"));
                ui->cam_btn_connect->setText("connect");
            }
            else {
                ui->kwcam_btn_connect->setIcon(QIcon(":/res/icon/connect.svg"));
                ui->kwcam_btn_connect->setText("connect");
            }
            ui->camComb->setEnabled(true);
            ui->cam_btn_search->setEnabled(true);
            ui->getCamBtn->setEnabled(true);
            camConnected = false;
        }
        else {
            if (camType != CameraFactory::CameraType::KWSeries) {
                ui->cam_btn_connect->setIcon(QIcon(":/res/icon/disconnect.svg"));
                ui->cam_btn_connect->setText("discon");
            }
            else {
                ui->kwcam_btn_connect->setIcon(QIcon(":/res/icon/disconnect.svg"));
                ui->kwcam_btn_connect->setText("discon");
            }
            ui->camComb->setEnabled(false);
            ui->cam_btn_search->setEnabled(false);
            ui->getCamBtn->setEnabled(false);
            camConnected = true;
        }
        }, Qt::QueuedConnection);

    connect(this->camera.get(), &ICamera::sendCamCloud, this, &MainWindow::GetCamData, Qt::QueuedConnection);

    disconnect(ui->cam_btn_search, &QPushButton::clicked, nullptr, nullptr);
    connect(ui->cam_btn_search, &QPushButton::clicked, this->camera.get(), &ICamera::searchDevice, Qt::QueuedConnection);

    disconnect(ui->cam_btn_connect, &QPushButton::clicked, nullptr, nullptr);
    connect(ui->cam_btn_connect, &QPushButton::clicked, this, [=]() {
        if (this->camera) {
            int index = ui->cam_cbox_device->currentIndex();
            this->camera->connectDevice(index);
        }
        }, Qt::QueuedConnection);

    disconnect(ui->cam_btn_capture, &QPushButton::clicked, nullptr, nullptr);
    connect(ui->cam_btn_capture, &QPushButton::clicked, this->camera.get(), &ICamera::captureDevice, Qt::QueuedConnection);

    disconnect(ui->cam_btn_add, &QPushButton::clicked, nullptr, nullptr);
    connect(ui->cam_btn_add, &QPushButton::clicked, this->camera.get(), &ICamera::add, Qt::QueuedConnection);

    disconnect(ui->kwcam_btn_connect, &QPushButton::clicked, nullptr, nullptr);
    connect(ui->kwcam_btn_connect, &QPushButton::clicked, this, [=]() {
        if (this->camera) {
            std::string camip = ui->kwcam_ip->text().toStdString();
            std::string camconfig = ui->kwCamConfigEdit->text().toStdString();
            this->camera->connectDevice(camip, camconfig);
        }
        }, Qt::QueuedConnection);

    disconnect(ui->kwcam_btn_reset, &QPushButton::clicked, nullptr, nullptr);
    connect(ui->kwcam_btn_reset, &QPushButton::clicked, this->camera.get(), &ICamera::reset, Qt::QueuedConnection);

    disconnect(ui->cam_btn_reset, &QPushButton::clicked, nullptr, nullptr);
    connect(ui->cam_btn_reset, &QPushButton::clicked, this->camera.get(), &ICamera::reset, Qt::QueuedConnection);

    disconnect(ui->kwcam_btn_capture, &QPushButton::clicked, nullptr, nullptr);
    connect(ui->kwcam_btn_capture, &QPushButton::clicked, this->camera.get(), &ICamera::captureDevice, Qt::QueuedConnection);

    disconnect(ui->kwcam_btn_add, &QPushButton::clicked, nullptr, nullptr);
    connect(ui->kwcam_btn_add, &QPushButton::clicked, this->camera.get(), &ICamera::add, Qt::QueuedConnection);

    camera->setCloudTree(ui->cloudtree);
    camera->setCloudView(ui->cloudview);
    camera->setConsole(ui->console);

    ui->cam_cbox_device->clear();
    ui->cam_txt_info->clear();
    ui->kwcam_txt_info->clear();

    ui->console->print(ct::LOG_INFO, tr("The Cam Initialized Done!"));
}

void MainWindow::GetCamData(const CamData& data, const QString& id) {

    if (data.color_Image.empty() || data.depth_Image.empty()) {
        ui->console->print(ct::LOG_ERROR, tr("Cap Image or Depth is empty!"));
        return;
    }

    if (isRunOnce) {
        proworker->setInputSceneData(data.cloud, data.color_Image, data.depth_Image);
        current_cloud_id = id;
        emit triggerVisionPro();
        isRunOnce = false;
    }

    if (ui->saveImageCheck->isChecked()) {
        cv::Mat depth = data.depth_Image.clone();
        cv::Mat color = data.color_Image.clone();
        safeImwrite("./camera_data/capImage/", depth, color);
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
            
            calibImageCount++;
            if (calibImageCount == calib_off.size()) {
                ui->goCapBtn->clicked();
                calibImageCount = 0;
                isAutoCalib = false;
                ui->console->print(ct::LOG_INFO, tr("Auto calib flow done!"));
                return;
            }
            else if (calibImageCount == calib_off.size() - 1) {
                QMetaObject::invokeMethod(this->robot.get(), "movePose", Qt::QueuedConnection, Q_ARG(std::vector<float>, current_endvec),
                    Q_ARG(QString, "calib"));
            }
            else {
                QMetaObject::invokeMethod(this->robot.get(), "movePose", Qt::QueuedConnection, Q_ARG(std::vector<float>, current_offset),
                    Q_ARG(QString, "calib"));
            }
        }
    }
    
    return;
}

void MainWindow::on_actionRunonce_triggered() {
    isRunOnce = true;
    
    if (camConnected) {
        ui->cam_btn_capture->click();
    }
    else if (ui->cloudtree->getSelectedClouds().size() != 0) {
        ui->console->print(ct::LOG_INFO, tr("Current use cloudtree load cloud data to test flow!"));
        std::vector<ct::Cloud::Ptr> selectPcd = ui->cloudtree->getSelectedClouds();
        proworker->setInputSceneData(selectPcd[0], cv::Mat(), cv::Mat());
        emit triggerVisionPro();
    }
    else {
        ui->console->print(ct::LOG_INFO, tr("Current use local data to test flow!"));
        QString pcd_path = QFileDialog::getOpenFileName(this, tr("Open pcd File"), "", tr("pcd File (*.ply);;所有文件 (*)"));
        if (pcd_path.isEmpty()) {
            ui->console->print(ct::LOG_ERROR, tr("No Check valid pcd file!"));
            return;
        }
        ct::Cloud::Ptr cloud_in(new ct::Cloud);
        pcl::io::loadPLYFile(pcd_path.toStdString(), *cloud_in);

		cv::Mat image = cv::imread("./camera_data/1.png", cv::IMREAD_UNCHANGED);
        cv::Mat depth = cv::imread("./camera_data/1.tiff", cv::IMREAD_UNCHANGED);

        proworker->setInputSceneData(cloud_in, image, depth);
        emit triggerVisionPro();
    }
    QMetaObject::invokeMethod(this->robot.get(), "robotStart", Qt::QueuedConnection);
}

void MainWindow::on_actionCycle_triggered() {

}

void MainWindow::on_loadToolBtn_clicked() {
    QString filePath = QFileDialog::getOpenFileName( this,tr("Open Json File"), "./config/toolkit/", tr("Json File (*.Json );;所有文件 (*)"));
    if (!filePath.isEmpty()) {
        ui->lineToolPath->setText(filePath);
    }
    else {
        ui->console->print(ct::LOG_WARNING, tr("Not choose the valid target json file!"));
        return;
    }
    proworker->loadToolkitConfig(filePath.toStdString());
}

void MainWindow::on_loadVisionConfBtn_clicked() {
    QString filePath = QFileDialog::getOpenFileName(this, tr("Open Json File"), "./config/visionConfig/", tr("Json File (*.Json );;所有文件 (*)"));
    if (!filePath.isEmpty()) {
        ui->visionEdit->setText(filePath);
    }
    else {
        ui->console->print(ct::LOG_WARNING, tr("Not choose the valid target json file!"));
        return;
    }
    proworker->loadVisionConfig(filePath.toStdString());
}

void MainWindow::on_loadAutoCalibBtn_clicked() {
    QString filePath = QFileDialog::getOpenFileName(this, tr("Open Json File"), "./config/robot/", tr("Json File (*.Json );;所有文件 (*)"));
    if (!filePath.isEmpty()) {
        ui->autocalibFile->setText(filePath);
    }
    else {
        ui->console->print(ct::LOG_WARNING, tr("Not choose the valid target json file!"));
        return;
    }
    std::ifstream file(filePath.toStdString());
    if (!file.is_open()) {
        ui->console->print(ct::LOG_ERROR, tr("Cant open robot auto calibration json!"));
        return;
    }
    json config;
    file >> config;
    file.close();
    calib_off = config["calib_offsets"].get<std::vector<std::vector<float>>>();
    ui->console->print(ct::LOG_ERROR, QString("robot calib offset pose size is [%1]!").arg(calib_off.size()));
}

void MainWindow::InitializeScrew() {
    if (sc != nullptr) {
        disconnect(sc.get(), nullptr, this, nullptr);
        disconnect(this, nullptr, sc.get(), nullptr);
    }

    sc = std::make_unique<ScrewComm>();
    sc->moveToThread(m_thread_scwork);

    connect(m_thread_scwork, &QThread::finished, this, &MainWindow::deleteLater);

    connect(this, &MainWindow::screwConnect, sc.get(), &ScrewComm::init, Qt::QueuedConnection);
    connect(this, &MainWindow::screwClose, sc.get(), &ScrewComm::close, Qt::QueuedConnection);
    connect(this, &MainWindow::screwStart, sc.get(), &ScrewComm::runStatus, Qt::QueuedConnection);

    disconnect(ui->screwResetBtn, &QPushButton::clicked, nullptr, nullptr);
    connect(ui->screwResetBtn, &QPushButton::clicked, sc.get(), &ScrewComm::reset_stop, Qt::QueuedConnection);

    disconnect(ui->screwBackBtn, &QPushButton::clicked, nullptr, nullptr);
    connect(ui->screwBackBtn, &QPushButton::clicked, sc.get(), &ScrewComm::backlash, Qt::QueuedConnection);

    connect(this->sc.get(), &ScrewComm::screwDone, this->robot.get(), &IRobot::getScrewStatus);
    connect(this->robot.get(), &IRobot::sigScrewMachine, this->sc.get(), &ScrewComm::start);
    //connect(this->robot.get(), &IRobot::screwBack, this->sc.get(), &ScrewComm::backlash);

    connect(sc.get(), &ScrewComm::sendScrewStatus, this, [=](bool status) {
        if (status == false) {
            ui->screwConnectBtn->setIcon(QIcon(":/res/icon/connect.svg"));
        }
        else {
            ui->screwConnectBtn->setIcon(QIcon(":/res/icon/disconnect.svg"));
        }
        screwConnected = status;
        }, Qt::QueuedConnection);

    connect(sc.get(), &ScrewComm::sendRunningStatus, this, [=](bool status) {
        if (status == false) {
            ui->screwStartBtn->setIcon(QIcon(":/res/icon/start_.svg"));
        }
        else {
            ui->screwStartBtn->setIcon(QIcon(":/res/icon/stop_.svg"));
        }
        screwRunning = status;
        }, Qt::QueuedConnection);

    connect(sc.get(), &ScrewComm::sendInfoMsg, this, [=](const QString& msg) {
        ui->console->print(ct::LOG_INFO, msg);
        }, Qt::QueuedConnection);

    connect(sc.get(), &ScrewComm::sendErrorMsg, this, [=](const QString& msg) {
        ui->console->print(ct::LOG_ERROR, msg);
        }, Qt::QueuedConnection);

    m_thread_scwork->start();

    ui->console->print(ct::LOG_INFO, tr("The ScrewMachine Initialized Done!"));
}

void MainWindow::InitializeRobot() {
    if (robot != nullptr) {
        disconnect(robot.get(), nullptr, this, nullptr);
        disconnect(this, nullptr, robot.get(), nullptr);
    }

    int robotIndex = ui->robotComb->currentIndex();
    if (robotIndex < 0) {
        ui->console->print(ct::LOG_ERROR, tr("Please ensure the robot type frist!"));
        return;
    }

    switch (robotIndex) {
    case 0: robotType = RobotFactory::RobotType::DOBOTCR5; break;
    case 1: robotType = RobotFactory::RobotType::JAKAZU12; break;
    /*case 1: robotType = RobotFactory::RobotType::DOBOTCRA10; break;
    case 3: robotType = RobotFactory::RobotType::RokaeCR12; break;*/
    default: robotType = RobotFactory::RobotType::UNKOWN; break;
    }

    if (robotType == RobotFactory::RobotType::UNKOWN) return;

    robot = RobotFactory::createRobot(robotType);
    robot->moveToThread(m_thread_rfwork);

    connect(m_thread_rfwork, &QThread::finished, this, &MainWindow::deleteLater);

    connect(this->robot.get(), &IRobot::sigError, this, [=](const QString& msg) {
        ui->console->print(ct::LOG_ERROR, msg);
        }, Qt::QueuedConnection);

    connect(this->robot.get(), &IRobot::sigRobotStatus, this, [=](const std::vector<float>& Joint, const std::vector<float>& Pose) {
        if (Joint.size() != 6 || Pose.size() != 6) {
            ui->console->print(ct::LOG_ERROR, tr("Receive robot data is wrong!"));
            return;
        }
        ui->j1->setValue(Joint[0]);
        ui->j2->setValue(Joint[1]);
        ui->j3->setValue(Joint[2]);
        ui->j4->setValue(Joint[3]);
        ui->j5->setValue(Joint[4]);
        ui->j6->setValue(Joint[5]);
        ui->robotX->setValue(Pose[0]);
        ui->robotY->setValue(Pose[1]);
        ui->robotZ->setValue(Pose[2]);
        ui->robotRx->setValue(Pose[3]);
        ui->robotRy->setValue(Pose[4]);
        ui->robotRz->setValue(Pose[5]);

        if (this->mdl) {
            std::vector<float> m_joint;
            m_joint.resize(6);
            for (int i = 0; i < 6; ++i) m_joint[i] = Joint[i] * M_PI / 180.0;
            rl::math::Vector j_array = Eigen::Map<const Eigen::VectorXf>(m_joint.data(), m_joint.size()).cast<rl::math::Real>();
            this->viewer->drawConfiguration(j_array);
        }
        }, Qt::QueuedConnection);

    connect(this->robot.get(), &IRobot::sigEnabled, this, [=](bool status) {
        if (status) {
            ui->robotEnableBtn->setIcon(QIcon(":/res/icon/disable.svg"));
        }
        else {
            ui->robotEnableBtn->setIcon(QIcon(":/res/icon/enable.svg"));
        }
        this->robotEnabled = status;
        }, Qt::QueuedConnection);

    connect(this->robot.get(), &IRobot::sigConnected, this, [=](bool status) {
        if (status) {
            ui->robotConBtn->setIcon(QIcon(":/res/icon/disconnect.svg"));
            ui->console->print(ct::LOG_INFO, tr("Robot Connected done!"));
        }
        else {
            ui->robotConBtn->setIcon(QIcon(":/res/icon/connect.svg"));
            ui->console->print(ct::LOG_INFO, tr("Robot disConnected done!"));
        }
        this->robotConnected = status;
        }, Qt::QueuedConnection);

    connect(this->robot.get(), &IRobot::sigPose, this, [=](const std::vector<float>& pose) {
        if (isAutoCalib) {
            const std::string fileName = "./config/Calibration/calibPose.json";
            json root;
            {
                std::ifstream fin(fileName);
                if (fin.is_open()) {
                    try {
                        fin >> root;
                    }
                    catch (...) {
                        root = json::object(); // 文件损坏或为空时重置
                    }
                }
            }
            if (!root.is_object()) {
                root = json::object();
            }
            json record;
            record = { pose[0], pose[1], pose[2], pose[3], pose[4], pose[5] };
            root[(calibImageCount < 10 ? "0" : "") + std::to_string(calibImageCount)] = record;
            std::ofstream fout(fileName);
            if (fout.is_open()) {
                fout << root.dump(4); // 缩进 4 空格，便于阅读
                fout.close();
            }
            this->camera->captureDevice();
        }
        }, Qt::QueuedConnection);

    disconnect(ui->goCapBtn, &QPushButton::clicked, nullptr, nullptr);
    connect(ui->goCapBtn, &QPushButton::clicked, this, [=]() {
        if (!robotEnabled || !robotConnected) return;
        std::ifstream file("./robot_config.json");
        if (!file.is_open()) {
            ui->console->print(ct::LOG_ERROR, tr("Cant open robot_config.json!"));
            return;
        }
        json config;
        file >> config;
        file.close();
        std::vector<float> capVector = config["Cap"].get<std::vector<float>>();
        if (capVector.size() != 6) return;
        this->robot->moveJoint(capVector, "move_cap");
        }, Qt::QueuedConnection);

    disconnect(ui->goHomeBtn, &QPushButton::clicked, nullptr, nullptr);
    connect(ui->goHomeBtn, &QPushButton::clicked, this, [=]() {
        if (!robotEnabled || !robotConnected) return;
        std::ifstream file("./robot_config.json");
        if (!file.is_open()) {
            ui->console->print(ct::LOG_ERROR, tr("Cant open robot_config.json!"));
            return;
        }
        json config;
        file >> config;
        file.close();
        std::vector<float> homeVector = config["Home"].get<std::vector<float>>();
        if (homeVector.size() != 6) return;
        this->robot->moveJoint(homeVector, "move_home");
        }, Qt::QueuedConnection);

    disconnect(ui->setCapBtn, &QPushButton::clicked, nullptr, nullptr);
    connect(ui->setCapBtn, &QPushButton::clicked, this, [=]() {
        if (!robotEnabled || !robotConnected) return;
        json config;
        std::ifstream inFile("robot_config.json");
        if (inFile.is_open()) {
            try {
                inFile >> config;
            }
            catch (json::parse_error& e) {
                ui->console->print(ct::LOG_ERROR, tr("robot_config.json parse error, will overwrite!"));
                config = json::object();
            }
            inFile.close();
        }
        config["Cap"] = { ui->j1->value(), ui->j2->value(), ui->j3->value(), ui->j4->value(), ui->j5->value(), ui->j6->value() };
        std::ofstream outFile("robot_config.json");
        if (!outFile.is_open()) {
            ui->console->print(ct::LOG_ERROR, tr("Cant open robot_config.json!"));
            return;
        }
        outFile << std::setw(4) << config << std::endl;
        outFile.close();
        ui->console->print(ct::LOG_INFO, tr("Cap position robot_config.json save done!"));
        }, Qt::QueuedConnection);

    disconnect(ui->setHomeBtn, &QPushButton::clicked, nullptr, nullptr);
    connect(ui->setHomeBtn, &QPushButton::clicked, this, [=]() {
        if (!robotEnabled || !robotConnected) return;
        json config;
        std::ifstream inFile("robot_config.json");
        if (inFile.is_open()) {
            try {
                inFile >> config;
            }
            catch (json::parse_error& e) {
                ui->console->print(ct::LOG_ERROR, tr("robot_config.json parse error, will overwrite!"));
                config = json::object();
            }
            inFile.close();
        }
        config["Home"] = { ui->j1->value(), ui->j2->value(), ui->j3->value(), ui->j4->value(), ui->j5->value(), ui->j6->value() };
        std::ofstream outFile("robot_config.json");
        if (!outFile.is_open()) {
            ui->console->print(ct::LOG_ERROR, tr("Cant open robot_config.json!"));
            return;
        }
        outFile << std::setw(4) << config << std::endl;
        outFile.close();
        ui->console->print(ct::LOG_INFO, tr("Home position robot_config.json save done!"));
        }, Qt::QueuedConnection);

    disconnect(ui->robotConBtn, &QPushButton::clicked, nullptr, nullptr);
    connect(ui->robotConBtn, &QPushButton::clicked, this, [=]() {
        RobotPara rp;
        rp.IpAddress = ui->RobotCommIp->text().toStdString();
        rp.iPortDashboard = ui->RobotCommPort->text().toShort();
        if (!robotConnected) {
            this->robot->connectRobot(rp);
        }
        else {
            if (robotEnabled) {
                ui->console->print(ct::LOG_WARNING, tr("Please disabled the robot frist!"));
                return;
            }
            this->robot->disconnectRobot();
        }
        }, Qt::QueuedConnection);

    disconnect(ui->robotEnableBtn, &QPushButton::clicked, nullptr, nullptr);
    connect(ui->robotEnableBtn, &QPushButton::clicked, this, [=]() {
        if (!robotConnected) return;
        this->robot->enableRobot(!robotEnabled);
        ui->console->print(ct::LOG_INFO, !robotEnabled ? tr("Robot Enabled done!") : tr("Robot disEnabled done!"));
        }, Qt::QueuedConnection);

    disconnect(ui->robotSpeedBtn, &QPushButton::clicked, nullptr, nullptr);
    connect(ui->robotSpeedBtn, &QPushButton::clicked, this, [=]() {
        if (!robotEnabled || !robotConnected) return;
        this->robot->setSpeedFactor(ui->speedSpin->value());
        }, Qt::QueuedConnection);

    //disconnect(ui->recordRobotBtn, &QPushButton::clicked, nullptr, nullptr);
    //connect(ui->recordRobotBtn, &QPushButton::clicked, this, [=]() {
    //    // 根据当前模式确定文件名与编号引用
    //    const std::string fileName = m_isJointMode ? "./config/robot/calib_joint.json" : "./config/robot/calib_pose.json";
    //    int& index = m_isJointMode ? m_jointIndex : m_poseIndex;
    //    json root;
    //    {
    //        std::ifstream fin(fileName);
    //        if (fin.is_open()) {
    //            try {
    //                fin >> root;
    //            }
    //            catch (...) {
    //                root = json::object(); // 文件损坏或为空时重置
    //            }
    //        }
    //    }
    //    if (!root.is_object()) {
    //        root = json::object();
    //    }
    //    // 2. 组装本次数据
    //    json record;
    //    if (m_isJointMode) {
    //        // 记录 6 个关节角
    //        record = { ui->j1->value(), ui->j2->value(), ui->j3->value(), ui->j4->value(), ui->j5->value(), ui->j6->value() };
    //    }
    //    else {
    //        // 记录法兰位姿
    //        record = { ui->robotX->value(), ui->robotY->value(), ui->robotZ->value(), ui->robotRx->value(), ui->robotRy->value(), ui->robotRz->value() };
    //    }
    //    QMessageBox::StandardButton reply;
    //    reply = QMessageBox::question(this, tr(u8"确认"), tr(u8"是否确认？"), QMessageBox::Yes | QMessageBox::No, QMessageBox::No); // 默认按钮
    //    if (reply != QMessageBox::Yes) {
    //        return;
    //    }
    //    // 3. 编号累加，以编号作为键写入
    //    ++index;
    //    root[std::to_string(index)] = record;
    //    // 4. 写回文件
    //    std::ofstream fout(fileName);
    //    if (fout.is_open()) {
    //        fout << root.dump(4); // 缩进 4 空格，便于阅读
    //        fout.close();
    //    }
    //    }, Qt::QueuedConnection);

    //disconnect(ui->clearRecordBtn, &QPushButton::clicked, nullptr, nullptr);
    //connect(ui->clearRecordBtn, &QPushButton::clicked, this, [this]() {
    //    const std::string fileName = m_isJointMode ? "./config/robot/calib_joint.json" : "./config/robot/calib_pose.json";
    //    int& index = m_isJointMode ? m_jointIndex : m_poseIndex;
    //    QMessageBox::StandardButton reply;
    //    reply = QMessageBox::question(this, tr(u8"确认"), tr(u8"是否确认？"), QMessageBox::Yes | QMessageBox::No, QMessageBox::No); // 默认按钮
    //    if (reply != QMessageBox::Yes) {
    //        return;
    //    }
    //    // 写入空的 json 对象，清空内容
    //    std::ofstream fout(fileName, std::ios::trunc);
    //    if (fout.is_open()) {
    //        fout << nlohmann::json::object().dump(4);
    //        fout.close();
    //    }
    //    index = 0;
    //    }, Qt::QueuedConnection);

    disconnect(ui->loadPlaceConfigBtn, &QPushButton::clicked, nullptr, nullptr);
    connect(ui->loadPlaceConfigBtn, &QPushButton::clicked, this, [=]() {
        QString json_path = QFileDialog::getOpenFileName(this, tr("Open json File"), "./config/place/", "All Formats (*.json)");
        if (json_path.isEmpty()) {
            ui->console->print(ct::LOG_ERROR, "Open place config file failed!");
            return;
        }
        ui->console->print(ct::LOG_INFO, "Open place config file done!");
        ui->placeConfigEdit->setText(json_path);
        this->robot.get()->loadPlaceConfig(json_path.toStdString());
        }, Qt::QueuedConnection);

    connect(this->proworker.get(), &ProcessWorker::sendScrewTransform, this->robot.get(), &IRobot::startScrewSequence, Qt::QueuedConnection);

    connect(this->robot.get(), &IRobot::sigGraspMachine, this, [=](const bool status, const std::string& tag) {
        if (this->gripper == nullptr) {
            ui->console->print(ct::LOG_WARNING, "Gripper tool not initialized!");
            return;
        }
        this->gripper.get()->setSpeed(ui->GripperSpeedSpin->value());
        this->gripper.get()->setPower(ui->GripperTorqueSpin->value());
        if (status) this->gripper.get()->close_gripper(tag);
        else this->gripper.get()->open_gripper(tag);
        }, Qt::QueuedConnection);

    connect(ui->actionStop, &QAction::triggered, this, [this]() {
        if (ui->actionStop->text() == "Start") {
            this->robot->robotStop();
            ui->actionStop->setIcon(QIcon(":/res/icon/stop_.svg"));
            ui->actionStop->setText("Stop");
            ui->console->print(ct::LOG_INFO, "Stop screw flow!");
        }
        else if (ui->actionStop->text() == "Stop") {
            this->robot->robotStart();
            ui->actionStop->setIcon(QIcon(":/res/icon/start_.svg"));
            ui->actionStop->setText("Start");
            ui->console->print(ct::LOG_INFO, "Start screw flow!");
        }
    }, Qt::QueuedConnection);

    m_thread_rfwork->start();

    ui->console->print(ct::LOG_INFO, tr("The Robot Initialized Done!"));
}

void MainWindow::recOffMatrix(const std::vector<Eigen::Affine3f>& before, 
    const std::vector<Eigen::Affine3f>& after, const Eigen::Affine3f& sceneTrans) {
    if (before.size() != after.size()) {
        ui->console->print(ct::LOG_ERROR, tr("Output Pose Size is Wrong!"));
        return;
    }
    
    ui->cloudview->removeAllCoordinateSystems();

    std::vector<ct::Cloud::Ptr> cloud_cluster = ui->cloudtree->getSelectedClouds();
    if (cloud_cluster.size() != 0) {
        current_cloud_id = cloud_cluster[0]->id();
    }
    else {
        if (current_cloud_id == "") {
            ui->console->print(ct::LOG_ERROR, tr("Current Cloud Id is empty!"));
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
}

void MainWindow::closeEvent(QCloseEvent* event)
{
    QMessageBox::StandardButton reply;
    reply = QMessageBox::question(this,
        tr("Exit"),
        tr("Ensure Exit NexusVIT?"),
        QMessageBox::Yes | QMessageBox::No,
        QMessageBox::No); // 默认按钮
    if (reply == QMessageBox::Yes) {
        event->accept(); // 接受关闭事件
    }
    else {
        event->ignore(); // 忽略关闭事件
    }
}

void MainWindow::InitializeGripper() {
    if (gripper != nullptr) {
        disconnect(gripper.get(), nullptr, this, nullptr);
        disconnect(this, nullptr, gripper.get(), nullptr);
    }

    int gripperIndex = ui->gripperComb->currentIndex();
    if (gripperIndex < 0) {
        ui->console->print(ct::LOG_ERROR, tr("Please ensure the gripper type frist!"));
        return;
    }

    switch (gripperIndex) {
    case 0:
        gripperType = GripperFactory::GripperType::YSGripper;
        break;
    case 1:
        gripperType = GripperFactory::GripperType::TekGripper;
        break;
    default:
        gripperType = GripperFactory::GripperType::UNKOWN;
        break;
    }

    if (gripperType == GripperFactory::GripperType::UNKOWN) return;

    gripper = GripperFactory::createGripper(gripperType);
    gripper->moveToThread(m_thread_gripperwork);

    connect(m_thread_gripperwork, &QThread::finished, this, &MainWindow::deleteLater);

    disconnect(ui->connectGripperBtn, &QPushButton::clicked, nullptr, nullptr);
    connect(ui->connectGripperBtn, &QPushButton::clicked, this, [=]() {
        if (gripperConnected)
            this->gripper.get()->disconnect();
        else {
            this->gripper.get()->setPort(ui->ComPortBox->currentText().toStdString());
            this->gripper.get()->connect();
        }
        }, Qt::QueuedConnection);

    disconnect(ui->enableGripperBtn, &QPushButton::clicked, nullptr, nullptr);
    connect(ui->enableGripperBtn, &QPushButton::clicked, this, [=]() {
        if (gripperEnabled)
            this->gripper.get()->disenable();
        else
            this->gripper.get()->enable();
        }, Qt::QueuedConnection);

    disconnect(ui->searchGripperBtn, &QPushButton::clicked, nullptr, nullptr);
    connect(ui->searchGripperBtn, &QPushButton::clicked, this->gripper.get(), &IGripper::search, Qt::QueuedConnection);

    disconnect(ui->openGripperBtn, &QPushButton::clicked, nullptr, nullptr);
    connect(ui->openGripperBtn, &QPushButton::clicked, this, [=]() {
        this->gripper.get()->setSpeed(ui->GripperSpeedSpin->value());
        this->gripper.get()->open_gripper("");
        }, Qt::QueuedConnection);

    disconnect(ui->closeGripperBtn, &QPushButton::clicked, nullptr, nullptr);
    connect(ui->closeGripperBtn, &QPushButton::clicked, this, [=]() {
        this->gripper.get()->setSpeed(ui->GripperSpeedSpin->value());
        this->gripper.get()->setPower(ui->GripperTorqueSpin->value());
        this->gripper.get()->close_gripper("");
        }, Qt::QueuedConnection);

    connect(this->gripper.get(), &IGripper::sendSearchCom, this, [=](const int& com) {
        ui->ComPortBox->setCurrentIndex(com - 1);
        }, Qt::QueuedConnection);

    connect(this->gripper.get(), &IGripper::sendConnectStatus, this, [=](bool status) {
        if (status == false) {
            ui->connectGripperBtn->setIcon(QIcon(":/res/icon/connect.svg"));
        }
        else {
            ui->connectGripperBtn->setIcon(QIcon(":/res/icon/disconnect.svg"));
        }
        gripperConnected = status;
        }, Qt::QueuedConnection);

    connect(this->gripper.get(), &IGripper::sendEnableStatus, this, [=](bool status) {
        if (status == false) {
            ui->enableGripperBtn->setIcon(QIcon(":/res/icon/disenable.svg"));
        }
        else {
            ui->enableGripperBtn->setIcon(QIcon(":/res/icon/enable_on.svg"));
        }
        gripperEnabled = status;
        }, Qt::QueuedConnection);

    connect(this->gripper.get(), &IGripper::sendErrorMsg, this, [=](const QString& msg) {
        ui->console->print(ct::LOG_ERROR, msg);
        }, Qt::QueuedConnection);

    connect(this->gripper.get(), &IGripper::sendInfoMsg, this, [=](const QString& msg) {
        ui->console->print(ct::LOG_INFO, msg);
        }, Qt::QueuedConnection);

   connect(this->gripper.get(), &IGripper::sigOpenDone, this, [=](const std::string& tag) {
        this->robot->moveNextPose(true, QString::fromStdString(tag));
        }, Qt::QueuedConnection);

    connect(this->gripper.get(), &IGripper::sigCloseDone, this, [=](const std::string& tag) {
        this->robot->moveNextPose(true, QString::fromStdString(tag));
        }, Qt::QueuedConnection);

    m_thread_gripperwork->start();

    ui->console->print(ct::LOG_INFO, tr("The Gripper Initialized Done!"));
}

void MainWindow::InitializeComm()
{
    // ── Re-init: disconnect previous UI hooks + tear down active session ───
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
        m_worker_comm = nullptr; // deleted via finished → deleteLater
    }

    if (m_comm_connected)
        setConnected(false);

    // ── Helpers ────────────────────────────────────────────────────────────
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
            cleanupComm();
            return;
        }

        // ── connect ──
        const QString modeText = ui->scoketComb->currentText().trimmed();
        const QString ip = ui->ipaddressEdit->text().trimmed();
        const QString portStr = ui->commPortEdit->text().trimmed();

        const bool isTcpServer = (modeText == QStringLiteral("TCP Server"));
        const bool isUdp = (modeText == QStringLiteral("UDP"));
        // anything else (including "TCP Client") → TCP Client

        if (portStr.isEmpty())
        {
            ui->console->print(ct::LOG_WARNING, "Input Error, Please enter Port.");
            return;
        }

        if (!isTcpServer && ip.isEmpty())
        {
            ui->console->print(ct::LOG_WARNING, "Input Error, Please enter IP and Port.");
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
                ui->console->print(ct::LOG_ERROR, msg);
                });

            connect(m_worker_comm, &SocketWorker::statusMessage, this, [=](const QString& msg) {
                ui->console->print(ct::LOG_INFO, msg);
                });

            connect(m_worker_comm, &SocketWorker::clientAccepted, this, [=](const QString& peer) {
                ui->console->print(ct::LOG_INFO,
                    QStringLiteral("TCP Server accepted client: %1").arg(peer));
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
            ui->console->print(ct::LOG_INFO, readyMsg);
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
            QMessageBox::critical(this, "Connection Failed", QString::fromStdString(e.what()));
        }
        });

    auto sendCurrent = [this]() {
        if (!m_comm_connected || !m_worker_comm)
            return;

        const QString msg = ui->sendEdit->text();
        if (msg.isEmpty())
            return;

        m_worker_comm->sendMsg(msg);
    };

    connect(ui->commSendBtn, &QPushButton::clicked, this, [=]() { sendCurrent(); });
    connect(ui->sendEdit, &QLineEdit::returnPressed, this, [=]() { sendCurrent(); });

    ui->console->print(ct::LOG_INFO, "Socket Communication is ready now!");
}

void MainWindow::setConnected(bool connected)
{
    m_comm_connected = connected;

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

void MainWindow::load(const QString& filename)
{
    QMutexLocker lock(&this->mutex);

    rl::xml::DomParser parser;

    rl::xml::Document document = parser.readFile(filename.toStdString(), "", XML_PARSE_NOENT | XML_PARSE_XINCLUDE);
    document.substitute(XML_PARSE_NOENT | XML_PARSE_XINCLUDE);

    rl::xml::Path path(document);

    this->scene = std::make_shared<rl::sg::ode::Scene>();

    rl::xml::NodeSet modelScene = path.eval("(/rl/plan|/rlplan)//model/scene").getValue<rl::xml::NodeSet>();
    std::string pathModel = modelScene[0].getUri(modelScene[0].getProperty("href"));
    this->scene->load(pathModel);
    this->sceneModel = this->scene->getModel(
        path.eval("number((/rl/plan|/rlplan)//model/model)").getValue<std::size_t>()
    );

    rl::xml::NodeSet mdl = path.eval("(/rl/plan|/rlplan)//model/kinematics").getValue<rl::xml::NodeSet>();
    rl::mdl::XmlFactory factory;
    std::shared_ptr<rl::mdl::Model> model = factory.create(mdl[0].getUri(mdl[0].getProperty("href")));
    this->mdl = std::dynamic_pointer_cast<rl::mdl::Dynamic>(model);

    if (rl::sg::DistanceScene* scene = dynamic_cast<rl::sg::DistanceScene*>(this->scene.get()))
    {
        this->model = std::make_shared<rl::plan::DistanceModel>();
    }
    else if (rl::sg::SimpleScene* scene = dynamic_cast<rl::sg::SimpleScene*>(this->scene.get()))
    {
        this->model = std::make_shared<rl::plan::SimpleModel>();
    }

    if (nullptr != this->kin)
    {
        this->model->kin = this->kin.get();
    }
    else if (nullptr != this->mdl)
    {
        this->model->mdl = this->mdl.get();
    }

    this->model->model = this->sceneModel;
    this->model->scene = this->scene.get();

    this->q = std::make_shared<rl::math::Vector>(this->model->getDofPosition());

    if (nullptr != this->scene2)
    {
        this->viewer->sceneGroup->removeChild(this->scene2->root);
    }

    this->scene2 = std::make_shared<rl::sg::so::Scene>();

    rl::xml::NodeSet viewerScene = path.eval("(/rl/plan|/rlplan)//viewer/model/scene").getValue<rl::xml::NodeSet>();
    this->scene2->load(viewerScene[0].getUri(viewerScene[0].getProperty("href")));
    this->sceneModel2 = static_cast<rl::sg::so::Model*>(this->scene2->getModel(
        path.eval("number((/rl/plan|/rlplan)//viewer/model/model)").getValue<std::size_t>()
    ));

    rl::xml::NodeSet mdl2 = path.eval("(/rl/plan|/rlplan)//viewer/model/kinematics").getValue<rl::xml::NodeSet>();
    std::shared_ptr<rl::mdl::Model> model_ =  factory.create(mdl2[0].getUri(mdl2[0].getProperty("href")));

    this->mdl2 = std::dynamic_pointer_cast<rl::mdl::Dynamic>(model_);

    this->model2 = std::make_shared<rl::plan::Model>();

    if (nullptr != this->kin2)
    {
        this->model2->kin = this->kin2.get();
    }
    else if (nullptr != this->mdl2)
    {
        this->model2->mdl = this->mdl2.get();
    }

    this->model2->model = this->sceneModel2;
    this->model2->scene = this->scene2.get();

    rl::xml::NodeSet start = path.eval("(/rl/plan|/rlplan)//start/q").getValue<rl::xml::NodeSet>();
    this->start = std::make_shared<rl::math::Vector>(start.size());

    for (int i = 0; i < start.size(); ++i)
    {
        (*this->start)(i) = std::atof(start[i].getContent().c_str());

        if ("deg" == start[i].getProperty("unit"))
        {
            (*this->start)(i) *= rl::math::DEG2RAD;
        }
    }

    *this->q = *this->start;

    rl::xml::NodeSet goal = path.eval("(/rl/plan|/rlplan)//goal/q").getValue<rl::xml::NodeSet>();
    this->goal = std::make_shared<rl::math::Vector>(goal.size());

    for (int i = 0; i < goal.size(); ++i)
    {
        (*this->goal)(i) = std::atof(goal[i].getContent().c_str());

        if ("deg" == goal[i].getProperty("unit"))
        {
            (*this->goal)(i) *= rl::math::DEG2RAD;
        }
    }

    if (path.eval("count((/rl/plan|/rlplan)//sigma) > 0").getValue<bool>())
    {
        rl::xml::NodeSet sigma = path.eval("(/rl/plan|/rlplan)//sigma/q").getValue<rl::xml::NodeSet>();
        this->sigma = std::make_shared<rl::math::Vector>(sigma.size());

        for (int i = 0; i < sigma.size(); ++i)
        {
            (*this->sigma)(i) = std::atof(sigma[i].getContent().c_str());

            if ("deg" == sigma[i].getProperty("unit"))
            {
                (*this->sigma)(i) *= rl::math::DEG2RAD;
            }
        }
    }

    this->sampler = std::make_shared<rl::plan::UniformSampler>();
    rl::plan::UniformSampler* uniformSampler = static_cast<rl::plan::UniformSampler*>(this->sampler.get());
    uniformSampler->seed(42);

    if (nullptr != this->sampler)
    {
        this->sampler->model = this->model.get();
    }

    this->sampler2 = std::make_shared<rl::plan::UniformSampler>();
    this->sampler2->model = this->model.get();

    this->verifier = std::make_shared<rl::plan::RecursiveVerifier>();
    this->verifier->delta = path.eval("number((/rl/plan|/rlplan)//recursiveVerifier/delta)").getValue<rl::math::Real>(1);

    if ("deg" == path.eval("string((/rl/plan|/rlplan)//recursiveVerifier/delta/@unit)").getValue<std::string>())
    {
        this->verifier->delta *= rl::math::DEG2RAD;
    }

    if (nullptr != this->verifier)
    {
        this->verifier->model = this->model.get();
    }

    if (path.eval("count((/rl/plan|/rlplan)//simpleOptimizer/recursiveVerifier) > 0").getValue<bool>())
    {
        this->verifier2 = std::make_shared<rl::plan::RecursiveVerifier>();
        this->verifier2->delta = path.eval("number((/rl/plan|/rlplan)//simpleOptimizer/recursiveVerifier/delta)").getValue<rl::math::Real>(1);

        if ("deg" == path.eval("string((/rl/plan|/rlplan)//simpleOptimizer/recursiveVerifier/delta/@unit)").getValue<std::string>())
        {
            this->verifier2->delta *= rl::math::DEG2RAD;
        }
    }
    else if (path.eval("count((/rl/plan|/rlplan)//advancedOptimizer/recursiveVerifier) > 0").getValue<bool>())
    {
        this->verifier2 = std::make_shared<rl::plan::RecursiveVerifier>();
        this->verifier2->delta = path.eval("number((/rl/plan|/rlplan)//advancedOptimizer/recursiveVerifier/delta)").getValue<rl::math::Real>(1);

        if ("deg" == path.eval("string((/rl/plan|/rlplan)//advancedOptimizer/recursiveVerifier/delta/@unit)").getValue<std::string>())
        {
            this->verifier2->delta *= rl::math::DEG2RAD;
        }
    }

    if (nullptr != this->verifier2)
    {
        this->verifier2->model = this->model.get();
    }

    this->optimizer.reset();

    if (path.eval("count((/rl/plan|/rlplan)//simpleOptimizer) > 0").getValue<bool>())
    {
        this->optimizer = std::make_shared<rl::plan::SimpleOptimizer>();
    }
    else if (path.eval("count((/rl/plan|/rlplan)//advancedOptimizer) > 0").getValue<bool>())
    {
        this->optimizer = std::make_shared<rl::plan::AdvancedOptimizer>();
        rl::plan::AdvancedOptimizer* advancedOptimizer = static_cast<rl::plan::AdvancedOptimizer*>(this->optimizer.get());
        advancedOptimizer->length = path.eval("number((/rl/plan|/rlplan)//advancedOptimizer/length)").getValue<rl::math::Real>(1);

        if ("deg" == path.eval("string((/rl/plan|/rlplan)//advancedOptimizer/length/@unit)").getValue<std::string>())
        {
            advancedOptimizer->length *= rl::math::DEG2RAD;
        }

        advancedOptimizer->ratio = path.eval("number((/rl/plan|/rlplan)//advancedOptimizer/ratio)").getValue<rl::math::Real>(0.1f);
    }

    if (nullptr != this->optimizer)
    {
        this->optimizer->model = this->model.get();
        this->optimizer->verifier = this->verifier2.get();
    }

    rl::xml::NodeSet planners = path.eval("(/rl/plan|/rlplan)//addRrtConCon|(/rl/plan|/rlplan)//eet|(/rl/plan|/rlplan)//prm|(/rl/plan|/rlplan)//prmUtilityGuided|(/rl/plan|/rlplan)//rrt|(/rl/plan|/rlplan)//rrtCon|(/rl/plan|/rlplan)//rrtConCon|(/rl/plan|/rlplan)//rrtConExt|(/rl/plan|/rlplan)//rrtDual|(/rl/plan|/rlplan)//rrtGoalBias|(/rl/plan|/rlplan)//rrtExtCon|(/rl/plan|/rlplan)//rrtExtExt").getValue<rl::xml::NodeSet>();

    for (int i = 0; i < std::min(1, planners.size()); ++i)
    {
        rl::xml::Path path(document, planners[i]);

        this->planner = std::make_shared<rl::plan::RrtConCon>();
        rl::plan::RrtConCon* rrtConCon = static_cast<rl::plan::RrtConCon*>(this->planner.get());
        rrtConCon->delta = path.eval("number(delta)").getValue<rl::math::Real>(1);

        if ("deg" == path.eval("string(delta/@unit)").getValue<std::string>())
        {
            rrtConCon->delta *= rl::math::DEG2RAD;
        }

        rrtConCon->epsilon = path.eval("number(epsilon)").getValue<rl::math::Real>(1.0e-3f);

        if ("deg" == path.eval("string(epsilon/@unit)").getValue<std::string>())
        {
            rrtConCon->epsilon *= rl::math::DEG2RAD;
        }

        rrtConCon->sampler = this->sampler.get();
    }

    std::size_t nearestNeighborsSize = 1;

    if (rl::plan::RrtDual* rrtDual = dynamic_cast<rl::plan::RrtDual*>(this->planner.get()))
    {
        nearestNeighborsSize = 2;
    }

    for (::std::size_t i = 0; i < nearestNeighborsSize; ++i)
    {
        std::shared_ptr<rl::plan::NearestNeighbors> nearestNeighbors;

        std::shared_ptr<rl::plan::KdtreeNearestNeighbors> kdtreeNearestNeighbors = std::make_shared<rl::plan::KdtreeNearestNeighbors>(this->model.get());

        if (path.eval("count((/rl/plan|/rlplan)//kdtreeNearestNeighbors/checks) > 0").getValue<bool>())
        {
            kdtreeNearestNeighbors->setChecks(
                path.eval("number((/rl/plan|/rlplan)//kdtreeNearestNeighbors/checks)").getValue<std::size_t>(0)
            );
        }

        if (path.eval("count((/rl/plan|/rlplan)//kdtreeNearestNeighbors/samples) > 0").getValue<bool>())
        {
            kdtreeNearestNeighbors->setSamples(
                path.eval("number((/rl/plan|/rlplan)//kdtreeNearestNeighbors/samples)").getValue<std::size_t>(100)
            );
        }

        nearestNeighbors = kdtreeNearestNeighbors;

        this->nearestNeighbors.push_back(nearestNeighbors);

        if (rl::plan::Prm* prm = dynamic_cast<rl::plan::Prm*>(this->planner.get()))
        {
            prm->setNearestNeighbors(nearestNeighbors.get());
        }
        if (rl::plan::Rrt* rrt = dynamic_cast<rl::plan::Rrt*>(this->planner.get()))
        {
            rrt->setNearestNeighbors(nearestNeighbors.get(), i);
        }
    }

    this->planner->duration = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<float>(
            path.eval("number((/rl/plan|/rlplan)//duration)").getValue<rl::math::Real>(std::numeric_limits<float>::max())
        )
    );

    this->planner->goal = this->goal.get();
    this->planner->model = this->model.get();
    this->planner->start = this->start.get();

    this->viewer->delta = path.eval("number((/rl/plan|/rlplan)//viewer/delta)").getValue<rl::math::Real>();

    if ("deg" == path.eval("string((/rl/plan|/rlplan)//viewer/delta/@unit)").getValue<std::string>())
    {
        this->viewer->delta *= rl::math::DEG2RAD;
    }

    this->viewer->deltaSwept = path.eval("number((/rl/plan|/rlplan)//viewer/swept)").getValue<rl::math::Real>(this->viewer->delta * 100);

    if ("deg" == path.eval("string((/rl/plan|/rlplan)//viewer/swept/@unit)").getValue<std::string>())
    {
        this->viewer->deltaSwept *= rl::math::DEG2RAD;
    }

    this->viewer->sceneGroup->addChild(this->scene2->root);
    this->viewer->model = this->model2.get();

    this->viewer->viewer->setCameraType(SoPerspectiveCamera::getClassTypeId());

    this->viewer->viewer->getCamera()->setToDefaults();

    this->viewer->viewer->viewAll();

    this->viewer->viewer->getCamera()->position.setValue(
        path.eval("number((/rl/plan|/rlplan)//viewer/camera/position/x)").getValue<rl::math::Real>(this->viewer->viewer->getCamera()->position.getValue()[0]),
        path.eval("number((/rl/plan|/rlplan)//viewer/camera/position/y)").getValue<rl::math::Real>(this->viewer->viewer->getCamera()->position.getValue()[1]),
        path.eval("number((/rl/plan|/rlplan)//viewer/camera/position/z)").getValue<rl::math::Real>(this->viewer->viewer->getCamera()->position.getValue()[2])
    );

    this->viewer->viewer->getCamera()->scaleHeight(
        path.eval("number((/rl/plan|/rlplan)//viewer/camera/scale)").getValue<rl::math::Real>(1.0f)
    );

    this->viewer->drawConfiguration(*this->start);

    this->viewer->sceneGroup->addChild(createWorldFrame(0.3f));

    this->planner->viewer = this->thread;

    if (nullptr != this->optimizer)
    {
        this->optimizer->viewer = this->thread;
    }

    for (std::vector<std::shared_ptr<rl::plan::WorkspaceSphereExplorer>>::iterator i = this->explorers.begin(); i != this->explorers.end(); ++i)
    {
        (*i)->viewer = this->thread;
    }

    this->view_connect(this->thread, this->viewer);
}

SoVRMLTransform* MainWindow::createWorldFrame(float length)
{
    SoVRMLIndexedLineSet* axes = new SoVRMLIndexedLineSet();
    axes->colorPerVertex = false;
    SoVRMLColor* color = new SoVRMLColor();
    color->color.set1Value(0, 1.0f, 0.0f, 0.0f); // X 红
    color->color.set1Value(1, 0.0f, 1.0f, 0.0f); // Y 绿
    color->color.set1Value(2, 0.0f, 0.0f, 1.0f); // Z 蓝
    axes->color = color;
    axes->colorIndex.set1Value(0, 0);
    axes->colorIndex.set1Value(1, 1);
    axes->colorIndex.set1Value(2, 2);
    SoVRMLCoordinate* coord = new SoVRMLCoordinate();
    coord->point.set1Value(0, 0.0f, 0.0f, 0.0f);
    coord->point.set1Value(1, length, 0.0f, 0.0f);
    coord->point.set1Value(2, 0.0f, length, 0.0f);
    coord->point.set1Value(3, 0.0f, 0.0f, length);
    axes->coord = coord;
    // 三条线段：原点→X / 原点→Y / 原点→Z
    axes->coordIndex.set1Value(0, 0);
    axes->coordIndex.set1Value(1, 1);
    axes->coordIndex.set1Value(2, -1);
    axes->coordIndex.set1Value(3, 0);
    axes->coordIndex.set1Value(4, 2);
    axes->coordIndex.set1Value(5, -1);
    axes->coordIndex.set1Value(6, 0);
    axes->coordIndex.set1Value(7, 3);
    axes->coordIndex.set1Value(8, -1);
    SoDrawStyle* style = new SoDrawStyle();
    style->lineWidth = 3.0f;
    SoVRMLTransform* world = new SoVRMLTransform();
    world->setName("worldFrame");
    world->translation.setValue(0.0f, 0.0f, 0.0f); // 世界原点
    world->addChild(style);
    world->addChild(axes);
    return world;
}

rl::sg::Model* MainWindow::environmentModel(bool createIfMissing)
{
    if (!this->scene)
    {
        return nullptr;
    }

    for (std::size_t i = 0; i < this->scene->getNumModels(); ++i)
    {
        rl::sg::Model* model = this->scene->getModel(i);
        if (model != this->sceneModel)
        {
            return model;
        }
    }

    if (!createIfMissing)
    {
        return nullptr;
    }

    rl::sg::Model* created = this->scene->create();
    created->setName("dynamic_obstacles");
    ui->console->print(ct::LOG_INFO, tr("Created planner environment model for dynamic WRL obstacles."));
    return created;
}

void MainWindow::syncObstaclesToPlanner()
{
    MovableWrlManager* manager = MovableWrlManager::instance();
    if (!manager)
    {
        return;
    }

    for (MovableWrl* item : manager->items())
    {
        if (item)
        {
            item->syncCollisionBody();
        }
    }
}

void MainWindow::loadSceneWrl() {
    QString wrl_path = QFileDialog::getOpenFileName(this, tr("Open wrl File"), "./scene_model", "All Formats (*.wrl)");
    if (wrl_path.isEmpty()) {
        ui->console->print(ct::LOG_WARNING, tr("No Check valid wrl file!"));
        return;
    }
    QFileInfo fileInfo(wrl_path);

    if (!this->scene || !this->viewer || !this->viewer->sceneGroup || !this->viewer->viewer)
    {
        ui->console->print(ct::LOG_ERROR, tr("Planner scene or viewer is not ready."));
        return;
    }

    this->thread->stop();
    QMutexLocker lock(&this->mutex);

    rl::sg::Model* envModel = environmentModel();
    if (!envModel)
    {
        ui->console->print(ct::LOG_ERROR, tr("No planner environment model available."));
        return;
    }

    obstacle = addMovableWrl(this->viewer->sceneGroup, this->viewer->viewer, wrl_path.toStdString());
    if (!obstacle)
    {
        ui->console->print(ct::LOG_ERROR, tr("Load movable WRL failed."));
        return;
    }

    rl::sg::Body* body = envModel->create();
    body->setName(fileInfo.baseName().toStdString());

    const std::size_t shapeCount = obstacle->addCollisionShapes(body);
    obstacle->bindCollisionBody(body);

    if (this->planner)
    {
        this->planner->reset();
    }

    const std::size_t num = envModel->getNumBodies();
    ui->console->print(ct::LOG_INFO,
        QString("Current Scene body num is %1, collision shapes is %2").arg(num).arg(shapeCount));
    if (shapeCount == 0)
    {
        ui->console->print(ct::LOG_ERROR,
            tr("WRL has no usable collision mesh. Planner will ignore this obstacle."));
    }
}

void MainWindow::removeSceneWrl() {
    this->thread->stop();
    QMutexLocker lock(&this->mutex);
    removeSelectedMovableWrl();
    if (this->planner)
    {
        this->planner->reset();
    }
    rl::sg::Model* envModel = environmentModel(false);
    const std::size_t num = envModel ? envModel->getNumBodies() : 0;
    ui->console->print(ct::LOG_INFO, QString("Current Scene body num is %1").arg(num));
}

void MainWindow::saveSceneWrl() {
    QString filepath = QFileDialog::getSaveFileName(this, tr("Save wrl file"), "", "(*.wrl)");
    if (filepath.isEmpty()) {
        ui->console->print(ct::LOG_WARNING, tr("No Check valid save path!"));
        return;
	}
    saveSelectedMovableWrl(filepath.toStdString());
    ui->console->print(ct::LOG_INFO, QString("Save file %1 done!").arg(filepath));
}

void MainWindow::showSceneCloud(const ct::Cloud::Ptr& scene_cloud) {

    if (scene_cloud->empty()) return;
	double scale = 0.001f; // mm to m
    SoSeparator* pointCloudRoot = new SoSeparator();
    pointCloudRoot->ref();

    SoDrawStyle* drawStyle = new SoDrawStyle();
    drawStyle->pointSize.setValue(2.0f);
    pointCloudRoot->addChild(drawStyle);

    SoVertexProperty* vertexProp = new SoVertexProperty();
    vertexProp->vertex.setNum(scene_cloud->size());
    vertexProp->orderedRGBA.setNum(scene_cloud->size());
    vertexProp->materialBinding = SoVertexProperty::PER_VERTEX;

    for (size_t i = 0; i < scene_cloud->size(); ++i) {
        const auto& p = scene_cloud->points[i];
        vertexProp->vertex.set1Value(i, SbVec3f(p.x * scale, p.y * scale, p.z * scale));
        SbColor color(p.r / 255.0f, p.g / 255.0f, p.b / 255.0f);
        vertexProp->orderedRGBA.set1Value(i, color.getPackedValue());
    }

    SoPointSet* pointSet = new SoPointSet();
    pointSet->vertexProperty = vertexProp;
    pointCloudRoot->addChild(pointSet);

    if (this->viewer == nullptr || this->viewer->sceneGroup == nullptr) {
        return;
    }

    SoGroup* root = this->viewer->sceneGroup;
    const SbName nodeName("scenePcd");

    for (int i = root->getNumChildren() - 1; i >= 0; --i) {
        SoNode* child = root->getChild(i);
        if (child != nullptr && child->getName() == nodeName) {
            root->removeChild(i);
        }
    }

    if (pointCloudRoot == nullptr) {
        return;
    }

    pointCloudRoot->setName(nodeName);
    if (root->findChild(pointCloudRoot) < 0) {
        root->addChild(pointCloudRoot);
    }
}

void MainWindow::safeImwrite(const QString& savePath, const cv::Mat& Depth, const cv::Mat& Color) {
    QString timestamp = QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss");

    QDir dir(QFileInfo(savePath).absolutePath());

    QString finalPath_color = dir.filePath(timestamp + ".png");
    QString finalPath_depth = dir.filePath(timestamp + ".tiff");
    if (!dir.exists()) {
        if (!dir.mkpath(".")) {
            ui->console->print(ct::LOG_ERROR, QStringLiteral("Failed to create output directory."));
            return;
        }
    }

    bool success = cv::imwrite(finalPath_depth.toStdString(), Depth);
    success = cv::imwrite(finalPath_color.toStdString(), Color);
    if (!success) {
        ui->console->print(ct::LOG_ERROR, QStringLiteral("Failed to save image: %1, %2").arg(finalPath_depth).arg(finalPath_color));
    }
    else {
        ui->console->print(ct::LOG_INFO, QStringLiteral("Image saved successfully: %1, %2").arg(finalPath_depth).arg(finalPath_color));
    }
}

void MainWindow::startAutoCalib() {
    QMessageBox::StandardButton reply;
    reply = QMessageBox::question(this,
        QStringLiteral("自动标定"),
        QStringLiteral("清确保相机连接，机械臂连接，标定板放置于相机视野中央!"),
        QMessageBox::Yes | QMessageBox::No,
        QMessageBox::No); // 默认按钮
    if (reply == QMessageBox::Yes) {
        isAutoCalib = true;
        calibImageCount = 0;
        if (!this->camera || !camConnected) {
            ui->console->print(ct::LOG_ERROR, "Camera not connected now!");
            return;
        }
        this->camera->captureDevice();
    }
    else return;
}