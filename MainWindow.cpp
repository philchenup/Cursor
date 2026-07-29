#include "mainwindow.h"

#include "edit/coordinate.h"
#include "edit/cutting.h"
#include "edit/sampling.h"
#include "edit/color.h"

#include "help/about.h"
#include "help/shortcutkey.h"

#include <QDebug>
#include <QDesktopWidget>
#include <QDir>
#include <QFile>
#include <QDateTime>
#include <QFileDialog>
#include <QRadioButton>

#include "base/customdock.h"
#include "base/customdialog.h"

#include "tool/readmodel.h"
#include "tool/occView.h"
#include "tool/TcpCalib.h"
#include "tool/WeldSeamTransition.h"
#include "tool/WeldCoordinateSystem.h"
#include "tool/IKWorker.h"
#include "tool/CamConfig.h"
#include "tool/HandEyeCalib.h"
#include "tool/VisionConfig.h"

#include "utils/processWorker.h"

#include "robot_control/ConfigurationDelegate.h"
#include "robot_control/ConfigurationModel.h"
#include "robot_control/OperationalDelegate.h"
#include "robot_control/OperationalModel.h"

Quantity_Color grayColor(194 / 255.0f, 186 / 255.0f, 181 / 255.0f, Quantity_TOC_RGB);
Quantity_Color kukaColor(1.0, 0.5, 0.0, Quantity_TOC_RGB);
Quantity_Color balckColor(0.2, 0.2, 0.2, Quantity_TOC_RGB);
Quantity_Color silveryColor(0.7725, 0.7843, 0.7843, Quantity_TOC_RGB);
Quantity_Color redColor(0.8196, 0.247, 0.0745, Quantity_TOC_RGB);
Quantity_Color yellowColor(0.7529, 0.74117, 0.3647, Quantity_TOC_RGB);

MainWindow::MainWindow(QWidget* parent)
	: QMainWindow(parent),
	ui(new Ui::MainWindow),
	translator(nullptr),
	myOccView(new OccView(this)),
	configurationDelegates(),
	operationalDelegates(),
	flushSceneTimer(new QTimer),
	flushTrajTimer(new QTimer),
	flushHomeTimer(new QTimer),
	ikwork(new IKWorker),
	m_thread_laserwork(new QThread),
	m_thread_ikwork(new QThread),
	m_thread_prowork(new QThread),
	m_thread_rworker(new QThread)
{
	MainWindow::singleton = this;

	ui->setupUi(this);

	ui->sceneWidget->addWidget(myOccView);
	ui->tabWidget->setCurrentIndex(0);

	connect(this->myOccView, &OccView::sendExtractEdges, this, &MainWindow::flushWeldList, Qt::QueuedConnection);
	ui->weldTableWidget->setSelectionBehavior(QAbstractItemView::SelectRows); 
	ui->weldTableWidget->setEditTriggers(QAbstractItemView::NoEditTriggers); 
	ui->weldTableWidget->setSelectionMode(QAbstractItemView::SingleSelection);
	ui->weldTableWidget->setColumnWidth(0, 100);
	ui->weldTableWidget->setColumnWidth(1, 100);

	connect(ui->actionCoordinate, &QAction::triggered, [=] { this->createDialog<Coordinate>("Coordinate", false); });
	connect(ui->actionCutting, &QAction::triggered, [=] { this->createDialog<Cutting>("Cutting", false); });
	connect(ui->actionSample, &QAction::triggered, [=] { this->createDialog<Sampling>("Sampling", false); });

	connect(ui->actionColor, &QAction::triggered, [=] { this->createLeftDock<Color>("Color"); });

	connect(ui->actionCamParaSet, &QAction::triggered, [=] { this->createDialog<CamConfig>("CamConfig", true); });
	connect(ui->actionTcpCalib, &QAction::triggered, [=] { this->createDialog<TcpCalib>("TcpCalib", true); });
	connect(ui->actionCalibration, &QAction::triggered, [=] { this->createDialog<HandEyeCalib>("HandEyeCalib", true); });
	connect(ui->actionVision, &QAction::triggered, [=] { this->createDialog<VisionConfig>("VisionConfig", true); });

	connect(ui->actionSimulation, &QAction::triggered, this, [=]() {
		if (!isSimulation) {
			if (wholeTrajectory.size() == 0) {
				ui->console->print(ct::LOG_WARNING, "Trajectory points not ready now!");
				return;
			}
			ui->actionSimulation->setIcon(QIcon(":/res/icon/simStop.svg"));
			flushTrajTimer->start(20);
			isSimulation = true;
			m_counter = 0;
		}
		else {
			ui->actionSimulation->setIcon(QIcon(":/res/icon/simulation.svg"));
			flushTrajTimer->stop();
			isSimulation = false;
			m_counter = 0;
		}
	});
	connect(ui->actionHome, &QAction::triggered, this, &MainWindow::robotGoHome, Qt::QueuedConnection);
	connect(ui->actionTrajectory, &QAction::triggered, this, &MainWindow::Trajectory, Qt::QueuedConnection);

	connect(ui->weldTableWidget->selectionModel(), &QItemSelectionModel::selectionChanged, this, &MainWindow::onWeldSelected, Qt::QueuedConnection);
	
	ui->laserTabWidget->setCurrentIndex(0);
	// init qrc
	Q_INIT_RESOURCE(res);

	connect(flushTrajTimer, &QTimer::timeout, this, [=]() { execSimulation(m_counter); m_counter++; });
	
	connect(flushHomeTimer, &QTimer::timeout, this, [=]() { execReturnHome(m_counter_home); m_counter_home++; });

	/*connect(heartMonitorTimer, &QTimer::timeout, this, [=]() { 
		if (robot_heart_beat_num_last == robot_heart_beat_num && robot_heart_beat_num == 0) {
			heart_beat_different_times++;
		}
		else if (robot_heart_beat_num_last == robot_heart_beat_num) {
			heart_beat_different_times++;
		}
		else if (robot_heart_beat_num == robot_heart_beat_num_last + 1) {
			heart_beat_different_times = 0;
			robot_heart_beat_num_last = robot_heart_beat_num;
			robotConnected = true;
			ui->actionRobotStatus->setIcon(QIcon(":/res/icon/robot_con.svg"));
		}
		else if (robot_heart_beat_num == 0 && robot_heart_beat_num_last == 9999) {
			heart_beat_different_times = 0;
			robot_heart_beat_num_last = robot_heart_beat_num;
			robotConnected = true;
			ui->actionRobotStatus->setIcon(QIcon(":/res/icon/robot_con.svg"));
		}
		if (heart_beat_different_times >= 3) {
			robotConnected = false;
			ui->actionRobotStatus->setIcon(QIcon(":/res/icon/robot_dis.svg"));
		}
	}, Qt::QueuedConnection);*/

	// connect pointer
	ui->cloudview->setBackgroundColor({ 127, 127, 127 });
	ui->laser3DWidget->setBackgroundColor({ 20, 20, 20 });
	ui->cloudtree->setCloudView(ui->cloudview);
	ui->cloudtree->setConsole(ui->console);
	ui->cloudtree->setParentIcon(QIcon(":/res/icon/document-open.svg"));
	ui->cloudtree->setChildIcon(QIcon(":/res/icon/view-calendar.svg"));

	ui->camStack->setContentsMargins(0, 3, 0, 0);
	// ui->weldStack->setContentsMargins(0, 3, 0, 0);
	ui->laserStack->setContentsMargins(0, 3, 0, 0);

	this->changeTheme(0);
	std::string result = "HWIWelding软件启动!";
	ui->console->print(ct::LOG_INFO, QString::fromLocal8Bit(result.c_str()));

	connect_init();

	robotWidgetInit();

	initSimulation();
}

MainWindow::~MainWindow() {
	MainWindow::singleton = nullptr;

	camera = nullptr;

	if (m_thread_prowork) {
		m_thread_prowork->quit();
		m_thread_prowork->wait(3000);
		delete m_thread_prowork;
		m_thread_prowork = nullptr;
	}

	if (m_thread_laserwork) {
		m_thread_laserwork->quit();
		m_thread_laserwork->wait(3000);
		delete m_thread_laserwork;
		m_thread_laserwork = nullptr;
	}

	if (ikwork && ikwork->isRunning()) ikwork->requestStop();
	if (m_thread_ikwork) {
		m_thread_ikwork->quit();
		m_thread_ikwork->wait(3000);
		delete m_thread_ikwork;
		m_thread_ikwork = nullptr;
	}
	ikwork = nullptr;

	// 先协议断开 + 关 TCP，再停机器人通信线程
	shutdownRobotComm();

	delete ui;
}

void MainWindow::shutdownRobotComm()
{
	if (m_comm) {
		std::array<double, 7> xp2{};
		{
			QMutexLocker locker(&g_robotMutex);
			for (int i = 0; i < 7; ++i) {
				xp2[i] = g_robotData.joint[i];
			}
		}

		if (m_thread_rworker && m_thread_rworker->isRunning()) {
			// 在工作线程同步执行：IsOut → EKI_Close → disconnectFromHost
			QMetaObject::invokeMethod(m_comm.get(), [comm = m_comm.get(), xp2]() {
				comm->gracefulStop(xp2);
			}, Qt::BlockingQueuedConnection);
		}

		disconnect(m_comm.get(), nullptr, this, nullptr);
		disconnect(this, nullptr, m_comm.get(), nullptr);
		m_comm.release()->deleteLater();
	}

	if (m_thread_rworker) {
		if (m_thread_rworker->isRunning()) {
			m_thread_rworker->quit();
			m_thread_rworker->wait(3000);
		}
		delete m_thread_rworker;
		m_thread_rworker = nullptr;
	}
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
	connect(ui->actionQuit, &QAction::triggered, this, &MainWindow::close);

	// options
	connect(ui->actionDark, &QAction::triggered, [=] { changeTheme(0); });
	connect(ui->actionMacOS, &QAction::triggered, [=] { changeTheme(1); });
	connect(ui->actionUbuntu, &QAction::triggered, [=] { changeTheme(2); });

	connect(ui->actionEnglish, &QAction::triggered, [=] { changeLanguage(0); });
	connect(ui->actionChinese, &QAction::triggered, [=] { changeLanguage(1); });

	// help
	About* about = new About(this);
	ShortcutKey* shortcutKey = new ShortcutKey(this);
	connect(ui->actionAbout, &QAction::triggered, about, &QDialog::show);
	connect(ui->actionShortcutKey, &QAction::triggered, shortcutKey, &QDialog::show);

	// robot control panel
	connect(ui->toolCoordinateBtn, &QRadioButton::toggled, this, &MainWindow::changeRobotCoorBtn);
	connect(ui->userCoordinateBtn, &QRadioButton::toggled, this, &MainWindow::changeRobotCoorBtn);

	connect(ui->actionOpen, &QAction::triggered, this, [this]() {
		if (ui->tabWidget->currentIndex() == 0)
			load3DModel();
		else if (ui->tabWidget->currentIndex() == 1)
			ui->cloudtree->addCloud();
		});
	connect(ui->actionSave, &QAction::triggered, this, [this]() {
		if (ui->tabWidget->currentIndex() == 0)
			saveModel();
		else if (ui->tabWidget->currentIndex() == 1)
			ui->cloudtree->saveSelectedClouds();
		});
	connect(ui->actionRemove, &QAction::triggered, this, [this]() {
		if (ui->tabWidget->currentIndex() == 0)
			removeModel();
		else if (ui->tabWidget->currentIndex() == 1)
			ui->cloudtree->removeSelectedClouds();
		});
	
	connect(ui->actionUpdateScene, &QAction::triggered, this, &MainWindow::actionStart_triggered);

	/*connect(ui->actionOpen, &QAction::triggered, ui->cloudtree, &ct::CloudTree::addCloud);
	connect(ui->actionSave, &QAction::triggered, ui->cloudtree, &ct::CloudTree::saveSelectedClouds);
	connect(ui->actionRemove, &QAction::triggered, ui->cloudtree, &ct::CloudTree::removeSelectedClouds);*/

	connect(ui->calibCheckBtn, &QCheckBox::toggled, [=](bool checked) {
		if (checked) {
			calibNumCount = 0;
		} 
	});

	proworker = std::make_unique<ProcessWorker>();
	proworker->moveToThread(m_thread_prowork);
	connect(m_thread_prowork, &QThread::finished, this, &MainWindow::deleteLater);
	connect(this, &MainWindow::triggerVisionPro, this->proworker.get(), &ProcessWorker::triggerVisionPro);
	connect(this->proworker.get(), &ProcessWorker::sendTargetTransform, this, [=](const Eigen::Affine3f& targetTrans) {
		if (!loadShape) return;
		const Eigen::Matrix4f& mat = targetTrans.matrix();
		gp_Trsf T0;
		T0.SetValues(
			mat(0, 0), mat(0, 1), mat(0, 2), mat(0, 3),
			mat(1, 0), mat(1, 1), mat(1, 2), mat(1, 3),
			mat(2, 0), mat(2, 1), mat(2, 2), mat(2, 3)
		);
		loadShape->SetLocalTransformation(T0);
		myOccView->getContext()->Display(loadShape, Standard_False);
		if (isTheFirstDraw) {
			myOccView->fitAll();
			isTheFirstDraw = false;
		}
		else {
			myOccView->Redraw();
		}
		if (sgShapes.size() < 1) return;
		rl::math::Transform trans;
		trans.matrix() = targetTrans.matrix().cast<double>();
		sgShapes[0]->setTransform(trans);
	});
	connect(this->proworker.get(), &ProcessWorker::sendMsQ, [this](int level, const QString& errorMsg) {
		switch (level)
		{
		case 0:
			ui->console->print(ct::LOG_INFO, errorMsg);
			break;
		case 1:
			ui->console->print(ct::LOG_WARNING, errorMsg);
			break;
		case 2:
			ui->console->print(ct::LOG_ERROR, errorMsg);
			break;
		default:
			break;
		}
	});
	m_thread_prowork->start();
}

void MainWindow::on_InitializeCamera_clicked() {

	if (camera != nullptr) {
		disconnect(camera.get(), nullptr, this, nullptr);
		disconnect(this, nullptr, camera.get(), nullptr);
		camera->reset();
		camera.release()->deleteLater();
	}

	int camIndex = ui->camComb->currentIndex();
	if (camIndex < 0) {
		ui->console->print(ct::LOG_ERROR, tr("Please ensure the cam type frist!"));
		return;
	}

	switch (camIndex) {
	case 0:
		camType = CameraFactory::CameraType::RVS_M;
		break;
	default:
		camType = CameraFactory::CameraType::UNKOWN;
		break;
	}

	if (camType == CameraFactory::CameraType::UNKOWN) return;
	camera = CameraFactory::createCamera(camType);

	// 相机信号绑定（因为上面已经断开了旧相机的连接，这里直接绑定新相机即可，不会重复）
	connect(this->camera.get(), &ICamera::sendCamModel, this, [=](const QString& camModel) {
		if (camModel == "") {
			ui->console->print(ct::LOG_ERROR, tr("The Cam mode is empty!"));
			return;
		}
		ui->cam_cbox_device->addItem(camModel);
		ui->cam_cbox_device->setCurrentIndex(0);
		}, Qt::QueuedConnection);

	connect(this->camera.get(), &ICamera::sendCamInfo, this, [=](const QString& info) {
		if (info.isEmpty()) {
			ui->console->print(ct::LOG_ERROR, tr("The Cam info is empty!"));
			return;
		}
		// ui->cam_txt_info->append(info);
		ui->console->print(ct::LOG_INFO, info);
		}, Qt::QueuedConnection);

	connect(this->camera.get(), &ICamera::sendCamConnectStatus, this, [=](const bool& status) {
		if (status == false) {
			ui->cam_btn_connect->setIcon(QIcon(":/res/icon/connect.svg"));
			ui->cam_btn_connect->setText("connect");
			ui->camComb->setEnabled(true);
			ui->cam_btn_search->setEnabled(true);
			ui->InitializeCamera->setEnabled(true);
			camConnected = false;
		}
		else {
			ui->cam_btn_connect->setIcon(QIcon(":/res/icon/disconnect.svg"));
			ui->cam_btn_connect->setText("discon");
			ui->camComb->setEnabled(false);
			ui->cam_btn_search->setEnabled(false);
			ui->InitializeCamera->setEnabled(false);
			camConnected = true;
		}
		}, Qt::QueuedConnection);

	connect(this->camera.get(), &ICamera::sendCamCloud, this, &MainWindow::GetCamData, Qt::QueuedConnection);

	connect(this->camera.get(), &ICamera::sendCamRunningStatus, this, [=](const bool& status) {
		this->camCaptureRunning = status;
		if (status) {
			ui->cam_btn_capture->setCheckable(true);
			ui->cam_btn_capture->setChecked(true);
			ui->cam_btn_capture->setStyleSheet(R"(
                QPushButton:checked {
                    background-color: #6a6a6a;
                }
            )");
		}
		else {
			ui->cam_btn_capture->setCheckable(false);
			ui->cam_btn_capture->setChecked(false);
		}
		}, Qt::QueuedConnection);

	// 【核心修改 2】：UI 按钮的 Lambda 连接，采用“先断开，再连接”策略
	// 无论 InitializeCamera 被点击多少次，按钮点击事件都只会触发一次

	disconnect(ui->cam_btn_search, &QPushButton::clicked, nullptr, nullptr);
	connect(ui->cam_btn_search, &QPushButton::clicked, this->camera.get(), [&]() {
		if (this->camera) {
			this->camera->searchDevice();
		}
		}, Qt::QueuedConnection);

	disconnect(ui->cam_btn_connect, &QPushButton::clicked, nullptr, nullptr);
	connect(ui->cam_btn_connect, &QPushButton::clicked, this->camera.get(), [&]() {
		if (this->camera) {
			int index = ui->cam_cbox_device->currentIndex();
			this->camera->connectDevice(index);
		}
		}, Qt::QueuedConnection);

	disconnect(ui->cam_btn_capture, &QPushButton::clicked, nullptr, nullptr);
	connect(ui->cam_btn_capture, &QPushButton::clicked, this, [=]() {
		json camConfig;
		std::ifstream file(ui->camJsonEdit->text().toStdString().c_str());
		try {
			file >> camConfig;
			file.close();
		}
		catch (const std::exception& e) {
			file.close();
			ui->console->print(ct::LOG_ERROR, tr("Camera config file open failed, %1!").arg(e.what()));
			return;
		}
		file.close();
		if (this->camCaptureRunning == false) {
			// 硬触发模式
			if (camConfig["trigger_mode"] == 1 && camConfig["capture_mode"] == 64) {
				this->camera->captureFixedLineScan(ui->camJsonEdit->text().toStdString());
			}
			else { // 面阵或摆动线扫
				this->camera->captureDevice(ui->camJsonEdit->text().toStdString());
			}
		}
		else {
			if (camConfig["trigger_mode"] == 1 && camConfig["capture_mode"] == 64) {
				this->camera->stopFixedLineScan();
			}
		}
		}, Qt::QueuedConnection);

	disconnect(ui->cam_btn_add, &QPushButton::clicked, nullptr, nullptr);
	connect(ui->cam_btn_add, &QPushButton::clicked, this->camera.get(), &ICamera::add, Qt::QueuedConnection);

	/*disconnect(ui->cam_btn_reset, &QPushButton::clicked, nullptr, nullptr);
	connect(ui->cam_btn_reset, &QPushButton::clicked, this->camera.get(), &ICamera::reset, Qt::QueuedConnection);*/

	disconnect(ui->loadCamJsonBtn, &QPushButton::clicked, nullptr, nullptr);
	connect(ui->loadCamJsonBtn, &QPushButton::clicked, this, [=]() {
		QString filePath = QFileDialog::getOpenFileName(this, tr("Open Json File"), "./config/cam/", tr("Json File (*.Json );;所有文件 (*)"));
		if (!filePath.isEmpty()) {
			ui->camJsonEdit->setText(filePath);
		}
		else {
			ui->console->print(ct::LOG_WARNING, tr("Not choose the valid target json file!"));
			return;
		}
		}, Qt::QueuedConnection);

	this->camera->setCloudTree(ui->cloudtree);
	this->camera->setCloudView(ui->cloudview);
	this->camera->setConsole(ui->console);

	ui->cam_cbox_device->clear();
	// ui->cam_txt_info->clear();

	ui->console->print(ct::LOG_INFO, tr("The Cam Initialized Done!"));
}

void MainWindow::on_InitializeLaser_clicked() {

	if (laser != nullptr) {
		disconnect(laser.get(), nullptr, this, nullptr);
		disconnect(this, nullptr, laser.get(), nullptr);

		laser.release()->deleteLater();
	}

	if (m_thread_laserwork->isRunning()) {
		m_thread_laserwork->quit();
		m_thread_laserwork->wait();  
	}

	int laserIndex = ui->laserComb->currentIndex();
	if (laserIndex < 0) {
		ui->console->print(ct::LOG_ERROR, tr("Please ensure the laser type frist!"));
		return;
	}

	switch (laserIndex) {
	case 0:
		laserType = LaserFactory::LaserType::ILaser_PF400;
		break;
	default:
		laserType = LaserFactory::LaserType::UNKOWN;
		break;
	}

	if (laserType == LaserFactory::LaserType::UNKOWN) return;
	laser = LaserFactory::createLaser(laserType);

	laser->moveToThread(m_thread_laserwork);
	// connect(m_thread_laserwork, &QThread::finished, this->laser.get(), &QObject::deleteLater);

	disconnect(ui->laserSearchBtn, &QPushButton::clicked, nullptr, nullptr);
	connect(ui->laserSearchBtn, &QPushButton::clicked, this->laser.get(), &ILaser::searchDevice, Qt::QueuedConnection);

	disconnect(ui->connectLaserBtn, &QPushButton::clicked, nullptr, nullptr);
	connect(ui->connectLaserBtn, &QPushButton::clicked, this->laser.get(), [&]() {
		if (this->laser) {
			this->laser->connectDevice(ui->laserIpComb->currentText().toStdString());
		}
		}, Qt::QueuedConnection);

	disconnect(ui->openLaserBtn, &QPushButton::clicked, nullptr, nullptr);
	connect(ui->openLaserBtn, &QPushButton::clicked, this->laser.get(), [&]() {
		if (this->laser) {
			this->laser->openLaser(ui->openLaserBtn->text().toStdString() == "Laser");
		}
		}, Qt::QueuedConnection);

	disconnect(ui->openCamBtn, &QPushButton::clicked, nullptr, nullptr);
	connect(ui->openCamBtn, &QPushButton::clicked, this->laser.get(), [&]() {
		if (this->laser) {
			this->laser->openSensor(ui->openCamBtn->text().toStdString() == "Cam");
		}
		}, Qt::QueuedConnection);

	disconnect(ui->capDataBtn, &QPushButton::clicked, nullptr, nullptr);
	connect(ui->capDataBtn, &QPushButton::clicked, this->laser.get(), [&]() {
		if (this->laser) {
			this->laser->captureDevice(ui->capDataBtn->text().toStdString() == "Capture");
		}
		}, Qt::QueuedConnection);

	connect(this->laser.get(), &ILaser::sendConStatus, this, [&](const bool& m_enable) {
		if (!m_enable) {
			ui->connectLaserBtn->setIcon(QIcon(":/res/icon/connect.svg"));
			ui->connectLaserBtn->setText("connect");
			ui->laserComb->setEnabled(true);
			ui->InitializeLaser->setEnabled(true);
			ui->laserSearchBtn->setEnabled(true);
			laserConnected = false;
		}
		else {
			ui->connectLaserBtn->setIcon(QIcon(":/res/icon/disconnect.svg"));
			ui->connectLaserBtn->setText("discon");
			ui->laserComb->setEnabled(false);
			ui->InitializeLaser->setEnabled(false);
			ui->laserSearchBtn->setEnabled(false);
			laserConnected = true;
		}
		}, Qt::QueuedConnection);

	connect(this->laser.get(), &ILaser::sendLaserStatus, this, [&](const bool& m_enable) {
		if (!m_enable) {
			ui->openLaserBtn->setIcon(QIcon(":/res/icon/start_.svg"));
			ui->openLaserBtn->setText("Laser");
		}
		else {
			ui->openLaserBtn->setIcon(QIcon(":/res/icon/stop_1.svg"));
			ui->openLaserBtn->setText("stop");
		}
		}, Qt::QueuedConnection);

	connect(this->laser.get(), &ILaser::sendSensorStatus, this, [&](const bool& m_enable) {
		if (!m_enable) {
			ui->openCamBtn->setIcon(QIcon(":/res/icon/start_.svg"));
			ui->openCamBtn->setText("Cam");
		}
		else {
			ui->openCamBtn->setIcon(QIcon(":/res/icon/stop_1.svg"));
			ui->openCamBtn->setText("stop");
		}
		}, Qt::QueuedConnection);

	connect(this->laser.get(), &ILaser::sendCaptureStatus, this, [&](const bool& m_enable) {
		if (!m_enable) {
			ui->capDataBtn->setIcon(QIcon(":/res/icon/start_.svg"));
			ui->capDataBtn->setText("Capture");
		}
		else {
			ui->capDataBtn->setIcon(QIcon(":/res/icon/stop_1.svg"));
			ui->capDataBtn->setText("stop");
		}
		}, Qt::QueuedConnection);

	connect(this->laser.get(), &ILaser::sendMsg, this, [&](const int& level, const QString& msg) {
		switch (level) {
		case 0:
			ui->console->print(ct::LOG_INFO, msg);
			break;
		case 1:
			ui->console->print(ct::LOG_WARNING, msg);
			break;
		case 2:
			ui->console->print(ct::LOG_ERROR, msg);
			break;
		default:
			break;
		}
		}, Qt::QueuedConnection);

	connect(this->laser.get(), &ILaser::sendLaserIp, this, [&](const QString& camIp) {
		ui->laserIpComb->clear();
		ui->laserIpComb->addItem(camIp);
		}, Qt::QueuedConnection);

	connect(this->laser.get(), &ILaser::sendCaptureImage, this, [=](const cv::Mat& image) {
		cv::Mat rgbMat;
		cv::cvtColor(image, rgbMat, cv::COLOR_BGR2RGB);
		QImage qImage(rgbMat.data, rgbMat.cols, rgbMat.rows, rgbMat.step, QImage::Format_RGB888);
		QPixmap pixmap = QPixmap::fromImage(qImage);
		QPixmap scaledPixmap = pixmap.scaled(ui->CaptureImageLabel->size() - QSize(2, 2), Qt::KeepAspectRatio, Qt::SmoothTransformation);
		ui->CaptureImageLabel->setPixmap(scaledPixmap);
		ui->CaptureImageLabel->setAlignment(Qt::AlignCenter);
		}, Qt::QueuedConnection);

	connect(this->laser.get(), &ILaser::sendMonitorImage, this, [=](const cv::Mat& image) {
		cv::Mat rgbMat;
		cv::cvtColor(image, rgbMat, cv::COLOR_BGR2RGB);
		QImage qImage(rgbMat.data, rgbMat.cols, rgbMat.rows, rgbMat.step, QImage::Format_RGB888);
		QPixmap pixmap = QPixmap::fromImage(qImage);
		QPixmap scaledPixmap = pixmap.scaled(ui->MonitorImageLabel->size() - QSize(2, 2), Qt::KeepAspectRatio, Qt::SmoothTransformation);
		ui->MonitorImageLabel->setPixmap(scaledPixmap);
		ui->MonitorImageLabel->setAlignment(Qt::AlignCenter);
		}, Qt::QueuedConnection);

	connect(this->laser.get(), &ILaser::send3DContour, this, [=](const ct::Cloud::Ptr& contour) {
		ui->laser3DWidget->removeAllCoordinateSystems();
		ui->laser3DWidget->removeAllPointClouds();
		ui->laser3DWidget->addPointCloud(contour);
		ui->laser3DWidget->addCoordinateSystem(ct::Coord("cloud", 100.0, Eigen::Affine3f::Identity()));
		ui->laser3DWidget->update();


		// 91.0675,-32.789,-157.778,1.12473,-1.59023,-1.33706
		Eigen::Vector3f translation(91.0675,-32.789,-157.778);
		Eigen::Vector3f rot_vec(1.12473,-1.59023,-1.33706);
		Eigen::Affine3f pose = Eigen::Translation3f(translation) * Eigen::AngleAxisf(rot_vec.norm(), rot_vec.normalized());
		ui->laser3DWidget->setViewerPose(pose);

		// ui->laser3DWidget->resetCameraViewpoint();
		/*Eigen::Affine3f pose = ui->laser3DWidget->getm_viewerPose();
		Eigen::Vector3f trans = pose.translation();
		Eigen::AngleAxisf rotation_vector(pose.rotation());
		Eigen::Vector3f axis = rotation_vector.axis();
		float angle = rotation_vector.angle();
		Eigen::Vector3f rot_vec = axis * angle;
		ui->console->print(ct::LOG_ERROR, QString("%1,%2,%3,%4,%5,%6").arg(trans.x()).arg(trans.y()).arg(trans.z()).arg(rot_vec.x()).arg(rot_vec.y()).arg(rot_vec.z()));*/
		
		}, Qt::QueuedConnection);

	m_thread_laserwork->start();

	ui->laserIpComb->clear();

	ui->console->print(ct::LOG_INFO, tr("The Laser Sensor Initialized Done!"));
}

void MainWindow::on_InitializeRobot_clicked()
{
	if (m_comm != nullptr) {
		disconnect(m_comm.get(), nullptr, this, nullptr);
		disconnect(this, nullptr, m_comm.get(), nullptr);
		m_comm.release()->deleteLater();
	}

	if (m_thread_rworker->isRunning()) {
		m_thread_rworker->quit();
		m_thread_rworker->wait();
	}

	m_comm = std::make_unique<KukaCommunicator>();
	m_comm->moveToThread(m_thread_rworker);

	connect(m_comm.get(), &KukaCommunicator::robotDataReceived, this, [=](const RobotData& data) {
		QMutexLocker locker(&g_robotMutex);
		g_robotData = data;
		rl::math::Vector rlVec = this->mdl->getHomePosition();
		for (size_t i = 1; i < 7; ++i) {
			rlVec[i] = g_robotData.joint[i] / 180.0 * M_PI;
		}
		rlVec[0] = g_robotData.joint[0];
		this->configurationModel->setData(rlVec);
		}, Qt::QueuedConnection);

	connect(m_comm.get(), &KukaCommunicator::statusMessage, this, [=](const QString& msg) {
		ui->console->print(ct::LOG_INFO, msg);
		}, Qt::QueuedConnection);

	connect(m_comm.get(), &KukaCommunicator::clientConnected, this, [=](bool connected) {
		if (connected) {
			ui->robotConnectBtn->setIcon(QIcon(":/res/icon/disconnect.svg"));
			ui->robotConnectBtn->setText("discon");
			ui->console->print(ct::LOG_INFO, "robot connect now!");
			ui->actionRobotStatus->setIcon(QIcon(":/res/icon/robot_con.svg"));
			ui->realRobotCheck->setChecked(true);
			ui->virtualRobotCheck->setChecked(false);
		}
		else {
			ui->robotConnectBtn->setIcon(QIcon(":/res/icon/connect.svg"));
			ui->robotConnectBtn->setText("connect");
			ui->console->print(ct::LOG_INFO, "robot disconnect done!");
			ui->actionRobotStatus->setIcon(QIcon(":/res/icon/robot_dis.svg"));
			ui->realRobotCheck->setChecked(false);
			ui->virtualRobotCheck->setChecked(true);
		}}, Qt::QueuedConnection);

	disconnect(ui->robotConnectBtn, &QPushButton::clicked, nullptr, nullptr);
	connect(ui->robotConnectBtn, &QPushButton::clicked, this, [=]() {
		KukaCommunicator* comm = m_comm.get();
		if (!comm) return;

		if (ui->robotConnectBtn->text() == QLatin1String("discon")) {
			std::array<double, 7> xp2{};
			{
				QMutexLocker locker(&g_robotMutex);
				for (int i = 0; i < 7; ++i) {
					xp2[i] = g_robotData.joint[i];
				}
			}
			// 优雅断开：IsOut=TRUE → 等待机器人 EKI_Close → 再关 TCP
			QMetaObject::invokeMethod(comm, [comm, xp2]() {
				comm->gracefulStop(xp2);
			}, Qt::QueuedConnection);
			return;
		}

		const QString ip = ui->robotIpEdit->text().trimmed();
		const quint16 port = static_cast<quint16>(ui->robotPortEdit->text().toUInt());
		if (ip.isEmpty() || port == 0) {
			ui->console->print(ct::LOG_WARNING, "invalid robot ip/port");
			return;
		}

		QMetaObject::invokeMethod(comm, [comm, ip, port]() {
			comm->start(ip, port);
			}, Qt::QueuedConnection);
		});

	struct JointBtn { QPushButton* btn; int axis; int dir; };
	const JointBtn items[] = {
		{ ui->E1SubBtn, 0, -1 }, { ui->E1AddBtn, 0, +1 },
		{ ui->J1SubBtn, 1, -1 }, { ui->J1AddBtn, 1, +1 },
		{ ui->J2SubBtn, 2, -1 }, { ui->J2AddBtn, 2, +1 },
		{ ui->J3SubBtn, 3, -1 }, { ui->J3AddBtn, 3, +1 },
		{ ui->J4SubBtn, 4, -1 }, { ui->J4AddBtn, 4, +1 },
		{ ui->J5SubBtn, 5, -1 }, { ui->J5AddBtn, 5, +1 },
		{ ui->J6SubBtn, 6, -1 }, { ui->J6AddBtn, 6, +1 },
	};
	auto stepJoint = [this](int axis, int dir) {
		KukaCommunicator* comm = m_comm.get();
		if (!comm) return;
		std::array<double, 7> xp2{};
		{
			QMutexLocker locker(&g_robotMutex);
			g_robotData.joint[axis] += dir * ui->rotateStepSpinbox->value();
			for (int i = 0; i < 7; ++i) xp2[i] = g_robotData.joint[i];
		}
		QMetaObject::invokeMethod(comm, [comm, xp2]() {
			comm->stepMove(1, xp2);
		}, Qt::QueuedConnection);
	};
	for (const auto& it : items) {
		disconnect(it.btn, &QPushButton::clicked, nullptr, nullptr);
		connect(it.btn, &QPushButton::clicked, this, [=]() { stepJoint(it.axis, it.dir); });
	}

	struct CartBtn { QPushButton* btn; int axis; int dir; };
	const CartBtn cartItems[] = {
		{ ui->XSubBtn, 0, -1 }, { ui->XAddBtn, 0, +1 },
		{ ui->YSubBtn, 1, -1 }, { ui->YAddBtn, 1, +1 },
		{ ui->ZSubBtn, 2, -1 }, { ui->ZAddBtn, 2, +1 },
		{ ui->ASubBtn, 3, -1 }, { ui->AAddBtn, 3, +1 },
		{ ui->BSubBtn, 4, -1 }, { ui->BAddBtn, 4, +1 },
		{ ui->CSubBtn, 5, -1 }, { ui->CAddBtn, 5, +1 },
	};
	auto stepCart = [this](int axis, int dir) {
		KukaCommunicator* comm = m_comm.get();
		if (!comm) return;
		std::array<double, 7> xp2{};
		{
			QMutexLocker locker(&g_robotMutex);
			g_robotData.pose[axis] += dir * ui->transStepSpinbox->value();
			for (int i = 0; i < 6; ++i) xp2[i] = g_robotData.pose[i];
			xp2[6] = 0;
		}
		QMetaObject::invokeMethod(comm, [comm, xp2]() {
			comm->stepMove(2, xp2);
			}, Qt::QueuedConnection);
	};
	for (const auto& it : cartItems) {
		disconnect(it.btn, &QPushButton::clicked, nullptr, nullptr);
		connect(it.btn, &QPushButton::clicked, this, [=]() { stepCart(it.axis, it.dir); });
	}

	connect(this, &MainWindow::sendTrajectory, this->m_comm.get(), [=](const rl::math::Vector& cur_joint) {
		KukaCommunicator* comm = m_comm.get();
		if (!comm) return;
		std::array<double, 7> xp2{};
		{
			for (int i = 0; i < 7; ++i) xp2[i] = (i > 1 ? cur_joint[i] / M_PI * 180.0 : cur_joint[i]);
		}
		QMetaObject::invokeMethod(comm, [comm, xp2]() {
			comm->stepMove(2, xp2);
			});
	}, Qt::QueuedConnection);

	m_thread_rworker->start();
	ui->console->print(ct::LOG_INFO, "Robot Initialized done! (client mode: connect to robot server)");
}

void MainWindow::GetCamData(const CamData& data, const QString& id) {
	if (isRunOnce) {
		isRunOnce = false;
		try
		{
			proworker->setInputSceneData(data.cloud);
			current_cloud_id = id;
			emit triggerVisionPro();
		}
		catch (const std::exception& e)
		{
			ui->console->print(ct::LOG_ERROR, QString("pcd process flow error happened! %1").arg(e.what()));
		}
		return;
	}

	if (ui->calibCheckBtn->isChecked()) { // 记录图片和机械臂位姿数据
		std::string savePath;
		if (calibNumCount < 10) {
			savePath = "./config/Calibration/CalibImage/color_0" + std::to_string(calibNumCount) + ".bmp";
		}
		else {
			savePath = "./config/Calibration/CalibImage/color_" + std::to_string(calibNumCount) + ".bmp";
		}

		safeImwrite(QString::fromStdString(savePath), data.color_Image);

		// cv::imwrite(savePath, data.color_Image);
		const std::string fileName = "./config/Calibration/calibPose.json";
		json root;
		{
			std::ifstream fin(fileName);
			if (fin.is_open()) {
				try {
					fin >> root;
					fin.close();
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

		QMutexLocker locker(&g_robotMutex);
		record = { g_robotData.pose[0], g_robotData.pose[1], g_robotData.pose[2], g_robotData.pose[3], g_robotData.pose[4], g_robotData.pose[5] };
		root[(calibNumCount < 10 ? "0" : "") + std::to_string(calibNumCount)] = record;
		std::ofstream fout(fileName);
		if (fout.is_open()) {
			fout << root.dump(4); // 缩进 4 空格，便于阅读
			fout.close();
		}
		if (calibNumCount == 17) {
			ui->console->print(ct::LOG_WARNING, "Current already capture 17 times data!");
			calibNumCount = 0;
			return;
		}
	}

	return;
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
		qss.setFileName(":/res/theme/MacOS.qss");
		qss.open(QFile::ReadOnly);
		qApp->setStyleSheet(qss.readAll());
		qss.close();
		ui->statusBar->showMessage(tr("MacOS Theme"), 2000);
		break;
	case 2:
		qss.setFileName(":/res/theme/Ubuntu.qss");
		qss.open(QFile::ReadOnly);
		qApp->setStyleSheet(qss.readAll());
		qss.close();
		ui->statusBar->showMessage(tr("Ubuntu Theme"), 2000);
		break;
	}
}

void MainWindow::changeLanguage(int index)
{
	switch (index)
	{
	case 0:
		if (translator != nullptr) {
			qApp->removeTranslator(translator);
			ui->retranslateUi(this);
		}
		break;
	case 1:
		if (translator == nullptr) {
			translator = new QTranslator;
			translator->load(":/res/trans/zh_CN.qm");
		}
		qApp->installTranslator(translator);
		ui->retranslateUi(this);
		break;
	}
}

void MainWindow::changeRobotCoorBtn(bool checked) {
	if (!checked) return;

	QRadioButton* senderBtn = qobject_cast<QRadioButton*>(sender());
	if (!senderBtn) return;

	int index = senderBtn == ui->toolCoordinateBtn ? 1 : 2;
	std::string result;
	switch (index) {
	case 1:
		result = "切换工具坐标系.";
		ui->console->print(ct::LOG_INFO, QString::fromLocal8Bit(result.c_str()));
		break;
	case 2:
		result = "切换用户坐标系.";
		ui->console->print(ct::LOG_INFO, QString::fromLocal8Bit(result.c_str()));
		break;
	default:
		break;
	}
}

void MainWindow::closeEvent(QCloseEvent* event)
{
	QMessageBox::StandardButton reply;
	reply = QMessageBox::question(this,
		tr(u8"退出程序"),
		tr(u8"确定要退出程序吗？"),
		QMessageBox::Yes | QMessageBox::No,
		QMessageBox::No);
	if (reply == QMessageBox::Yes) {
		shutdownRobotComm();
		event->accept();
	}
	else {
		event->ignore();
	}
}

void MainWindow::on_loadVisionConfigBtn_clicked() {
	QString filePath = QFileDialog::getOpenFileName(this, tr("Open Json File"), "./config/visionConfig/", tr("Json File (*.Json );;所有文件 (*)"));
	if (!filePath.isEmpty()) {
		ui->visionConfigEdit->setText(filePath);
	}
	else {
		ui->console->print(ct::LOG_WARNING, tr("Not choose the valid target json file!"));
		return;
	}
	proworker->loadVisionConfig(filePath.toStdString());
}

void MainWindow::actionStart_triggered() {
	isRunOnce = true;
	if (camConnected) {
		ui->cam_btn_capture->click();
	}
	else if (ui->cloudtree->getSelectedClouds().size() != 0) {
		ui->console->print(ct::LOG_INFO, tr("Current use cloudtree load cloud data to test flow!"));
		std::vector<ct::Cloud::Ptr> selectPcd = ui->cloudtree->getSelectedClouds();
		proworker->setInputSceneData(selectPcd[0]);
		emit triggerVisionPro();
		isRunOnce = false;
	}
	else {
		ui->console->print(ct::LOG_INFO, tr("Current use local data to test flow!"));
		QString pcd_path = QFileDialog::getOpenFileName(this, tr("Open pcd File"), "", tr("pcd File (*.ply);;所有文件 (*)"));
		ct::Cloud::Ptr cloud_in(new ct::Cloud);
		pcl::io::loadPLYFile(pcd_path.toStdString(), *cloud_in);
		proworker->setInputSceneData(cloud_in);
		emit triggerVisionPro();
		isRunOnce = false;
	}
}

/*occt 可视化*/
void MainWindow::loadOcctModel(std::string type) {
	//return;
	links.clear();

	Interface_Static::SetIVal("read.step.product.mode", 0);  // 禁用产品结构严格检查
	Interface_Static::SetIVal("read.step.sequence.mode", 0); // 禁用序列化检查

	Interface_Static::SetIVal("read.step.memory.optimize", 1);  // 启用内存优化
	Interface_Static::SetIVal("read.step.progressive.mode", 1); // 启用渐进加载
	AIS_Shape* myais_shape;

	for (int i = 0; i < 4; ++i) {
		QString robot_file = QString::fromStdString("./robot_env/" + type + "/step/base" + std::to_string(i) + ".stl");
		TopoDS_Shape link = ReadModel::readStlModel(robot_file.toStdString().c_str());
		links.append(link);
		myais_shape = new AIS_Shape(link);
		displinks.append(myais_shape);
	}

	for (int i = 0; i < 9; i++)
	{
		QString linkPath = QString::fromStdString("./robot_env/" + type +  "/step/link" + std::to_string(i) + ".stl");
		TopoDS_Shape link = ReadModel::readStlModel(linkPath.toStdString().c_str());
		links.append(link);
		myais_shape = new AIS_Shape(link);
		displinks.append(myais_shape);
	}

	occtUpdate();
}

void MainWindow::occtUpdate() {
	//return;
	if (rl::mdl::Kinematic* kinematic = dynamic_cast<rl::mdl::Kinematic*>(this->mdl.get())) {
		kinematic->forwardPosition();
		for (std::size_t i = 0; i < this->sceneModel->getNumBodies(); ++i)
		{
			rl::math::Transform tf = kinematic->getBodyFrame(i);
			// this->sceneModel->getBody(i)->setFrame(kinematic->getFrame(i));

			gp_Trsf T0;
			T0.SetValues(tf(0, 0), tf(0, 1), tf(0, 2), tf(0, 3),
				tf(1, 0), tf(1, 1), tf(1, 2), tf(1, 3),
				tf(2, 0), tf(2, 1), tf(2, 2), tf(2, 3));

			displinks[i]->SetLocalTransformation(T0);

			if (i == 0) displinks[i]->SetColor(redColor);
			else if (i == 1 || i == 2 || i == 3) displinks[i]->SetColor(yellowColor);
			else if (i == 11) displinks[i]->SetColor(silveryColor);
			else if (i == 12) displinks[i]->SetColor(balckColor);
			else  displinks[i]->SetColor(kukaColor);

			myOccView->getContext()->Display(displinks[i], Standard_False);
			myOccView->getContext()->SetSelectionModeActive(displinks[i], -1, Standard_False);
		}
		if (isTheFirstDraw) {
			myOccView->fitAll();
			isTheFirstDraw = false;
		}
		else {
			myOccView->Redraw();
		}
	}
}

void MainWindow::robotWidgetInit() {

	/*tcp_transform(0, 0) = 0.70710678; tcp_transform(0, 1) = 0.0; tcp_transform(0, 2) = 0.70710678; tcp_transform(0, 3) = 79.90306;
	tcp_transform(1, 0) = 0.0; tcp_transform(1, 1) = 1.0; tcp_transform(1, 2) = 0.0; tcp_transform(1, 3) = 0.0;
	tcp_transform(2, 0) = -0.70710678; tcp_transform(2, 1) = 0.0; tcp_transform(2, 2) = 0.7071067; tcp_transform(2, 3) = 517.903;
	tcp_transform(3, 0) = 0.0; tcp_transform(3, 1) = 0.0; tcp_transform(3, 2) = 0.0; tcp_transform(3, 3) = 1.0;

	QString filename = QFileDialog::getOpenFileName(this, "", "", "All Formats (*.xml)");
	this->loadRobotWidget(filename);*/

	std::ifstream fin("./config/config.json");
	if (!fin) {
		ui->console->print(ct::LOG_ERROR, "cannot open config.json!");
		return;
	}
	json config = json::parse(fin);

	// 2. 读取机器人型号
	std::string ro_type = config["RoType"].get<std::string>();

	// 3. 读取 4x4 矩阵 -> rl::math::Transform
	tcp_transform.setIdentity();
	const auto& m = config["tcp_transform"];
	for (int i = 0; i < 4; ++i) {
		for (int j = 0; j < 4; ++j) {
			tcp_transform.matrix()(i, j) = m[i][j].get<double>();
		}
	}

	QString filename;
	if (ro_type == "kuka16" || ro_type == "kuka20") {
		filename = QDir::currentPath() + QString::fromStdString("/robot_env/" + ro_type + "/" + ro_type + ".xml");
	}
	else {
		ui->console->print(ct::LOG_ERROR, "Robot Type read from config.json is wrong, please check!");
		return;
	}
	this->loadRobotWidget(filename);

	std::string type;
	if (filename.contains("kuka16")) {
		type = "kuka16";
	}
	else if(filename.contains("kuka20")){
		type = "kuka20";
	}
	loadOcctModel(type);
}

void MainWindow::loadRobotWidget(const QString& filename) {

	QMutexLocker lock(&this->mutex);

	rl::xml::DomParser parser;
	rl::xml::Document document = parser.readFile(filename.toStdString(), "", XML_PARSE_NOENT | XML_PARSE_XINCLUDE);
	document.substitute(XML_PARSE_NOENT | XML_PARSE_XINCLUDE);

	rl::xml::Path path(document);
	this->scene = std::make_shared<rl::sg::ode::Scene>();

	rl::xml::NodeSet modelScene = path.eval("(/rl/plan|/rlplan)//model/scene").getValue<rl::xml::NodeSet>();
	this->scene->load(modelScene[0].getUri(modelScene[0].getProperty("href")));
	this->sceneModel = this->scene->getModel(0);
	if (this->scene->getNumModels() > 1) {
		this->sceneBody = this->scene->getModel(1)->create();
	}
	else {
		ui->console->print(ct::LOG_INFO, u8"场景模型个数小于2!");
		return;
	}

	rl::xml::NodeSet mdl = path.eval("(/rl/plan|/rlplan)//model/kinematics")
		.getValue<rl::xml::NodeSet>();
	rl::mdl::XmlFactory factory;

	std::shared_ptr<rl::mdl::Model> model =
		factory.create(mdl[0].getUri(mdl[0].getProperty("href")));

	this->mdl = std::dynamic_pointer_cast<rl::mdl::Dynamic>(model);

	this->mdl->setHomePosition(this->mdl->getHomePosition());
	this->mdl->setPosition(this->mdl->getHomePosition());
	this->mdl->forwardPosition();

	/*rl::xml::NodeSet mdl = path.eval("(/rl/plan|/rlplan)//model/kinematics").getValue<rl::xml::NodeSet>();
	rl::mdl::XmlFactory factory;
	this->mdl.reset(dynamic_cast<rl::mdl::Dynamic*>(factory.create(mdl[0].getUri(mdl[0].getProperty("href")))));*/

	if (rl::sg::DistanceScene* scene = dynamic_cast<rl::sg::DistanceScene*>(this->scene.get())) {
		this->model = std::make_shared<rl::plan::DistanceModel>();
	}
	else if (rl::sg::SimpleScene* scene = dynamic_cast<rl::sg::SimpleScene*>(this->scene.get())) {
		this->model = std::make_shared<rl::plan::SimpleModel>();
	}
	else {
		throw std::runtime_error("selected engine does not support collision queries");
	}

	if (nullptr != this->mdl) {
		this->model->mdl = this->mdl.get();
	}

	this->model->model = this->sceneModel;
	this->model->scene = this->scene.get();

	this->verifier = std::make_shared<rl::plan::RecursiveVerifier>();
	this->verifier->delta = path.eval("number((/rl/plan|/rlplan)//advancedOptimizer/recursiveVerifier/delta)").getValue<rl::math::Real>(1);
	if ("deg" == path.eval("string((/rl/plan|/rlplan)//advancedOptimizer/recursiveVerifier/delta/@unit)").getValue<std::string>()) {
		this->verifier->delta *= rl::math::DEG2RAD;
	}

	if (nullptr != this->verifier) {
		this->verifier->model = this->model.get();
	}

	this->optimizer.reset();
	this->optimizer = std::make_shared<rl::plan::AdvancedOptimizer>();
	rl::plan::AdvancedOptimizer* advancedOptimizer = static_cast<rl::plan::AdvancedOptimizer*>(this->optimizer.get());
	advancedOptimizer->length = path.eval("number((/rl/plan|/rlplan)//advancedOptimizer/length)").getValue<rl::math::Real>(1);
	if ("deg" == path.eval("string((/rl/plan|/rlplan)//advancedOptimizer/length/@unit)").getValue<std::string>()) {
		advancedOptimizer->length *= rl::math::DEG2RAD;
	}
	advancedOptimizer->ratio = path.eval("number((/rl/plan|/rlplan)//advancedOptimizer/ratio)").getValue<rl::math::Real>(0.1f);

	if (nullptr != this->optimizer) {
		this->optimizer->model = this->model.get();
		this->optimizer->verifier = this->verifier.get();
	}

	this->sampler = std::make_shared<rl::plan::UniformSampler>();
	rl::plan::UniformSampler* uniformSampler = static_cast<rl::plan::UniformSampler*>(this->sampler.get());
	if (nullptr != this->sampler) {
		this->sampler->model = this->model.get();
	}

	rl::xml::NodeSet planners = path.eval("(/rl/plan|/rlplan)//addRrtConCon|(/rl/plan|/rlplan)//eet|(/rl/plan|/rlplan)//rrt|(/rl/plan|/rlplan)//rrtCon|(/rl/plan|/rlplan)//rrtConCon|(/rl/plan|/rlplan)//rrtConExt|(/rl/plan|/rlplan)//rrtDual|(/rl/plan|/rlplan)//rrtExtCon").getValue<rl::xml::NodeSet>();
	for (int i = 0; i < std::min(1, planners.size()); ++i) {
		rl::xml::Path path(document, planners[i]);
		this->planner = std::make_shared<rl::plan::AddRrtConCon>();
		rl::plan::AddRrtConCon* addRrtConCon = static_cast<rl::plan::AddRrtConCon*>(this->planner.get());
		addRrtConCon->alpha = path.eval("number(alpha)").getValue<rl::math::Real>(0.5f);
		addRrtConCon->delta = path.eval("number(delta)").getValue<rl::math::Real>(20);

		if ("deg" == path.eval("string(delta/@unit)").getValue<std::string>()) {
			addRrtConCon->delta *= rl::math::DEG2RAD;
		}

		addRrtConCon->epsilon = path.eval("number(epsilon)").getValue<rl::math::Real>(1.0e-1f);

		if ("deg" == path.eval("string(epsilon/@unit)").getValue<std::string>()) {
			addRrtConCon->epsilon *= rl::math::DEG2RAD;
		}

		addRrtConCon->lower = path.eval("number(lower)").getValue<rl::math::Real>(20);

		if ("deg" == path.eval("string(lower/@unit)").getValue<std::string>()) {
			addRrtConCon->lower *= rl::math::DEG2RAD;
		}

		addRrtConCon->radius = path.eval("number(radius)").getValue<rl::math::Real>(200);

		if ("deg" == path.eval("string(radius/@unit)").getValue<std::string>()) {
			addRrtConCon->radius *= rl::math::DEG2RAD;
		}

		addRrtConCon->sampler = this->sampler.get();
	}

	std::size_t nearestNeighborsSize = 1;
	if (rl::plan::RrtDual* rrtDual = dynamic_cast<rl::plan::RrtDual*>(this->planner.get())) {
		nearestNeighborsSize = 2;
	}

	for (::std::size_t i = 0; i < nearestNeighborsSize; ++i)
	{
		std::shared_ptr<rl::plan::NearestNeighbors> nearestNeighbors;
		std::shared_ptr<rl::plan::GnatNearestNeighbors> gnatNearestNeighbors = std::make_shared<rl::plan::GnatNearestNeighbors>(this->model.get());
		nearestNeighbors = gnatNearestNeighbors;
		this->nearestNeighbors.push_back(nearestNeighbors);
		if (rl::plan::Prm* prm = dynamic_cast<rl::plan::Prm*>(this->planner.get())) {
			prm->setNearestNeighbors(nearestNeighbors.get());
		}
		if (rl::plan::Rrt* rrt = dynamic_cast<rl::plan::Rrt*>(this->planner.get())) {
			rrt->setNearestNeighbors(nearestNeighbors.get(), i);
		}
	}
	this->planner->duration = std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<float>(100));
	this->planner->model = this->model.get();

	ConfigurationDelegate* configurationDelegate = new ConfigurationDelegate(this);
	this->configurationDelegates = configurationDelegate;

	configurationModel = new ConfigurationModel(this);

	QTableView* configurationView = new QTableView(this);
	configurationView->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
	configurationView->horizontalHeader()->hide();
	configurationView->setAlternatingRowColors(true);
	configurationView->setItemDelegate(configurationDelegate);
	configurationView->setModel(configurationModel);

	OperationalDelegate* operationalDelegate = new OperationalDelegate(this);
	this->operationalDelegates = operationalDelegate;

	OperationalModel* operationalModel = new OperationalModel(this);
	this->operationalModels = operationalModel;

	QTableView* operationalView = new QTableView(this);
	operationalView->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
	operationalView->horizontalHeader()->hide();
	operationalView->setAlternatingRowColors(true);
	operationalView->setItemDelegate(operationalDelegate);
	operationalView->setModel(operationalModel);

	// ui->robotControlWidget->tabBar()->setExpanding(true);
	ui->robotControlWidget->tabBar()->setUsesScrollButtons(true);
	ui->robotControlWidget->addTab(configurationView, QStringLiteral("轴空间"));
	ui->robotControlWidget->addTab(operationalView, QStringLiteral("笛卡尔"));

	QObject::connect(
		configurationModel,
		SIGNAL(dataChanged(const QModelIndex&, const QModelIndex&)),
		operationalModel,
		SLOT(configurationChanged(const QModelIndex&, const QModelIndex&))
	);

	QObject::connect(
		operationalModel,
		SIGNAL(dataChanged(const QModelIndex&, const QModelIndex&)),
		configurationModel,
		SLOT(operationalChanged(const QModelIndex&, const QModelIndex&))
	);

	QObject::connect(ui->rotateStepSpinbox, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
		configurationDelegate, &ConfigurationDelegate::setSingleStep);

	QObject::connect(ui->transStepSpinbox, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
		operationalDelegate, &OperationalDelegate::setSingleStep);

	configurationModel->setData(this->mdl->getHomePosition());

	QObject::connect(flushSceneTimer, &QTimer::timeout, this, &MainWindow::occtUpdate);
	flushSceneTimer->start(50);
}

void MainWindow::load3DModel() {
	QString filename = QFileDialog::getOpenFileName(this, "", "", "All Formats (*.stl | *.step)");
	QFileInfo fileInfo(filename);
	QString suffix = fileInfo.suffix().toLower();
	QString wrlFilename = fileInfo.absolutePath() + "/" + fileInfo.baseName() + ".wrl";

	QMutexLocker locker(&mapMutex);
	ReadModel rm;
	TopoDS_Shape scene_shape;
	if (suffix == "stl") {
		scene_shape = ReadModel::readStlModel(filename.toStdString().c_str());
	}
	else if (suffix == "step") {
		scene_shape = ReadModel::readStepModel(filename.toStdString().c_str());
	}
	else {
		ui->console->print(ct::LOG_INFO, QStringLiteral("导入文件格式非stl或step文件!"));
		return;
	}

	loadShape = new AIS_Shape(scene_shape);
	loadWrlModel(wrlFilename.toStdString());

	myOccView->getContext()->Display(loadShape, Standard_False);
	if (isTheFirstDraw) {
		myOccView->fitAll();
		isTheFirstDraw = false;
	}
	else {
		myOccView->Redraw();
	}
}

void MainWindow::saveModel() {
	
	TopoDS_Shape currentSelectionShape = this->myOccView->getSelectShape();
	if (currentSelectionShape.IsNull()) {
		ui->console->print(ct::LOG_WARNING, QStringLiteral("未选中任何实体!"));
		return;
	}

	ReadModel rm;
	QString fileName = QFileDialog::getSaveFileName( this, QStringLiteral("模型导出"),"",tr("STEP Files (*.step *.stp);;STL Files (*.stl)"));
	if (fileName.isEmpty()) {
		return;
	}

	QFileInfo fileInfo(fileName);
	QString suffix = fileInfo.suffix().toLower();
	bool success = false;
	if (suffix == "stl") {
		rm.writeStlModel(currentSelectionShape, fileName.toUtf8().constData());
		success = true;
	}
	else if (suffix == "step" || suffix == "stp") {
		rm.writeStepModel(currentSelectionShape, fileName.toUtf8().constData());
		success = true;
	}
	else {
		ui->console->print(ct::LOG_WARNING, QStringLiteral("格式错误，不支持的文件格式，请选择 .stl 或 .step"));
		return;
	}
	if (success) {
		ui->console->print(ct::LOG_INFO, QStringLiteral("文件保存成功") + fileName);
	}
	else {
		ui->console->print(ct::LOG_INFO, QStringLiteral("文件保存失败") + fileName);
	}
}

void MainWindow::removeModel() {
	std::vector<Handle(AIS_InteractiveObject)> selectedObjects;
	myOccView->getContext()->InitSelected();

	// 选取线段，获取端点
	while (myOccView->getContext()->MoreSelected())
	{
		Handle(AIS_InteractiveObject) aAis = myOccView->getContext()->SelectedInteractive();
		selectedObjects.push_back(aAis);
		myOccView->getContext()->NextSelected();
	}
	for (auto aAis : selectedObjects)
	{
		myOccView->getContext()->Remove(aAis, false);
	}
	myOccView->getContext()->UpdateCurrentViewer();

	for (int i = 0; i < sgShapes.size(); ++i) {
		this->sceneBody->remove(sgShapes[i]);
	}
}

void MainWindow::loadWrlModel(const std::string& wrlName) {

	SoInput input;
	if (!input.openFile(wrlName.c_str())) return;
	SoNode* wrlRoot = SoDB::readAll(&input);
	input.closeFile();

	SoSearchAction searchAction;
	searchAction.setType(SoVRMLShape::getClassTypeId());
	searchAction.setInterest(SoSearchAction::ALL);
	searchAction.apply(wrlRoot);

	const SoPathList& paths = searchAction.getPaths();

	std::vector<SoVRMLShape*> shapes;
	for (int i = 0; i < paths.getLength(); ++i) {
		SoVRMLShape* shape = static_cast<SoVRMLShape*>(paths[i]->getTail());
		rl::sg::Shape* rlShape = this->sceneBody->create(shape);
		sgShapes.push_back(rlShape);
	}

	this->model->scene = this->scene.get();

	if (nullptr != this->verifier) {
		this->verifier->model = this->model.get();
	}

	if (nullptr != this->verifier) {
		this->verifier->model = this->model.get();
	}

	if (nullptr != this->optimizer) {
		this->optimizer->model = this->model.get();
		this->optimizer->verifier = this->verifier.get();
	}

	if (nullptr != this->sampler) {
		this->sampler->model = this->model.get();
	}

	this->planner->model = this->model.get();
}

void MainWindow::reset() {
	/*std::chrono::steady_clock::duration duration = this->planner->duration;

	this->planner->reset();
	this->model->reset();*/
}

void MainWindow::on_actionGoOrigin_triggered() {
	rl::math::Vector home = this->mdl->getHomePosition();
	this->model->setPosition(home);

	occtUpdate();
}

void MainWindow::flushWeldList(const TopoDS_Shape& selectShape, const std::vector<TopoDS_Edge>& weldEdges) {
	ui->weldTableWidget->setRowCount(0);

	if (weldEdges.size() == 0) {
		ui->console->print(ct::LOG_WARNING, tr("Not extract valid weld!"));
		return;
	}
	if (selectShape.IsNull()) {
		ui->console->print(ct::LOG_WARNING, tr("Not choose vaile model!"));
		return;
	}

	m_selectShape = std::move(selectShape);
	std::vector<TopoDS_Edge> m_weldEdges = std::move(weldEdges);

	WeldingPathBuilder getPath(20.0, 1e-3);
	std::vector<ProcessedEdge> sortedEdges;
	sortedClusters.clear();
	getPath.SortEdges(m_weldEdges, sortedEdges, sortedClusters);
	if (sortedEdges.size() < 1) return;

	auto Pt2Qstring = [](const gp_Pnt& point, QString& result) {
		double x = point.X();
		double y = point.Y();
		double z = point.Z();
		QString xStr = QString::number(x, 'f', 3);
		QString yStr = QString::number(y, 'f', 3);
		QString zStr = QString::number(z, 'f', 3);
		result = QString("(%1, %2, %3)").arg(xStr).arg(yStr).arg(zStr);
	};

	int dataCount = sortedEdges.size();
	for (int i = 0; i < dataCount; i++) {
		ui->weldTableWidget->insertRow(i);

		QComboBox* pathCombo = new QComboBox();
		pathCombo->addItems({ "Line", "Circle" });
		pathCombo->setCurrentIndex(0);
		ui->weldTableWidget->setCellWidget(i, 0, pathCombo);

		QComboBox* craftCombo = new QComboBox();
		craftCombo->addItems({ QStringLiteral("I口"), QStringLiteral("V口"), QStringLiteral("角焊"), QStringLiteral("多层多道") });
		ui->weldTableWidget->setCellWidget(i, 1, craftCombo);

		ProcessedEdge current_edge = sortedEdges[i];

		QString ptString;
		Pt2Qstring(current_edge.StartPoint, ptString);
		QTableWidgetItem* coordItem_start = new QTableWidgetItem(ptString);
		coordItem_start->setTextAlignment(Qt::AlignCenter);
		coordItem_start->setFlags(coordItem_start->flags() & ~Qt::ItemIsEditable);
		ui->weldTableWidget->setItem(i, 2, coordItem_start);

		QDoubleSpinBox* StartOffset = new QDoubleSpinBox();
		StartOffset->setMaximum(200);
		StartOffset->setMinimum(30);
		StartOffset->setValue(50);
		StartOffset->setDecimals(2);
		ui->weldTableWidget->setCellWidget(i, 3, StartOffset);

		Pt2Qstring(current_edge.EndPoint, ptString);
		QTableWidgetItem* coordItem_end = new QTableWidgetItem(ptString);
		coordItem_end->setTextAlignment(Qt::AlignCenter);
		coordItem_end->setFlags(coordItem_end->flags() & ~Qt::ItemIsEditable);
		ui->weldTableWidget->setItem(i, 4, coordItem_end);

		QDoubleSpinBox* EndOffset = new QDoubleSpinBox();
		EndOffset->setMaximum(200);
		EndOffset->setMinimum(30);
		EndOffset->setValue(50);
		EndOffset->setDecimals(2);
		ui->weldTableWidget->setCellWidget(i, 5, EndOffset);

		QDoubleSpinBox* TransitionSpin = new QDoubleSpinBox();
		TransitionSpin->setMaximum(180);
		TransitionSpin->setMinimum(-180);
		TransitionSpin->setValue(-45);
		TransitionSpin->setDecimals(2);
		ui->weldTableWidget->setCellWidget(i, 6, TransitionSpin);
	}

	ui->console->print(ct::LOG_INFO, tr("Extract weld done!"));
}

void MainWindow::onWeldSelected(const QItemSelection& selected, const QItemSelection& deselected) {
	Q_UNUSED(deselected); 
	if (selected.indexes().isEmpty()) return;
	int row = selected.indexes().first().row();
	QTableWidget* table = qobject_cast<QTableWidget*>(sender()->parent());
	if (table) {
		QTableWidgetItem* itemStart = table->item(row, 2); // 起点
		QTableWidgetItem* itemEnd = table->item(row, 4); // 终点

		QString start_point_string = itemStart ? itemStart->text() : "";
		QString end_point_string = itemEnd ? itemEnd->text() : "";

		QRegularExpression re(R"(\(([\d.+-]+),\s*([\d.+-]+),\s*([\d.+-]+)\))");
		QRegularExpressionMatch match_start = re.match(start_point_string);
		QRegularExpressionMatch match_end = re.match(end_point_string);

		gp_Pnt m_start_point, m_end_point;
		if (match_start.hasMatch()) {
			double x = match_start.captured(1).toDouble();
			double y = match_start.captured(2).toDouble();
			double z = match_start.captured(3).toDouble();
			gp_Pnt start_point(x, y, z);
			m_start_point = start_point;
		}

		if (match_end.hasMatch()) {
			double x = match_end.captured(1).toDouble();
			double y = match_end.captured(2).toDouble();
			double z = match_end.captured(3).toDouble();
			gp_Pnt start_point(x, y, z);
			m_end_point = start_point;
		}

		Handle(AIS_Shape) blueArrow = myOccView->CreateBlueArrow(m_start_point, m_end_point, 4.0, 12.0, 6.0);
		this->myOccView->getContext()->SetDisplayMode(blueArrow, 1, Standard_True);
		this->myOccView->getContext()->Display(blueArrow, Standard_False);
		this->myOccView->Redraw();
	}
}

void MainWindow::initSimulation() {
	ikwork->moveToThread(m_thread_ikwork);
	QObject::connect(m_thread_ikwork, &QThread::finished, ikwork, &QObject::deleteLater);
	ikwork->setKinematic(this->mdl);

	connect(ikwork, &IKWorker::started, this, [=]() {
		ui->trajProgressBar->setRange(0, 100);
		ui->trajProgressBar->setValue(0);
		}, Qt::QueuedConnection);

	connect(ikwork, &IKWorker::progress, this, [=](int percent) {
		ui->trajProgressBar->setValue(percent);
		}, Qt::QueuedConnection);

	connect(ikwork, &IKWorker::finished_return, this, [=](const std::vector<rl::math::Vector>& jointTrajectory,
		const double& ratio) {
			m_jointTrajectory_home = jointTrajectory;
			flushHomeTimer->start(20);
			ui->console->print(ct::LOG_INFO, QString("Trajctory return to home done, Completion rate %1%").arg(QString::number(ratio)));
		}, Qt::QueuedConnection);

	connect(ikwork, &IKWorker::finished_start, this, [=](const std::vector<rl::math::Vector>& jointTrajectory,
		const double& ratio) {
			if(jointTrajectory.size() < 1){return;}
			wholeTrajectory.clear();
			wholeTrajectory.insert(wholeTrajectory.end(), jointTrajectory.begin(), jointTrajectory.end());

			IKSolveParams params;
			params.trajectory = mergedTraj;
			params.T_flange_to_tcp = tcp_transform;
			params.q_initial = jointTrajectory.back();
			if (params.q_initial.size() == 0) {
				params.q_initial = this->mdl->getPosition();
			}
			params.constrainRail = true;
			params.railWindow = 10;
			params.timeoutMs = 500;
			QMetaObject::invokeMethod(ikwork, "doSolve", Qt::QueuedConnection, Q_ARG(IKSolveParams, params));

			ui->console->print(ct::LOG_INFO, QString("Trajctory home to start done, Completion rate %1%").arg(QString::number(ratio)));
		}, Qt::QueuedConnection);

	connect(ikwork, &IKWorker::finished, this, [=](const std::vector<rl::math::Vector>& jointTrajectory,
		const double& ratio, const DiscretePoint& start) {
			if (jointTrajectory.size() < 1) { return; }
			ui->trajProgressBar->setValue(100);
			ui->console->print(ct::LOG_INFO, QString("Trajctory weld done, Completion rate %1%").arg(QString::number(ratio)));
			if (ui->virtualRobotCheck->isChecked()) {
				wholeTrajectory.insert(wholeTrajectory.end(), jointTrajectory.begin(), jointTrajectory.end());
			}
			else if (ui->realRobotCheck->isChecked()) {
				wholeTrajectory.push_back(jointTrajectory[0]);
				wholeTrajectory.push_back(jointTrajectory[jointTrajectory.size() - 1]);
			}
		}, Qt::QueuedConnection);

	connect(ikwork, &IKWorker::failed, this, [=](const QString& errorMessage) {
		ui->console->print(ct::LOG_ERROR, errorMessage);
		}, Qt::QueuedConnection);

	connect(ikwork, &IKWorker::aborted, this, [=]() {
		ui->console->print(ct::LOG_ERROR, "Trajctory Progree Aborted!");
		}, Qt::QueuedConnection);

	m_thread_ikwork->start();
}

void MainWindow::execSimulation(int& inter) {
	
	if (wholeTrajectory.size() == 0 || inter >= wholeTrajectory.size()) {
		ui->actionSimulation->setIcon(QIcon(":/res/icon/simulation.svg"));
		flushTrajTimer->stop();
		isSimulation = false;
		m_counter = 0;
		return;
	}
	rl::math::Vector current_joint = wholeTrajectory[inter];
	if (ui->realRobotCheck->isChecked()) {
		emit sendTrajectory(current_joint);
	}
	else if (ui->virtualRobotCheck->isChecked()) {
		configurationModel->setData(current_joint);
	}
}

void MainWindow::robotGoHome() {
	m_jointTrajectory_home.clear();

	IKReturnHomeParams p;
	p.q_current = this->mdl->getPosition();
	p.q_home = this->mdl->getHomePosition();
	p.T_flange_to_tcp = tcp_transform;
	p.tcpStepBack = 100.0;
	p.jointStepRad = M_PI / 180.0;
	p.railStepLen = 10.0;
	p.timeoutMs = 500;

	QMetaObject::invokeMethod(ikwork, "doReturnHome", Qt::QueuedConnection, Q_ARG(IKReturnHomeParams, p));
}

void MainWindow::execReturnHome(int& inter) {
	if (m_jointTrajectory_home.size() == 0 || inter >= m_jointTrajectory_home.size()) {
		inter = 0;
		flushHomeTimer->stop();
		return;
	}
	rl::math::Vector current_joint = m_jointTrajectory_home[inter];

	if (ui->realRobotCheck->isChecked()) {
		emit sendTrajectory(current_joint);
	}
	else if (ui->virtualRobotCheck->isChecked()) {
		configurationModel->setData(current_joint);
	}
}

void MainWindow::showWeldTrajPt(const std::vector<DiscretePoint>& trajPt) {
	if (trajPt.size() < 2) return;
	if (this->myOccView == nullptr) return;

	for (const auto& info : trajPt)
	{
		// 用 Trihedron 显示局部坐标系
		// gp_Ax2 需要 (origin, mainDir=Z, xDir)
		gp_Ax2 anAx2(info.position, info.zDir, info.xDir);
		Handle(Geom_Axis2Placement) anAxisPlacement =
			new Geom_Axis2Placement(anAx2);

		Handle(AIS_Trihedron) aTrihedron = new AIS_Trihedron(anAxisPlacement);
		aTrihedron->SetSize(5.0);

		// 设置坐标轴颜色：X-红 Y-绿 Z-蓝
		Handle(Prs3d_DatumAspect) aDatumAspect = new Prs3d_DatumAspect();
		aDatumAspect->LineAspect(Prs3d_DatumParts_XAxis)
			->SetColor(Quantity_NOC_RED);
		aDatumAspect->LineAspect(Prs3d_DatumParts_YAxis)
			->SetColor(Quantity_NOC_GREEN);
		aDatumAspect->LineAspect(Prs3d_DatumParts_ZAxis)
			->SetColor(Quantity_NOC_BLUE1);
		aTrihedron->Attributes()->SetDatumAspect(aDatumAspect);

		// 同时显示一个点标记（可选）
		Handle(Geom_CartesianPoint) aGeomPnt =
			new Geom_CartesianPoint(info.position);
		Handle(AIS_Point) anAISPoint = new AIS_Point(aGeomPnt);
		Handle(Prs3d_PointAspect) aPointAspect = new Prs3d_PointAspect(
			Aspect_TOM_O_PLUS, Quantity_NOC_YELLOW, 2.0);
		anAISPoint->Attributes()->SetPointAspect(aPointAspect);

		this->myOccView->getContext()->Display(aTrihedron, Standard_False);
		this->myOccView->getContext()->Display(anAISPoint, Standard_False);
	}
	this->myOccView->getContext()->UpdateCurrentViewer();
}

void MainWindow::Trajectory() {

	if (m_selectShape.IsNull()) {
		ui->console->print(ct::LOG_WARNING, tr("Current not choose valid model!"));
		return;
	}
	/*std::vector<DiscretePoint> mergedTraj;*/
	mergedTraj.clear();
	std::vector<std::vector<DiscretePoint>> wholeTrajPt;
	WeldingPathBuilder getPath(20.0, 1e-3);
	if (sortedClusters.size() >= 2) {
		for (auto clusterEdge : sortedClusters) {
			TopoDS_Wire wires;
			if (!clusterEdge.isVertical) {
				getPath.GetCirclePath(clusterEdge.edges, wires);
				if (wires.IsNull()) return;
				std::vector<DiscretePoint> trajPt = getPath.DiscretizeWireWithCustomFrame(wires, m_selectShape, 20.0, 1e-3);
				showWeldTrajPt(trajPt);
				wholeTrajPt.push_back(trajPt);
			}
			else {
				if (clusterEdge.edges.size() < 1) return;
				BRepBuilderAPI_MakeWire mkWire(clusterEdge.edges[0].Edge);
				wires = mkWire.Wire();
				std::vector<DiscretePoint> trajPt = getPath.DiscretizeWireWithCustomFrame(wires, m_selectShape, 20.0, 1e-3);
				trajPt.insert(trajPt.begin(), trajPt.back());
				showWeldTrajPt(trajPt);
				wholeTrajPt.push_back(trajPt);
			}
		}
		size_t totalSize = 0;
		for (const auto& traj : wholeTrajPt) {
			totalSize += traj.size();
		}

		mergedTraj.reserve(totalSize);
		for (const auto& traj : wholeTrajPt) {
			mergedTraj.insert(mergedTraj.end(), traj.begin(), traj.end());
		}
	}
	else if(sortedClusters.size() == 1){
		if (!DiscretizeWeldTrajectory(m_selectShape, sortedClusters[0].edges[0].Edge, mergedTraj, 5.0, Standard_True, 100)) {
			return;
		}
		showWeldTrajPt(mergedTraj);
	}
	else {
		ui->console->print(ct::LOG_ERROR, "Input Edge is empty!");
		return;
	}

	if (mergedTraj.size() > 2) {
		IKGoToStartParams p;
		p.q_home = this->mdl->getHomePosition();
		p.startPoint = mergedTraj[0];
		p.T_flange_to_tcp = tcp_transform;
		p.baseUpDistance = 100.0;
		p.jointStepRad = 1 * M_PI / 180.0;
		p.railStepLen = 5.0;
		p.cartStepLen = 5.0;
		p.timeoutMs = 500;

		QMetaObject::invokeMethod(ikwork, "doGoToStart", Qt::QueuedConnection, Q_ARG(IKGoToStartParams, p));
	}
}

void MainWindow::safeImwrite(const QString& savePath, const cv::Mat& image) {
	QString dirPath = QFileInfo(savePath).absolutePath();

	QDir dir;
	if (!dir.exists(dirPath)) {
		if (!dir.mkpath(dirPath)) {
			ui->console->print(ct::LOG_ERROR, QStringLiteral("Error: cannot create valid directory!"));
			return;
		}
	}

	bool success = cv::imwrite(savePath.toStdString(), image);
	if (!success) {
		ui->console->print(ct::LOG_ERROR, QStringLiteral("Error: Image save failed!"));
	}
	else {
		ui->console->print(ct::LOG_ERROR, QStringLiteral("Success: Image save done, location in %1!").arg(savePath));
	}
}