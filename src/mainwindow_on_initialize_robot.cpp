// 片段：配合客户端版 KukaCommunicator 使用
// 说明：robotIpEdit / robotPortEdit 现为「机器人服务端」地址，上位机主动连接

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
		}
	}, Qt::QueuedConnection);

	// 客户端模式：按钮切换「连接机器人 / 断开」
	disconnect(ui->robotConnectBtn, &QPushButton::clicked, nullptr, nullptr);
	connect(ui->robotConnectBtn, &QPushButton::clicked, this, [=]() {
		KukaCommunicator* comm = m_comm.get();
		if (!comm) {
			return;
		}

		// 已连接则断开；未连接则主动连接机器人（IP/Port = 机器人服务端）
		if (ui->robotConnectBtn->text() == QLatin1String("discon")) {
			QMetaObject::invokeMethod(comm, "stop", Qt::QueuedConnection);
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
		if (!comm) {
			return;
		}
		std::array<double, 7> xp2{};
		{
			QMutexLocker locker(&g_robotMutex);
			g_robotData.joint[axis] += dir * ui->rotateStepSpinbox->value();
			for (int i = 0; i < 7; ++i) {
				xp2[i] = g_robotData.joint[i];
			}
		}
		QMetaObject::invokeMethod(comm, [comm, xp2]() {
			comm->stepMove(1, xp2);
		}, Qt::QueuedConnection);
	};

	for (const auto& it : items) {
		disconnect(it.btn, &QPushButton::clicked, nullptr, nullptr);
		connect(it.btn, &QPushButton::clicked, this, [=]() {
			stepJoint(it.axis, it.dir);
		});
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
		if (!comm) {
			return;
		}
		// buildSensorXml(se1, SJ1..SJ6)：笛卡尔用 SJ1..SJ6=X,Y,Z,A,B,C，SE1=0
		std::array<double, 7> xp2{};
		{
			QMutexLocker locker(&g_robotMutex);
			g_robotData.pose[axis] += dir * ui->transStepSpinbox->value();
			xp2[0] = 0;
			for (int i = 0; i < 6; ++i) {
				xp2[i + 1] = g_robotData.pose[i];
			}
		}
		QMetaObject::invokeMethod(comm, [comm, xp2]() {
			comm->stepMove(2, xp2);
		}, Qt::QueuedConnection);
	};

	for (const auto& it : cartItems) {
		disconnect(it.btn, &QPushButton::clicked, nullptr, nullptr);
		connect(it.btn, &QPushButton::clicked, this, [=]() {
			stepCart(it.axis, it.dir);
		});
	}

	m_thread_rworker->start();

	ui->console->print(ct::LOG_INFO, "Robot Initialized done! (client mode: connect to robot server)");
}
