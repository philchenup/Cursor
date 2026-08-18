void MainWindow::removeModel() {
	logInfo("Action triggered: RemoveModel");
	std::vector<Handle(AIS_InteractiveObject)> selectedObjects;
	myOccView->getContext()->InitSelected();

	while (myOccView->getContext()->MoreSelected())
	{
		Handle(AIS_InteractiveObject) aAis = myOccView->getContext()->SelectedInteractive();
		selectedObjects.push_back(aAis);
		myOccView->getContext()->NextSelected();
	}
	for (auto aAis : selectedObjects)
	{
		if (aAis.IsNull())
			continue;
		if (aAis->IsKind(STANDARD_TYPE(AIS_ViewCube))
			|| aAis->IsKind(STANDARD_TYPE(AIS_Manipulator))
			|| aAis->IsKind(STANDARD_TYPE(AIS_Trihedron))
			|| aAis->IsKind(STANDARD_TYPE(AIS_Point)))
			continue;

		// 先解绑当前 AIS 对应的那一个 RL Body，其它模型的 Body 不动
		aisRl.unbind(aAis);

		myOccView->getContext()->Remove(aAis, false);
	}
	myOccView->getContext()->UpdateCurrentViewer();

	rl::sg::Model* env = (this->scene != nullptr && this->scene->getNumModels() > 1)
		? this->scene->getModel(1) : nullptr;
	const int nBodies = (env != nullptr) ? static_cast<int>(env->getNumBodies()) : 0;
	const int nShapes = (this->sceneBody != nullptr) ? static_cast<int>(this->sceneBody->getNumShapes()) : 0;
	ui->console->print(ct::LOG_INFO,
		QString("Current scene model bodies num is %1, scene body shape num is %2.")
			.arg(nBodies).arg(nShapes));
}

// 构造里连接：拖动只同步当前 AIS
// connect(myOccView, &OccView::aisPoseChanged, this, &MainWindow::onAisPoseChanged);
// connect(myOccView, &OccView::aisDragFinished, this, &MainWindow::onAisDragFinished);

void MainWindow::onAisPoseChanged(AIS_Shape* ais)
{
	aisRl.sync(ais);
}

void MainWindow::onAisDragFinished(AIS_Shape* ais)
{
	if (ais == nullptr)
		return;
	aisRl.bind(ais, environmentModel());
}

void MainWindow::occtUpdate()
{
	// 不要 aisRl.syncAll()：拖一个工件时不得把其它 Body 当同一位姿刷新。
	// 工件位姿只在 aisPoseChanged / aisDragFinished 里按当前 AIS 更新。
	// ... 原来的机器人 link 刷新
}
