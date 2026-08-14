// 替换 initSimulation() 里对 finished_start 的连接（信号多了 toolPoints）
connect(ikwork, &IKWorker::finished_start, this,
	[=](const std::vector<rl::math::Vector>& jointTrajectory,
		const std::vector<rl::math::Vector3>& toolPoints,
		const double& ratio) {
		if (jointTrajectory.size() < 1) { return; }
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

		logInfo(QString("Trajectory home-to-start completed, progress %1%, points %2")
			.arg(QString::number(ratio)).arg(static_cast<int>(toolPoints.size())));
	}, Qt::QueuedConnection);
