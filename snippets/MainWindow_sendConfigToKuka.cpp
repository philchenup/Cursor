// Paste into MainWindow after the operational ↔ configuration model connections.
//
// Flow:
//   operationalModel::setData (IK ok)
//     → mdl joint positions already updated
//     → emit dataChanged
//     → configurationModel::operationalChanged() only refreshes the table
//       (it does NOT emit dataChanged!)
//
// Therefore: send joints to Kuka on operationalModel::dataChanged,
// and also on configurationModel::dataChanged for direct axis-space edits.

auto sendConfigurationToKuka = [this]() {
	KukaCommunicator* comm = m_comm.get();
	if (!comm || !mdl) {
		return;
	}

	const rl::math::Vector q = mdl->getPosition(); // radians / meters (rl units)
	const auto qUnits = mdl->getPositionUnits();

	SendRobot sr;
	{
		QMutexLocker locker(&g_robotMutex);
		// Adjust field name to match your RobotData (e.g. axes / joints / q).
		// Convert radian joints to degrees if your Kuka protocol expects deg.
		const int n = std::min(static_cast<int>(q.size()), 6);
		for (int i = 0; i < n; ++i) {
			if (qUnits(i) == rl::math::UNIT_RADIAN) {
				g_robotData.axes[i] = q(i) * rl::math::RAD2DEG;
			} else {
				g_robotData.axes[i] = q(i);
			}
		}
		// If SendRobot needs a copy of the command payload, fill sr here.
	}

	QMetaObject::invokeMethod(comm, [comm, sr]() {
		comm->stepMove(sr); // or moveAbs / sendJoints — use your joint command API
	}, Qt::QueuedConnection);
};

// Cartesian / operational change → configuration synced in mdl → send to Kuka
QObject::connect(
	operationalModel,
	&QAbstractItemModel::dataChanged,
	this,
	[sendConfigurationToKuka](const QModelIndex&, const QModelIndex&) {
		sendConfigurationToKuka();
	}
);

// Direct axis-space spinbox edit → also send to Kuka
QObject::connect(
	configurationModel,
	&QAbstractItemModel::dataChanged,
	this,
	[sendConfigurationToKuka](const QModelIndex&, const QModelIndex&) {
		sendConfigurationToKuka();
	}
);
