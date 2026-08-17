	/// 用局部坐标系显示焊缝离散点；坐标轴仅显示、不可选中。
	void showWeldTrajPt(const std::vector<DiscretePoint>& trajPt);

	/// 用绿点显示 toolPoints，并用直径 5 mm 的绿线依次连接。
	void showToolPoints(const std::vector<rl::math::Vector3>& toolPoints);
