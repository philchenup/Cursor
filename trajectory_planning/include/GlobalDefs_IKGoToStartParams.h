// 粘贴到工程 GlobalDefs.h, 替换原 IKGoToStartParams。
// DiscretePoint startPoint → Eigen::Affine3f T_base_flange (目标点法兰位姿)。
// timeoutMs 默认 500。

struct IKGoToStartParams
{
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	rl::math::Vector q_home;              // Home 关节角(含地轨)
	Eigen::Affine3f  T_base_flange;       // 目标点法兰位姿 T_base_flange, 替代 DiscretePoint startPoint
	rl::math::Transform T_flange_to_tcp;  // 法兰 → TCP

	double jointStepRad = 0.5 * M_PI / 180.0;
	double railStepLen = 5.0;
	double cartStepLen = 5.0;
	int    timeoutMs = 500;
};
