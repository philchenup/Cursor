#ifndef TCP_POSE_H
#define TCP_POSE_H

#include <rl/math/Transform.h>
#include <rl/math/Vector.h>

/// 法兰坐标系下的 TCP：R 的三列是 TCP 的 X/Y/Z 轴在法兰系中的方向。
/// T_base_tcp = T_base_flange * T_flange_tcp
inline rl::math::Transform flangeToTcp(
	const rl::math::Transform& T_base_flange,
	const rl::math::Transform::LinearMatrixType& R_flange_tcp,
	const rl::math::Vector3& t_flange_tcp = rl::math::Vector3::Zero())
{
	rl::math::Transform T_flange_tcp = rl::math::Transform::Identity();
	T_flange_tcp.linear() = R_flange_tcp;
	T_flange_tcp.translation() = t_flange_tcp;
	return T_base_flange * T_flange_tcp;
}

/// 显示/编辑 TCP 后，反算法兰位姿再送给 IK。
inline rl::math::Transform tcpToFlange(
	const rl::math::Transform& T_base_tcp,
	const rl::math::Transform::LinearMatrixType& R_flange_tcp,
	const rl::math::Vector3& t_flange_tcp = rl::math::Vector3::Zero())
{
	rl::math::Transform T_flange_tcp = rl::math::Transform::Identity();
	T_flange_tcp.linear() = R_flange_tcp;
	T_flange_tcp.translation() = t_flange_tcp;
	return T_base_tcp * T_flange_tcp.inverse();
}

#endif // TCP_POSE_H
