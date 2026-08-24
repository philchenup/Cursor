// 合入: GlobalDefs.h 替换 IKGoToStartParams 后, Thread 按 doGoToStart 规划。
// 输入: Eigen::Affine3f 法兰位姿, IK 超时 500ms。

#include "MainWindow.h"
#include "Thread.h"
#include "GlobalDefs.h"

void MainWindow::planGoToStartFromFlange(const Eigen::Affine3f& T_base_flange)
{
	if (!this->thread || !this->mdl)
	{
		return;
	}

	IKGoToStartParams p;
	p.q_home = this->mdl->getHomePosition();
	p.T_base_flange = T_base_flange;
	p.T_flange_to_tcp = this->tcp_transform;
	p.railStepLen = 5.0;
	p.cartStepLen = 5.0;
	p.timeoutMs = 500;

	this->thread->animate = true;
	this->thread->planGoToStart(
		p.q_home,
		p.T_base_flange,
		p.T_flange_to_tcp,
		p.railStepLen,
		p.cartStepLen,
		p.timeoutMs);
}

// 原 Trajectory() 里用 DiscretePoint / pointToTransform 的写法改为:
//   Eigen::Affine3f T_base_flange = ...;  // 已是法兰位姿
//   this->planGoToStartFromFlange(T_base_flange);
//
// 若仍是 TCP:
//   rl::math::Transform T_tcp = pointToTransform(mergedTraj.front());
//   rl::math::Transform T_flange = T_tcp * this->tcp_transform.inverse();
//   Eigen::Affine3f pose;
//   pose.matrix() = T_flange.matrix().cast<float>();
//   this->planGoToStartFromFlange(pose);
