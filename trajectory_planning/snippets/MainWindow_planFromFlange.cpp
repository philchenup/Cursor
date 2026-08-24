// 合入 MainWindow 的调用示例: 当前输入为目标点法兰位姿 Eigen::Affine3f。
// 将 Thread.h / Thread.cpp 替换工程内同名文件后, 按下面方式触发规划。

#include "MainWindow.h"
#include "Thread.h"

#include <Eigen/Geometry>
#include <rl/math/Transform.h>

// ---------- 1) 直接拿到法兰位姿时 ----------
void MainWindow::planFromFlangePose(const Eigen::Affine3f& T_base_flange)
{
	if (!this->thread || !this->mdl || !this->planner)
	{
		return;
	}

	this->thread->animate = true;
	this->thread->setIkTimeoutMs(500);
	this->thread->setStartConfiguration(this->mdl->getPosition());
	this->thread->planToFlange(T_base_flange);
}

// ---------- 2) 视觉 / 配准回调里 (ProcessWorker::sendTargetTransform 同型) ----------
// connect(proworker.get(), &ProcessWorker::sendTargetTransform, this,
//     &MainWindow::planFromFlangePose);

// ---------- 3) 若当前是 TCP 位姿, 先换成法兰再规划 ----------
void MainWindow::planFromTcpPose(const rl::math::Transform& T_base_tcp)
{
	const rl::math::Transform T_base_flange = T_base_tcp * this->tcp_transform.inverse();

	Eigen::Affine3f pose;
	pose.matrix() = T_base_flange.matrix().cast<float>();
	this->planFromFlangePose(pose);
}

// ---------- 4) 收取规划结果 (可选, 替代只靠 Viewer 动画) ----------
void MainWindow::connectThreadPlanningFinished()
{
	connect(this->thread, &Thread::planningFinished, this,
		[this](const rl::plan::VectorList& path, bool solved, double plannerMs) {
			if (!solved)
			{
				ui->console->print(ct::LOG_ERROR,
					tr("Trajectory planning failed (%1 ms).").arg(plannerMs, 0, 'f', 1));
				return;
			}

			ui->console->print(ct::LOG_INFO,
				tr("Trajectory planned: %1 waypoints, %2 ms.")
					.arg(static_cast<int>(path.size()))
					.arg(plannerMs, 0, 'f', 1));

			// 如需写入 wholeTrajectory 供 flushTrajTimer / execSimulation 播放:
			// this->wholeTrajectory.assign(path.begin(), path.end());
		});
}
