#include "servo_j_trajectory.h"

/*
 * 将原先按 viewer->delta 做空间加密的循环，换成 8 ms + 梯形速度轮廓。
 * 下面按你现有的 RL / Qt 代码书写，复制到规划线程中即可。
 *
 * 原代码问题：
 *   steps = ceil(distance(i, j) / delta) 只保证显示间距，
 *   与 servo_j 的 8 ms 周期、180 deg/s 单轴限速无关。
 *
 * 用法：路点为弧度时用 Params::forRadianPath()；发给 JAKA 前再转成度。
 */

#if defined(SERVO_J_RL_SNIPPET)

std::vector<std::vector<float> > toJakaDegreeTrajFromRl(const rl::plan::VectorList& path)
{
	std::vector<std::vector<float> > traj;
	traj.reserve(path.size());
	for (rl::plan::VectorList::const_iterator it = path.begin(); it != path.end(); ++it)
	{
		std::vector<float> q(static_cast<std::size_t>(it->size()));
		for (int d = 0; d < it->size(); ++d)
		{
			q[static_cast<std::size_t>(d)] = static_cast<float>(servo_j::rad2deg((*it)[d]));
		}
		traj.push_back(q);
	}
	return traj;
}

void Thread::interpolateForServoJ(const rl::plan::VectorList& path)
{
	const servo_j::Params params = servo_j::Params::forRadianPath(90.0, 400.0);

	rl::plan::VectorList interplotPath = servo_j::densifyPath(
		path,
		MainWindow::instance()->model,
		params,
		this->running);

	this->drawConfigurationPath(interplotPath);

	std::chrono::steady_clock::time_point stop = std::chrono::steady_clock::now();
	double plannerDuration =
		std::chrono::duration_cast<std::chrono::duration<double> >(stop - start).count() * 1000.0;

	// 规划器内部是弧度；JAKA servo_j 的 JointValue 是角度
	std::vector<std::vector<float> > traj = toJakaDegreeTrajFromRl(interplotPath);
	emit planningFinished(traj, plannerDuration);
}

/*
 * 展开后的等价循环（与 densifyPath 相同），便于对照原 delta 版本：
 *
 *	rl::plan::VectorList interplotPath;
 *	if (!path.empty())
 *	{
 *		interplotPath.push_back(*path.begin());
 *	}
 *
 *	rl::math::Vector inter(MainWindow::instance()->model->getDofPosition());
 *	const servo_j::Params params = servo_j::Params::forRadianPath(90.0, 400.0);
 *
 *	rl::plan::VectorList::const_iterator i = path.begin();
 *	rl::plan::VectorList::const_iterator j = path.begin();
 *	if (j != path.end())
 *	{
 *		++j;
 *	}
 *
 *	for (; i != path.end() && j != path.end(); ++i, ++j)
 *	{
 *		if (!this->running) break;
 *
 *		rl::math::Real sMax = 0;
 *		for (int d = 0; d < i->size(); ++d)
 *		{
 *			sMax = std::max(sMax, std::abs((*j)[d] - (*i)[d]));
 *		}
 *
 *		const servo_j::Profile profile = servo_j::makeProfile(sMax, params);
 *		for (int k = 1; k <= profile.N; ++k)
 *		{
 *			if (!this->running) break;
 *			const rl::math::Real u = servo_j::normalizedPosition(k * params.dt, profile);
 *			MainWindow::instance()->model->interpolate(*i, *j, u, inter);
 *			interplotPath.push_back(inter);
 *		}
 *	}
 *
 * 发送侧（另一线程）必须 8 ms 一拍、ABS 模式：
 *
 *	robot.servo_move_enable(TRUE);
 *	auto next = std::chrono::steady_clock::now();
 *	for (const auto& qDeg : traj)
 *	{
 *		JointValue joint_pos{};
 *		for (int d = 0; d < 6 && d < (int)qDeg.size(); ++d)
 *		{
 *			joint_pos.jVal[d] = qDeg[d];
 *		}
 *		robot.servo_j(&joint_pos, ABS);
 *		next += std::chrono::microseconds(8000);
 *		std::this_thread::sleep_until(next);
 *	}
 *	robot.servo_move_enable(FALSE);
 */

#endif
