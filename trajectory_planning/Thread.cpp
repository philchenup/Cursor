//
// Copyright (c) 2009, Markus Rickert
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <vector>
#include <QApplication>
#include <QDateTime>
#include <QMutexLocker>
#include <rl/math/Quaternion.h>
#include <rl/math/Unit.h>
#include <rl/mdl/JacobianInverseKinematics.h>
#include <rl/mdl/Joint.h>
#include <rl/mdl/Kinematic.h>
#include <rl/plan/Eet.h>
#include <rl/plan/GnatNearestNeighbors.h>
#include <rl/plan/KdtreeBoundingBoxNearestNeighbors.h>
#include <rl/plan/KdtreeNearestNeighbors.h>
#include <rl/plan/LinearNearestNeighbors.h>
#include <rl/plan/Prm.h>
#include <rl/plan/Rrt.h>

#include "MainWindow.h"
#include "Thread.h"
#include "Viewer.h"

namespace
{

struct RailLimitGuard
{
	rl::mdl::Joint* joint;
	rl::math::Vector min0;
	rl::math::Vector max0;
	bool armed;
	
	RailLimitGuard() :
		joint(nullptr),
		armed(false)
	{
	}
	
	~RailLimitGuard()
	{
		restore();
	}
	
	void restore()
	{
		if (this->armed && nullptr != this->joint && this->min0.size() > 0)
		{
			this->joint->setMinimum(this->min0);
			this->joint->setMaximum(this->max0);
			this->armed = false;
		}
	}
};

} // namespace

Thread::Thread(QObject* parent) :
	QThread(parent),
	animate(true),
	quit(false),
	swept(false),
	lastPlannerMs(0.0),
	lastSolved(false),
	running(false),
	targetFlangeMatrix(FlangeMatrix::Identity()),
	flangeToTcpMatrix(TransformMatrix::Identity()),
	hasTargetFlangePose(false),
	qHome(),
	qStart(),
	qGoal(),
	hasExplicitStart(false),
	ikTimeoutMs(500),
	railStepLen(5.0),
	cartStepLen(5.0)
{
}

Thread::~Thread()
{
}

void
Thread::drawConfiguration(const rl::math::Vector& q)
{
	emit configurationRequested(q);
}

void
Thread::drawConfigurationEdge(const rl::math::Vector& q0, const rl::math::Vector& q1, const bool& free)
{
	emit configurationEdgeRequested(q0, q1, free);
}

void
Thread::drawConfigurationPath(const rl::plan::VectorList& path)
{
	emit configurationPathRequested(path);
}

void
Thread::drawConfigurationVertex(const rl::math::Vector& q, const bool& free)
{
	emit configurationVertexRequested(q, free);
}

void
Thread::drawLine(const rl::math::Vector& xyz0, const rl::math::Vector& xyz1)
{
	emit lineRequested(xyz0, xyz1);
}

void
Thread::drawPoint(const rl::math::Vector& xyz)
{
	emit pointRequested(xyz);
}

void
Thread::drawSphere(const rl::math::Vector& center, const rl::math::Real& radius)
{
	emit sphereRequested(center, radius);
}

void
Thread::drawSweptVolume(const rl::plan::VectorList& path)
{
	emit sweptVolumeRequested(path);
}

void
Thread::drawWork(const rl::math::Transform& t)
{
	emit workRequested(t);
}

void
Thread::drawWorkEdge(const rl::math::Vector& q0, const rl::math::Vector& q1)
{
//	emit workEdgeRequested(q0, q1);
}

void
Thread::drawWorkPath(const rl::plan::VectorList& path)
{
	emit workPathRequested(path);
}

void
Thread::drawWorkVertex(const rl::math::Vector& q)
{
//	emit workVertexRequested(q);
}

void
Thread::reset()
{
	emit resetRequested();
}

void
Thread::resetEdges()
{
	emit edgeResetRequested();
}

void
Thread::resetLines()
{
	emit lineResetRequested();
}

void
Thread::resetPoints()
{
	emit pointResetRequested();
}

void
Thread::resetSpheres()
{
	emit sphereResetRequested();
}

void
Thread::resetVertices()
{
	emit vertexResetRequested();
}

void
Thread::setTargetFlangePose(const Eigen::Affine3f& T_base_flange)
{
	this->targetFlangeMatrix = T_base_flange.matrix();
	this->hasTargetFlangePose = true;
}

void
Thread::clearTargetFlangePose()
{
	this->hasTargetFlangePose = false;
	this->targetFlangeMatrix = FlangeMatrix::Identity();
}

void
Thread::copyConfiguration(rl::math::Vector& dst, const rl::math::Vector& src)
{
	dst.resize(src.size());
	
	for (Eigen::Index i = 0; i < src.size(); ++i)
	{
		dst[i] = src[i];
	}
}

void
Thread::appendToPath(rl::plan::VectorList& path, const std::vector<rl::math::Vector>& joints, std::size_t begin)
{
	for (std::size_t i = begin; i < joints.size(); ++i)
	{
		path.push_back(joints[i]);
	}
}

void
Thread::setStartConfiguration(const rl::math::Vector& q_home)
{
	Thread::copyConfiguration(this->qHome, q_home);
	this->hasExplicitStart = true;
}

void
Thread::clearStartConfiguration()
{
	this->hasExplicitStart = false;
}

void
Thread::setFlangeToTcp(const rl::math::Transform& T_flange_to_tcp)
{
	this->flangeToTcpMatrix = T_flange_to_tcp.matrix();
}

void
Thread::setIkTimeoutMs(int ms)
{
	this->ikTimeoutMs = (ms > 0) ? ms : 1;
}

void
Thread::setRailStepLen(double mm)
{
	this->railStepLen = (mm > 0.0) ? mm : 5.0;
}

void
Thread::setCartStepLen(double step)
{
	this->cartStepLen = (step > 0.0) ? step : 5.0;
}

void
Thread::planGoToStart(const rl::math::Vector& q_home,
	const Eigen::Affine3f& T_base_flange,
	const rl::math::Transform& T_flange_to_tcp,
	double railStepMm,
	double cartStep,
	int timeoutMs)
{
	this->stop();
	this->setStartConfiguration(q_home);
	this->setFlangeToTcp(T_flange_to_tcp);
	this->setRailStepLen(railStepMm);
	this->setCartStepLen(cartStep);
	this->setIkTimeoutMs(timeoutMs);
	this->setTargetFlangePose(T_base_flange);
	this->start();
}

void
Thread::planToFlange(const Eigen::Affine3f& T_base_flange)
{
	this->stop();
	this->setTargetFlangePose(T_base_flange);
	this->start();
}

bool
Thread::planGoToStartFromFlange()
{
	MainWindow* mw = MainWindow::instance();
	
	if (nullptr == mw || nullptr == mw->mdl || nullptr == mw->planner || nullptr == mw->model)
	{
		this->showMessage("Planner context is not initialized.");
		return false;
	}
	
	rl::mdl::Kinematic* kinematic = dynamic_cast<rl::mdl::Kinematic*>(mw->mdl.get());
	
	if (nullptr == kinematic)
	{
		this->showMessage("Kinematic model is not available.");
		return false;
	}
	
	const std::size_t dof = kinematic->getDof();
	
	if (!this->hasExplicitStart)
	{
		Thread::copyConfiguration(this->qHome, kinematic->getHomePosition());
	}
	
	if (this->qHome.size() != static_cast<Eigen::Index>(dof))
	{
		this->showMessage("q_home size mismatch with model DOF.");
		return false;
	}
	
	// Affine3f 法兰位姿 → RL Transform; TCP 仅用于地轨 Y 对齐
	rl::math::Transform T_base_flange;
	T_base_flange.matrix() = this->targetFlangeMatrix.cast<rl::math::Real>();
	
	rl::math::Transform T_flange_to_tcp;
	T_flange_to_tcp.matrix() = this->flangeToTcpMatrix;
	
	const rl::math::Transform T_world_tcp_start = T_base_flange * T_flange_to_tcp;
	const double yWeld = T_world_tcp_start.translation().y();
	
	RailLimitGuard railGuard;
	railGuard.joint = kinematic->getJoint(0);
	
	if (nullptr != railGuard.joint)
	{
		Thread::copyConfiguration(railGuard.min0, railGuard.joint->getMinimum());
		Thread::copyConfiguration(railGuard.max0, railGuard.joint->getMaximum());
		railGuard.armed = true;
	}
	
	auto checkStop = [&]() -> bool {
		return !this->running;
	};
	
	// ========== path1：只动地轨 Joint0，对齐焊接起点 Y，其它关节保持 Home ==========
	const double kRailStepMm = this->railStepLen;
	double yTarget = yWeld;
	
	if (nullptr != railGuard.joint && railGuard.joint->getDofPosition() == 1 && railGuard.min0.size() > 0)
	{
		yTarget = std::min(std::max(yWeld, railGuard.min0(0)), railGuard.max0(0));
	}
	
	std::vector<rl::math::Vector> path1;
	rl::math::Vector qRail;
	Thread::copyConfiguration(qRail, this->qHome);
	path1.push_back(qRail);
	
	const double y0 = this->qHome(0);
	const double yDir = (yTarget >= y0) ? 1.0 : -1.0;
	double y = y0;
	
	while (std::abs(yTarget - y) > kRailStepMm)
	{
		if (checkStop())
		{
			this->showMessage("Planning aborted.");
			return false;
		}
		
		y += yDir * kRailStepMm;
		qRail(0) = y;
		path1.push_back(qRail);
	}
	
	if (std::abs(yTarget - y) > 1e-9)
	{
		qRail(0) = yTarget;
		path1.push_back(qRail);
	}
	
	// ========== 固定 Joint0，后续 IK / RRT 不再采样地轨 ==========
	if (nullptr != railGuard.joint && railGuard.joint->getDofPosition() == 1)
	{
		const double eps = 1e-6;
		rl::math::Vector lo(1), hi(1);
		lo << (yTarget - eps);
		hi << (yTarget + eps);
		railGuard.joint->setMinimum(lo);
		railGuard.joint->setMaximum(hi);
	}
	
	// ========== 焊接起点关节解（path2 终点），IK 超时 500ms ==========
	rl::mdl::JacobianInverseKinematics ik(kinematic);
	ik.setDuration(std::chrono::milliseconds(this->ikTimeoutMs));
	ik.addGoal(T_base_flange, 0);
	kinematic->setPosition(path1.back());
	
	if (!ik.solve())
	{
		this->showMessage("IK failed: weld start configuration.");
		return false;
	}
	
	rl::math::Vector qGoalIk;
	Thread::copyConfiguration(qGoalIk, kinematic->getPosition());
	qGoalIk(0) = yTarget;
	
	// ========== path2：Joint0 锁定后的 RRT（path1 终点 → 焊接起点）==========
	Thread::copyConfiguration(this->qStart, path1.back());
	Thread::copyConfiguration(this->qGoal, qGoalIk);
	mw->planner->start = &this->qStart;
	mw->planner->goal = &this->qGoal;
	mw->planner->viewer = nullptr;
	
	if (!mw->planner->verify())
	{
		this->showMessage("Invalid start or goal configuration.");
		return false;
	}
	
	if (checkStop())
	{
		this->showMessage("Planning aborted.");
		return false;
	}
	
	this->showMessage("Solving...");
	
	const std::chrono::steady_clock::time_point solveBegin = std::chrono::steady_clock::now();
	const bool solved = mw->planner->solve();
	const std::chrono::steady_clock::time_point solveEnd = std::chrono::steady_clock::now();
	this->lastPlannerMs = std::chrono::duration_cast<std::chrono::duration<double>>(solveEnd - solveBegin).count() * 1000.0;
	
	if (!solved)
	{
		this->showMessage("Planner failed: home-rail pose to weld start.");
		return false;
	}
	
	rl::plan::VectorList sparse = mw->planner->getPath();
	
	if (nullptr != mw->optimizer)
	{
		mw->optimizer->setViewer(nullptr);
		mw->optimizer->process(sparse);
	}
	
	const double delta = this->cartStepLen;
	rl::math::Vector inter(mw->model->getDofPosition());
	std::vector<rl::math::Vector> path2;
	
	if (!sparse.empty())
	{
		path2.push_back(*sparse.begin());
	}
	
	rl::plan::VectorList::iterator it = sparse.begin();
	rl::plan::VectorList::iterator jt = sparse.begin();
	
	if (jt != sparse.end())
	{
		++jt;
	}
	
	for (; it != sparse.end() && jt != sparse.end(); ++it, ++jt)
	{
		if (checkStop())
		{
			this->showMessage("Planning aborted.");
			return false;
		}
		
		const rl::math::Real steps = std::ceil(mw->model->distance(*it, *jt) / delta);
		const rl::math::Real n = (steps < 1.0) ? 1.0 : steps;
		
		for (std::size_t k = 1; k < static_cast<std::size_t>(n) + 1; ++k)
		{
			mw->model->interpolate(*it, *jt, static_cast<rl::math::Real>(k) / n, inter);
			path2.push_back(inter);
		}
	}
	
	// 地轨保持 yTarget，避免限位窗口 eps 漂移
	for (rl::math::Vector& q : path2)
	{
		q(0) = yTarget;
	}
	
	railGuard.restore();
	
	// ========== 拼接 path1 + path2 ==========
	std::size_t p2Begin = 0;
	
	if (!path2.empty() && !path1.empty()
		&& (path2.front() - path1.back()).cwiseAbs().maxCoeff() < 1e-6)
	{
		p2Begin = 1;
	}
	
	this->lastPath.clear();
	Thread::appendToPath(this->lastPath, path1, 0);
	Thread::appendToPath(this->lastPath, path2, p2Begin);
	
	if (this->lastPath.empty())
	{
		this->showMessage("Planned path is empty.");
		return false;
	}
	
	kinematic->setPosition(kinematic->getHomePosition());
	kinematic->forwardPosition();
	mw->model->setPosition(this->qHome);
	mw->model->updateFrames();
	
	this->lastSolved = true;
	return true;
}

void
Thread::run()
{
	QMutexLocker lock(&MainWindow::instance()->mutex);
	
	this->running = true;
	this->lastSolved = false;
	this->lastPlannerMs = 0.0;
	this->lastPath.clear();
	
	bool solved = false;
	rl::plan::VectorList path;
	
	if (this->hasTargetFlangePose)
	{
		solved = this->planGoToStartFromFlange();
		path = this->lastPath;
		
		if (!solved)
		{
			emit planningFinished(this->lastPath, false, this->lastPlannerMs);
			return;
		}
	}
	else
	{
		if (!MainWindow::instance()->planner->verify())
		{
			this->showMessage("Invalid start or goal configuration.");
			emit planningFinished(this->lastPath, false, 0.0);
			return;
		}
		
		this->showMessage("Solving...");
		
		std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
		solved = MainWindow::instance()->planner->solve();
		std::chrono::steady_clock::time_point stop = std::chrono::steady_clock::now();
		
		this->lastPlannerMs = std::chrono::duration_cast<std::chrono::duration<double>>(stop - start).count() * 1000;
		this->lastSolved = solved;
		
		if (solved)
		{
			path = MainWindow::instance()->planner->getPath();
			this->lastPath = path;
		}
	}
	
	if (!MainWindow::instance()->nearestNeighbors.empty())
	{
		if (rl::plan::GnatNearestNeighbors* gnatNearestNeighbors = dynamic_cast<rl::plan::GnatNearestNeighbors*>(MainWindow::instance()->nearestNeighbors.front().get()))
		{
			boost::optional<std::size_t> checks = gnatNearestNeighbors->getChecks();
			(void)checks;
		}
		else if (rl::plan::KdtreeBoundingBoxNearestNeighbors* kdtreeBoundingBoxNearestNeighbors = dynamic_cast<rl::plan::KdtreeBoundingBoxNearestNeighbors*>(MainWindow::instance()->nearestNeighbors.front().get()))
		{
			boost::optional<std::size_t> checks = kdtreeBoundingBoxNearestNeighbors->getChecks();
			(void)checks;
		}
		else if (rl::plan::KdtreeNearestNeighbors* kdtreeNearestNeighbors = dynamic_cast<rl::plan::KdtreeNearestNeighbors*>(MainWindow::instance()->nearestNeighbors.front().get()))
		{
			boost::optional<std::size_t> checks = kdtreeNearestNeighbors->getChecks();
			(void)checks;
		}
	}
	
	if (!this->running)
	{
		emit planningFinished(this->lastPath, solved, this->lastPlannerMs);
		return;
	}
	
	if (solved)
	{
		this->drawConfigurationPath(path);
		
		if (!this->hasTargetFlangePose && nullptr != MainWindow::instance()->optimizer)
		{
			MainWindow::instance()->optimizer->setViewer(nullptr);
			MainWindow::instance()->optimizer->process(path);
			this->lastPath = path;
			this->drawConfigurationPath(path);
		}
		
		emit planningFinished(this->lastPath, true, this->lastPlannerMs);
		
		rl::math::Vector diff(MainWindow::instance()->model->getDofPosition());
		rl::math::Vector inter(MainWindow::instance()->model->getDofPosition());
		
		while (this->animate)
		{
			if (!this->running) break;
			
			rl::plan::VectorList::iterator i = path.begin();
			rl::plan::VectorList::iterator j = ++path.begin();
			
			if (i != path.end() && j != path.end())
			{
				this->drawConfiguration(*i);
				usleep(static_cast<std::size_t>(0.05f * 1000.0f * 1000.0f));
			}
			
			rl::math::Real delta = MainWindow::instance()->viewer->delta;
			
			for (; i != path.end() && j != path.end(); ++i, ++j)
			{
				diff = *j - *i;
				
				rl::math::Real steps = std::ceil(MainWindow::instance()->model->distance(*i, *j) / delta);
				
				for (std::size_t k = 1; k < steps + 1; ++k)
				{
					if (!this->running) break;
					
					MainWindow::instance()->model->interpolate(*i, *j, k / steps, inter);
					this->drawConfiguration(inter);
					usleep(static_cast<std::size_t>(0.05f * 1000.0f * 1000.0f));
				}
			}
			
			if (!this->running) break;
			
			rl::plan::VectorList::reverse_iterator ri = path.rbegin();
			rl::plan::VectorList::reverse_iterator rj = ++path.rbegin();
			
			if (ri != path.rend() && rj != path.rend())
			{
				this->drawConfiguration(*ri);
				usleep(static_cast<std::size_t>(0.05f * 1000.0f * 1000.0f));
			}
			
			for (; ri != path.rend() && rj != path.rend(); ++ri, ++rj)
			{
				diff = *rj - *ri;
				
				rl::math::Real steps = std::ceil(MainWindow::instance()->model->distance(*ri, *rj) / delta);
				
				for (std::size_t k = 1; k < steps + 1; ++k)
				{
					if (!this->running) break;
					
					MainWindow::instance()->model->interpolate(*ri, *rj, k / steps, inter);
					this->drawConfiguration(inter);
					usleep(static_cast<std::size_t>(0.05f * 1000.0f * 1000.0f));
				}
			}
		}
	}
	else
	{
		this->showMessage("Planner failed, no collision-free path to flange pose.");
		emit planningFinished(this->lastPath, false, this->lastPlannerMs);
	}
}

void
Thread::showMessage(const std::string& message)
{
	emit messageRequested(message);
}

void
Thread::stop()
{
	if (this->running)
	{
		this->running = false;
		
		while (!this->isFinished())
		{
			QThread::usleep(0);
		}
	}
}
