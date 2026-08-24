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

#include <chrono>
#include <fstream>
#include <QApplication>
#include <QDateTime>
#include <QMutexLocker>
#include <rl/math/Quaternion.h>
#include <rl/math/Unit.h>
#include <rl/mdl/JacobianInverseKinematics.h>
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

Thread::Thread(QObject* parent) :
	QThread(parent),
	animate(true),
	quit(false),
	swept(false),
	lastPlannerMs(0.0),
	lastSolved(false),
	running(false),
	targetFlangeMatrix(FlangeMatrix::Identity()),
	hasTargetFlangePose(false),
	qStart(),
	qGoal(),
	hasExplicitStart(false),
	ikTimeoutMs(500)
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
	// 按元素拷贝, 避免跨 DLL 时 Eigen 对动态 Vector 做 move/对齐释放
	dst.resize(src.size());
	
	for (Eigen::Index i = 0; i < src.size(); ++i)
	{
		dst[i] = src[i];
	}
}

void
Thread::setStartConfiguration(const rl::math::Vector& q)
{
	Thread::copyConfiguration(this->qStart, q);
	this->hasExplicitStart = true;
}

void
Thread::clearStartConfiguration()
{
	this->hasExplicitStart = false;
}

void
Thread::setIkTimeoutMs(int ms)
{
	this->ikTimeoutMs = (ms > 0) ? ms : 1;
}

void
Thread::planToFlange(const Eigen::Affine3f& T_base_flange)
{
	this->stop();
	this->setTargetFlangePose(T_base_flange);
	this->start();
}

bool
Thread::resolveGoalFromFlangePose()
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
	
	const std::size_t dof = kinematic->getDofPosition();
	
	if (!this->hasExplicitStart)
	{
		Thread::copyConfiguration(this->qStart, kinematic->getPosition());
	}
	
	if (this->qStart.size() != static_cast<Eigen::Index>(dof))
	{
		this->showMessage("Start configuration size mismatch with model DOF.");
		return false;
	}
	
	// 以起点为 IK 初值, 目标为法兰系位姿 T_base_flange (不再乘 TCP 偏移)
	kinematic->setPosition(this->qStart);
	kinematic->forwardPosition();
	
	rl::math::Transform T_base_flange;
	T_base_flange.matrix() = this->targetFlangeMatrix.cast<rl::math::Real>();
	
	rl::mdl::JacobianInverseKinematics ik(kinematic);
	ik.setDuration(std::chrono::milliseconds(this->ikTimeoutMs));
	ik.addGoal(T_base_flange, 0);
	
	if (!ik.solve())
	{
		this->showMessage("Flange-pose IK failed, target pose may be unreachable.");
		return false;
	}
	
	Thread::copyConfiguration(this->qGoal, kinematic->getPosition());
	
	mw->model->setPosition(this->qStart);
	mw->model->updateFrames();
	
	if (mw->model->isColliding())
	{
		this->showMessage("Start configuration is in collision.");
		return false;
	}
	
	mw->model->setPosition(this->qGoal);
	mw->model->updateFrames();
	
	if (mw->model->isColliding())
	{
		this->showMessage("Goal configuration from flange pose is in collision.");
		return false;
	}
	
	mw->model->setPosition(this->qStart);
	mw->model->updateFrames();
	
	// planner->start / goal 是裸指针, 指向本线程持有的 qStart/qGoal, 保证 solve() 期间有效
	mw->planner->start = &this->qStart;
	mw->planner->goal = &this->qGoal;
	
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
	
	if (this->hasTargetFlangePose)
	{
		if (!this->resolveGoalFromFlangePose())
		{
			emit planningFinished(this->lastPath, false, 0.0);
			return;
		}
	}
	
	if (!MainWindow::instance()->planner->verify())
	{
		this->showMessage("Invalid start or goal configuration.");
		emit planningFinished(this->lastPath, false, 0.0);
		return;
	}
	
	this->showMessage("Solving...");
	
	std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
	bool solved = MainWindow::instance()->planner->solve();
	std::chrono::steady_clock::time_point stop = std::chrono::steady_clock::now();
	
	double plannerDuration = std::chrono::duration_cast<std::chrono::duration<double>>(stop - start).count() * 1000;
	this->lastPlannerMs = plannerDuration;
	this->lastSolved = solved;
	
	if (!MainWindow::instance()->nearestNeighbors.empty())
	{
		if (rl::plan::GnatNearestNeighbors* gnatNearestNeighbors = dynamic_cast<rl::plan::GnatNearestNeighbors*>(MainWindow::instance()->nearestNeighbors.front().get()))
		{
			
			boost::optional<std::size_t> checks = gnatNearestNeighbors->getChecks();
		
		}
		else if (rl::plan::KdtreeBoundingBoxNearestNeighbors* kdtreeBoundingBoxNearestNeighbors = dynamic_cast<rl::plan::KdtreeBoundingBoxNearestNeighbors*>(MainWindow::instance()->nearestNeighbors.front().get()))
		{
			boost::optional<std::size_t> checks = kdtreeBoundingBoxNearestNeighbors->getChecks();
			
		}
		else if (rl::plan::KdtreeNearestNeighbors* kdtreeNearestNeighbors = dynamic_cast<rl::plan::KdtreeNearestNeighbors*>(MainWindow::instance()->nearestNeighbors.front().get()))
		{
			boost::optional<std::size_t> checks = kdtreeNearestNeighbors->getChecks();
		}
	}

	rl::plan::VectorList path;
	
	if (solved)
	{
		path = MainWindow::instance()->planner->getPath();
		
		rl::plan::VectorList::iterator i = path.begin();
		rl::plan::VectorList::iterator j = ++path.begin();
		
		rl::math::Real length = 0;
		
		for (; i != path.end() && j != path.end(); ++i, ++j)
		{
			length += MainWindow::instance()->model->distance(*i, *j);
		}
	}

	if (solved)
	{
		this->lastPath = path;
	}
	
	if (!this->running)
	{
		emit planningFinished(this->lastPath, solved, plannerDuration);
		return;
	}

	if (solved)
	{
		this->drawConfigurationPath(path);
		
		if (!this->running)
		{
			emit planningFinished(this->lastPath, true, plannerDuration);
			return;
		}
		
		if (nullptr != MainWindow::instance()->optimizer)
		{
			MainWindow::instance()->optimizer->setViewer(nullptr);
			MainWindow::instance()->optimizer->process(path);
			this->lastPath = path;
			this->drawConfigurationPath(path);
		}
		
		emit planningFinished(this->lastPath, true, plannerDuration);
		
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
		emit planningFinished(this->lastPath, false, plannerDuration);
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
