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

namespace 
{
	std::vector<float> RlVector2StdVector(const rl::math::Vector& q) {
		std::vector<float> pt(static_cast<std::size_t>(q.size()));
		for (int i = 0; i < q.size(); ++i)
		{
			pt[static_cast<std::size_t>(i)] = static_cast<float>(q[i]);
		}
		return pt;
	}
	rl::math::Vector StdVector2RlVector(const std::vector<float>& pt) {
		rl::math::Vector q(static_cast<int>(pt.size()));
		for (std::size_t i = 0; i < pt.size(); ++i)
		{
			q[static_cast<int>(i)] = static_cast<rl::math::Real>(pt[i]) / 180.0 * M_PI;
		}
		return q;
	}
}

Thread::Thread(QObject* parent) :
	QThread(parent),
	animate(true),
	quit(false),
	swept(false),
	running(false),
	qStart(),
	qGoal(),
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
Thread::setIkTimeoutMs(int ms)
{
	this->ikTimeoutMs = (ms > 0) ? ms : 1;
}

//void
//Thread::sortPose(const std::vector<std::pair<Eigen::Affine3f, Eigen::Affine3f>>& target_pose)
//{
//	this->stop();
//	if (nullptr == MainWindow::instance()->planner || nullptr == MainWindow::instance()->goal)
//	{
//		emit sendErrorMessage("sortPose: planner/goal/candidates not available.");
//		return;
//	}
//
//	rl::mdl::Kinematic* kinematic = dynamic_cast<rl::mdl::Kinematic*>(MainWindow::instance()->mdl.get());
//	if (nullptr == kinematic)
//	{
//		emit sendErrorMessage("sortPose: kinematic / planner not available.");
//		return;
//	}
//
//	rl::mdl::JacobianInverseKinematics ik(kinematic);
//	ik.setDuration(std::chrono::milliseconds(this->ikTimeoutMs));
//
//	bool found = false;
//	for (const auto& pt : target_pose)
//	{
//		rl::math::Transform T_goal;
//		T_goal.matrix() = pt.first.matrix().cast<rl::math::Real>();
//		ik.addGoal(T_goal, 0);
//		kinematic->setPosition(*MainWindow::instance()->start.get());
//		if (!ik.solve()) continue;
//
//		*MainWindow::instance()->goal = kinematic->getPosition();
//		MainWindow::instance()->planner->goal = MainWindow::instance()->goal.get();
//		MainWindow::instance()->planner->reset();
//		if (!MainWindow::instance()->planner->verify()) continue;
//
//		T_goal.matrix() = pt.second.matrix().cast<rl::math::Real>();
//		ik.addGoal(T_goal, 0);
//		kinematic->setPosition(*MainWindow::instance()->start.get());
//		if (!ik.solve()) continue;
//
//		tgt_joint = kinematic->getPosition();
//
//		found = true;
//		break;
//	}
//	if (!found)
//	{
//		emit sendErrorMessage("sortPose: no valid goal configuration.");
//		return;
//	}
//	this->start();
//}

void
Thread::sortPose(const std::vector<std::pair<Eigen::Affine3f, Eigen::Affine3f>>& target_pose)
{
	this->stop();

	MainWindow* mw = MainWindow::instance();
	if (nullptr == mw || nullptr == mw->planner || nullptr == mw->goal || nullptr == mw->start || nullptr == mw->model)
	{
		emit sendErrorMessage("sortPose: planner/start/goal/model not available.");
		return;
	}

	rl::mdl::Kinematic* kinematic = dynamic_cast<rl::mdl::Kinematic*>(mw->mdl.get());
	if (nullptr == kinematic)
	{
		emit sendErrorMessage("sortPose: kinematic / planner not available.");
		return;
	}

	const rl::math::Vector qStart = *mw->start;
	mw->planner->start = mw->start.get();
	mw->syncObstaclesToPlanner();

	auto solveIk = [&](const Eigen::Affine3f& pose, const rl::math::Vector& qSeed, rl::math::Vector& qOut) -> bool
		{
			rl::mdl::JacobianInverseKinematics ik(kinematic);
			ik.setDuration(std::chrono::milliseconds(this->ikTimeoutMs));
			rl::math::Transform T;
			Eigen::Affine3f temp = pose;
			temp.translation() *= 0.001;
			T.matrix() = temp.matrix().cast<rl::math::Real>();
			ik.addGoal(T, 0);
			kinematic->setPosition(qSeed);
			if (!ik.solve())
			{
				return false;
			}
			qOut = kinematic->getPosition();
			return true;
		};

	auto colliding = [&](const rl::math::Vector& q) -> bool
		{
			mw->model->setPosition(q);
			return mw->model->isColliding();
		};

	auto segmentFree = [&](const rl::math::Vector& q0, const rl::math::Vector& q1) -> bool
		{
			if (colliding(q0) || colliding(q1))
			{
				return false;
			}
			const rl::math::Real delta = (nullptr != mw->viewer) ? mw->viewer->delta : static_cast<rl::math::Real>(0.05);
			rl::math::Real steps = std::ceil(mw->model->distance(q0, q1) / delta);
			if (steps < 1.0)
			{
				steps = 1.0;
			}
			rl::math::Vector q(q0.size());
			for (int k = 1; k <= static_cast<int>(steps); ++k)
			{
				mw->model->interpolate(q0, q1, static_cast<rl::math::Real>(k) / steps, q);
				if (colliding(q))
				{
					return false;
				}
			}
			return true;
		};

	bool found = false;
	for (const auto& pt : target_pose)
	{
		rl::math::Vector qPre;
		if (!solveIk(pt.first, qStart, qPre))
		{
			continue;
		}
		if (colliding(qPre))
		{
			continue;
		}

		rl::math::Vector qGrasp;
		if (!solveIk(pt.second, qPre, qGrasp))
		{
			continue;
		}
		if (!segmentFree(qPre, qGrasp))
		{
			continue;
		}

		*mw->goal = qPre;
		mw->planner->goal = mw->goal.get();
		mw->planner->reset();
		if (!mw->planner->verify())
		{
			continue;
		}

		this->tgt_joint = qGrasp;
		found = true;
		break;
	}

	if (!found)
	{
		emit sendErrorMessage("sortPose: no valid pre-grasp / grasp pair.");
		return;
	}

	this->start();
}

void
Thread::run()
{
	QMutexLocker lock(&MainWindow::instance()->mutex);

	this->running = true;
	MainWindow::instance()->syncObstaclesToPlanner();

	std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
	bool solved = MainWindow::instance()->planner->solve();

	if (solved)
	{
		rl::plan::VectorList path;
		path = MainWindow::instance()->planner->getPath();

		if (!this->running) { this->running = false; return; }

		if (nullptr != MainWindow::instance()->optimizer)
		{
			MainWindow::instance()->optimizer->setViewer(nullptr);
			MainWindow::instance()->optimizer->process(path);
			if (tgt_joint.size() == 6) path.push_back(tgt_joint);
			this->drawConfigurationPath(path);
		}
		
		rl::plan::VectorList interplotPath;
		rl::math::Vector diff(MainWindow::instance()->model->getDofPosition());
		rl::math::Vector inter(MainWindow::instance()->model->getDofPosition());

		rl::plan::VectorList::iterator i = path.begin();
		rl::plan::VectorList::iterator j = ++path.begin();

		rl::math::Real delta = MainWindow::instance()->viewer->delta;
		for (; i != path.end() && j != path.end(); ++i, ++j)
		{
			diff = *j - *i;
			rl::math::Real steps = std::ceil(MainWindow::instance()->model->distance(*i, *j) / delta);
			for (std::size_t k = 1; k < steps + 1; ++k)
			{
				if (!this->running) { this->running = false; break; }
				MainWindow::instance()->model->interpolate(*i, *j, k / steps, inter);
				interplotPath.push_back(inter);
				/*this->drawConfiguration(inter);
				usleep(static_cast<std::size_t>(0.05f * 1000.0f * 1000.0f));*/
			}
		}
		this->drawConfigurationPath(interplotPath);
		std::vector<std::vector<float>> traj = toFloatTraj(interplotPath);

		std::chrono::steady_clock::time_point stop = std::chrono::steady_clock::now();
		double plannerDuration = std::chrono::duration_cast<std::chrono::duration<double>>(stop - start).count() * 1000;

		emit planningFinished(traj, plannerDuration);

		/*while (this->animate)
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
		}*/
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

std::vector<std::vector<float>> Thread::toFloatTraj(const rl::plan::VectorList& path)
{
	std::vector<std::vector<float>> traj;
	traj.reserve(path.size());
	for (const rl::math::Vector& q : path)
	{
		std::vector<float> pt(static_cast<std::size_t>(q.size()));
		for (int i = 0; i < q.size(); ++i)
		{
			pt[static_cast<std::size_t>(i)] = static_cast<float>(q[i]);
		}
		traj.push_back(std::move(pt));
	}
	return traj;
}

rl::plan::VectorList Thread::toVectorList(const std::vector<std::vector<float>>& traj)
{
	rl::plan::VectorList path;
	for (const std::vector<float>& pt : traj)
	{
		rl::math::Vector q(static_cast<int>(pt.size()));
		for (std::size_t i = 0; i < pt.size(); ++i)
		{
			q[static_cast<int>(i)] = static_cast<rl::math::Real>(pt[i]);
		}
		path.push_back(q);
	}
	return path;
}