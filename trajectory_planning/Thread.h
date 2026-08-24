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

#ifndef THREAD_H
#define THREAD_H

#include <QThread>
#include <Eigen/Geometry>
#include <rl/math/Transform.h>
#include <rl/math/Vector.h>
#include <rl/plan/Viewer.h>

// QObject/QThread 的 operator new 不保证 16 字节对齐, 不能把 Affine3f 等
// 定长可向量化类型直接做成成员, 否则会写坏相邻的 rl::math::Vector.
typedef Eigen::Matrix<float, 4, 4, Eigen::DontAlign> FlangeMatrix;

class Thread : public QThread, public rl::plan::Viewer
{
	Q_OBJECT
	
public:
	Thread(QObject* parent = nullptr);
	
	virtual ~Thread();
	
	void drawConfiguration(const rl::math::Vector& q);
	
	void drawConfigurationEdge(const rl::math::Vector& q0, const rl::math::Vector& q1, const bool& free = true);
	
	void drawConfigurationPath(const rl::plan::VectorList& path);
	
	void drawConfigurationVertex(const rl::math::Vector& q, const bool& free = true);
	
	void drawLine(const rl::math::Vector& xyz0, const rl::math::Vector& xyz1);
	
	void drawPoint(const rl::math::Vector& xyz);
	
	void drawSphere(const rl::math::Vector& center, const rl::math::Real& radius);
	
	void drawSweptVolume(const rl::plan::VectorList& path);
	
	void drawWork(const rl::math::Transform& t);
	
	void drawWorkEdge(const rl::math::Vector& q0, const rl::math::Vector& q1);
	
	void drawWorkPath(const rl::plan::VectorList& path);
	
	void drawWorkVertex(const rl::math::Vector& q);
	
	void reset();
	
	void resetEdges();
	
	void resetLines();
	
	void resetPoints();
	
	void resetSpheres();
	
	void resetVertices();
	
	void run();
	
	void showMessage(const std::string& message);
	
	void stop();
	
	// 设置目标点法兰位姿 T_base_flange (Eigen::Affine3f).
	// run() 时先对该位姿做 Jacobian IK 得到 goal 关节角, 再走原 RRT 规划.
	void setTargetFlangePose(const Eigen::Affine3f& T_base_flange);
	
	void clearTargetFlangePose();
	
	// 可选: 显式指定起点关节角. 未设置时用 mdl->getPosition().
	void setStartConfiguration(const rl::math::Vector& q);
	
	void clearStartConfiguration();
	
	void setIkTimeoutMs(int ms);
	
	// 写入法兰目标后启动规划线程 (会先 stop 上一次规划).
	void planToFlange(const Eigen::Affine3f& T_base_flange);
	
	bool animate;
	
	bool quit;
	
	bool swept;
	
	// 最近一次规划结果, 规划结束后可直接读取
	rl::plan::VectorList lastPath;
	
	double lastPlannerMs;
	
	bool lastSolved;
	
protected:
	
private:
	bool resolveGoalFromFlangePose();
	
	static void copyConfiguration(rl::math::Vector& dst, const rl::math::Vector& src);
	
	bool running;
	
	FlangeMatrix targetFlangeMatrix;
	
	bool hasTargetFlangePose;
	
	rl::math::Vector qStart;
	
	rl::math::Vector qGoal;
	
	bool hasExplicitStart;
	
	int ikTimeoutMs;
	
signals:
	void configurationRequested(const rl::math::Vector& q);
	
	void configurationEdgeRequested(const rl::math::Vector& q0, const rl::math::Vector& q1, const bool& free);
	
	void configurationVertexRequested(const rl::math::Vector& q, const bool& free);
	
	void configurationPathRequested(const rl::plan::VectorList& path);
	
	void edgeResetRequested();
	
	void lineRequested(const rl::math::Vector& xyz0, const rl::math::Vector& xyz1);
	
	void lineResetRequested();
	
	void messageRequested(const std::string& message);
	
	void pointRequested(const rl::math::Vector& xyz);
	
	void pointResetRequested();
	
	void resetRequested();
	
	void sphereRequested(const rl::math::Vector& center, const rl::math::Real& radius);
	
	void sphereResetRequested();
	
	void sweptVolumeRequested(const rl::plan::VectorList& path);
	
	void vertexResetRequested();
	
	void workRequested(const rl::math::Transform& t);
	
	void workEdgeRequested(const rl::math::Vector& q0, const rl::math::Vector& q1);
	
	void workPathRequested(const rl::plan::VectorList& path);
	
	void workVertexRequested(const rl::math::Vector& q);
	
	// 规划结束: solved=false 时 lastPath 为空
	void planningFinished(const rl::plan::VectorList& path, bool solved, double plannerMs);
};

#endif // THREAD_H
