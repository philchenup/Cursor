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
#include <vector>
#include <Eigen/Geometry>
#include <rl/math/Transform.h>
#include <rl/math/Vector.h>
#include <rl/plan/Viewer.h>

// QObject/QThread 的 operator new 不保证 16 字节对齐, 不能把 Affine3f / Transform
// 等定长可向量化类型直接做成成员, 否则会写坏相邻的 rl::math::Vector.
typedef Eigen::Matrix<float, 4, 4, Eigen::DontAlign> FlangeMatrix;
typedef Eigen::Matrix<double, 4, 4, Eigen::DontAlign> TransformMatrix;

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
	
	// 目标点法兰位姿 T_base_flange (替代 DiscretePoint startPoint)
	void setTargetFlangePose(const Eigen::Affine3f& T_base_flange);
	
	void clearTargetFlangePose();
	
	// q_home: Home 关节角(含地轨). 未设置时用 mdl->getHomePosition().
	void setStartConfiguration(const rl::math::Vector& q_home);
	
	void clearStartConfiguration();
	
	void setFlangeToTcp(const rl::math::Transform& T_flange_to_tcp);
	
	void setIkTimeoutMs(int ms);
	
	void setRailStepLen(double mm);
	
	void setCartStepLen(double step);
	
	// 按 IKGoToStartParams 写入参数后启动: 地轨 path1 + 锁 Joint0 + IK(500ms) + RRT path2
	void planGoToStart(const rl::math::Vector& q_home,
		const Eigen::Affine3f& T_base_flange,
		const rl::math::Transform& T_flange_to_tcp,
		double railStepLen = 5.0,
		double cartStepLen = 5.0,
		int timeoutMs = 500);
	
	void planToFlange(const Eigen::Affine3f& T_base_flange);
	
	bool animate;
	
	bool quit;
	
	bool swept;
	
	rl::plan::VectorList lastPath;
	
	double lastPlannerMs;
	
	bool lastSolved;
	
protected:
	
private:
	bool planGoToStartFromFlange();
	
	static void copyConfiguration(rl::math::Vector& dst, const rl::math::Vector& src);
	
	static void appendToPath(rl::plan::VectorList& path, const std::vector<rl::math::Vector>& joints, std::size_t begin = 0);
	
	bool running;
	
	FlangeMatrix targetFlangeMatrix;
	
	TransformMatrix flangeToTcpMatrix;
	
	bool hasTargetFlangePose;
	
	rl::math::Vector qHome;
	
	rl::math::Vector qStart;
	
	rl::math::Vector qGoal;
	
	bool hasExplicitStart;
	
	int ikTimeoutMs;
	
	double railStepLen;
	
	double cartStepLen;
	
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
	
	void planningFinished(const rl::plan::VectorList& path, bool solved, double plannerMs);
};

#endif // THREAD_H
