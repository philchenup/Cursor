#include "servo_j_trajectory.h"

#include <cmath>
#include <iostream>
#include <vector>

namespace {

int gFailures = 0;

void expect(bool cond, const char* msg)
{
	if (!cond)
	{
		std::cerr << "FAIL: " << msg << std::endl;
		++gFailures;
	}
}

void expectNear(double a, double b, double eps, const char* msg)
{
	if (std::abs(a - b) > eps)
	{
		std::cerr << "FAIL: " << msg << " (" << a << " vs " << b << ")" << std::endl;
		++gFailures;
	}
}

struct DummyModel
{
	void interpolate(const std::vector<double>& a, const std::vector<double>& b, double u, std::vector<double>& out)
	{
		servo_j::lerp(a, b, u, out);
	}
};

} // namespace

int main()
{
	const servo_j::Params rad = servo_j::Params::forRadianPath(90.0, 400.0);
	const servo_j::Params deg = servo_j::Params::forDegreePath(90.0, 400.0);

	// 1) 时间对齐到 8 ms，终点到达
	{
		const double s = servo_j::deg2rad(20.0);
		const servo_j::Profile pr = servo_j::makeProfile(s, rad);
		expect(pr.N >= 1, "profile has samples");
		expectNear(pr.T, pr.N * rad.dt, 1e-12, "T is N*dt");
		expectNear(servo_j::position(0.0, pr), 0.0, 1e-12, "start at 0");
		expectNear(servo_j::position(pr.T, pr), s, 1e-9, "end at s");
		expect(pr.v <= rad.maxVel * 1.001, "cruise below maxVel");
	}

	// 2) 相邻点单轴增量不超过 180 deg/s * 8 ms
	{
		std::vector<std::vector<double> > waypoints;
		std::vector<double> q0(6, 0.0);
		std::vector<double> q1(6, 0.0);
		q1[0] = servo_j::deg2rad(45.0);
		q1[3] = servo_j::deg2rad(10.0);
		waypoints.push_back(q0);
		waypoints.push_back(q1);

		const std::vector<std::vector<double> > traj = servo_j::densify(waypoints, rad);
		expect(!traj.empty(), "traj not empty");
		expectNear(traj.front()[0], 0.0, 1e-12, "starts at q0");
		expectNear(traj.back()[0], q1[0], 1e-6, "ends at q1");

		const double dqHard = rad.hardVel * rad.dt;
		double peakVel = 0.0;
		for (std::size_t k = 1; k < traj.size(); ++k)
		{
			const double dq = servo_j::maxAbsDiff(traj[k - 1], traj[k]);
			peakVel = std::max(peakVel, dq / rad.dt);
			if (dq > dqHard * 1.001)
			{
				std::cerr << "FAIL: step " << k << " dq=" << servo_j::rad2deg(dq)
						  << " deg exceeds 1.44 deg" << std::endl;
				++gFailures;
			}
		}
		expect(peakVel <= rad.maxVel * 1.001, "peak vel near cruise cap");
		expect(static_cast<double>(traj.size() - 1) * rad.dt > 0.05, "move takes real time");
	}

	// 3) 多轴同步：直线段，慢轴不独自提前到点
	{
		std::vector<double> a(2);
		a[0] = 0.0;
		a[1] = 0.0;
		std::vector<double> b(2);
		b[0] = 30.0;
		b[1] = 6.0;
		std::vector<std::vector<double> > waypoints;
		waypoints.push_back(a);
		waypoints.push_back(b);
		const std::vector<std::vector<double> > traj = servo_j::densify(waypoints, deg);

		for (std::size_t k = 1; k + 1 < traj.size(); ++k)
		{
			const double r0 = traj[k][0] / 30.0;
			const double r1 = traj[k][1] / 6.0;
			expectNear(r0, r1, 1e-6, "shared path parameter");
		}
		expectNear(traj.back()[0], 30.0, 1e-6, "joint0 end");
		expectNear(traj.back()[1], 6.0, 1e-6, "joint1 end");
	}

	// 4) 起点速度接近 0：第一拍增量远小于巡航步长
	{
		const double s = 40.0;
		const servo_j::Profile pr = servo_j::makeProfile(s, deg);
		const double ds0 = servo_j::position(deg.dt, pr);
		const double dsCruise = pr.v * deg.dt;
		expect(ds0 < 0.5 * dsCruise || pr.Tflat <= deg.dt, "first step smaller than cruise");
	}

	// 5) 短位移到不了巡航速度
	{
		const servo_j::Profile pr = servo_j::makeProfile(0.2, deg);
		expect(pr.v < deg.maxVel, "short segment peak below cruise");
		expectNear(servo_j::position(pr.T, pr), 0.2, 1e-9, "short segment reaches s");
	}

	// 6) 零位移 / 单路点
	{
		std::vector<std::vector<double> > one(1, std::vector<double>(6, 0.1));
		const std::vector<std::vector<double> > traj = servo_j::densify(one, rad);
		expect(traj.size() == 1, "single waypoint");

		std::vector<std::vector<double> > dup;
		dup.push_back(std::vector<double>(3, 1.0));
		dup.push_back(std::vector<double>(3, 1.0));
		const std::vector<std::vector<double> > trajDup = servo_j::densify(dup, rad);
		expect(trajDup.size() == 1, "duplicate waypoint skipped");
	}

	// 7) 模板路径接口（模拟 RL VectorList = vector<vector>）
	{
		DummyModel model;
		bool running = true;
		std::vector<std::vector<double> > path;
		path.push_back(std::vector<double>(2, 0.0));
		path.push_back(std::vector<double>(2, servo_j::deg2rad(12.0)));
		const std::vector<std::vector<double> > out = servo_j::densifyPath(path, &model, rad, running);
		expect(out.size() >= 2, "template densifyPath produced points");
		expectNear(out.back()[0], path.back()[0], 1e-6, "template ends at goal");
	}

	// 8) 转角度后仍满足 1.44 deg/拍
	{
		std::vector<std::vector<double> > waypoints;
		waypoints.push_back(std::vector<double>(1, 0.0));
		waypoints.push_back(std::vector<double>(1, servo_j::deg2rad(60.0)));
		const std::vector<std::vector<double> > radTraj = servo_j::densify(waypoints, rad);
		const std::vector<std::vector<float> > degTraj = servo_j::toJakaDegreeTraj(radTraj);
		expect(degTraj.size() == radTraj.size(), "degree traj same length");
		for (std::size_t k = 1; k < degTraj.size(); ++k)
		{
			const double dq = std::abs(degTraj[k][0] - degTraj[k - 1][0]);
			expect(dq <= 1.44 * 1.001, "JAKA degree step <= 1.44");
		}
	}

	// 9) float 关节接口：未加密 -> 8 ms 加密
	{
		std::vector<std::vector<float> > sparse(2, std::vector<float>(6, 0.0f));
		sparse[1][0] = 30.0f;
		sparse[1][1] = -12.0f;

		const std::vector<std::vector<float> > dense = servo_j::densifyJoints8ms(sparse);
		expect(dense.size() > sparse.size(), "float densify adds samples");
		expectNear(dense.front()[0], 0.0, 1e-5, "float starts at first waypoint");
		expectNear(dense.back()[0], 30.0, 1e-4, "float ends at last waypoint");
		expectNear(dense.back()[1], -12.0, 1e-4, "float joint1 end");

		const double dqHard = 180.0 * 0.008;
		for (std::size_t k = 1; k < dense.size(); ++k)
		{
			double dq = 0.0;
			for (std::size_t d = 0; d < dense[k].size(); ++d)
			{
				dq = std::max(dq, static_cast<double>(std::abs(dense[k][d] - dense[k - 1][d])));
			}
			expect(dq <= dqHard * 1.001, "float step <= 1.44 deg");
		}

		const std::vector<std::vector<float> > empty = servo_j::densifyJoints8ms(std::vector<std::vector<float> >());
		expect(empty.empty(), "empty input stays empty");
	}

	if (gFailures == 0)
	{
		std::cout << "All servo_j trajectory tests passed." << std::endl;
		return 0;
	}
	std::cerr << gFailures << " test(s) failed." << std::endl;
	return 1;
}
