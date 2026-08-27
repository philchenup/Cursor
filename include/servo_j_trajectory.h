#ifndef SERVO_J_TRAJECTORY_H
#define SERVO_J_TRAJECTORY_H

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <functional>
#include <vector>

/**
 * JAKA servo_j 轨迹加密：按 8 ms 周期、关节速度/加速度约束生成绝对关节角。
 *
 * 手册约束：
 * - 发送周期 8 ms
 * - 关节速度硬限 180 deg/s（相邻点单轴增量 <= 1.44 deg）
 * - 用户侧完成插值；多轴共用同一时间参数，保持规划器的关节空间直线段
 *
 * 路径单位必须与 Params 一致：
 * - Robotics Library 规划结果一般为弧度，用 Params::forRadianPath()
 * - 若路点已是角度，用 Params::forDegreePath()
 */
namespace servo_j {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDt = 0.008;            // s，与控制器周期一致
constexpr double kHardVelDeg = 180.0;    // deg/s，手册硬限
constexpr double kCruiseVelDeg = 90.0;   // deg/s，规划巡航（低于硬限）
constexpr double kAccDeg = 400.0;        // deg/s^2

inline double deg2rad(double deg)
{
	return deg * kPi / 180.0;
}

inline double rad2deg(double rad)
{
	return rad * 180.0 / kPi;
}

struct Params
{
	double dt = kDt;
	double maxVel = deg2rad(kCruiseVelDeg);
	double maxAcc = deg2rad(kAccDeg);
	double hardVel = deg2rad(kHardVelDeg);
	double minDisp = 1e-9;

	/** RL / 规划器路点为弧度时使用。velDeg、accDeg 仍用角度填写，内部换成 rad/s。 */
	static Params forRadianPath(double velDeg = kCruiseVelDeg, double accDeg = kAccDeg)
	{
		Params p;
		p.dt = kDt;
		p.maxVel = deg2rad(velDeg);
		p.maxAcc = deg2rad(accDeg);
		p.hardVel = deg2rad(kHardVelDeg);
		return p;
	}

	/** 路点已是角度（即将写入 JAKA JointValue）时使用。 */
	static Params forDegreePath(double velDeg = kCruiseVelDeg, double accDeg = kAccDeg)
	{
		Params p;
		p.dt = kDt;
		p.maxVel = velDeg;
		p.maxAcc = accDeg;
		p.hardVel = kHardVelDeg;
		return p;
	}

	void clampToHardLimit()
	{
		if (maxVel > hardVel)
		{
			maxVel = hardVel;
		}
	}
};

struct Profile
{
	double T = 0.0;
	double Tacc = 0.0;
	double Tflat = 0.0;
	double v = 0.0;
	double a = 0.0;
	double s = 0.0;
	int N = 0;
};

/**
 * 以位移 s（最快跑的那根轴的 |Δq|）生成段间停稳的梯形/三角形轮廓，
 * 总时间对齐到 dt 的整数倍。
 */
inline Profile makeProfile(double s, Params params)
{
	Profile pr;
	params.clampToHardLimit();
	s = std::abs(s);
	pr.s = s;

	if (s <= params.minDisp || params.dt <= 0.0 || params.maxAcc <= 0.0)
	{
		return pr;
	}

	const double vMax = std::max(params.maxVel, 1e-12);
	const double aMax = params.maxAcc;
	const double sCruise = vMax * vMax / aMax;
	double Tmin = (s >= sCruise) ? (s / vMax + vMax / aMax) : (2.0 * std::sqrt(s / aMax));

	pr.N = std::max(1, static_cast<int>(std::ceil(Tmin / params.dt)));
	const double dqHard = params.hardVel * params.dt;
	if (dqHard > 0.0)
	{
		pr.N = std::max(pr.N, static_cast<int>(std::ceil(s / dqHard)));
	}

	for (int guard = 0; guard < 16; ++guard)
	{
		pr.T = static_cast<double>(pr.N) * params.dt;
		pr.a = aMax;
		const double disc = pr.a * pr.a * pr.T * pr.T - 4.0 * pr.a * s;
		if (disc < 0.0)
		{
			++pr.N;
			continue;
		}

		// 取较小根：同样时间内用 aMax 加速，巡航速度最低，且不超过 vMax
		pr.v = 0.5 * (pr.a * pr.T - std::sqrt(disc));
		pr.Tacc = pr.v / pr.a;
		pr.Tflat = pr.T - 2.0 * pr.Tacc;
		if (pr.Tflat < 0.0)
		{
			pr.Tflat = 0.0;
			pr.Tacc = pr.T / 2.0;
			pr.v = pr.a * pr.Tacc;
		}
		if (pr.v > vMax * 1.000001)
		{
			++pr.N;
			continue;
		}
		break;
	}

	return pr;
}

/** 轮廓上 t∈[0,T] 已走过的标量位移，范围 [0, s]。 */
inline double position(double t, const Profile& pr)
{
	if (t <= 0.0 || pr.s <= 0.0)
	{
		return 0.0;
	}
	if (t >= pr.T)
	{
		return pr.s;
	}
	if (t < pr.Tacc)
	{
		return 0.5 * pr.a * t * t;
	}
	if (t < pr.Tacc + pr.Tflat)
	{
		return 0.5 * pr.a * pr.Tacc * pr.Tacc + pr.v * (t - pr.Tacc);
	}

	const double tDec = t - pr.Tacc - pr.Tflat;
	const double sAcc = 0.5 * pr.a * pr.Tacc * pr.Tacc;
	const double sFlat = pr.v * pr.Tflat;
	return sAcc + sFlat + pr.v * tDec - 0.5 * pr.a * tDec * tDec;
}

/** 归一化路径参数 u∈[0,1]，供 model->interpolate(..., u, ...) 使用。 */
inline double normalizedPosition(double t, const Profile& pr)
{
	if (pr.s <= 0.0)
	{
		return 1.0;
	}
	const double u = position(t, pr) / pr.s;
	return std::min(1.0, std::max(0.0, u));
}

inline double maxAbsDiff(const std::vector<double>& a, const std::vector<double>& b)
{
	const std::size_t n = std::min(a.size(), b.size());
	double m = 0.0;
	for (std::size_t d = 0; d < n; ++d)
	{
		m = std::max(m, std::abs(b[d] - a[d]));
	}
	return m;
}

inline void lerp(const std::vector<double>& a, const std::vector<double>& b, double u, std::vector<double>& out)
{
	out.resize(a.size());
	for (std::size_t d = 0; d < a.size(); ++d)
	{
		const double bd = d < b.size() ? b[d] : a[d];
		out[d] = a[d] + u * (bd - a[d]);
	}
}

/**
 * 将路点加密为 8 ms 绝对关节角序列。
 * interpolate(a, b, u, out) 应实现关节空间插值（可换成 model->interpolate）。
 */
inline std::vector<std::vector<double> > densify(
	const std::vector<std::vector<double> >& waypoints,
	const Params& params,
	const std::function<void(const std::vector<double>&, const std::vector<double>&, double, std::vector<double>&)>& interpolate
		= lerp,
	const std::function<bool()>& isRunning = std::function<bool()>())
{
	std::vector<std::vector<double> > out;
	if (waypoints.empty())
	{
		return out;
	}

	out.push_back(waypoints.front());
	std::vector<double> inter = waypoints.front();

	for (std::size_t i = 0; i + 1 < waypoints.size(); ++i)
	{
		if (isRunning && !isRunning())
		{
			break;
		}

		const std::vector<double>& a = waypoints[i];
		const std::vector<double>& b = waypoints[i + 1];
		const double sMax = maxAbsDiff(a, b);
		const Profile profile = makeProfile(sMax, params);
		if (profile.N <= 0)
		{
			continue;
		}

		for (int k = 1; k <= profile.N; ++k)
		{
			if (isRunning && !isRunning())
			{
				break;
			}
			const double u = normalizedPosition(static_cast<double>(k) * params.dt, profile);
			interpolate(a, b, u, inter);
			out.push_back(inter);
		}
	}

	return out;
}

/** 弧度轨迹转为 JAKA JointValue 所用的角度。 */
inline std::vector<std::vector<float> > toJakaDegreeTraj(const std::vector<std::vector<double> >& pathRad)
{
	std::vector<std::vector<float> > traj;
	traj.reserve(pathRad.size());
	for (std::size_t i = 0; i < pathRad.size(); ++i)
	{
		std::vector<float> q(pathRad[i].size());
		for (std::size_t d = 0; d < pathRad[i].size(); ++d)
		{
			q[d] = static_cast<float>(rad2deg(pathRad[i][d]));
		}
		traj.push_back(q);
	}
	return traj;
}

/**
 * 替换原 viewer->delta 空间加密。Model 需提供：
 *   interpolate(const Vector&, const Vector&, Real u, Vector& out)
 *
 * Path 需支持 begin/end、empty、push_back，元素为可分量访问的关节向量
 * （rl::plan::VectorList / rl::math::Vector 满足）。
 */
template <typename Path, typename Model>
Path densifyPath(const Path& path, Model* model, const Params& params, const bool& running)
{
	Path out;
	if (path.empty() || model == 0)
	{
		return out;
	}

	out.push_back(*path.begin());
	typename Path::value_type inter = *path.begin();

	typename Path::const_iterator i = path.begin();
	typename Path::const_iterator j = path.begin();
	++j;

	for (; i != path.end() && j != path.end(); ++i, ++j)
	{
		if (!running)
		{
			break;
		}

		double sMax = 0.0;
		const int dof = static_cast<int>(i->size());
		for (int d = 0; d < dof; ++d)
		{
			sMax = std::max(sMax, std::abs((*j)[d] - (*i)[d]));
		}

		const Profile profile = makeProfile(sMax, params);
		if (profile.N <= 0)
		{
			continue;
		}

		for (int k = 1; k <= profile.N; ++k)
		{
			if (!running)
			{
				break;
			}
			const double u = normalizedPosition(static_cast<double>(k) * params.dt, profile);
			model->interpolate(*i, *j, u, inter);
			out.push_back(inter);
		}
	}

	return out;
}

} // namespace servo_j

#endif
