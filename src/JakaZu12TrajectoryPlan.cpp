#include "JakaZu12Trajectory.h"

#include <algorithm>
#include <cmath>
#include <vector>

namespace jaka {
namespace {

constexpr double kDuplicateEps = 1e-9;

double effectiveVel(const JakaZu12TrajectoryOptions& options, int joint)
{
    const double scale = std::max(options.velocity_scale, 1e-3);
    return std::max(options.max_joint_vel_rad_s[static_cast<std::size_t>(joint)] * scale, 1e-3);
}

double samplePeriod(const JakaZu12TrajectoryOptions& options)
{
    const double period = options.sample_period_s > 0.0 ? options.sample_period_s : kJakaServoPeriodS;
    const double steps = std::max(1.0, std::round(period / kJakaServoPeriodS));
    return steps * kJakaServoPeriodS;
}

bool nearlyEqual(const std::array<double, kZu12Dof>& a, const std::array<double, kZu12Dof>& b)
{
    for (int i = 0; i < kZu12Dof; ++i)
    {
        if (std::fabs(a[static_cast<std::size_t>(i)] - b[static_cast<std::size_t>(i)]) > kDuplicateEps)
        {
            return false;
        }
    }
    return true;
}

std::vector<std::array<double, kZu12Dof>> uniqueWaypoints(
    const std::vector<std::array<double, kZu12Dof>>& waypoints)
{
    std::vector<std::array<double, kZu12Dof>> unique;
    unique.reserve(waypoints.size());
    for (const auto& point : waypoints)
    {
        if (unique.empty() || !nearlyEqual(unique.back(), point))
        {
            unique.push_back(point);
        }
    }
    return unique;
}

double segmentDuration(
    const std::array<double, kZu12Dof>& from,
    const std::array<double, kZu12Dof>& to,
    const JakaZu12TrajectoryOptions& options)
{
    double dt = samplePeriod(options);
    for (int j = 0; j < kZu12Dof; ++j)
    {
        const double dq = std::fabs(to[static_cast<std::size_t>(j)] - from[static_cast<std::size_t>(j)]);
        dt = std::max(dt, dq / effectiveVel(options, j));
    }
    return dt;
}

// 钳制三次样条：端点速度为 0，保证起步/停止无冲击。
void clampedCubicSlopes(
    const std::vector<double>& t,
    const std::vector<double>& y,
    std::vector<double>* slopes)
{
    const int n = static_cast<int>(t.size());
    slopes->assign(static_cast<std::size_t>(n), 0.0);
    if (n < 2)
    {
        return;
    }
    if (n == 2)
    {
        return; // 两端速度为 0 的 Hermite 三次
    }

    const int interior = n - 2;
    std::vector<double> a(static_cast<std::size_t>(interior), 0.0);
    std::vector<double> b(static_cast<std::size_t>(interior), 0.0);
    std::vector<double> c(static_cast<std::size_t>(interior), 0.0);
    std::vector<double> d(static_cast<std::size_t>(interior), 0.0);

    for (int i = 1; i <= n - 2; ++i)
    {
        const double h0 = t[static_cast<std::size_t>(i)] - t[static_cast<std::size_t>(i - 1)];
        const double h1 = t[static_cast<std::size_t>(i + 1)] - t[static_cast<std::size_t>(i)];
        const int row = i - 1;
        b[static_cast<std::size_t>(row)] = 2.0 * (h0 + h1);
        if (row > 0)
        {
            a[static_cast<std::size_t>(row)] = h0;
        }
        if (row < interior - 1)
        {
            c[static_cast<std::size_t>(row)] = h1;
        }
        d[static_cast<std::size_t>(row)] =
            3.0 * ((y[static_cast<std::size_t>(i + 1)] - y[static_cast<std::size_t>(i)]) * h0 / h1 +
                   (y[static_cast<std::size_t>(i)] - y[static_cast<std::size_t>(i - 1)]) * h1 / h0);
    }

    // Thomas 追赶法
    for (int i = 1; i < interior; ++i)
    {
        const double w = a[static_cast<std::size_t>(i)] / b[static_cast<std::size_t>(i - 1)];
        b[static_cast<std::size_t>(i)] -= w * c[static_cast<std::size_t>(i - 1)];
        d[static_cast<std::size_t>(i)] -= w * d[static_cast<std::size_t>(i - 1)];
    }
    std::vector<double> k(static_cast<std::size_t>(interior), 0.0);
    k[static_cast<std::size_t>(interior - 1)] =
        d[static_cast<std::size_t>(interior - 1)] / b[static_cast<std::size_t>(interior - 1)];
    for (int i = interior - 2; i >= 0; --i)
    {
        k[static_cast<std::size_t>(i)] =
            (d[static_cast<std::size_t>(i)] - c[static_cast<std::size_t>(i)] * k[static_cast<std::size_t>(i + 1)]) /
            b[static_cast<std::size_t>(i)];
    }
    for (int i = 0; i < interior; ++i)
    {
        (*slopes)[static_cast<std::size_t>(i + 1)] = k[static_cast<std::size_t>(i)];
    }
}

double hermiteEval(double y0, double y1, double k0, double k1, double h, double s)
{
    const double s2 = s * s;
    const double s3 = s2 * s;
    const double h00 = 2.0 * s3 - 3.0 * s2 + 1.0;
    const double h10 = s3 - 2.0 * s2 + s;
    const double h01 = -2.0 * s3 + 3.0 * s2;
    const double h11 = s3 - s2;
    return h00 * y0 + h10 * h * k0 + h01 * y1 + h11 * h * k1;
}

int findSegment(const std::vector<double>& t, double query)
{
    const int last = static_cast<int>(t.size()) - 2;
    if (query <= t.front())
    {
        return 0;
    }
    if (query >= t.back())
    {
        return last;
    }
    auto it = std::upper_bound(t.begin(), t.end(), query);
    int idx = static_cast<int>(std::distance(t.begin(), it)) - 1;
    return std::max(0, std::min(idx, last));
}

std::array<double, kZu12Dof> evalSpline(
    const std::vector<double>& t,
    const std::array<std::vector<double>, kZu12Dof>& y,
    const std::array<std::vector<double>, kZu12Dof>& k,
    double query)
{
    const int seg = findSegment(t, query);
    const double h = t[static_cast<std::size_t>(seg + 1)] - t[static_cast<std::size_t>(seg)];
    const double s = h > 0.0 ? (query - t[static_cast<std::size_t>(seg)]) / h : 0.0;
    const double clamped_s = std::min(1.0, std::max(0.0, s));

    std::array<double, kZu12Dof> q{};
    for (int j = 0; j < kZu12Dof; ++j)
    {
        q[static_cast<std::size_t>(j)] = hermiteEval(
            y[static_cast<std::size_t>(j)][static_cast<std::size_t>(seg)],
            y[static_cast<std::size_t>(j)][static_cast<std::size_t>(seg + 1)],
            k[static_cast<std::size_t>(j)][static_cast<std::size_t>(seg)],
            k[static_cast<std::size_t>(j)][static_cast<std::size_t>(seg + 1)],
            h,
            clamped_s);
    }
    return q;
}

double peakVelRatio(
    const std::vector<std::array<double, kZu12Dof>>& samples,
    double dt,
    const JakaZu12TrajectoryOptions& options)
{
    double ratio = 0.0;
    for (std::size_t i = 1; i < samples.size(); ++i)
    {
        for (int j = 0; j < kZu12Dof; ++j)
        {
            const double v = std::fabs(
                                 samples[i][static_cast<std::size_t>(j)] -
                                 samples[i - 1][static_cast<std::size_t>(j)]) /
                             dt;
            ratio = std::max(ratio, v / effectiveVel(options, j));
        }
    }
    return ratio;
}

} // namespace

JakaZu12ServoPlan planJakaZu12ServoTrajectory(
    const std::vector<std::array<double, kZu12Dof>>& joint_waypoints_rad,
    const JakaZu12TrajectoryOptions& options)
{
    JakaZu12ServoPlan plan;
    const double dt = samplePeriod(options);
    plan.step_num = static_cast<unsigned int>(std::max(1.0, std::round(dt / kJakaServoPeriodS)));

    if (joint_waypoints_rad.empty())
    {
        plan.error = ERR_INVALID_PARAMETER;
        return plan;
    }

    const std::vector<std::array<double, kZu12Dof>> waypoints = uniqueWaypoints(joint_waypoints_rad);
    if (waypoints.size() == 1)
    {
        plan.samples.push_back(waypoints.front());
        return plan;
    }

    std::vector<double> t(waypoints.size(), 0.0);
    for (std::size_t i = 1; i < waypoints.size(); ++i)
    {
        t[i] = t[i - 1] + segmentDuration(waypoints[i - 1], waypoints[i], options);
    }

    auto buildSamples = [&](const std::vector<double>& times) {
        std::array<std::vector<double>, kZu12Dof> y;
        std::array<std::vector<double>, kZu12Dof> k;
        for (int j = 0; j < kZu12Dof; ++j)
        {
            y[static_cast<std::size_t>(j)].resize(waypoints.size());
            for (std::size_t i = 0; i < waypoints.size(); ++i)
            {
                y[static_cast<std::size_t>(j)][i] = waypoints[i][static_cast<std::size_t>(j)];
            }
            clampedCubicSlopes(times, y[static_cast<std::size_t>(j)], &k[static_cast<std::size_t>(j)]);
        }

        std::vector<std::array<double, kZu12Dof>> samples;
        const double total = times.back();
        const int count = std::max(2, static_cast<int>(std::ceil(total / dt)) + 1);
        samples.reserve(static_cast<std::size_t>(count));
        for (int i = 0; i < count; ++i)
        {
            const double query = std::min(total, static_cast<double>(i) * dt);
            samples.push_back(evalSpline(times, y, k, query));
        }
        if (!nearlyEqual(samples.back(), waypoints.back()))
        {
            samples.push_back(waypoints.back());
        }
        return samples;
    };

    std::vector<double> times = t;
    std::vector<std::array<double, kZu12Dof>> samples = buildSamples(times);
    for (int iter = 0; iter < 4; ++iter)
    {
        const double ratio = peakVelRatio(samples, dt, options);
        if (ratio <= 1.001)
        {
            break;
        }
        for (std::size_t i = 1; i < times.size(); ++i)
        {
            times[i] *= ratio;
        }
        samples = buildSamples(times);
    }

    // 相邻点仍超速时再插入中间点（应对局部尖峰）
    std::vector<std::array<double, kZu12Dof>> densified;
    densified.reserve(samples.size() * 2);
    densified.push_back(samples.front());
    for (std::size_t i = 1; i < samples.size(); ++i)
    {
        const auto& prev = densified.back();
        const auto& next = samples[i];
        double needed = 1.0;
        for (int j = 0; j < kZu12Dof; ++j)
        {
            const double dq = std::fabs(next[static_cast<std::size_t>(j)] - prev[static_cast<std::size_t>(j)]);
            needed = std::max(needed, dq / (effectiveVel(options, j) * dt));
        }
        const int splits = std::max(1, static_cast<int>(std::ceil(needed)));
        for (int s = 1; s <= splits; ++s)
        {
            const double alpha = static_cast<double>(s) / static_cast<double>(splits);
            std::array<double, kZu12Dof> mid{};
            for (int j = 0; j < kZu12Dof; ++j)
            {
                mid[static_cast<std::size_t>(j)] =
                    prev[static_cast<std::size_t>(j)] * (1.0 - alpha) +
                    next[static_cast<std::size_t>(j)] * alpha;
            }
            densified.push_back(mid);
        }
    }

    plan.samples = std::move(densified);
    return plan;
}

} // namespace jaka
