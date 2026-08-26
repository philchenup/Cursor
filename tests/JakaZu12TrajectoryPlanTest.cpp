#include "JakaZu12Trajectory.h"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>

namespace {

int g_failures = 0;

void expect(bool cond, const std::string& msg)
{
    if (!cond)
    {
        std::cerr << "FAIL: " << msg << std::endl;
        ++g_failures;
    }
}

double maxAbs(const std::array<double, jaka::kZu12Dof>& a, const std::array<double, jaka::kZu12Dof>& b)
{
    double m = 0.0;
    for (int i = 0; i < jaka::kZu12Dof; ++i)
    {
        m = std::max(m, std::fabs(a[static_cast<std::size_t>(i)] - b[static_cast<std::size_t>(i)]));
    }
    return m;
}

double peakVel(
    const std::vector<std::array<double, jaka::kZu12Dof>>& samples,
    double dt)
{
    double peak = 0.0;
    for (std::size_t i = 1; i < samples.size(); ++i)
    {
        for (int j = 0; j < jaka::kZu12Dof; ++j)
        {
            peak = std::max(
                peak,
                std::fabs(samples[i][static_cast<std::size_t>(j)] - samples[i - 1][static_cast<std::size_t>(j)]) / dt);
        }
    }
    return peak;
}

std::array<double, jaka::kZu12Dof> joints(double j0, double j1 = 0, double j2 = 0, double j3 = 0, double j4 = 0, double j5 = 0)
{
    return {j0, j1, j2, j3, j4, j5};
}

} // namespace

int main()
{
    using jaka::planJakaZu12ServoTrajectory;
    using jaka::JakaZu12TrajectoryOptions;
    using jaka::kJakaServoPeriodS;

    {
        const auto plan = planJakaZu12ServoTrajectory({}, {});
        expect(plan.error == ERR_INVALID_PARAMETER, "empty trajectory should be invalid");
        expect(plan.samples.empty(), "empty trajectory has no samples");
    }

    {
        const auto plan = planJakaZu12ServoTrajectory({joints(0.1, 0.2, 0.3, 0.4, 0.5, 0.6)}, {});
        expect(plan.error == ERR_SUCC, "single waypoint ok");
        expect(plan.samples.size() == 1, "single waypoint stays one sample");
        expect(maxAbs(plan.samples.front(), joints(0.1, 0.2, 0.3, 0.4, 0.5, 0.6)) < 1e-12, "single waypoint copied");
        expect(plan.step_num == 1, "default step_num is 1 (8ms)");
    }

    {
        JakaZu12TrajectoryOptions opt;
        opt.velocity_scale = 0.5;
        const auto start = joints(0, 0, 0, 0, 0, 0);
        const auto goal = joints(0.3, 0, 0, 0, 0, 0);
        const auto plan = planJakaZu12ServoTrajectory({start, goal}, opt);
        expect(plan.error == ERR_SUCC, "two-point plan ok");
        expect(plan.samples.size() >= 2, "two-point plan has samples");
        expect(maxAbs(plan.samples.front(), start) < 1e-6, "starts at first waypoint");
        expect(maxAbs(plan.samples.back(), goal) < 1e-6, "ends at last waypoint");

        bool monotonic = true;
        for (std::size_t i = 1; i < plan.samples.size(); ++i)
        {
            if (plan.samples[i][0] + 1e-9 < plan.samples[i - 1][0])
            {
                monotonic = false;
            }
        }
        expect(monotonic, "joint0 of two-point cubic should be monotonic");

        const double vmax = opt.max_joint_vel_rad_s[0] * opt.velocity_scale * 1.05;
        expect(peakVel(plan.samples, kJakaServoPeriodS * plan.step_num) <= vmax, "two-point respects velocity limit");
    }

    {
        JakaZu12TrajectoryOptions opt;
        opt.velocity_scale = 0.4;
        const std::vector<std::array<double, jaka::kZu12Dof>> wps = {
            joints(0, 0, 0, 0, 0, 0),
            joints(0.15, 0.08, -0.05, 0.02, 0, 0),
            joints(0.28, 0.04, 0.06, -0.03, 0.01, 0),
        };
        const auto plan = planJakaZu12ServoTrajectory(wps, opt);
        expect(plan.error == ERR_SUCC, "three-point plan ok");
        expect(plan.samples.size() > wps.size(), "spline is denser than waypoints");

        for (const auto& wp : wps)
        {
            double best = 1e9;
            for (const auto& s : plan.samples)
            {
                best = std::min(best, maxAbs(s, wp));
            }
            expect(best < 0.02, "samples pass near each waypoint");
        }

        const double vmax = opt.max_joint_vel_rad_s[0] * opt.velocity_scale * 1.08;
        expect(peakVel(plan.samples, kJakaServoPeriodS * plan.step_num) <= vmax, "three-point respects velocity limit");
    }

    {
        JakaZu12TrajectoryOptions opt;
        opt.sample_period_s = 0.016;
        const auto plan = planJakaZu12ServoTrajectory({joints(0), joints(0.1)}, opt);
        expect(plan.step_num == 2, "16ms sample period maps to step_num=2");
    }

    {
        std::vector<rl::math::Vector> traj;
        for (int i = 0; i < 4; ++i)
        {
            rl::math::Vector q(6);
            for (int j = 0; j < 6; ++j)
            {
                q[j] = 0.05 * i * (j == 0 ? 1.0 : 0.2);
            }
            traj.push_back(q);
        }

        std::vector<std::array<double, jaka::kZu12Dof>> wps;
        for (const auto& v : traj)
        {
            expect(v.size() == 6, "rl::math::Vector size is 6");
            std::array<double, jaka::kZu12Dof> six{};
            for (int j = 0; j < 6; ++j)
            {
                six[static_cast<std::size_t>(j)] = v[j];
            }
            wps.push_back(six);
        }
        const auto plan = planJakaZu12ServoTrajectory(wps, {});
        expect(plan.error == ERR_SUCC, "RL vector converted trajectory plans");
        expect(plan.samples.size() > 4, "RL vector trajectory is densified");
        expect(std::fabs(plan.samples.back()[0] - traj.back()[0]) < 1e-6, "RL last joint matches");
    }

    {
        const auto a = joints(0.1);
        const auto plan = planJakaZu12ServoTrajectory({a, a, a}, {});
        expect(plan.error == ERR_SUCC, "duplicate waypoints ok");
        expect(plan.samples.size() == 1, "duplicates collapse to one sample");
    }

    if (g_failures != 0)
    {
        std::cerr << g_failures << " check(s) failed" << std::endl;
        return EXIT_FAILURE;
    }
    std::cout << "JakaZu12TrajectoryPlan tests passed" << std::endl;
    return EXIT_SUCCESS;
}
