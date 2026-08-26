#include "JakaZu12Trajectory.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <thread>

namespace {

constexpr int kDof = 6;
constexpr double kPeriodS = 0.008;
constexpr double kMaxVel = 1.0; // rad/s，低于伺服 180 deg/s 硬限，利于平滑
constexpr double kApproachTol = 0.02;

JointValue toJoint(const rl::math::Vector& q)
{
    JointValue j{};
    for (int i = 0; i < kDof; ++i)
    {
        j.jVal[i] = q[i];
    }
    return j;
}

double maxAbsDiff(const JointValue& a, const JointValue& b)
{
    double d = 0.0;
    for (int i = 0; i < kDof; ++i)
    {
        d = std::max(d, std::fabs(a.jVal[i] - b.jVal[i]));
    }
    return d;
}

} // namespace

errno_t runJakaZu12Trajectory(JAKAZuRobot& robot, const std::vector<rl::math::Vector>& traj)
{
    if (traj.empty())
    {
        return ERR_INVALID_PARAMETER;
    }
    for (const auto& q : traj)
    {
        if (static_cast<int>(q.size()) < kDof)
        {
            return ERR_INVALID_PARAMETER;
        }
    }

    JointValue current{};
    errno_t ret = robot.get_joint_position(&current);
    if (ret != ERR_SUCC)
    {
        return ret;
    }

    const JointValue first = toJoint(traj.front());
    if (maxAbsDiff(current, first) > kApproachTol)
    {
        BOOL in_servo = FALSE;
        robot.is_in_servomove(&in_servo);
        if (in_servo)
        {
            robot.servo_move_enable(FALSE);
        }
        ret = robot.joint_move(&first, ABS, TRUE, 0.5);
        if (ret != ERR_SUCC)
        {
            return ret;
        }
    }

    robot.servo_move_use_joint_LPF(0.5);
    ret = robot.servo_move_enable(TRUE);
    if (ret != ERR_SUCC)
    {
        return ret;
    }

    JointValue prev = first;
    auto tick = std::chrono::steady_clock::now();
    const auto period = std::chrono::milliseconds(8);

    for (std::size_t n = 1; n < traj.size(); ++n)
    {
        const JointValue target = toJoint(traj[n]);
        const int steps = std::max(1, static_cast<int>(std::ceil(maxAbsDiff(prev, target) / kMaxVel / kPeriodS)));
        for (int k = 1; k <= steps; ++k)
        {
            const double s = static_cast<double>(k) / static_cast<double>(steps);
            JointValue cmd{};
            for (int i = 0; i < kDof; ++i)
            {
                cmd.jVal[i] = prev.jVal[i] + (target.jVal[i] - prev.jVal[i]) * s;
            }
            ret = robot.servo_j(&cmd, ABS, 1);
            if (ret != ERR_SUCC)
            {
                robot.motion_abort();
                robot.servo_move_enable(FALSE);
                return ret;
            }
            tick += period;
            std::this_thread::sleep_until(tick);
        }
        prev = target;
    }

    robot.servo_move_enable(FALSE);
    return ERR_SUCC;
}
