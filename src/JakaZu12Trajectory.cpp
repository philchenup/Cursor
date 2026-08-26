#include "JakaZu12Trajectory.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <thread>

namespace jaka {
namespace {

template <typename Vec>
bool copySix(const Vec& v, std::array<double, kZu12Dof>* out)
{
    if (static_cast<int>(v.size()) < kZu12Dof)
    {
        return false;
    }
    for (int i = 0; i < kZu12Dof; ++i)
    {
        (*out)[static_cast<std::size_t>(i)] = static_cast<double>(v[i]);
    }
    return true;
}

double maxAbsDiff(const JointValue& a, const std::array<double, kZu12Dof>& b)
{
    double m = 0.0;
    for (int i = 0; i < kZu12Dof; ++i)
    {
        m = std::max(m, std::fabs(a.jVal[i] - b[static_cast<std::size_t>(i)]));
    }
    return m;
}

bool jointsReached(const JointValue& actual, const std::array<double, kZu12Dof>& target, double tol)
{
    return maxAbsDiff(actual, target) <= tol;
}

JointValue toJointValue(const std::array<double, kZu12Dof>& q)
{
    JointValue j{};
    for (int i = 0; i < kZu12Dof; ++i)
    {
        j.jVal[i] = q[static_cast<std::size_t>(i)];
    }
    return j;
}

bool looksLikeMeters(const std::vector<std::array<double, kZu12Dof>>& poses)
{
    double max_xyz = 0.0;
    for (const auto& p : poses)
    {
        max_xyz = std::max(max_xyz, std::max({std::fabs(p[0]), std::fabs(p[1]), std::fabs(p[2])}));
    }
    return max_xyz <= 5.0;
}

CartesianPose toCartesianPose(const std::array<double, kZu12Dof>& p, double trans_scale)
{
    CartesianPose pose{};
    pose.tran.x = p[0] * trans_scale;
    pose.tran.y = p[1] * trans_scale;
    pose.tran.z = p[2] * trans_scale;
    pose.rpy.rx = p[3];
    pose.rpy.ry = p[4];
    pose.rpy.rz = p[5];
    return pose;
}

class ServoModeGuard
{
public:
    explicit ServoModeGuard(JAKAZuRobot& robot) : robot_(robot) {}
    ~ServoModeGuard()
    {
        if (enabled_)
        {
            robot_.servo_move_enable(FALSE);
        }
    }
    errno_t enable()
    {
        const errno_t ret = robot_.servo_move_enable(TRUE);
        enabled_ = (ret == ERR_SUCC);
        return ret;
    }
    void release()
    {
        if (enabled_)
        {
            robot_.servo_move_enable(FALSE);
            enabled_ = false;
        }
    }

private:
    JAKAZuRobot& robot_;
    bool enabled_ = false;
};

errno_t cartesianToJoints(
    JAKAZuRobot& robot,
    const std::vector<std::array<double, kZu12Dof>>& poses,
    const JakaZu12TrajectoryOptions& options,
    std::vector<std::array<double, kZu12Dof>>* joints)
{
    double trans_scale = 1.0;
    if (options.cartesian_unit == JakaCartesianLengthUnit::Meter)
    {
        trans_scale = 1000.0;
    }
    else if (options.cartesian_unit == JakaCartesianLengthUnit::Auto && looksLikeMeters(poses))
    {
        trans_scale = 1000.0;
    }

    JointValue seed{};
    errno_t ret = robot.get_joint_position(&seed);
    if (ret != ERR_SUCC)
    {
        return ret;
    }

    joints->clear();
    joints->reserve(poses.size());
    for (const auto& pose_vec : poses)
    {
        const CartesianPose pose = toCartesianPose(pose_vec, trans_scale);
        JointValue q{};
        ret = robot.kine_inverse(&seed, &pose, &q);
        if (ret != ERR_SUCC)
        {
            return ERR_KINE_INVERSE_ERR;
        }
        std::array<double, kZu12Dof> arr{};
        for (int i = 0; i < kZu12Dof; ++i)
        {
            arr[static_cast<std::size_t>(i)] = q.jVal[i];
        }
        joints->push_back(arr);
        seed = q;
    }
    return ERR_SUCC;
}

} // namespace

errno_t runJakaZu12Trajectory(
    JAKAZuRobot& robot,
    const std::vector<rl::math::Vector>& traj,
    const JakaZu12TrajectoryOptions& options)
{
    if (traj.empty())
    {
        return ERR_INVALID_PARAMETER;
    }

    std::vector<std::array<double, kZu12Dof>> raw;
    raw.reserve(traj.size());
    for (const auto& point : traj)
    {
        std::array<double, kZu12Dof> six{};
        if (!copySix(point, &six))
        {
            return ERR_INVALID_PARAMETER;
        }
        raw.push_back(six);
    }

    std::vector<std::array<double, kZu12Dof>> joint_waypoints = raw;
    if (options.space == JakaZu12Space::Cartesian)
    {
        const errno_t ik_ret = cartesianToJoints(robot, raw, options, &joint_waypoints);
        if (ik_ret != ERR_SUCC)
        {
            return ik_ret;
        }
    }

    const JakaZu12ServoPlan plan = planJakaZu12ServoTrajectory(joint_waypoints, options);
    if (plan.error != ERR_SUCC)
    {
        return plan.error;
    }
    if (plan.samples.empty())
    {
        return ERR_INVALID_PARAMETER;
    }

    RobotStatus_simple status{};
    errno_t ret = robot.get_robot_status_simple(&status);
    if (ret != ERR_SUCC)
    {
        return ret;
    }
    if (status.powered_on == 0)
    {
        return ERR_NOT_POWERED;
    }
    if (status.enabled == 0)
    {
        return ERR_NOT_ENABLED;
    }

    JointValue current{};
    ret = robot.get_joint_position(&current);
    if (ret != ERR_SUCC)
    {
        return ret;
    }

    const std::array<double, kZu12Dof>& first = plan.samples.front();
    if (options.approach_first_with_movej && maxAbsDiff(current, first) > options.start_joint_tol_rad)
    {
        BOOL in_servo = FALSE;
        robot.is_in_servomove(&in_servo);
        if (in_servo)
        {
            robot.servo_move_enable(FALSE);
        }
        const JointValue first_j = toJointValue(first);
        ret = robot.joint_move(
            &first_j,
            ABS,
            TRUE,
            std::max(options.approach_joint_speed_rad_s, 0.05),
            std::max(options.approach_joint_acc_rad_s2, 0.1),
            0.0,
            nullptr);
        if (ret != ERR_SUCC)
        {
            return ret;
        }
    }

    // 滤波必须在进入伺服模式前设置（与官方 SDK 说明一致）
    if (options.use_joint_lpf)
    {
        ret = robot.servo_move_use_joint_LPF(options.lpf_cutoff_hz);
        if (ret != ERR_SUCC)
        {
            return ret;
        }
    }
    if (options.use_speed_foresight)
    {
        // 部分 SDK 版本对前视参数更敏感；失败时仍可用 LPF 继续跟踪
        robot.servo_speed_foresight(
            std::max(options.foresight_buf, 3),
            std::max(options.foresight_kp, 0.1));
    }

    ServoModeGuard servo(robot);
    ret = servo.enable();
    if (ret != ERR_SUCC)
    {
        return ret;
    }

    const auto period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(plan.step_num * kJakaServoPeriodS));
    auto next_tick = std::chrono::steady_clock::now();

    for (const auto& sample : plan.samples)
    {
        const JointValue cmd = toJointValue(sample);
        ret = robot.servo_j(&cmd, ABS, plan.step_num);
        if (ret != ERR_SUCC)
        {
            robot.motion_abort();
            return ret;
        }
        next_tick += period;
        std::this_thread::sleep_until(next_tick);
    }

    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::milliseconds(options.reach_timeout_ms);
    const auto& last = plan.samples.back();
    while (std::chrono::steady_clock::now() < deadline)
    {
        ret = robot.get_joint_position(&current);
        if (ret != ERR_SUCC)
        {
            return ret;
        }
        if (jointsReached(current, last, options.reach_tol_rad))
        {
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(8));
    }

    servo.release();
    return ERR_SUCC;
}

} // namespace jaka
