#include "JakaTrajectoryRunner.h"

#include <chrono>
#include <iostream>
#include <thread>

namespace jaka_traj {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr int kErrInvalidTraj = -2;  // 与 SDK ERR_INVALID_PARAMETER 一致

bool Ok(errno_t ret, const char* what)
{
    if (ret != ERR_SUCC) {
        std::cerr << "[JAKA] " << what << " 失败, ret=" << ret << std::endl;
        return false;
    }
    return true;
}

double ToRad(double value, AngleUnit unit)
{
    return (unit == AngleUnit::Deg) ? (value * kPi / 180.0) : value;
}

bool PointValid(const std::vector<float>& pt)
{
    return pt.size() >= 6;
}

CartesianPose ToCartesian(const std::vector<float>& pt, AngleUnit unit)
{
    CartesianPose pose{};
    pose.tran.x = pt[0];
    pose.tran.y = pt[1];
    pose.tran.z = pt[2];
    pose.rpy.rx = ToRad(pt[3], unit);
    pose.rpy.ry = ToRad(pt[4], unit);
    pose.rpy.rz = ToRad(pt[5], unit);
    return pose;
}

JointValue ToJoint(const std::vector<float>& pt, AngleUnit unit)
{
    JointValue j{};
    for (int i = 0; i < 6; ++i) {
        j.jVal[i] = ToRad(pt[i], unit);
    }
    return j;
}

errno_t MoveToStart(JAKAZuRobot& robot,
                    const std::vector<float>& first,
                    const RunOptions& opt)
{
    if (opt.space == Space::Cartesian) {
        CartesianPose start = ToCartesian(first, opt.angle_unit);
        // 阻塞直线运动到首点，规划由控制器完成
        return robot.linear_move(&start, ABS, TRUE, opt.linear_speed_mm_s);
    }

    JointValue start = ToJoint(first, opt.angle_unit);
    return robot.joint_move(&start, ABS, TRUE, opt.joint_speed_rad_s);
}

void WaitSendPeriod(std::chrono::steady_clock::time_point& next_tick)
{
    next_tick += std::chrono::milliseconds(kSendPeriodMs);
    std::this_thread::sleep_until(next_tick);
}

errno_t SendCartesian(JAKAZuRobot& robot,
                      const std::vector<std::vector<float>>& traj,
                      const RunOptions& opt)
{
    auto next_tick = std::chrono::steady_clock::now();

    for (size_t i = 0; i < traj.size(); ++i) {
        if (!PointValid(traj[i])) {
            std::cerr << "[JAKA] 第 " << i << " 个笛卡尔点长度不足 6" << std::endl;
            return kErrInvalidTraj;
        }

        CartesianPose pose = ToCartesian(traj[i], opt.angle_unit);
        // ABS：绝对位姿；step_num=10 → 该点在 80ms 内由控制器按 8ms 插补执行
        // V2.1.14：servo_p(pose, move_mode, step_num)
        errno_t ret = robot.servo_p(&pose, ABS, kStepNum);
        if (!Ok(ret, "servo_p")) {
            return ret;
        }

        WaitSendPeriod(next_tick);
    }
    return ERR_SUCC;
}

errno_t SendJoint(JAKAZuRobot& robot,
                  const std::vector<std::vector<float>>& traj,
                  const RunOptions& opt)
{
    auto next_tick = std::chrono::steady_clock::now();

    for (size_t i = 0; i < traj.size(); ++i) {
        if (!PointValid(traj[i])) {
            std::cerr << "[JAKA] 第 " << i << " 个关节点长度不足 6" << std::endl;
            return kErrInvalidTraj;
        }

        JointValue jpos = ToJoint(traj[i], opt.angle_unit);
        errno_t ret = robot.servo_j(&jpos, ABS, kStepNum);
        if (!Ok(ret, "servo_j")) {
            return ret;
        }

        WaitSendPeriod(next_tick);
    }
    return ERR_SUCC;
}

}  // namespace

errno_t RunTrajectory(JAKAZuRobot& robot,
                      const std::vector<std::vector<float>>& traj,
                      const RunOptions& opt)
{
    if (traj.empty()) {
        std::cerr << "[JAKA] 轨迹为空" << std::endl;
        return kErrInvalidTraj;
    }
    if (!PointValid(traj.front())) {
        std::cerr << "[JAKA] 轨迹点必须至少包含 6 个数值" << std::endl;
        return kErrInvalidTraj;
    }

    // 先退出伺服，才能设置滤波器（手册：伺服模式下不可改滤波器）
    errno_t ret = robot.servo_move_enable(FALSE);
    if (!Ok(ret, "servo_move_enable(FALSE)")) {
        return ret;
    }

    // 轨迹已插值加密，关闭额外滤波，避免滞后（部分固件若无此接口可忽略失败）
    ret = robot.servo_move_use_none_filter();
    if (ret != ERR_SUCC) {
        std::cerr << "[JAKA] servo_move_use_none_filter 未生效, ret=" << ret
                  << "，继续按下发" << std::endl;
    }

    if (opt.move_to_start) {
        ret = MoveToStart(robot, traj.front(), opt);
        if (!Ok(ret, "move_to_start")) {
            return ret;
        }
    }

    // 进入伺服位置控制模式（v20 起为阻塞接口）
    ret = robot.servo_move_enable(TRUE);
    if (!Ok(ret, "servo_move_enable(TRUE)")) {
        return ret;
    }

    if (opt.space == Space::Cartesian) {
        ret = SendCartesian(robot, traj, opt);
    } else {
        ret = SendJoint(robot, traj, opt);
    }

    // 等待最后一个点执行完再退出伺服
    std::this_thread::sleep_for(std::chrono::milliseconds(kSendPeriodMs));

    errno_t disable_ret = robot.servo_move_enable(FALSE);
    Ok(disable_ret, "servo_move_enable(FALSE) 结束");
    return (ret != ERR_SUCC) ? ret : disable_ret;
}

}  // namespace jaka_traj
