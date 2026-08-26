#ifndef JAKA_TRAJECTORY_RUNNER_H
#define JAKA_TRAJECTORY_RUNNER_H

#include "JAKAZuRobot.h"
#include "jkerr.h"

#include <vector>

/**
 * JAKA 伺服轨迹下发（对应 C++ SDK 用户手册 V2.1.14）
 *
 * 控制器伺服插补周期为 8ms。step_num 为倍分周期，实际执行周期 = step_num * 8ms。
 * 本模块按 10 倍周期发送，即 step_num = 10，每 80ms 下发一个已经插值加密后的轨迹点。
 *
 * 轨迹格式：std::vector<std::vector<float>>
 *   - 笛卡尔：每点 6 个数 [x, y, z, rx, ry, rz]，xyz 单位 mm，姿态默认弧度
 *   - 关节：  每点 6 个数 [j1, j2, j3, j4, j5, j6]，默认弧度
 *
 * 注意：servo_p / servo_j 不走控制器运动规划器，必须使用预先插值好的轨迹。
 * 每个轨迹点应对应 80ms 的时间间隔；若按 8ms 加密，请先抽稀到 80ms 再调用。
 */
namespace jaka_traj {

/// 控制器伺服基周期（手册：8ms）
constexpr unsigned int kServoBasePeriodMs = 8;
/// 10 倍周期发送
constexpr unsigned int kStepNum = 10;
/// 实际发送/执行周期：10 * 8ms = 80ms
constexpr unsigned int kSendPeriodMs = kStepNum * kServoBasePeriodMs;

enum class Space {
    Cartesian,  ///< servo_p，笛卡尔空间
    Joint       ///< servo_j，关节空间
};

enum class AngleUnit {
    Rad,  ///< SDK 原生单位（关节、RPY 均为 rad）
    Deg   ///< 轨迹中的角度为度，内部会转换成 rad
};

struct RunOptions {
    Space space = Space::Cartesian;
    AngleUnit angle_unit = AngleUnit::Rad;
    /// 下发前是否用阻塞运动先走到轨迹首点（强烈建议开启，避免伺服跳变超速）
    bool move_to_start = true;
    /// 笛卡尔先到首点时的直线速度 mm/s
    double linear_speed_mm_s = 80.0;
    /// 关节先到首点时的速度 rad/s
    double joint_speed_rad_s = 0.4;
};

/**
 * 按 80ms 周期把已插值轨迹发送给 JAKA。
 * 调用前机器人需已 login_in / power_on / enable_robot。
 *
 * @return ERR_SUCC(0) 成功，否则为 SDK 错误码
 */
errno_t RunTrajectory(JAKAZuRobot& robot,
                      const std::vector<std::vector<float>>& traj,
                      const RunOptions& opt = RunOptions());

}  // namespace jaka_traj

#endif  // JAKA_TRAJECTORY_RUNNER_H
