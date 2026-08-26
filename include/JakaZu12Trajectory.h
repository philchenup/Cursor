#ifndef JAKA_ZU12_TRAJECTORY_H
#define JAKA_ZU12_TRAJECTORY_H

#include "JAKAZuRobot.h"
#include "jaka_rl_compat.h"

#include <array>
#include <vector>

/**
 * 让 JAKA Zu12（6 轴）沿一段轨迹点流畅运行。
 *
 * 实现对照 JAKA 官方 GitHub：
 * - jakasdk-cpp：servo_move_enable + servo_j / servo_p
 * - jaka_ros2/src/jaka_planner/src/moveit_server.cpp：
 *     servo_move_use_joint_LPF(0.5) + servo_j(ABS, step_num=dt/0.008)
 * - jaka_ros2/src/jaka_driver/src/servoj_demo.cpp：先使能伺服再连续下发
 *
 * 控制器伺服周期为 8ms，且不对 servo_j 做规划；本函数在关节空间做
 * 钳制三次样条插值，再按 8ms 重采样后流式下发，以保证连续平滑。
 */
namespace jaka {

constexpr int kZu12Dof = 6;
constexpr double kJakaServoPeriodS = 0.008;

enum class JakaZu12Space
{
    Joint,     ///< 每个点为 6 维关节角，单位 rad（RL 惯例）
    Cartesian  ///< 每个点为 [x,y,z,rx,ry,rz]；平移默认米，姿态 rad
};

enum class JakaCartesianLengthUnit
{
    Auto,        ///< |x,y,z| 最大值 <= 5 视为米，否则视为毫米
    Meter,       ///< RL 常用 SI 单位，发送前乘 1000 转为 JAKA 的 mm
    Millimeter   ///< 与 JAKA SDK CartesianPose.tran 一致
};

struct JakaZu12TrajectoryOptions
{
    JakaZu12Space space = JakaZu12Space::Joint;
    JakaCartesianLengthUnit cartesian_unit = JakaCartesianLengthUnit::Auto;

    /// Zu12 关节最大角速度（rad/s）。伺服层硬限约 180 deg/s，此处取产品常用值并可用 scale 再降速。
    std::array<double, kZu12Dof> max_joint_vel_rad_s = {
        120.0 * 3.14159265358979323846 / 180.0,
        120.0 * 3.14159265358979323846 / 180.0,
        120.0 * 3.14159265358979323846 / 180.0,
        180.0 * 3.14159265358979323846 / 180.0,
        180.0 * 3.14159265358979323846 / 180.0,
        180.0 * 3.14159265358979323846 / 180.0};

    /// (0, 1]，对 max_joint_vel_rad_s 再缩放。默认 0.5，更利于流畅、少抖动。
    double velocity_scale = 0.5;

    /// 重采样周期，必须是 8ms 的整数倍。
    double sample_period_s = kJakaServoPeriodS;

    /// 与官方 moveit_server 一致：进入伺服前设置一阶关节低通。
    bool use_joint_lpf = true;
    double lpf_cutoff_hz = 0.5;

    /// 速度前视均值滤波（SDK servo_speed_foresight），进一步抑制噪声。
    bool use_speed_foresight = true;
    int foresight_buf = 20;
    double foresight_kp = 1.0;

    /// 当前关节与首点偏差过大时，先用阻塞 joint_move 到位，再切伺服跟踪。
    bool approach_first_with_movej = true;
    double approach_joint_speed_rad_s = 0.6;
    double approach_joint_acc_rad_s2 = 1.2;
    double start_joint_tol_rad = 0.02; ///< ~1.15 deg

    /// 等待末端点到位的容差（官方 moveit_server 使用 ±0.2 deg）。
    double reach_tol_rad = 0.2 * 3.14159265358979323846 / 180.0;
    int reach_timeout_ms = 5000;
};

struct JakaZu12ServoPlan
{
    int error = ERR_SUCC;
    std::vector<std::array<double, kZu12Dof>> samples; ///< 关节 rad，按 sample_period_s 排列
    unsigned int step_num = 1;                         ///< servo_j 的 step_num，= sample_period / 8ms
};

/**
 * 纯规划：把稀疏轨迹点变成 Zu12 可流畅跟踪的 8ms 关节序列。
 * 不访问机械臂，便于单测。
 */
JakaZu12ServoPlan planJakaZu12ServoTrajectory(
    const std::vector<std::array<double, kZu12Dof>>& joint_waypoints_rad,
    const JakaZu12TrajectoryOptions& options = {});

/**
 * 在已登录、已上电、已使能的 JAKAZuRobot 上，流畅执行一段 RL 轨迹。
 *
 * @param robot     已 login_in / power_on / enable_robot 的 SDK 句柄
 * @param traj      轨迹点。Joint 空间为 6 维 rad；Cartesian 为 6 维位姿
 * @param options   速度、滤波、坐标系等
 * @return ERR_SUCC 成功，其余为 jkerr.h 中的错误码
 */
errno_t runJakaZu12Trajectory(
    JAKAZuRobot& robot,
    const std::vector<rl::math::Vector>& traj,
    const JakaZu12TrajectoryOptions& options = {});

} // namespace jaka

#endif // JAKA_ZU12_TRAJECTORY_H
