#ifndef JAKA_ZU12_TRAJECTORY_H
#define JAKA_ZU12_TRAJECTORY_H

#include "JAKAZuRobot.h"

#include <rl/math/Vector.h>
#include <vector>

/**
 * 让已使能的 JAKA Zu12 流畅跟踪一段关节轨迹。
 *
 * traj 每个点为 6 维关节角，单位 rad（rl::math::Vector）。
 * 实现对照 JAKA 官方 servo_j：8ms 周期、先 LPF 再连续下发绝对关节角。
 * robot 需已 login_in / power_on / enable_robot。
 */
errno_t runJakaZu12Trajectory(JAKAZuRobot& robot, const std::vector<rl::math::Vector>& traj);

#endif
