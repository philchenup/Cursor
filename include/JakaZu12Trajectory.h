#ifndef JAKA_ZU12_TRAJECTORY_H
#define JAKA_ZU12_TRAJECTORY_H

#include "JAKAZuRobot.h"

#include <rl/math/Vector.h>
#include <vector>

/**
 * 让已使能的 JAKA Zu12 跟踪一段已经加密（8ms 密采样）的关节轨迹。
 *
 * traj 每个点为 6 维关节角，单位 rad，点间距应对应控制器 8ms 周期。
 * 不再二次插值，按点直接 servo_j 下发。
 * robot 需已 login_in / power_on / enable_robot。
 */
errno_t runJakaZu12Trajectory(JAKAZuRobot& robot, const std::vector<rl::math::Vector>& traj);

#endif
