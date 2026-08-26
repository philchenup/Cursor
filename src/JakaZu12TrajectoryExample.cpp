#include "JakaZu12Trajectory.h"

#include <iostream>
#include <vector>

/**
 * 用法示例：将 Robotics Library 给出的关节轨迹交给 JAKA Zu12 流畅执行。
 *
 * 对照 JAKA 官方仓库：
 *   https://github.com/JAKARobotics/jakasdk-cpp
 *   https://github.com/JAKARobotics/jaka_ros2  (moveit_server.cpp / servoj_demo.cpp)
 *
 * 编译并链接 libjakaAPI.so 后，替换控制器 IP 即可在实机上运行。
 */
int main()
{
    JAKAZuRobot robot;
    const char* ip = "192.168.137.100";

    if (robot.login_in(ip) != ERR_SUCC)
    {
        std::cerr << "login_in failed" << std::endl;
        return 1;
    }
    robot.power_on();
    robot.enable_robot();

    std::vector<rl::math::Vector> traj;
    for (int i = 0; i <= 20; ++i)
    {
        rl::math::Vector q(6);
        const double s = static_cast<double>(i) / 20.0;
        q[0] = 0.2 * s;
        q[1] = 0.5 + 0.1 * s;
        q[2] = -0.4;
        q[3] = 1.57;
        q[4] = 0.3;
        q[5] = 0.0;
        traj.push_back(q);
    }

    jaka::JakaZu12TrajectoryOptions opt;
    opt.space = jaka::JakaZu12Space::Joint;
    opt.velocity_scale = 0.4;

    const errno_t ret = jaka::runJakaZu12Trajectory(robot, traj, opt);
    std::cout << "runJakaZu12Trajectory returned " << ret << std::endl;

    robot.disable_robot();
    robot.power_off();
    robot.login_out();
    return ret == ERR_SUCC ? 0 : 1;
}
