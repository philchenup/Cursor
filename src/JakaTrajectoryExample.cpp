/**
 * JAKA 按已插值轨迹运行示例（C++ SDK V2.1.14）
 *
 * 编译前请把 JAKASDK 的头文件/库加入工程，例如：
 *   头文件：E:/algorithm/JAKASDK 下 inc_of_c++ （JAKAZuRobot.h / jktypes.h / jkerr.h）
 *   库文件：jakaAPI.lib / jakaAPI.dll（Windows）或 libjakaAPI.so（Linux）
 *
 * 用法：把已经插值加密的轨迹赋给 std::vector<std::vector<float>>，
 * 本程序每 80ms（10*8ms）发送一个点。
 */
#include "JakaTrajectoryRunner.h"

#include <iostream>
#include <vector>

int main()
{
    // ---------- 已插值加密的轨迹（示例数据，请替换为实际轨迹）----------
    // 笛卡尔空间：每点 [x, y, z, rx, ry, rz]
    // xyz: mm；rx/ry/rz: rad（若是度，把 angle_unit 设为 Deg）
    std::vector<std::vector<float>> traj = {
        // 仅作格式示意；真实轨迹应由规划/插值模块生成
        {-251.054f, -48.360f, 374.000f, 3.14f, 0.0f, 1.57f},
        {-252.054f, -50.360f, 374.000f, 3.14f, 0.0f, 1.57f},
        {-253.054f, -52.360f, 374.000f, 3.14f, 0.0f, 1.57f},
    };

    JAKAZuRobot robot;

    // 将 IP 换成控制器地址；若 App 设置了 SDK 密码，使用带用户名密码的重载：
    // robot.login_in("192.168.2.194", true, "jaka_sdk", "password");
    errno_t ret = robot.login_in("192.168.2.194");
    if (ret != ERR_SUCC) {
        std::cerr << "login_in 失败, ret=" << ret << std::endl;
        return ret;
    }

    ret = robot.power_on();
    if (ret != ERR_SUCC) {
        std::cerr << "power_on 失败, ret=" << ret << std::endl;
        return ret;
    }

    ret = robot.enable_robot();
    if (ret != ERR_SUCC) {
        std::cerr << "enable_robot 失败, ret=" << ret << std::endl;
        return ret;
    }

    jaka_traj::RunOptions opt;
    opt.space = jaka_traj::Space::Cartesian;   // 关节轨迹改为 Space::Joint
    opt.angle_unit = jaka_traj::AngleUnit::Rad;
    opt.move_to_start = true;

    std::cout << "开始下发轨迹, 点数=" << traj.size()
              << ", 周期=" << jaka_traj::kSendPeriodMs << "ms"
              << ", step_num=" << jaka_traj::kStepNum << std::endl;

    ret = jaka_traj::RunTrajectory(robot, traj, opt);
    if (ret != ERR_SUCC) {
        std::cerr << "轨迹执行失败, ret=" << ret << std::endl;
        robot.login_out();
        return ret;
    }

    std::cout << "轨迹执行完成" << std::endl;
    robot.login_out();
    return 0;
}
