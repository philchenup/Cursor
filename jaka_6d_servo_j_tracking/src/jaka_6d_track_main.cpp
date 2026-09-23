#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <iostream>
#include <thread>

#include "jaka_6d_tracker.hpp"

namespace {

std::atomic<bool> g_quit{false};

void OnSignal(int) {
    g_quit.store(true);
}

double NowSeconds() {
    using clock = std::chrono::steady_clock;
    return std::chrono::duration<double>(clock::now().time_since_epoch()).count();
}

// Simulated moving workpiece in the robot base frame.
// Replace BuildCameraMeasurement() with your 30 Hz 6D tracker output.
jaka_se3::Pose SimulatedObjectBase(double t_s) {
    jaka_se3::Pose pose{};
    pose.p = {400.0 + 40.0 * std::cos(0.4 * t_s),
              0.0 + 40.0 * std::sin(0.4 * t_s),
              150.0};
    const double yaw = 0.15 * std::sin(0.3 * t_s);
    pose.q = {std::cos(0.5 * yaw), 0.0, 0.0, std::sin(0.5 * yaw)};
    return pose;
}

errno_t BuildCameraMeasurement(JAKAZuRobot& robot,
                               const jaka_se3::Pose& T_tcp_cam,
                               const jaka_se3::Pose& T_base_obj,
                               CartesianPose* T_cam_obj_jaka,
                               CartesianPose* T_base_tcp_jaka) {
    if (robot.get_tcp_position(T_base_tcp_jaka) != ERR_SUCC) {
        return ERR_COMMUNICATION_ERR;
    }
    RotMatrix rot{};
    if (robot.rpy_to_rot_matrix(&T_base_tcp_jaka->rpy, &rot) != ERR_SUCC) {
        return ERR_FUCTION_CALL_ERROR;
    }
    const jaka_se3::Pose T_base_tcp = jaka_se3::FromCartesian(*T_base_tcp_jaka, rot);
    const jaka_se3::Pose T_cam_obj =
        jaka_se3::Mul(jaka_se3::Inverse(jaka_se3::Mul(T_base_tcp, T_tcp_cam)), T_base_obj);

    *T_cam_obj_jaka = jaka_se3::ToCartesianTran(T_cam_obj);
    const RotMatrix cam_rot = jaka_se3::ToRotMatrix(T_cam_obj.q);
    return robot.rot_matrix_to_rpy(&cam_rot, &T_cam_obj_jaka->rpy);
}

}  // namespace

int main(int argc, char** argv) {
    const char* ip = argc > 1 ? argv[1] : "192.168.137.101";
    std::signal(SIGINT, OnSignal);
    std::signal(SIGTERM, OnSignal);

    JAKAZuRobot robot;
    errno_t err = robot.login_in(ip);
    if (err != ERR_SUCC) {
        std::cerr << "login_in failed: " << err << std::endl;
        return 1;
    }

    err = robot.power_on();
    if (err != ERR_SUCC) {
        std::cerr << "power_on failed: " << err << std::endl;
        robot.login_out();
        return 1;
    }
    err = robot.enable_robot();
    if (err != ERR_SUCC) {
        std::cerr << "enable_robot failed: " << err << std::endl;
        robot.login_out();
        return 1;
    }
    robot.set_rapidrate(1.0);

    // Move to a reachable watch pose before entering servo_j. Adjust for your cell.
    JointValue home{};
    home.jVal[0] = 0.0;
    home.jVal[1] = -0.6;
    home.jVal[2] = 1.4;
    home.jVal[3] = -0.8;
    home.jVal[4] = 1.57;
    home.jVal[5] = 0.0;
    err = robot.joint_move(&home, ABS, TRUE, 0.4);
    if (err != ERR_SUCC) {
        std::cerr << "joint_move home failed: " << err << std::endl;
        robot.login_out();
        return 1;
    }

    Jaka6dServoJTracker::Config cfg;
    // Example eye-in-hand: camera 80 mm along tool +Z, no extra rotation.
    // Replace with your calibrated T_tcp_cam.
    cfg.T_tcp_cam.tran = {0.0, 0.0, 80.0};
    cfg.T_tcp_cam.rpy = {0.0, 0.0, 0.0};
    cfg.lock_relative_pose_on_first_see = true;
    cfg.vision_delay_s = 0.040;

    Jaka6dServoJTracker tracker(robot, cfg);
    err = tracker.Start();
    if (err != ERR_SUCC) {
        std::cerr << "tracker.Start failed: " << err << std::endl;
        robot.login_out();
        return 1;
    }

    jaka_se3::Pose T_tcp_cam{};
    RotMatrix cam_rot{};
    robot.rpy_to_rot_matrix(&cfg.T_tcp_cam.rpy, &cam_rot);
    T_tcp_cam = jaka_se3::FromCartesian(cfg.T_tcp_cam, cam_rot);

    std::cout << "servo_j 6D tracking started. Ctrl+C to stop.\n"
              << "Vision thread: 30 Hz. Control thread: 125 Hz (8 ms servo_j).\n";

    const auto t0 = std::chrono::steady_clock::now();
    auto next_vis = t0;
    while (!g_quit.load()) {
        next_vis += std::chrono::milliseconds(33);  // ~30 Hz

        const double t = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        const jaka_se3::Pose T_base_obj = SimulatedObjectBase(t);

        Jaka6dServoJTracker::VisionSample sample;
        sample.stamp_s = NowSeconds();
        sample.valid = true;
        sample.has_tcp_stamp = true;
        if (BuildCameraMeasurement(robot, T_tcp_cam, T_base_obj, &sample.T_cam_obj,
                                   &sample.T_base_tcp_at_capture) == ERR_SUCC) {
            tracker.OnVision(sample);
        }

        std::this_thread::sleep_until(next_vis);
    }

    tracker.Stop();
    robot.disable_robot();
    robot.power_off();
    robot.login_out();
    std::cout << "stopped." << std::endl;
    return 0;
}
