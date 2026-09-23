#pragma once

#include <atomic>
#include <mutex>
#include <thread>

#include "jaka_se3.hpp"
#include "JAKAZuRobot.h"

// 6D object tracking on JAKA using servo_j.
//
// Vision runs at ~30 Hz and only updates the object pose in the base frame.
// A 125 Hz (8 ms) control thread then:
//   1) predicts the moving object with velocity feedforward
//   2) builds the desired TCP in SE(3)
//   3) interpolates / rate-limits toward that TCP
//   4) solves IK with the previous joints as seed
//   5) streams servo_j(ABS) every controller cycle
//
// Do not send the raw 30 Hz IK solution to servo_j. JAKA does not interpolate
// servo commands; sparse joint targets produce steps or vibration.

class Jaka6dServoJTracker {
public:
    struct Config {
        // Eye-in-hand: camera pose expressed in the current TCP / tool frame.
        // Fill this from hand-eye calibration (AX=XB). Units: mm, rad.
        CartesianPose T_tcp_cam{};

        // If true, the first valid vision sample locks the current TCP-to-object
        // relative pose and the robot keeps that viewpoint while the object moves.
        // If false, T_obj_tcp_desired is used as the tracking offset.
        bool lock_relative_pose_on_first_see = true;
        CartesianPose T_obj_tcp_desired{};

        // Extra prediction to cover camera exposure + inference + network, seconds.
        double vision_delay_s = 0.040;

        // Drop the track if no valid vision arrives within this window.
        double vision_timeout_s = 0.150;

        // Complementary-filter blends for object pose / twist in the base frame.
        double pose_alpha = 0.35;
        double twist_alpha = 0.25;

        // Per-cycle Cartesian limits at 125 Hz. 150 mm/s and 40 deg/s by default.
        double max_tcp_step_mm = 1.2;
        double max_tcp_step_rad = 40.0 * jaka_se3::kDeg2Rad / 125.0;

        // Per-cycle joint limit. JAKA rejects commands above 180 deg/s.
        double max_joint_step_rad = 80.0 * jaka_se3::kDeg2Rad / 125.0;

        // Reject an IK solution if any joint jumps more than this from the seed.
        double ik_jump_limit_rad = 15.0 * jaka_se3::kDeg2Rad;

        // Joint-space NLF, applied before servo_move_enable. Units: deg*.
        double nlf_max_vr_deg = 80.0;
        double nlf_max_ar_deg = 400.0;
        double nlf_max_jr_deg = 2000.0;

        // Abort motion if SDK comms drop.
        float network_lost_ms = 100.0f;
    };

    // One 6D measurement from the eye-in-hand tracker.
    // T_cam_obj: object pose in the camera frame (mm, JAKA RPY rad).
    // T_base_tcp_at_capture: TCP at image time. If has_tcp_stamp is false the
    // control thread reads the live TCP, which is less accurate while moving.
    struct VisionSample {
        double stamp_s = 0.0;
        CartesianPose T_cam_obj{};
        CartesianPose T_base_tcp_at_capture{};
        bool has_tcp_stamp = false;
        bool valid = false;
    };

    explicit Jaka6dServoJTracker(JAKAZuRobot& robot, Config cfg);

    Jaka6dServoJTracker(const Jaka6dServoJTracker&) = delete;
    Jaka6dServoJTracker& operator=(const Jaka6dServoJTracker&) = delete;

    // Call from the vision thread at ~30 Hz.
    void OnVision(const VisionSample& sample);

    // Enable servo mode and start the 8 ms joint stream. Blocking until Stop().
    errno_t Start();
    void Stop();

    bool running() const { return running_.load(); }
    bool locked() const { return locked_.load(); }

private:
    struct ObjectState {
        bool valid = false;
        double stamp_s = 0.0;
        jaka_se3::Pose T_base_obj{};
        jaka_se3::Vec3 v_mm_s{};
        jaka_se3::Vec3 w_rad_s{};
    };

    errno_t PoseFromJaka(const CartesianPose& in, jaka_se3::Pose* out);
    errno_t PoseToJaka(const jaka_se3::Pose& in, CartesianPose* out);
    errno_t EnableServo();
    void DisableServo();
    void ControlLoop();
    static jaka_se3::Pose PredictObject(const ObjectState& state, double now_s, double delay_s);
    bool LimitJoints(const JointValue& seed, JointValue* q) const;

    JAKAZuRobot& robot_;
    Config cfg_;
    jaka_se3::Pose T_tcp_cam_{};
    jaka_se3::Pose T_obj_tcp_{};

    std::mutex state_mu_;
    ObjectState object_{};
    bool offset_locked_ = false;

    std::atomic<bool> running_{false};
    std::atomic<bool> stop_{false};
    std::atomic<bool> locked_{false};
    std::atomic<bool> cam_ready_{false};
    std::thread control_thread_;
};
