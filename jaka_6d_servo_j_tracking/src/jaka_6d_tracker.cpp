#include "jaka_6d_tracker.hpp"

#include <chrono>
#include <cmath>
#include <iostream>

namespace {

double NowSeconds() {
    using clock = std::chrono::steady_clock;
    return std::chrono::duration<double>(clock::now().time_since_epoch()).count();
}

void PrintErr(const char* what, errno_t err) {
    std::cerr << "[jaka_6d] " << what << " failed, err=" << err << std::endl;
}

}  // namespace

Jaka6dServoJTracker::Jaka6dServoJTracker(JAKAZuRobot& robot, Config cfg)
    : robot_(robot), cfg_(std::move(cfg)) {}

errno_t Jaka6dServoJTracker::PoseFromJaka(const CartesianPose& in, jaka_se3::Pose* out) {
    RotMatrix rot{};
    const errno_t err = robot_.rpy_to_rot_matrix(&in.rpy, &rot);
    if (err != ERR_SUCC) {
        return err;
    }
    *out = jaka_se3::FromCartesian(in, rot);
    return ERR_SUCC;
}

errno_t Jaka6dServoJTracker::PoseToJaka(const jaka_se3::Pose& in, CartesianPose* out) {
    *out = jaka_se3::ToCartesianTran(in);
    const RotMatrix rot = jaka_se3::ToRotMatrix(in.q);
    return robot_.rot_matrix_to_rpy(&rot, &out->rpy);
}

void Jaka6dServoJTracker::OnVision(const VisionSample& sample) {
    if (!sample.valid || !cam_ready_.load()) {
        return;
    }

    CartesianPose tcp_jaka = sample.T_base_tcp_at_capture;
    if (!sample.has_tcp_stamp) {
        if (robot_.get_tcp_position(&tcp_jaka) != ERR_SUCC) {
            return;
        }
    }

    jaka_se3::Pose T_base_tcp{};
    jaka_se3::Pose T_cam_obj{};
    if (PoseFromJaka(tcp_jaka, &T_base_tcp) != ERR_SUCC) {
        return;
    }
    if (PoseFromJaka(sample.T_cam_obj, &T_cam_obj) != ERR_SUCC) {
        return;
    }

    // Eye-in-hand: object in base = TCP * cam_in_tcp * obj_in_cam.
    // Using the TCP at image time removes most camera-motion leakage.
    const jaka_se3::Pose T_base_obj = jaka_se3::Mul(jaka_se3::Mul(T_base_tcp, T_tcp_cam_), T_cam_obj);

    std::lock_guard<std::mutex> lock(state_mu_);
    const double stamp = sample.stamp_s > 0.0 ? sample.stamp_s : NowSeconds();

    if (!object_.valid) {
        object_.valid = true;
        object_.stamp_s = stamp;
        object_.T_base_obj = T_base_obj;
        object_.v_mm_s = {};
        object_.w_rad_s = {};
    } else {
        const double dt = std::max(1e-4, stamp - object_.stamp_s);
        const jaka_se3::Vec3 v_meas = jaka_se3::Scale(jaka_se3::Sub(T_base_obj.p, object_.T_base_obj.p), 1.0 / dt);
        const jaka_se3::Vec3 w_meas = jaka_se3::AngularVelocity(object_.T_base_obj.q, T_base_obj.q, dt);
        const double a = cfg_.pose_alpha;
        const double b = cfg_.twist_alpha;
        object_.T_base_obj.p = jaka_se3::Add(jaka_se3::Scale(object_.T_base_obj.p, 1.0 - a),
                                             jaka_se3::Scale(T_base_obj.p, a));
        object_.T_base_obj.q = jaka_se3::Slerp(object_.T_base_obj.q, T_base_obj.q, a);
        object_.v_mm_s = jaka_se3::Add(jaka_se3::Scale(object_.v_mm_s, 1.0 - b), jaka_se3::Scale(v_meas, b));
        object_.w_rad_s = jaka_se3::Add(jaka_se3::Scale(object_.w_rad_s, 1.0 - b), jaka_se3::Scale(w_meas, b));
        object_.stamp_s = stamp;
    }

    if (!offset_locked_) {
        if (cfg_.lock_relative_pose_on_first_see) {
            T_obj_tcp_ = jaka_se3::Mul(jaka_se3::Inverse(object_.T_base_obj), T_base_tcp);
        }
        offset_locked_ = true;
        locked_.store(true);
    }
}

jaka_se3::Pose Jaka6dServoJTracker::PredictObject(const ObjectState& state, double now_s, double delay_s) {
    const double dt = (now_s - state.stamp_s) + delay_s;
    return jaka_se3::Integrate(state.T_base_obj, state.v_mm_s, state.w_rad_s, dt);
}

bool Jaka6dServoJTracker::LimitJoints(const JointValue& seed, JointValue* q) const {
    for (int i = 0; i < 6; ++i) {
        const double dq = q->jVal[i] - seed.jVal[i];
        if (std::abs(dq) > cfg_.ik_jump_limit_rad) {
            return false;
        }
        const double limited = std::max(-cfg_.max_joint_step_rad, std::min(cfg_.max_joint_step_rad, dq));
        q->jVal[i] = seed.jVal[i] + limited;
    }
    return true;
}

errno_t Jaka6dServoJTracker::EnableServo() {
    errno_t err = PoseFromJaka(cfg_.T_tcp_cam, &T_tcp_cam_);
    if (err != ERR_SUCC) {
        PrintErr("parse T_tcp_cam", err);
        return err;
    }
    if (!cfg_.lock_relative_pose_on_first_see) {
        err = PoseFromJaka(cfg_.T_obj_tcp_desired, &T_obj_tcp_);
        if (err != ERR_SUCC) {
            PrintErr("parse T_obj_tcp_desired", err);
            return err;
        }
        offset_locked_ = true;
        locked_.store(true);
    }

    err = robot_.set_network_exception_handle(cfg_.network_lost_ms, MOT_ABORT);
    if (err != ERR_SUCC) {
        PrintErr("set_network_exception_handle", err);
        return err;
    }

    // Filters must be set before entering servo mode. Joint NLF applies to servo_j.
    err = robot_.servo_move_use_joint_NLF(cfg_.nlf_max_vr_deg, cfg_.nlf_max_ar_deg, cfg_.nlf_max_jr_deg);
    if (err != ERR_SUCC) {
        PrintErr("servo_move_use_joint_NLF", err);
        return err;
    }

    err = robot_.servo_move_enable(TRUE);
    if (err != ERR_SUCC) {
        PrintErr("servo_move_enable", err);
        return err;
    }
    cam_ready_.store(true);
    return ERR_SUCC;
}

void Jaka6dServoJTracker::DisableServo() {
    cam_ready_.store(false);
    robot_.servo_move_enable(FALSE);
}

errno_t Jaka6dServoJTracker::Start() {
    if (running_.exchange(true)) {
        return ERR_INVALID_PARAMETER;
    }
    stop_.store(false);
    {
        std::lock_guard<std::mutex> lock(state_mu_);
        if (cfg_.lock_relative_pose_on_first_see) {
            offset_locked_ = false;
            locked_.store(false);
            object_ = {};
        }
    }

    const errno_t err = EnableServo();
    if (err != ERR_SUCC) {
        running_.store(false);
        return err;
    }

    control_thread_ = std::thread([this] { ControlLoop(); });
    return ERR_SUCC;
}

void Jaka6dServoJTracker::Stop() {
    stop_.store(true);
    if (control_thread_.joinable()) {
        control_thread_.join();
    }
    if (running_.exchange(false)) {
        DisableServo();
    }
}

void Jaka6dServoJTracker::ControlLoop() {
    constexpr double kDt = 0.008;  // JAKA servo cycle
    JointValue q_cmd{};
    if (robot_.get_joint_position(&q_cmd) != ERR_SUCC) {
        PrintErr("get_joint_position", ERR_COMMUNICATION_ERR);
        return;
    }

    CartesianPose tcp_now{};
    jaka_se3::Pose T_cmd{};
    if (robot_.get_tcp_position(&tcp_now) != ERR_SUCC || PoseFromJaka(tcp_now, &T_cmd) != ERR_SUCC) {
        PrintErr("get_tcp_position", ERR_COMMUNICATION_ERR);
        return;
    }

    auto next = std::chrono::steady_clock::now();
    while (!stop_.load()) {
        next += std::chrono::microseconds(8000);
        const double now = NowSeconds();

        ObjectState snap;
        jaka_se3::Pose T_offset{};
        bool have_offset = false;
        {
            std::lock_guard<std::mutex> lock(state_mu_);
            snap = object_;
            have_offset = offset_locked_;
            T_offset = T_obj_tcp_;
        }

        const bool fresh = snap.valid && (now - snap.stamp_s) <= cfg_.vision_timeout_s;
        jaka_se3::Pose T_des = T_cmd;
        if (fresh && have_offset) {
            const jaka_se3::Pose T_obj = PredictObject(snap, now, cfg_.vision_delay_s);
            T_des = jaka_se3::Mul(T_obj, T_offset);
        } else if (snap.valid) {
            // Vision dropped: decay twist so the arm eases to a stop instead of coasting.
            std::lock_guard<std::mutex> lock(state_mu_);
            object_.v_mm_s = jaka_se3::Scale(object_.v_mm_s, 0.85);
            object_.w_rad_s = jaka_se3::Scale(object_.w_rad_s, 0.85);
        }

        T_cmd = jaka_se3::StepToward(T_cmd, T_des, cfg_.max_tcp_step_mm, cfg_.max_tcp_step_rad);

        CartesianPose cmd_pose{};
        JointValue q_ik = q_cmd;
        errno_t err = PoseToJaka(T_cmd, &cmd_pose);
        if (err == ERR_SUCC) {
            err = robot_.kine_inverse(&q_cmd, &cmd_pose, &q_ik);
        }

        if (err == ERR_SUCC && LimitJoints(q_cmd, &q_ik)) {
            q_cmd = q_ik;
        }
        // Always stream a command. A gap in servo_j makes JAKA stutter.

        err = robot_.servo_j(&q_cmd, ABS, 1);
        if (err != ERR_SUCC) {
            PrintErr("servo_j", err);
            if (err == ERR_DISABLE_SERVOMODE || err == ERR_PROTECTIVE_STOP ||
                err == ERR_EMERGENCY_STOP || err == ERR_EMERGENCY_PRESSED) {
                break;
            }
        }

        std::this_thread::sleep_until(next);
    }
}
