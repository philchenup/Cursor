#pragma once

#include <cmath>
#include <algorithm>

#include "jktypes.h"

// Lightweight SE(3) helpers. Translation is in millimetres to match JAKA.
// Orientation is stored as a unit quaternion; convert to/from JAKA RPY only
// through JAKAZuRobot::rpy_to_rot_matrix / rot_matrix_to_rpy so the Euler
// convention stays identical to kine_inverse.

namespace jaka_se3 {

constexpr double kPi = 3.14159265358979323846;
constexpr double kRad2Deg = 180.0 / kPi;
constexpr double kDeg2Rad = kPi / 180.0;

struct Vec3 {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

struct Quat {
    double w = 1.0;
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

struct Pose {
    Vec3 p;
    Quat q;
};

inline double Dot(const Vec3& a, const Vec3& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

inline double Norm(const Vec3& v) {
    return std::sqrt(Dot(v, v));
}

inline Vec3 Add(const Vec3& a, const Vec3& b) {
    return {a.x + b.x, a.y + b.y, a.z + b.z};
}

inline Vec3 Sub(const Vec3& a, const Vec3& b) {
    return {a.x - b.x, a.y - b.y, a.z - b.z};
}

inline Vec3 Scale(const Vec3& a, double s) {
    return {a.x * s, a.y * s, a.z * s};
}

inline Vec3 Cross(const Vec3& a, const Vec3& b) {
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

inline Quat Normalize(Quat q) {
    const double n = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    if (n < 1e-12) {
        return {1.0, 0.0, 0.0, 0.0};
    }
    return {q.w / n, q.x / n, q.y / n, q.z / n};
}

inline Quat Conjugate(const Quat& q) {
    return {q.w, -q.x, -q.y, -q.z};
}

inline Quat Mul(const Quat& a, const Quat& b) {
    return Normalize({
        a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
        a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
        a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
        a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
    });
}

inline Vec3 Rotate(const Quat& q, const Vec3& v) {
    const Quat qv{0.0, v.x, v.y, v.z};
    const Quat r = Mul(Mul(q, qv), Conjugate(q));
    return {r.x, r.y, r.z};
}

inline Pose Inverse(const Pose& t) {
    const Quat qi = Conjugate(t.q);
    return {Scale(Rotate(qi, t.p), -1.0), qi};
}

inline Pose Mul(const Pose& a, const Pose& b) {
    return {Add(a.p, Rotate(a.q, b.p)), Mul(a.q, b.q)};
}

inline double QuatDot(const Quat& a, const Quat& b) {
    return a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z;
}

inline Quat AlignHemisphere(const Quat& ref, Quat q) {
    if (QuatDot(ref, q) < 0.0) {
        return {-q.w, -q.x, -q.y, -q.z};
    }
    return q;
}

inline Quat Slerp(Quat a, Quat b, double t) {
    a = Normalize(a);
    b = AlignHemisphere(a, Normalize(b));
    double c = QuatDot(a, b);
    c = std::max(-1.0, std::min(1.0, c));
    if (c > 0.9995) {
        return Normalize({
            a.w + t * (b.w - a.w),
            a.x + t * (b.x - a.x),
            a.y + t * (b.y - a.y),
            a.z + t * (b.z - a.z),
        });
    }
    const double theta = std::acos(c);
    const double s = std::sin(theta);
    const double w1 = std::sin((1.0 - t) * theta) / s;
    const double w2 = std::sin(t * theta) / s;
    return Normalize({
        w1 * a.w + w2 * b.w,
        w1 * a.x + w2 * b.x,
        w1 * a.y + w2 * b.y,
        w1 * a.z + w2 * b.z,
    });
}

inline Pose Interp(const Pose& a, const Pose& b, double t) {
    t = std::max(0.0, std::min(1.0, t));
    return {Add(a.p, Scale(Sub(b.p, a.p), t)), Slerp(a.q, b.q, t)};
}

// Geodesic angle between two orientations, radians.
inline double Angle(const Quat& a, const Quat& b) {
    double c = std::abs(QuatDot(Normalize(a), Normalize(b)));
    c = std::max(-1.0, std::min(1.0, c));
    return 2.0 * std::acos(c);
}

// Advance current toward target without exceeding linear/angular step limits.
inline Pose StepToward(const Pose& current, const Pose& target, double max_dp_mm, double max_dtheta_rad) {
    const Vec3 dp = Sub(target.p, current.p);
    const double dist = Norm(dp);
    const double ang = Angle(current.q, target.q);
    double t_p = 1.0;
    double t_r = 1.0;
    if (dist > max_dp_mm && dist > 1e-9) {
        t_p = max_dp_mm / dist;
    }
    if (ang > max_dtheta_rad && ang > 1e-9) {
        t_r = max_dtheta_rad / ang;
    }
    return Interp(current, target, std::min(t_p, t_r));
}

inline Pose Integrate(const Pose& pose, const Vec3& v_mm_s, const Vec3& w_rad_s, double dt) {
    Pose next = pose;
    next.p = Add(pose.p, Scale(v_mm_s, dt));
    const double wn = Norm(w_rad_s);
    if (wn > 1e-9 && std::abs(dt) > 1e-9) {
        const double half = 0.5 * wn * dt;
        const double s = std::sin(half) / wn;
        const Quat dq = Normalize({std::cos(half), w_rad_s.x * s, w_rad_s.y * s, w_rad_s.z * s});
        next.q = Mul(dq, pose.q);
    }
    return next;
}

inline Quat FromRotMatrix(const RotMatrix& m) {
    const double m00 = m.x.x, m01 = m.y.x, m02 = m.z.x;
    const double m10 = m.x.y, m11 = m.y.y, m12 = m.z.y;
    const double m20 = m.x.z, m21 = m.y.z, m22 = m.z.z;
    Quat q;
    const double tr = m00 + m11 + m22;
    if (tr > 0.0) {
        const double s = std::sqrt(tr + 1.0) * 2.0;
        q.w = 0.25 * s;
        q.x = (m21 - m12) / s;
        q.y = (m02 - m20) / s;
        q.z = (m10 - m01) / s;
    } else if (m00 > m11 && m00 > m22) {
        const double s = std::sqrt(1.0 + m00 - m11 - m22) * 2.0;
        q.w = (m21 - m12) / s;
        q.x = 0.25 * s;
        q.y = (m01 + m10) / s;
        q.z = (m02 + m20) / s;
    } else if (m11 > m22) {
        const double s = std::sqrt(1.0 + m11 - m00 - m22) * 2.0;
        q.w = (m02 - m20) / s;
        q.x = (m01 + m10) / s;
        q.y = 0.25 * s;
        q.z = (m12 + m21) / s;
    } else {
        const double s = std::sqrt(1.0 + m22 - m00 - m11) * 2.0;
        q.w = (m10 - m01) / s;
        q.x = (m02 + m20) / s;
        q.y = (m12 + m21) / s;
        q.z = 0.25 * s;
    }
    return Normalize(q);
}

inline RotMatrix ToRotMatrix(const Quat& q_in) {
    const Quat q = Normalize(q_in);
    const double xx = q.x * q.x, yy = q.y * q.y, zz = q.z * q.z;
    const double xy = q.x * q.y, xz = q.x * q.z, yz = q.y * q.z;
    const double wx = q.w * q.x, wy = q.w * q.y, wz = q.w * q.z;
    RotMatrix m{};
    m.x = {1.0 - 2.0 * (yy + zz), 2.0 * (xy + wz), 2.0 * (xz - wy)};
    m.y = {2.0 * (xy - wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz + wx)};
    m.z = {2.0 * (xz + wy), 2.0 * (yz - wx), 1.0 - 2.0 * (xx + yy)};
    return m;
}

inline Pose FromCartesian(const CartesianPose& pose, const RotMatrix& rot) {
    return {{pose.tran.x, pose.tran.y, pose.tran.z}, FromRotMatrix(rot)};
}

inline CartesianPose ToCartesianTran(const Pose& pose) {
    CartesianPose out{};
    out.tran = {pose.p.x, pose.p.y, pose.p.z};
    out.rpy = {0.0, 0.0, 0.0};
    return out;
}

inline Vec3 AngularVelocity(const Quat& q_prev, const Quat& q_now, double dt) {
    if (dt < 1e-6) {
        return {};
    }
    const Quat dq = Mul(q_now, Conjugate(q_prev));
    const double w = std::max(-1.0, std::min(1.0, dq.w));
    const double ang = 2.0 * std::acos(std::abs(w));
    const double s = std::sqrt(std::max(0.0, 1.0 - w * w));
    Vec3 axis{dq.x, dq.y, dq.z};
    if (s > 1e-8) {
        axis = Scale(axis, 1.0 / s);
    }
    if (dq.w < 0.0) {
        axis = Scale(axis, -1.0);
    }
    return Scale(axis, ang / dt);
}

}  // namespace jaka_se3
