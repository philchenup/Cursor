#include "arm_avoidance/SixAxisArm.h"

#include <algorithm>
#include <cmath>

namespace arm_avoidance {
namespace {

Mat4 dhTransform(double a, double alpha, double d, double theta) {
    const double ct = std::cos(theta);
    const double st = std::sin(theta);
    const double ca = std::cos(alpha);
    const double sa = std::sin(alpha);
    Mat4 T;
    T.m[0][0] = ct;
    T.m[0][1] = -st * ca;
    T.m[0][2] = st * sa;
    T.m[0][3] = a * ct;
    T.m[1][0] = st;
    T.m[1][1] = ct * ca;
    T.m[1][2] = -ct * sa;
    T.m[1][3] = a * st;
    T.m[2][0] = 0;
    T.m[2][1] = sa;
    T.m[2][2] = ca;
    T.m[2][3] = d;
    T.m[3][0] = 0;
    T.m[3][1] = 0;
    T.m[3][2] = 0;
    T.m[3][3] = 1;
    return T;
}

}  // namespace

SixAxisArm::SixAxisArm(std::array<LinkDH, 6> links, std::array<double, 6> link_radius)
    : links_(links), link_radius_(link_radius) {}

SixAxisArm SixAxisArm::DefaultIndustrial() {
    std::array<LinkDH, 6> links{{
        {0.0, kPi / 2, 0.30, 0, -kPi, kPi},
        {0.40, 0.0, 0.0, 0, -kPi, kPi},
        {0.30, 0.0, 0.0, 0, -kPi, kPi},
        {0.0, kPi / 2, 0.10, 0, -kPi, kPi},
        {0.0, -kPi / 2, 0.10, 0, -kPi, kPi},
        {0.0, 0.0, 0.05, 0, -kPi, kPi},
    }};
    const std::array<double, 6> radius{0.045, 0.040, 0.036, 0.030, 0.026, 0.022};
    return SixAxisArm(links, radius);
}

ChainState SixAxisArm::forward(const std::array<double, 6>& q) const {
    ChainState chain;
    chain.frames[0].rotation = Mat3::identity();
    chain.frames[0].position = {0, 0, 0};
    Mat4 T = Mat4::identity();
    for (int i = 0; i < 6; ++i) {
        const LinkDH& link = links_[static_cast<size_t>(i)];
        T = T.mul(dhTransform(link.a, link.alpha, link.d, q[static_cast<size_t>(i)] + link.theta_offset));
        chain.frames[static_cast<size_t>(i + 1)].rotation = T.rotation();
        chain.frames[static_cast<size_t>(i + 1)].position = T.translation();
    }
    return chain;
}

void SixAxisArm::positionalJacobian(const ChainState& chain, int link, const Vec3& local,
                                    double J[3][6]) const {
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 6; ++c) {
            J[r][c] = 0;
        }
    }
    const Vec3 point = worldPoint(chain, link, local);
    const int last = std::max(0, std::min(link, 6));
    for (int joint = 1; joint <= last; ++joint) {
        const Vec3 axis = chain.frames[static_cast<size_t>(joint - 1)].rotation.col(2);
        const Vec3 origin = chain.frames[static_cast<size_t>(joint - 1)].position;
        const Vec3 column = axis.cross(point - origin);
        J[0][joint - 1] = column.x;
        J[1][joint - 1] = column.y;
        J[2][joint - 1] = column.z;
    }
}

Vec3 SixAxisArm::worldPoint(const ChainState& chain, int link, const Vec3& local) const {
    const FramePose& frame = chain.frames[static_cast<size_t>(std::max(0, std::min(link, 6)))];
    return frame.rotation.mul(local) + frame.position;
}

std::vector<ControlPoint> SixAxisArm::controlPoints(const ChainState& chain) const {
    std::vector<ControlPoint> points;
    points.reserve(18);
    const double samples[] = {1.0, 2.0 / 3.0, 1.0 / 3.0};
    for (int link = 1; link <= 6; ++link) {
        const Vec3 start = chain.frames[static_cast<size_t>(link - 1)].position;
        const Vec3 end = chain.frames[static_cast<size_t>(link)].position;
        const Mat3 rotation_t = chain.frames[static_cast<size_t>(link)].rotation.transpose();
        const double radius = link_radius_[static_cast<size_t>(link - 1)];
        for (double t : samples) {
            const Vec3 world = start * (1.0 - t) + end * t;
            ControlPoint point;
            point.link = link;
            point.local = rotation_t.mul(world - end);
            point.radius = radius;
            points.push_back(point);
        }
    }
    return points;
}

std::array<double, 6> SixAxisArm::clampJoints(const std::array<double, 6>& q) const {
    std::array<double, 6> clamped = q;
    for (int i = 0; i < 6; ++i) {
        const LinkDH& link = links_[static_cast<size_t>(i)];
        clamped[static_cast<size_t>(i)] =
            clampDouble(q[static_cast<size_t>(i)], link.q_min, link.q_max);
    }
    return clamped;
}

}  // namespace arm_avoidance
