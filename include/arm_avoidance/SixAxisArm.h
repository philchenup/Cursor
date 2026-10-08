#pragma once

#include "arm_avoidance/Types.h"

#include <array>
#include <vector>

namespace arm_avoidance {

constexpr double kPi = 3.14159265358979323846;

// 经典 DH：T = Rot_z(θ) Trans_z(d) Trans_x(a) Rot_x(α)
struct LinkDH {
    double a = 0;
    double alpha = 0;
    double d = 0;
    double theta_offset = 0;
    double q_min = -kPi;
    double q_max = kPi;
};

// 固连在某一连杆上的碰撞球。local 随该连杆运动。
struct ControlPoint {
    int link = 1;  // 1..6
    Vec3 local;
    double radius = 0.03;
};

struct FramePose {
    Mat3 rotation = Mat3::identity();
    Vec3 position;
};

struct ChainState {
    // frames[0] 是基座，frames[i] 是第 i 个关节之后的连杆坐标系。
    std::array<FramePose, 7> frames;
};

// 六轴串联机械臂。控制点取各连杆关节原点以及连杆上 1/3、2/3 处，
// 用来近似包络连杆，供实时距离查询。
class SixAxisArm {
public:
    SixAxisArm(std::array<LinkDH, 6> links, std::array<double, 6> link_radius);

    static SixAxisArm DefaultIndustrial();

    const std::array<LinkDH, 6>& links() const { return links_; }
    const std::array<double, 6>& linkRadius() const { return link_radius_; }

    ChainState forward(const std::array<double, 6>& q) const;

    // 位置雅可比，3x6，把关节速度映射到 base 系下该点的线速度。
    void positionalJacobian(const ChainState& chain, int link, const Vec3& local,
                            double J[3][6]) const;

    Vec3 worldPoint(const ChainState& chain, int link, const Vec3& local) const;

    std::vector<ControlPoint> controlPoints(const ChainState& chain) const;

    std::array<double, 6> clampJoints(const std::array<double, 6>& q) const;

private:
    std::array<LinkDH, 6> links_;
    std::array<double, 6> link_radius_;
};

}  // namespace arm_avoidance
