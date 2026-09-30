#ifndef COMPUTE_TWO_POINT_POSES_H
#define COMPUTE_TWO_POINT_POSES_H

#include <algorithm>
#include <cmath>
#include <utility>

#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

/**
 * 免示教里路径方向和枪头朝向必须拆开。
 *
 * 鹅颈枪的枪头大致沿工具 +X。若强制 TCP-X = 行走，X 朝天则枪头朝天。
 * 工业做法：
 * 1) 工艺先定上坡/下坡（换起终点），不要用绕 Z 转 180° 去“选方向”；
 * 2) 行走角绕 Y，符号按重力取，让枪头更朝下，与 X 正负解耦；
 * 3) 电缆/枪身朝下用绕缝（绕 X）的工作角，而不是再翻 X。
 */
enum class WeldTravelPolicy {
    KeepGiven,       ///< 尊重 trajectory[0] → [1]
    PreferDownhill,  ///< 免示教默认：travel·up ≤ 0（ISO PG，薄板/CO2 常用）
    PreferUphill     ///< ISO PF，厚板/角焊常用
};

struct ComputeTwoPointPosesOptions {
    Eigen::Vector3f world_up = Eigen::Vector3f::UnitZ();
    WeldTravelPolicy travel_policy = WeldTravelPolicy::PreferDownhill;
    /// |travel · world_up| ≥ 此值视为陡焊缝，禁止为抬 Y 而反转 X。
    float steep_seam_abs_cos = 0.5f;
    Eigen::Vector3f preferred_torch = -Eigen::Vector3f::UnitZ();
    float max_torch_tilt_deg = 25.f;
    /// 绕工具 Y 的行走角（推进/拖曳）。符号由重力决定：枪头更朝下的一侧。
    float travel_angle_deg = 10.f;
    bool gravity_signed_travel_angle = true;
    /// 工具系枪头方向。默认 +X（鹅颈沿行走）；喷嘴沿 Z 则改为 UnitZ。
    Eigen::Vector3f tool_head_axis = Eigen::Vector3f::UnitX();
    /// 仅缓焊缝且显式打开：绕 Z 转 180° 抬 Y。免示教默认关闭，避免枪头随 X 翻面。
    bool flip_travel_to_raise_y = false;
};

namespace weld_pose_detail {

constexpr float kDegToRad = 0.017453292519943295f;

inline Eigen::Vector3f finiteUnit(const Eigen::Vector3f& v,
                                 const Eigen::Vector3f& fallback)
{
    if (!std::isfinite(v.x()) || v.squaredNorm() < 1e-12f)
        return fallback;
    return v.normalized();
}

inline Eigen::Vector3f projectPerp(const Eigen::Vector3f& v,
                                  const Eigen::Vector3f& unit_axis)
{
    return v - unit_axis * v.dot(unit_axis);
}

inline Eigen::Vector3f tiltTorchToward(const Eigen::Vector3f& n,
                                      const Eigen::Vector3f& preferred,
                                      float max_deg)
{
    if (preferred.squaredNorm() < 1e-12f || max_deg <= 0.f)
        return n;
    const Eigen::Vector3f p = preferred.normalized();
    const float c = std::max(-1.f, std::min(1.f, n.dot(p)));
    if (c < 0.f)
        return n;
    const float ang = std::acos(c);
    const float max_rad = max_deg * kDegToRad;
    if (ang <= max_rad)
        return n;
    Eigen::Vector3f axis = n.cross(p);
    if (axis.squaredNorm() < 1e-12f)
        return n;
    return Eigen::AngleAxisf(max_rad, axis.normalized()) * n;
}

inline Eigen::Vector3f travelAxisInTorchPlane(const Eigen::Vector3f& z,
                                             const Eigen::Vector3f& travel,
                                             const Eigen::Vector3f& world_up)
{
    Eigen::Vector3f x = projectPerp(travel, z);
    if (x.squaredNorm() < 1e-12f) {
        x = projectPerp(world_up, z);
        if (x.squaredNorm() < 1e-12f) {
            const Eigen::Vector3f alt = (std::fabs(z.x()) < 0.9f)
                                            ? Eigen::Vector3f::UnitX()
                                            : Eigen::Vector3f::UnitY();
            x = projectPerp(alt, z);
        }
    }
    x = finiteUnit(x, Eigen::Vector3f::UnitX());
    if (x.dot(travel) < 0.f)
        x = -x;
    return x;
}

struct Frame {
    Eigen::Vector3f x, y, z;
};

inline Frame makeFrame(const Eigen::Vector3f& x_in,
                       const Eigen::Vector3f& y_in,
                       const Eigen::Vector3f& z_in)
{
    Frame f;
    f.x = finiteUnit(x_in, Eigen::Vector3f::UnitX());
    f.y = finiteUnit(y_in, Eigen::Vector3f::UnitY());
    f.z = finiteUnit(f.x.cross(f.y), z_in);
    f.y = finiteUnit(f.z.cross(f.x), f.y);
    return f;
}

/** 绕工具 Y 转 rad：改变行走角，不反转路径顺序。 */
inline Frame rotateAroundY(const Frame& f, float rad)
{
    const float c = std::cos(rad);
    const float s = std::sin(rad);
    return makeFrame(f.x * c + f.z * s, f.y, -f.x * s + f.z * c);
}

inline float headUp(const Frame& f,
                    const Eigen::Vector3f& tool_head,
                    const Eigen::Vector3f& world_up)
{
    Eigen::Matrix3f R;
    R.col(0) = f.x;
    R.col(1) = f.y;
    R.col(2) = f.z;
    const Eigen::Vector3f head =
        R * finiteUnit(tool_head, Eigen::Vector3f::UnitX());
    return head.dot(world_up);
}

inline float signedTravelAngleRad(const Frame& probe,
                                  const ComputeTwoPointPosesOptions& opt,
                                  const Eigen::Vector3f& world_up)
{
    const float mag = std::fabs(opt.travel_angle_deg) * kDegToRad;
    if (mag < 1e-8f)
        return 0.f;
    const float plus = (opt.travel_angle_deg >= 0.f) ? mag : -mag;
    if (!opt.gravity_signed_travel_angle)
        return plus;
    const Frame a = rotateAroundY(probe, plus);
    const Frame b = rotateAroundY(probe, -plus);
    return (headUp(a, opt.tool_head_axis, world_up) <=
            headUp(b, opt.tool_head_axis, world_up))
               ? plus
               : -plus;
}

inline Eigen::Vector3f applyTravelPolicy(Eigen::Vector3f travel,
                                        Eigen::Vector3f& t0,
                                        Eigen::Vector3f& t1,
                                        Eigen::Vector3f& z0,
                                        Eigen::Vector3f& z1,
                                        const ComputeTwoPointPosesOptions& opt,
                                        const Eigen::Vector3f& world_up)
{
    const float du = travel.dot(world_up);
    const bool going_up = du > 1e-6f;
    const bool going_down = du < -1e-6f;
    const bool swap =
        (opt.travel_policy == WeldTravelPolicy::PreferDownhill && going_up) ||
        (opt.travel_policy == WeldTravelPolicy::PreferUphill && going_down);
    if (swap) {
        std::swap(t0, t1);
        std::swap(z0, z1);
        travel = -travel;
    }
    return travel;
}

} // namespace weld_pose_detail

/**
 * @brief 由两点轨迹构造免示教焊接 TCP。
 *
 * 默认 PreferDownhill：高点为起点、X 朝下，避免鹅颈枪头朝天。
 * 行走角按重力选符号，两端共用，不靠绕 Z 翻 180° 来选枪头朝向。
 */
inline bool computeTwoPointPoses(
    const pcl::PointCloud<pcl::PointNormal>& trajectory,
    Eigen::Affine3f& pose_start,
    Eigen::Affine3f& pose_end,
    const ComputeTwoPointPosesOptions& opt = ComputeTwoPointPosesOptions())
{
    if (trajectory.size() < 2)
        return false;

    const pcl::PointNormal& p0 = trajectory.points[0];
    const pcl::PointNormal& p1 = trajectory.points[1];
    Eigen::Vector3f t0(p0.x, p0.y, p0.z);
    Eigen::Vector3f t1(p1.x, p1.y, p1.z);
    const Eigen::Vector3f weld = t1 - t0;
    if (weld.squaredNorm() < 1e-12f)
        return false;

    const Eigen::Vector3f world_up =
        weld_pose_detail::finiteUnit(opt.world_up, Eigen::Vector3f::UnitZ());

    auto prepareZ = [&](const pcl::PointNormal& q) {
        Eigen::Vector3f n(q.normal_x, q.normal_y, q.normal_z);
        n = weld_pose_detail::finiteUnit(n, world_up);
        return weld_pose_detail::tiltTorchToward(
            n, opt.preferred_torch, opt.max_torch_tilt_deg);
    };

    Eigen::Vector3f z0 = prepareZ(p0);
    Eigen::Vector3f z1 = prepareZ(p1);
    Eigen::Vector3f travel = weld_pose_detail::applyTravelPolicy(
        weld.normalized(), t0, t1, z0, z1, opt, world_up);

    const bool steep = std::fabs(travel.dot(world_up)) >= opt.steep_seam_abs_cos;
    Eigen::Vector3f z_mean = weld_pose_detail::finiteUnit(z0 + z1, z0);
    const Eigen::Vector3f y_probe = z_mean.cross(travel);
    const bool flip_xy = opt.flip_travel_to_raise_y && !steep &&
                         y_probe.squaredNorm() >= 1e-12f &&
                         y_probe.dot(world_up) < 0.f;

    auto assemble = [&](const Eigen::Vector3f& z_in) {
        const Eigen::Vector3f z = weld_pose_detail::finiteUnit(z_in, world_up);
        Eigen::Vector3f x =
            weld_pose_detail::travelAxisInTorchPlane(z, travel, world_up);
        Eigen::Vector3f y = z.cross(x);
        if (y.squaredNorm() < 1e-12f)
            y = world_up.cross(x);
        if (flip_xy) {
            x = -x;
            y = -y;
        }
        return weld_pose_detail::makeFrame(x, y, z);
    };

    const float signed_rad = weld_pose_detail::signedTravelAngleRad(
        assemble(z_mean), opt, world_up);

    auto makePose = [&](const Eigen::Vector3f& t, const Eigen::Vector3f& z_in) {
        weld_pose_detail::Frame f = assemble(z_in);
        if (std::fabs(signed_rad) > 1e-8f)
            f = weld_pose_detail::rotateAroundY(f, signed_rad);
        Eigen::Affine3f T = Eigen::Affine3f::Identity();
        T.linear().col(0) = f.x;
        T.linear().col(1) = f.y;
        T.linear().col(2) = f.z;
        T.translation() = t;
        return T;
    };

    pose_start = makePose(t0, z0);
    pose_end = makePose(t1, z1);
    return true;
}

#endif // COMPUTE_TWO_POINT_POSES_H
