#ifndef COMPUTE_TWO_POINT_POSES_H
#define COMPUTE_TWO_POINT_POSES_H

#include <algorithm>
#include <cmath>

#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

/**
 * 竖直 / 倾斜焊缝的姿态约束。
 *
 * 焊枪 TCP 必须是正交右手系：Z 沿枪轴（表面法向），X 在 ⊥Z 平面内沿焊缝。
 * 若把 X 设成三维起点→终点、不向 Z 投影，倾斜焊缝会得到非正交旋转，IK 会拧腕。
 * 若仅用 Y·world_Z 决定是否把 X、Y 一起取反，接近竖直时该点积在 0 附近抖动，
 * 行走方向会突然反向，两端还可能各翻一次，路径上出现 180° 扭转。
 */
struct ComputeTwoPointPosesOptions {
    Eigen::Vector3f world_up = Eigen::Vector3f::UnitZ();
    /// |travel · world_up| ≥ 此值视为陡焊缝（默认约 60° 仰角），禁止为抬 Y 而反转 X。
    float steep_seam_abs_cos = 0.5f;
    /// 希望的枪轴（TCP Z）。默认朝世界 -Z，立焊缝会把枪略向下倾，避免平端对着墙。
    Eigen::Vector3f preferred_torch = -Eigen::Vector3f::UnitZ();
    /// 法向朝 preferred_torch 最多转过的角度；异侧半球不转，以免枪穿到工件背面。
    float max_torch_tilt_deg = 25.f;
    /// 仅缓焊缝：若 Y·up < 0 则绕 Z 转 180°（X、Y 同翻），让 Y 朝上。
    bool flip_travel_to_raise_y = true;
};

namespace weld_pose_detail {

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

/** 同侧半球内，把 n 转向 preferred，转角不超过 max_deg。 */
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
    const float max_rad = max_deg * 0.017453292519943295f;
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

} // namespace weld_pose_detail

/**
 * @brief 由轨迹云的起点/终点及其法向构造两个焊接 TCP。
 *
 * `trajectory.points[0]` 起点，`points[1]` 终点。法向为该端表面/坡口方向。
 *
 * - Z：法向，可在同侧半球内向 preferred_torch 限幅倾斜（立墙焊缝枪口略朝下）。
 * - X：焊缝行走方向投到 ⊥Z，且与起点→终点同向；陡焊缝不再为抬 Y 而反向。
 * - Y：Z × X，再正交化，保证 det(R) = +1。
 * - 两端共用同一次“是否绕 Z 翻 180°”的决定，避免路径中拧转。
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
    const Eigen::Vector3f t0(p0.x, p0.y, p0.z);
    const Eigen::Vector3f t1(p1.x, p1.y, p1.z);
    const Eigen::Vector3f weld = t1 - t0;
    if (weld.squaredNorm() < 1e-12f)
        return false;

    const Eigen::Vector3f travel = weld.normalized();
    const Eigen::Vector3f world_up =
        weld_pose_detail::finiteUnit(opt.world_up, Eigen::Vector3f::UnitZ());
    const bool steep = std::fabs(travel.dot(world_up)) >= opt.steep_seam_abs_cos;

    auto prepareZ = [&](const pcl::PointNormal& q) {
        Eigen::Vector3f n(q.normal_x, q.normal_y, q.normal_z);
        n = weld_pose_detail::finiteUnit(n, world_up);
        return weld_pose_detail::tiltTorchToward(
            n, opt.preferred_torch, opt.max_torch_tilt_deg);
    };

    const Eigen::Vector3f z0 = prepareZ(p0);
    const Eigen::Vector3f z1 = prepareZ(p1);

    Eigen::Vector3f z_mean = z0 + z1;
    z_mean = weld_pose_detail::finiteUnit(z_mean, z0);
    const Eigen::Vector3f y_probe = z_mean.cross(travel);
    const bool flip_xy = opt.flip_travel_to_raise_y && !steep &&
                         y_probe.squaredNorm() >= 1e-12f &&
                         y_probe.dot(world_up) < 0.f;

    auto makePose = [&](const Eigen::Vector3f& t, const Eigen::Vector3f& z_in) {
        const Eigen::Vector3f z = weld_pose_detail::finiteUnit(z_in, world_up);
        Eigen::Vector3f x =
            weld_pose_detail::travelAxisInTorchPlane(z, travel, world_up);
        Eigen::Vector3f y = z.cross(x);
        if (y.squaredNorm() < 1e-12f)
            y = world_up.cross(x);
        y = weld_pose_detail::finiteUnit(y, Eigen::Vector3f::UnitY());
        if (flip_xy) {
            x = -x;
            y = -y;
        }
        const Eigen::Vector3f z_rh = x.cross(y);
        Eigen::Affine3f T = Eigen::Affine3f::Identity();
        T.linear().col(0) = x;
        T.linear().col(1) = y;
        T.linear().col(2) = weld_pose_detail::finiteUnit(z_rh, z);
        T.translation() = t;
        return T;
    };

    pose_start = makePose(t0, z0);
    pose_end = makePose(t1, z1);
    return true;
}

#endif // COMPUTE_TWO_POINT_POSES_H
