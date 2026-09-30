#ifndef COMPUTE_TWO_POINT_POSES_H
#define COMPUTE_TWO_POINT_POSES_H

#include <algorithm>
#include <cmath>
#include <utility>

#include <Eigen/Geometry>

/**
 * 平焊 vs 立焊：TCP 轴分配不同。
 *
 * 约定：Z = 焊枪指向。平焊（PA）X = 沿缝行走。
 * 立焊（PF）若仍用 X 作竖直行走，鹅颈枪头（工具 +X）会朝天。
 * 工业做法：立缝把行走放到 Y（下→上），X 保持水平，Z 仍指向工件。
 */
enum class WeldTravelPolicy {
    KeepGiven,       ///< 尊重 pose_start → pose_end
    PreferDownhill,  ///< 平焊/斜板：travel·up ≤ 0
    PreferUphill     ///< 强制上坡；立焊 Auto 时也会走这条
};

enum class WeldPosition {
    Auto,      ///< |seam · world_up| ≥ vertical_seam_abs_cos → 立焊，否则平焊
    Flat,      ///< 平焊 PA：X = 行走，Y = 侧向
    Vertical   ///< 立焊 PF：Y = 下→上行走，X = 水平
};

struct ComputeTwoPointPosesOptions {
    Eigen::Vector3f world_up = Eigen::Vector3f::UnitZ();
    WeldTravelPolicy travel_policy = WeldTravelPolicy::PreferDownhill;
    WeldPosition weld_position = WeldPosition::Auto;
    /// |seam · world_up| ≥ 此值判定为立缝（默认约 60° 仰角）。
    float vertical_seam_abs_cos = 0.5f;
    float steep_seam_abs_cos = 0.5f;
    Eigen::Vector3f preferred_torch = -Eigen::Vector3f::UnitZ();
    float max_torch_tilt_deg = 25.f;
    float travel_angle_deg = 10.f;
    bool gravity_signed_travel_angle = true;
    /// 鹅颈沿工具 +X。立焊时 +X 在水平面，不再跟着竖直行走。
    Eigen::Vector3f tool_head_axis = Eigen::Vector3f::UnitX();
    bool flip_travel_to_raise_y = false;
    /// 当前焊枪 TCP 的 X（来自 mdl 法兰×工具）。非零则路径跟着枪 +X，避免起终点把 X 翻 180°。
    Eigen::Vector3f torch_x = Eigen::Vector3f::Zero();
    /// |travel · torch_x| 小于此值不换向，避免近 90° 抖动/腕部奇异。
    float x_align_hysteresis = 0.2f;
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

inline bool isVerticalSeam(const Eigen::Vector3f& seam,
                          const Eigen::Vector3f& world_up,
                          const ComputeTwoPointPosesOptions& opt)
{
    if (opt.weld_position == WeldPosition::Flat)
        return false;
    if (opt.weld_position == WeldPosition::Vertical)
        return true;
    return std::fabs(seam.dot(world_up)) >= opt.vertical_seam_abs_cos;
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
    Eigen::Vector3f t = projectPerp(travel, z);
    if (t.squaredNorm() < 1e-12f) {
        t = projectPerp(world_up, z);
        if (t.squaredNorm() < 1e-12f) {
            const Eigen::Vector3f alt = (std::fabs(z.x()) < 0.9f)
                                            ? Eigen::Vector3f::UnitX()
                                            : Eigen::Vector3f::UnitY();
            t = projectPerp(alt, z);
        }
    }
    t = finiteUnit(t, Eigen::Vector3f::UnitX());
    if (t.dot(travel) < 0.f)
        t = -t;
    return t;
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

/** 平焊：行走在 X，行走角绕 Y。 */
inline Frame rotateAroundY(const Frame& f, float rad)
{
    const float c = std::cos(rad);
    const float s = std::sin(rad);
    return makeFrame(f.x * c + f.z * s, f.y, -f.x * s + f.z * c);
}

/** 立焊：行走在 Y，行走角绕 X，鹅颈（X）保持水平。 */
inline Frame rotateAroundX(const Frame& f, float rad)
{
    const float c = std::cos(rad);
    const float s = std::sin(rad);
    return makeFrame(f.x, f.y * c + f.z * s, -f.y * s + f.z * c);
}

inline Frame rotateTravelAngle(const Frame& f, float rad, bool vertical)
{
    return vertical ? rotateAroundX(f, rad) : rotateAroundY(f, rad);
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
                                  const Eigen::Vector3f& world_up,
                                  bool vertical)
{
    const float mag = std::fabs(opt.travel_angle_deg) * kDegToRad;
    if (mag < 1e-8f)
        return 0.f;
    const float plus = (opt.travel_angle_deg >= 0.f) ? mag : -mag;
    if (!opt.gravity_signed_travel_angle)
        return plus;
    const Frame a = rotateTravelAngle(probe, plus, vertical);
    const Frame b = rotateTravelAngle(probe, -plus, vertical);
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
                                        WeldTravelPolicy policy,
                                        const Eigen::Vector3f& world_up)
{
    const float du = travel.dot(world_up);
    const bool going_up = du > 1e-6f;
    const bool going_down = du < -1e-6f;
    const bool swap =
        (policy == WeldTravelPolicy::PreferDownhill && going_up) ||
        (policy == WeldTravelPolicy::PreferUphill && going_down);
    if (swap) {
        std::swap(t0, t1);
        std::swap(z0, z1);
        travel = -travel;
    }
    return travel;
}

/** 路径跟着焊枪 +X：明显反向才对调起终点，不在 90° 附近翻转。 */
inline Eigen::Vector3f alignTravelToTorchX(Eigen::Vector3f travel,
                                          Eigen::Vector3f& t0,
                                          Eigen::Vector3f& t1,
                                          Eigen::Vector3f& z0,
                                          Eigen::Vector3f& z1,
                                          const Eigen::Vector3f& torch_x,
                                          const Eigen::Vector3f& z_mean,
                                          float hysteresis)
{
    if (torch_x.squaredNorm() < 1e-12f)
        return travel;
    Eigen::Vector3f xref = projectPerp(torch_x, z_mean);
    if (xref.squaredNorm() < 1e-12f)
        xref = torch_x;
    xref = finiteUnit(xref, travel);
    if (travel.dot(xref) < -hysteresis) {
        std::swap(t0, t1);
        std::swap(z0, z1);
        travel = -travel;
    }
    return travel;
}

inline Frame pickFrameFacingTorchX(const Frame& f,
                                  const Eigen::Vector3f& torch_x,
                                  float hysteresis)
{
    if (torch_x.squaredNorm() < 1e-12f)
        return f;
    const float d = f.x.dot(torch_x);
    if (d >= -hysteresis)
        return f;
    return makeFrame(-f.x, -f.y, f.z);
}

} // namespace weld_pose_detail

/**
 * @brief 由起点/终点 TCP 就地修正免示教焊接姿态。
 *
 * 输入：`pose_start/end` 的平移为焊点，Z 列为枪轴。
 * 输出：同一对姿态，X 与焊枪正向一致。无需点云。
 */
inline bool computeTwoPointPoses(
    Eigen::Affine3f& pose_start,
    Eigen::Affine3f& pose_end,
    const ComputeTwoPointPosesOptions& opt = ComputeTwoPointPosesOptions())
{
    Eigen::Vector3f t0 = pose_start.translation();
    Eigen::Vector3f t1 = pose_end.translation();
    const Eigen::Vector3f weld = t1 - t0;
    if (weld.squaredNorm() < 1e-12f)
        return false;

    const Eigen::Vector3f world_up =
        weld_pose_detail::finiteUnit(opt.world_up, Eigen::Vector3f::UnitZ());
    const Eigen::Vector3f seam = weld.normalized();
    const bool vertical = weld_pose_detail::isVerticalSeam(seam, world_up, opt);
    const Eigen::Vector3f& torch_x = opt.torch_x;

    auto prepareZ = [&](const Eigen::Vector3f& z_in) {
        const Eigen::Vector3f n = weld_pose_detail::finiteUnit(z_in, world_up);
        return weld_pose_detail::tiltTorchToward(
            n, opt.preferred_torch, opt.max_torch_tilt_deg);
    };

    Eigen::Vector3f z0 = prepareZ(pose_start.linear().col(2));
    Eigen::Vector3f z1 = prepareZ(pose_end.linear().col(2));

    WeldTravelPolicy policy = opt.travel_policy;
    if (vertical && policy != WeldTravelPolicy::KeepGiven)
        policy = WeldTravelPolicy::PreferUphill;

    Eigen::Vector3f travel = weld_pose_detail::applyTravelPolicy(
        seam, t0, t1, z0, z1, policy, world_up);

    Eigen::Vector3f z_mean = weld_pose_detail::finiteUnit(z0 + z1, z0);
    travel = weld_pose_detail::alignTravelToTorchX(
        travel, t0, t1, z0, z1, torch_x, z_mean, opt.x_align_hysteresis);
    z_mean = weld_pose_detail::finiteUnit(z0 + z1, z0);

    const bool steep = std::fabs(travel.dot(world_up)) >= opt.steep_seam_abs_cos;
    const Eigen::Vector3f y_probe = z_mean.cross(travel);
    const bool have_torch_x = torch_x.squaredNorm() >= 1e-12f;
    const bool flip_xy = !vertical && !have_torch_x && opt.flip_travel_to_raise_y && !steep &&
                         y_probe.squaredNorm() >= 1e-12f &&
                         y_probe.dot(world_up) < 0.f;

    auto assemble = [&](const Eigen::Vector3f& z_in) {
        const Eigen::Vector3f z = weld_pose_detail::finiteUnit(z_in, world_up);
        const Eigen::Vector3f along =
            weld_pose_detail::travelAxisInTorchPlane(z, travel, world_up);
        if (vertical) {
            const Eigen::Vector3f y = along;
            Eigen::Vector3f x = y.cross(z);
            if (x.squaredNorm() < 1e-12f)
                x = world_up.cross(z);
            return weld_pose_detail::makeFrame(x, y, z);
        }
        Eigen::Vector3f x = along;
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
        assemble(z_mean), opt, world_up, vertical);

    auto makePose = [&](const Eigen::Vector3f& t, const Eigen::Vector3f& z_in) {
        weld_pose_detail::Frame f = assemble(z_in);
        if (std::fabs(signed_rad) > 1e-8f)
            f = weld_pose_detail::rotateTravelAngle(f, signed_rad, vertical);
        if (!vertical)
            f = weld_pose_detail::pickFrameFacingTorchX(
                f, torch_x, opt.x_align_hysteresis);
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

/**
 * 近→远焊接：X 从近到远，起终点共用同一 X。
 * 只绕 Y 向焊缝内部倾 inward_deg（默认 30°），不绕 X，避免枪体撞两侧壁。
 * 起点 Ry(-θ)：Z 指向终点；终点 Ry(+θ)：Z 指向起点。
 */
inline bool computeWeldTcpStartEnd(
    Eigen::Affine3f& tcp_weld_start,
    Eigen::Affine3f& tcp_weld_end,
    float inward_deg = 30.f)
{
    Eigen::Vector3f t0 = tcp_weld_start.translation();
    Eigen::Vector3f t1 = tcp_weld_end.translation();
    Eigen::Vector3f z0 = tcp_weld_start.linear().col(2);
    Eigen::Vector3f z1 = tcp_weld_end.linear().col(2);
    const Eigen::Vector3f weld = t1 - t0;
    if (weld.squaredNorm() < 1e-12f)
        return false;

    if (t0.squaredNorm() > t1.squaredNorm()) {
        std::swap(t0, t1);
        std::swap(z0, z1);
    }

    const Eigen::Vector3f x_world = (t1 - t0).normalized();
    const Eigen::Vector3f world_up = Eigen::Vector3f::UnitZ();
    const float rad = inward_deg * weld_pose_detail::kDegToRad;

    auto poseAt = [&](const Eigen::Vector3f& t,
                      const Eigen::Vector3f& z_in,
                      float y_rad) {
        const Eigen::Vector3f z =
            weld_pose_detail::finiteUnit(z_in, world_up);
        const Eigen::Vector3f x =
            weld_pose_detail::travelAxisInTorchPlane(z, x_world, world_up);
        Eigen::Vector3f y = z.cross(x);
        if (y.squaredNorm() < 1e-12f)
            y = world_up.cross(x);
        weld_pose_detail::Frame f = weld_pose_detail::makeFrame(x, y, z);
        if (std::fabs(y_rad) > 1e-8f)
            f = weld_pose_detail::rotateAroundY(f, y_rad);
        Eigen::Affine3f T = Eigen::Affine3f::Identity();
        T.linear().col(0) = f.x;
        T.linear().col(1) = f.y;
        T.linear().col(2) = f.z;
        T.translation() = t;
        return T;
    };

    tcp_weld_start = poseAt(t0, z0, -rad);
    tcp_weld_end = poseAt(t1, z1, rad);
    return true;
}

#endif // COMPUTE_TWO_POINT_POSES_H
