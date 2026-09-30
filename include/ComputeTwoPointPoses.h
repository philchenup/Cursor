#ifndef COMPUTE_TWO_POINT_POSES_H
#define COMPUTE_TWO_POINT_POSES_H

#include <cmath>

#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

/**
 * @brief 由轨迹云的起点/终点及其法向构造两个焊接位姿。
 *
 * `trajectory` 至少含两个点：`points[0]` 为起点，`points[1]` 为终点。
 * 每个点的 `normal_*` 直接作为该端 Z，不再对场景做半径邻域搜索。
 *
 * X：焊缝 3D 方向 `normalize(终点 − 起点)`，不投到 ⊥Z 平面。
 * Y：`Y = Z × X`；若 `Y·world_Z < 0` 则 X、Y 一起取反，使 Y 朝世界 +Z。
 *
 * @param trajectory 含起点、终点及法向的 `pcl::PointNormal` 云
 * @param pose_start 输出起点 TCP
 * @param pose_end   输出终点 TCP
 */
inline bool computeTwoPointPoses(
    const pcl::PointCloud<pcl::PointNormal>& trajectory,
    Eigen::Affine3f& pose_start,
    Eigen::Affine3f& pose_end)
{
    if (trajectory.size() < 2)
        return false;

    auto axisZ = [](const pcl::PointNormal& q) {
        Eigen::Vector3f n(q.normal_x, q.normal_y, q.normal_z);
        if (!std::isfinite(n.x()) || n.squaredNorm() < 1e-12f)
            n = Eigen::Vector3f::UnitZ();
        return Eigen::Vector3f(n.normalized());
    };

    auto makePose = [](const Eigen::Vector3f& t,
                       const Eigen::Vector3f& z_in,
                       const Eigen::Vector3f& weld_dir) {
        Eigen::Vector3f z = z_in.normalized();
        // X = 起点→终点的 3D 方向，不向 Z 做投影
        Eigen::Vector3f x = weld_dir.normalized();
        Eigen::Vector3f y = z.cross(x);
        if (y.squaredNorm() < 1e-12f) {
            const Eigen::Vector3f axis = (std::fabs(z.z()) < 0.9f)
                                             ? Eigen::Vector3f::UnitZ()
                                             : Eigen::Vector3f::UnitX();
            y = z.cross(axis);
        }
        if (y.dot(Eigen::Vector3f::UnitZ()) < 0.f) {
            y = -y;
            x = -x;
        }
        y.normalize();
        Eigen::Affine3f T = Eigen::Affine3f::Identity();
        T.linear().col(0) = x;
        T.linear().col(1) = y;
        T.linear().col(2) = z;
        T.translation() = t;
        return T;
    };

    const pcl::PointNormal& p0 = trajectory.points[0];
    const pcl::PointNormal& p1 = trajectory.points[1];
    const Eigen::Vector3f t0(p0.x, p0.y, p0.z);
    const Eigen::Vector3f t1(p1.x, p1.y, p1.z);
    const Eigen::Vector3f weld_dir = t1 - t0;
    if (weld_dir.squaredNorm() < 1e-12f)
        return false;

    pose_start = makePose(t0, axisZ(p0), weld_dir);
    pose_end = makePose(t1, axisZ(p1), weld_dir);
    return true;
}

#endif // COMPUTE_TWO_POINT_POSES_H
