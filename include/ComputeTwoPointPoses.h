#ifndef COMPUTE_TWO_POINT_POSES_H
#define COMPUTE_TWO_POINT_POSES_H

#include <cmath>
#include <vector>

#include <Eigen/Geometry>
#include <pcl/kdtree/kdtree_flann.h>

/**
 * @brief 由起点/终点 XYZ 构造相机系下的两个焊接位姿（右手系）。
 *
 * Z：邻域法线均值，计算方式不变。
 * X：与焊缝方向 weld_dir = 终点 − 起点 平行。先把 weld_dir 投到垂直于 Z 的
 *    平面（保证与 Z 正交），再按 Y 朝上取 ±，不要求和起点→终点同向。
 * Y：Y = Z × X；若 Y·world_Z < 0 则 X、Y 一起取反，与世界 +Z 夹角 ≤ 90°。
 * 右手系：X × Y = Z，det(R) = +1。
 *
 * @param start_end 含起点、终点（points[0]/points[1]）
 * @param scene     已带法线的场景点云（用于邻域搜索）
 */
inline bool computeTwoPointPoses(const ct::Cloud::Ptr& start_end,
                                 const ct::Cloud::Ptr& scene,
                                 float radius,
                                 Eigen::Affine3f& pose_start,
                                 Eigen::Affine3f& pose_end)
{
    if (!start_end || start_end->size() < 2 || !scene || scene->empty())
        return false;

    pcl::KdTreeFLANN<ct::PointXYZRGBN> tree;
    tree.setInputCloud(scene);

    auto axisZ = [&](const ct::PointXYZRGBN& q) {
        std::vector<int> ids;
        std::vector<float> d;
        Eigen::Vector3f sum(0.f, 0.f, 0.f);
        if (tree.radiusSearch(q, radius, ids, d) > 0) {
            for (int i : ids)
                sum += Eigen::Vector3f(scene->points[static_cast<size_t>(i)].normal_x,
                                       scene->points[static_cast<size_t>(i)].normal_y,
                                       scene->points[static_cast<size_t>(i)].normal_z);
        } else {
            sum = Eigen::Vector3f(q.normal_x, q.normal_y, q.normal_z);
        }
        if (!std::isfinite(sum.x()) || sum.squaredNorm() < 1e-12f)
            sum = Eigen::Vector3f::UnitZ();
        return Eigen::Vector3f(sum.normalized());
    };

    auto makePose = [](const Eigen::Vector3f& t,
                       const Eigen::Vector3f& z_in,
                       const Eigen::Vector3f& weld_dir) {
        Eigen::Vector3f z = z_in.normalized();
        // X 与起点→终点平行：投到 ⊥Z 平面，保证正交且仍与焊缝共线
        Eigen::Vector3f x = weld_dir - z * z.dot(weld_dir);
        if (x.squaredNorm() < 1e-12f) {
            const Eigen::Vector3f axis = (std::fabs(z.z()) < 0.9f)
                                             ? Eigen::Vector3f::UnitZ()
                                             : Eigen::Vector3f::UnitX();
            x = axis.cross(z);
        }
        x.normalize();
        Eigen::Vector3f y = z.cross(x); // Y = Z × X
        if (y.dot(Eigen::Vector3f::UnitZ()) < 0.f) {
            y = -y;
            x = -x; // 与焊缝仍平行，仅反向
        }
        y.normalize();
        Eigen::Affine3f T = Eigen::Affine3f::Identity();
        T.linear().col(0) = x;
        T.linear().col(1) = y;
        T.linear().col(2) = z;
        T.translation() = t;
        return T;
    };

    const auto& p0 = start_end->points[0];
    const auto& p1 = start_end->points[1];
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
