#ifndef COMPUTE_TWO_POINT_POSES_H
#define COMPUTE_TWO_POINT_POSES_H

#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <vector>

/**
 * @brief 由起点/终点构造相机系下的两个位姿。
 *
 * Z = 半径内法线均值的反方向；Y = 起点→终点；X = Y × Z。
 * scene 无法线时，按 radius 估计法线（视点为相机原点）。
 *
 * @param start_end 含起点、终点（points[0]/points[1]）
 * @param scene     原场景点云（用于邻域与法线）
 */
inline bool computeTwoPointPoses(const ct::Cloud::Ptr& start_end,
                                 const ct::Cloud::Ptr& scene,
                                 float radius,
                                 Eigen::Affine3f& pose_start,
                                 Eigen::Affine3f& pose_end)
{
    if (!start_end || start_end->size() < 2 || !scene || scene->empty())
        return false;

    auto hasNormals = [](const ct::Cloud::Ptr& c) {
        const size_t n = c->size();
        const size_t step = std::max<size_t>(1, n / 32);
        for (size_t i = 0; i < n; i += step) {
            const auto& p = c->points[i];
            if (std::isfinite(p.normal_x) &&
                (p.normal_x != 0.f || p.normal_y != 0.f || p.normal_z != 0.f))
                return true;
        }
        return false;
    };

    if (!hasNormals(scene)) {
        pcl::PointCloud<pcl::Normal> nrm;
        pcl::NormalEstimation<ct::PointXYZRGBN, pcl::Normal> ne;
        ne.setInputCloud(scene);
        ne.setRadiusSearch(radius);
        ne.setViewPoint(0.f, 0.f, 0.f);
        ne.compute(nrm);
        for (size_t i = 0; i < scene->size(); ++i) {
            scene->points[i].normal_x = nrm[i].normal_x;
            scene->points[i].normal_y = nrm[i].normal_y;
            scene->points[i].normal_z = nrm[i].normal_z;
        }
    }

    auto axisZ = [&](const ct::PointXYZRGBN& q) {
        pcl::KdTreeFLANN<ct::PointXYZRGBN> tree;
        tree.setInputCloud(scene);
        std::vector<int> ids;
        std::vector<float> d;
        Eigen::Vector3f sum(0, 0, 0);
        if (tree.radiusSearch(q, radius, ids, d) > 0) {
            for (int i : ids)
                sum += Eigen::Vector3f(scene->points[i].normal_x,
                                       scene->points[i].normal_y,
                                       scene->points[i].normal_z);
        } else {
            sum = Eigen::Vector3f(q.normal_x, q.normal_y, q.normal_z);
        }
        if (!std::isfinite(sum.x()) || sum.squaredNorm() < 1e-12f)
            sum = Eigen::Vector3f::UnitZ();
        return Eigen::Vector3f(-sum.normalized());
    };

    auto makePose = [](const Eigen::Vector3f& t,
                       const Eigen::Vector3f& z_in,
                       const Eigen::Vector3f& y_in) {
        Eigen::Vector3f z = z_in.normalized();
        Eigen::Vector3f x = y_in.cross(z);
        if (x.squaredNorm() < 1e-12f)
            x = ((std::fabs(z.z()) < 0.9f) ? Eigen::Vector3f::UnitZ()
                                           : Eigen::Vector3f::UnitX()).cross(z);
        x.normalize();
        Eigen::Vector3f y = z.cross(x).normalized();
        Eigen::Affine3f T = Eigen::Affine3f::Identity();
        T.linear().col(0) = x;
        T.linear().col(1) = y;
        T.linear().col(2) = z;
        T.translation() = t;
        return T;
    };

    const auto& p0 = start_end->points[0];
    const auto& p1 = start_end->points[1];
    Eigen::Vector3f t0(p0.x, p0.y, p0.z), t1(p1.x, p1.y, p1.z);
    Eigen::Vector3f y_dir = t1 - t0;
    if (y_dir.squaredNorm() < 1e-12f)
        return false;

    pose_start = makePose(t0, axisZ(p0), y_dir);
    pose_end   = makePose(t1, axisZ(p1), y_dir);
    return true;
}

#endif // COMPUTE_TWO_POINT_POSES_H
