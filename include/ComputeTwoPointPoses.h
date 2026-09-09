#ifndef COMPUTE_TWO_POINT_POSES_H
#define COMPUTE_TWO_POINT_POSES_H

#include <cmath>
#include <vector>

#include <Eigen/Geometry>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

/**
 * @brief 由起点/终点 XYZ 构造相机系下的两个焊接位姿。
 *
 * 对每个查询点：
 *   1. 只提取半径 2R 内的场景点（R = radius）；
 *   2. 仅在该局部点云上以半径 R 估算法线（视点为相机原点）。
 *      裁到 2R 后，内层 R 内每个点的 R 邻域都完整；
 *   3. 对内层 R 的有效法线取平均，Z 取该均值的反方向（焊枪指向工件）。
 *      Y 为起点→终点，X = Y × Z。
 *
 * 不改写 scene，也不对整幅点云估算法线。整幅场景只建一次 KdTree。
 *
 * @param start_end 含起点、终点（points[0]/points[1] 的 XYZ）
 * @param scene     原场景点云（用于邻域搜索）
 */
inline bool computeTwoPointPoses(const ct::Cloud::Ptr& start_end,
                                 const ct::Cloud::Ptr& scene,
                                 float radius,
                                 Eigen::Affine3f& pose_start,
                                 Eigen::Affine3f& pose_end)
{
    if (!start_end || start_end->size() < 2 || !scene || scene->empty() || radius <= 0.f)
        return false;

    const auto& p0 = start_end->points[0];
    const auto& p1 = start_end->points[1];
    const Eigen::Vector3f t0(p0.x, p0.y, p0.z);
    const Eigen::Vector3f t1(p1.x, p1.y, p1.z);
    const Eigen::Vector3f y_dir = t1 - t0;
    if (y_dir.squaredNorm() < 1e-12f)
        return false;

    pcl::KdTreeFLANN<ct::PointXYZRGBN> tree;
    tree.setInputCloud(scene);

    pcl::PointCloud<pcl::PointXYZ>::Ptr local(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::Normal> nrm;
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    // PointXYZ specialization is in the prebuilt pcl_features import lib.
    // PointXYZRGBNormal is not, and linking it fails with LNK2001 on MSVC.
    ne.setRadiusSearch(radius);
    ne.setViewPoint(0.f, 0.f, 0.f);

    std::vector<int> ids;
    std::vector<float> d2;

    auto estimateZ = [&](const ct::PointXYZRGBN& q) -> Eigen::Vector3f {
        ids.clear();
        d2.clear();
        if (tree.radiusSearch(q, radius * 2.f, ids, d2) <= 0)
            return Eigen::Vector3f::UnitZ();

        local->resize(ids.size());
        for (size_t i = 0; i < ids.size(); ++i) {
            const auto& p = scene->points[static_cast<size_t>(ids[i])];
            (*local)[i].x = p.x;
            (*local)[i].y = p.y;
            (*local)[i].z = p.z;
        }

        ne.setInputCloud(local);
        ne.compute(nrm);

        auto accumulate = [&](bool inner_only) {
            Eigen::Vector3f sum(0.f, 0.f, 0.f);
            const float r2 = radius * radius;
            for (size_t i = 0; i < ids.size() && i < nrm.size(); ++i) {
                if (inner_only && d2[i] > r2)
                    continue;
                const pcl::Normal& n = nrm[i];
                if (!std::isfinite(n.normal_x) || !std::isfinite(n.normal_y) ||
                    !std::isfinite(n.normal_z))
                    continue;
                const Eigen::Vector3f v(n.normal_x, n.normal_y, n.normal_z);
                if (v.squaredNorm() < 1e-12f)
                    continue;
                sum += v;
            }
            return sum;
        };

        Eigen::Vector3f sum = accumulate(true);
        if (!std::isfinite(sum.x()) || sum.squaredNorm() < 1e-12f)
            sum = accumulate(false);
        if (!std::isfinite(sum.x()) || sum.squaredNorm() < 1e-12f)
            return Eigen::Vector3f::UnitZ();
        return Eigen::Vector3f(-sum.normalized());
    };

    auto makePose = [](const Eigen::Vector3f& t,
                       const Eigen::Vector3f& z_in,
                       const Eigen::Vector3f& y_in) {
        Eigen::Vector3f z = z_in.normalized();
        Eigen::Vector3f x = y_in.cross(z);
        if (x.squaredNorm() < 1e-12f) {
            x = ((std::fabs(z.z()) < 0.9f) ? Eigen::Vector3f::UnitZ()
                                           : Eigen::Vector3f::UnitX())
                    .cross(z);
        }
        x.normalize();
        Eigen::Vector3f y = z.cross(x).normalized();
        Eigen::Affine3f T = Eigen::Affine3f::Identity();
        T.linear().col(0) = x;
        T.linear().col(1) = y;
        T.linear().col(2) = z;
        T.translation() = t;
        return T;
    };

    pose_start = makePose(t0, estimateZ(p0), y_dir);
    pose_end = makePose(t1, estimateZ(p1), y_dir);
    return true;
}

#endif // COMPUTE_TWO_POINT_POSES_H
