#include "FitCircle3D.h"

#include <pcl/filters/filter.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <Eigen/Dense>

#include <cmath>
#include <cstddef>
#include <random>
#include <vector>

namespace {

constexpr int kSampleNum = 3;       // 空间圆最少 3 个点
constexpr int kMaxIters = 500;
constexpr float kDistTh = 0.02f;    // 距离阈值
constexpr float kConfidence = 0.999f;

bool IsFinitePoint(const pcl::PointXYZ& p)
{
    return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
}

Eigen::Vector3d ToVec(const pcl::PointXYZ& p)
{
    return Eigen::Vector3d(p.x, p.y, p.z);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr RemoveInvalidPoints(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<int> index;
    pcl::removeNaNFromPointCloud(*cloud, *filtered, index);

    pcl::PointCloud<pcl::PointXYZ>::Ptr valid(new pcl::PointCloud<pcl::PointXYZ>);
    valid->reserve(filtered->size());
    for (const auto& p : filtered->points) {
        if (IsFinitePoint(p)) {
            valid->push_back(p);
        }
    }
    return valid;
}

void SampleDistinctIndices(int nums, int sample[3], std::mt19937& rng)
{
    std::uniform_int_distribution<int> dist(0, nums - 1);
    sample[0] = dist(rng);
    do {
        sample[1] = dist(rng);
    } while (sample[1] == sample[0]);
    do {
        sample[2] = dist(rng);
    } while (sample[2] == sample[0] || sample[2] == sample[1]);
}

// 三点确定唯一空间圆：圆心在平面上且到三点等距。
// 线性方程与球体 RANSAC 相同，第三行改为平面约束 n·c = n·p0。
bool FitCircleFromThreePoints(const Eigen::Vector3d& p0,
                              const Eigen::Vector3d& p1,
                              const Eigen::Vector3d& p2,
                              Eigen::Vector3d& center,
                              Eigen::Vector3d& normal,
                              double& radius)
{
    normal = (p1 - p0).cross(p2 - p0);
    const double n_norm = normal.norm();
    if (n_norm < 1e-8) {
        return false;
    }
    normal /= n_norm;

    Eigen::Matrix3d A = Eigen::Matrix3d::Zero();
    Eigen::Vector3d b = Eigen::Vector3d::Zero();

    A(0, 0) = p0.x() - p1.x();
    A(0, 1) = p0.y() - p1.y();
    A(0, 2) = p0.z() - p1.z();
    A(1, 0) = p0.x() - p2.x();
    A(1, 1) = p0.y() - p2.y();
    A(1, 2) = p0.z() - p2.z();
    A(2, 0) = normal.x();
    A(2, 1) = normal.y();
    A(2, 2) = normal.z();

    b(0) = ((p0.x() * p0.x() - p1.x() * p1.x()) +
            (p0.y() * p0.y() - p1.y() * p1.y()) +
            (p0.z() * p0.z() - p1.z() * p1.z())) / 2.0;
    b(1) = ((p0.x() * p0.x() - p2.x() * p2.x()) +
            (p0.y() * p0.y() - p2.y() * p2.y()) +
            (p0.z() * p0.z() - p2.z() * p2.z())) / 2.0;
    b(2) = normal.dot(p0);

    const double d = std::abs(A.determinant());
    if (d < 1e-5) {
        return false;
    }

    center = A.inverse() * b;
    radius = (center - p0).norm();
    return std::isfinite(radius) && radius > 1e-5;
}

int CountCircleInliers(const pcl::KdTreeFLANN<pcl::PointXYZ>& kdtree,
                        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                        const Eigen::Vector3d& center,
                        const Eigen::Vector3d& normal,
                        double radius,
                        float dist_th)
{
    pcl::PointXYZ query;
    query.x = static_cast<float>(center.x());
    query.y = static_cast<float>(center.y());
    query.z = static_cast<float>(center.z());

    std::vector<int> indice_outer;
    std::vector<float> sqr_dist;
    const int found = kdtree.radiusSearch(
        query, radius + static_cast<double>(dist_th), indice_outer, sqr_dist);
    if (found <= 0) {
        return 0;
    }

    const double r_min = radius - static_cast<double>(dist_th);
    const double r_max = radius + static_cast<double>(dist_th);
    int total = 0;
    for (int idx : indice_outer) {
        const Eigen::Vector3d p = ToVec(cloud->points[static_cast<std::size_t>(idx)]);
        const Eigen::Vector3d delta = p - center;
        // 须贴近圆所在平面，否则会把同心球壳上的点算作内点
        if (std::abs(delta.dot(normal)) > static_cast<double>(dist_th)) {
            continue;
        }
        const double radial = delta.norm();
        if (radial >= r_min && radial <= r_max) {
            ++total;
        }
    }
    return total;
}

} // namespace

bool fitCircle3D(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                 cv::Point3f& center,
                 float& radius)
{
    if (!cloud || cloud->empty()) {
        return false;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr points = RemoveInvalidPoints(cloud);
    const int nums = static_cast<int>(points->size());
    if (kSampleNum > nums) {
        return false;
    }

    int inner = 0;
    float circle_radius = 0.f;
    cv::Point3f circle_center(0.f, 0.f, 0.f);

    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(points);

    std::mt19937 rng{std::random_device{}()};
    int iters = 0;
    while (iters < kMaxIters) {
        int idx[3];
        SampleDistinctIndices(nums, idx, rng);
        const Eigen::Vector3d p0 = ToVec(points->points[static_cast<std::size_t>(idx[0])]);
        const Eigen::Vector3d p1 = ToVec(points->points[static_cast<std::size_t>(idx[1])]);
        const Eigen::Vector3d p2 = ToVec(points->points[static_cast<std::size_t>(idx[2])]);

        Eigen::Vector3d c;
        Eigen::Vector3d normal;
        double r = 0.0;
        if (!FitCircleFromThreePoints(p0, p1, p2, c, normal, r)) {
            continue;
        }

        const int total = CountCircleInliers(kdtree, points, c, normal, r, kDistTh);
        if (total > inner) {
            inner = total;
            circle_center = cv::Point3f(static_cast<float>(c.x()),
                                         static_cast<float>(c.y()),
                                         static_cast<float>(c.z()));
            circle_radius = static_cast<float>(r);
        }

        // 内点数大于总点数的 99.9% 则停止，0.999 即为置信度
        if (inner > kConfidence * static_cast<float>(nums)) {
            break;
        }
        ++iters;
    }

    if (circle_radius < 1e-5f) {
        return false;
    }

    center = circle_center;
    radius = circle_radius;
    return true;
}
