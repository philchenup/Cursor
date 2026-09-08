#include "FitCircle3D.h"

#include <pcl/kdtree/kdtree_flann.h>

#include <Eigen/Dense>

#include <cmath>
#include <random>
#include <vector>

namespace {

void SampleFourIndices(int nums, int idx[4], std::mt19937& rng)
{
    std::uniform_int_distribution<int> dist(0, nums - 1);
    idx[0] = dist(rng);
    do {
        idx[1] = dist(rng);
    } while (idx[1] == idx[0]);
    do {
        idx[2] = dist(rng);
    } while (idx[2] == idx[0] || idx[2] == idx[1]);
    do {
        idx[3] = dist(rng);
    } while (idx[3] == idx[0] || idx[3] == idx[1] || idx[3] == idx[2]);
}

} // namespace

bool fitCircle3D(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                 cv::Point3f& center,
                 float& radius)
{
    const float dist_th = 0.02f;
    const int max_iters = 500;
    const int sample_num = 4;

    int inner = 0;
    float sphere_radius = 0.f;
    cv::Point3f sphere_center(0.f, 0.f, 0.f);

    if (!cloud || cloud->empty()) {
        return false;
    }

    const int nums = static_cast<int>(cloud->size());
    if (sample_num > nums) {
        return false;
    }

    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloud);

    Eigen::Matrix3d A = Eigen::Matrix3d::Zero();
    Eigen::Vector3d b = Eigen::Vector3d::Zero();

    std::mt19937 rng{std::random_device{}()};
    int iters = 0;
    while (iters < max_iters) {
        int idx[4];
        SampleFourIndices(nums, idx, rng);

        const float x[4] = {
            cloud->points[idx[0]].x, cloud->points[idx[1]].x,
            cloud->points[idx[2]].x, cloud->points[idx[3]].x};
        const float y[4] = {
            cloud->points[idx[0]].y, cloud->points[idx[1]].y,
            cloud->points[idx[2]].y, cloud->points[idx[3]].y};
        const float z[4] = {
            cloud->points[idx[0]].z, cloud->points[idx[1]].z,
            cloud->points[idx[2]].z, cloud->points[idx[3]].z};

        A(0, 0) = x[0] - x[1];
        A(0, 1) = y[0] - y[1];
        A(0, 2) = z[0] - z[1];
        A(1, 0) = x[0] - x[2];
        A(1, 1) = y[0] - y[2];
        A(1, 2) = z[0] - z[2];
        A(2, 0) = x[0] - x[3];
        A(2, 1) = y[0] - y[3];
        A(2, 2) = z[0] - z[3];

        b(0) = ((x[0] * x[0] - x[1] * x[1]) +
                (y[0] * y[0] - y[1] * y[1]) +
                (z[0] * z[0] - z[1] * z[1])) / 2.0;
        b(1) = ((x[0] * x[0] - x[2] * x[2]) +
                (y[0] * y[0] - y[2] * y[2]) +
                (z[0] * z[0] - z[2] * z[2])) / 2.0;
        b(2) = ((x[0] * x[0] - x[3] * x[3]) +
                (y[0] * y[0] - y[3] * y[3]) +
                (z[0] * z[0] - z[3] * z[3])) / 2.0;

        const double d = std::abs(A.determinant());
        if (d < 1e-5) {
            ++iters;
            continue;
        }

        const Eigen::Vector3d c = A.inverse() * b;
        const Eigen::Vector3d p0(x[0], y[0], z[0]);
        const double r = (c - p0).norm();
        if (!std::isfinite(r) || r <= 0.0) {
            ++iters;
            continue;
        }

        pcl::PointXYZ query;
        query.x = static_cast<float>(c.x());
        query.y = static_cast<float>(c.y());
        query.z = static_cast<float>(c.z());

        std::vector<int> indice1;
        std::vector<int> indice2;
        std::vector<float> sqr1;
        std::vector<float> sqr2;
        int n1 = 0;
        if (r > dist_th) {
            n1 = kdtree.radiusSearch(query, r - dist_th, indice1, sqr1);
        }
        const int n2 = kdtree.radiusSearch(query, r + dist_th, indice2, sqr2);
        const int total = n2 - n1;

        if (total > inner) {
            inner = total;
            sphere_center = cv::Point3f(static_cast<float>(c.x()),
                                           static_cast<float>(c.y()),
                                           static_cast<float>(c.z()));
            sphere_radius = static_cast<float>(r);
        }

        // 内点数大于总点数的 99.9% 则停止，0.999 即为置信度
        if (inner > 0.999f * static_cast<float>(nums)) {
            break;
        }
        ++iters;
    }

    if (sphere_radius < 1e-5f) {
        return false;
    }

    center = sphere_center;
    radius = sphere_radius;
    return true;
}
