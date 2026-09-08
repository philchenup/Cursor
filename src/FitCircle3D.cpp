#include "FitCircle3D.h"

#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/filters/filter.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

namespace {

bool IsFinitePoint(const pcl::PointXYZ& p)
{
    return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
}

bool IsValidCircle(const cv::Point3f& center, float radius)
{
    return std::isfinite(center.x) && std::isfinite(center.y) && std::isfinite(center.z)
        && std::isfinite(radius) && radius > 0.f;
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

// PCA 求最佳平面，投影到平面坐标系后用代数最小二乘拟合圆：
// x^2 + y^2 + D x + E y + F = 0
bool FitCircle3DAlgebraic(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                          cv::Point3f& center,
                          float& radius)
{
    const std::size_t n = cloud->size();
    if (n < 3) {
        return false;
    }

    Eigen::Vector4f centroid;
    if (pcl::compute3DCentroid(*cloud, centroid) == 0) {
        return false;
    }

    Eigen::Matrix3f covariance;
    pcl::computeCovarianceMatrixNormalized(*cloud, centroid, covariance);

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(covariance);
    if (solver.info() != Eigen::Success) {
        return false;
    }

    const Eigen::Vector3f normal = solver.eigenvectors().col(0);
    Eigen::Vector3f u = solver.eigenvectors().col(2);
    Eigen::Vector3f v = normal.cross(u);
    if (v.norm() < 1e-8f) {
        return false;
    }
    v.normalize();
    u = v.cross(normal);
    u.normalize();

    const Eigen::Vector3f origin = centroid.head<3>();

    Eigen::MatrixXd A(static_cast<Eigen::Index>(n), 3);
    Eigen::VectorXd b(static_cast<Eigen::Index>(n));
    for (std::size_t i = 0; i < n; ++i) {
        const auto& p = cloud->points[i];
        const Eigen::Vector3f rel(p.x - origin.x(), p.y - origin.y(), p.z - origin.z());
        const double x = static_cast<double>(rel.dot(u));
        const double y = static_cast<double>(rel.dot(v));
        A(static_cast<Eigen::Index>(i), 0) = x;
        A(static_cast<Eigen::Index>(i), 1) = y;
        A(static_cast<Eigen::Index>(i), 2) = 1.0;
        b(static_cast<Eigen::Index>(i)) = -(x * x + y * y);
    }

    const Eigen::Vector3d def = A.colPivHouseholderQr().solve(b);
    const double cx = -def(0) * 0.5;
    const double cy = -def(1) * 0.5;
    const double r2 = cx * cx + cy * cy - def(2);
    if (!(r2 > 0.0) || !std::isfinite(r2)) {
        return false;
    }

    const Eigen::Vector3f c3 = origin
        + static_cast<float>(cx) * u
        + static_cast<float>(cy) * v;
    center = cv::Point3f(c3.x(), c3.y(), c3.z());
    radius = static_cast<float>(std::sqrt(r2));
    return IsValidCircle(center, radius);
}

float CloudScale(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
    pcl::PointXYZ min_pt;
    pcl::PointXYZ max_pt;
    pcl::getMinMax3D(*cloud, min_pt, max_pt);
    const float dx = max_pt.x - min_pt.x;
    const float dy = max_pt.y - min_pt.y;
    const float dz = max_pt.z - min_pt.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

bool FitCircle3DRansac(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                       cv::Point3f& center,
                       float& radius)
{
    const float scale = CloudScale(cloud);
    if (!(scale > 1e-8f)) {
        return false;
    }

    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_CIRCLE3D);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(std::max(1e-4f, scale * 0.01f));
    seg.setMaxIterations(1000);
    seg.setRadiusLimits(0.0, static_cast<double>(scale) * 2.0);
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);

    // CIRCLE3D 系数: [cx, cy, cz, radius, nx, ny, nz]
    if (inliers->indices.empty() || coefficients->values.size() < 4) {
        return false;
    }

    if (inliers->indices.size() >= 3) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr inlier_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        inlier_cloud->reserve(inliers->indices.size());
        for (int idx : inliers->indices) {
            inlier_cloud->push_back(cloud->points[static_cast<std::size_t>(idx)]);
        }
        if (FitCircle3DAlgebraic(inlier_cloud, center, radius)) {
            return true;
        }
    }

    center.x = coefficients->values[0];
    center.y = coefficients->values[1];
    center.z = coefficients->values[2];
    radius = coefficients->values[3];
    return IsValidCircle(center, radius);
}

} // namespace

bool fitCircle3D(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                 cv::Point3f& center,
                 float& radius)
{
    if (!cloud || cloud->empty()) {
        return false;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr valid = RemoveInvalidPoints(cloud);
    if (valid->size() < 3) {
        return false;
    }

    if (FitCircle3DRansac(valid, center, radius)) {
        return true;
    }

    return FitCircle3DAlgebraic(valid, center, radius);
}
