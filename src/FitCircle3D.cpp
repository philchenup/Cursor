#include "FitCircle3D.h"

#include <pcl/common/centroid.h>
#include <pcl/filters/filter.h>

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

// 对应 Python spherrors：平面圆代数残差 (x-a)^2 + (y-b)^2 - r^2
Eigen::VectorXd CircleErrors(const Eigen::Vector3d& para,
                             const std::vector<Eigen::Vector2d>& pts)
{
    const double a = para(0);
    const double b = para(1);
    const double r = para(2);
    Eigen::VectorXd f(static_cast<Eigen::Index>(pts.size()));
    for (std::size_t i = 0; i < pts.size(); ++i) {
        const double dx = pts[i].x() - a;
        const double dy = pts[i].y() - b;
        f(static_cast<Eigen::Index>(i)) = dx * dx + dy * dy - r * r;
    }
    return f;
}

Eigen::MatrixXd CircleJacobian(const Eigen::Vector3d& para,
                              const std::vector<Eigen::Vector2d>& pts)
{
    const double a = para(0);
    const double b = para(1);
    const double r = para(2);
    Eigen::MatrixXd J(static_cast<Eigen::Index>(pts.size()), 3);
    for (std::size_t i = 0; i < pts.size(); ++i) {
        J(static_cast<Eigen::Index>(i), 0) = -2.0 * (pts[i].x() - a);
        J(static_cast<Eigen::Index>(i), 1) = -2.0 * (pts[i].y() - b);
        J(static_cast<Eigen::Index>(i), 2) = -2.0 * r;
    }
    return J;
}

// 对应 Python opt.leastsq：Levenberg-Marquardt 最小化残差平方和
bool LeastsqCircle(const std::vector<Eigen::Vector2d>& pts, Eigen::Vector3d& para)
{
    if (pts.size() < 3) {
        return false;
    }

    double lambda = 1e-3;
    for (int iter = 0; iter < 50; ++iter) {
        const Eigen::VectorXd f = CircleErrors(para, pts);
        const double cost = f.squaredNorm();
        if (!std::isfinite(cost)) {
            return false;
        }

        const Eigen::MatrixXd J = CircleJacobian(para, pts);
        Eigen::Matrix3d A = J.transpose() * J;
        const Eigen::Vector3d g = J.transpose() * f;
        A.diagonal().array() += lambda;

        const Eigen::Vector3d delta = A.ldlt().solve(-g);
        if (!delta.allFinite()) {
            return false;
        }

        Eigen::Vector3d trial = para + delta;
        const double trial_cost = CircleErrors(trial, pts).squaredNorm();
        if (std::isfinite(trial_cost) && trial_cost < cost) {
            para = trial;
            lambda = std::max(lambda * 0.1, 1e-12);
            if (delta.norm() < 1e-10) {
                break;
            }
        } else {
            lambda = std::min(lambda * 10.0, 1e12);
        }
    }

    para(2) = std::abs(para(2));
    return para.allFinite() && para(2) > 1e-8;
}

bool InitCirclePara(const std::vector<Eigen::Vector2d>& pts, Eigen::Vector3d& para)
{
    const Eigen::Index n = static_cast<Eigen::Index>(pts.size());
    Eigen::MatrixXd A(n, 3);
    Eigen::VectorXd b(n);
    for (Eigen::Index i = 0; i < n; ++i) {
        const double x = pts[static_cast<std::size_t>(i)].x();
        const double y = pts[static_cast<std::size_t>(i)].y();
        A(i, 0) = x;
        A(i, 1) = y;
        A(i, 2) = 1.0;
        b(i) = -(x * x + y * y);
    }

    const Eigen::Vector3d def = A.colPivHouseholderQr().solve(b);
    const double cx = -def(0) * 0.5;
    const double cy = -def(1) * 0.5;
    const double r2 = cx * cx + cy * cy - def(2);
    if (r2 > 0.0 && std::isfinite(r2)) {
        para = Eigen::Vector3d(cx, cy, std::sqrt(r2));
        return true;
    }

    // 对应 Python 初值 [1, 1, 1, 1]：平面内用质心 + 平均半径
    Eigen::Vector2d mean = Eigen::Vector2d::Zero();
    for (const auto& p : pts) {
        mean += p;
    }
    mean /= static_cast<double>(pts.size());
    double r_mean = 0.0;
    for (const auto& p : pts) {
        r_mean += (p - mean).norm();
    }
    r_mean /= static_cast<double>(pts.size());
    para = Eigen::Vector3d(mean.x(), mean.y(), std::max(r_mean, 1.0));
    return true;
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
    const std::size_t n = points->size();
    if (n < 3) {
        return false;
    }

    Eigen::Vector4f centroid;
    if (pcl::compute3DCentroid(*points, centroid) == 0) {
        return false;
    }

    Eigen::Matrix3f covariance;
    pcl::computeCovarianceMatrixNormalized(*points, centroid, covariance);
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
    std::vector<Eigen::Vector2d> pts2d;
    pts2d.reserve(n);
    for (const auto& p : points->points) {
        const Eigen::Vector3f rel(p.x - origin.x(), p.y - origin.y(), p.z - origin.z());
        pts2d.emplace_back(static_cast<double>(rel.dot(u)),
                           static_cast<double>(rel.dot(v)));
    }

    Eigen::Vector3d para;
    if (!InitCirclePara(pts2d, para)) {
        return false;
    }
    if (!LeastsqCircle(pts2d, para)) {
        return false;
    }

    const Eigen::Vector3f c3 = origin
        + static_cast<float>(para(0)) * u
        + static_cast<float>(para(1)) * v;
    center = cv::Point3f(c3.x(), c3.y(), c3.z());
    radius = static_cast<float>(para(2));
    return std::isfinite(center.x) && std::isfinite(center.y) && std::isfinite(center.z)
        && std::isfinite(radius) && radius > 0.f;
}
