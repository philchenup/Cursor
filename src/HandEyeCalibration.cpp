#include "HandEyeCalibration.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/transformation_estimation_svd.h>

#include <Eigen/Dense>
#include <Eigen/SVD>
#include <cmath>
#include <sstream>

namespace {

constexpr int kMinSamples = 3;
constexpr double kCollinearRatio = 1e-6;

pcl::PointXYZ ToPcl(const Eigen::Vector3d& p)
{
    return pcl::PointXYZ(
        static_cast<float>(p.x()),
        static_cast<float>(p.y()),
        static_cast<float>(p.z()));
}

bool PointsAreCollinear(const Eigen::Matrix3Xd& pts, std::string* why)
{
    const Eigen::Vector3d mean = pts.rowwise().mean();
    const Eigen::Matrix3Xd centered = pts.colwise() - mean;
    Eigen::JacobiSVD<Eigen::Matrix3Xd> svd(centered, Eigen::ComputeThinU | Eigen::ComputeThinV);
    const auto& sv = svd.singularValues();
    const double s0 = sv.size() > 0 ? sv(0) : 0.0;
    const double s1 = sv.size() > 1 ? sv(1) : 0.0;
    if (s0 <= 0.0 || s1 / s0 < kCollinearRatio) {
        if (why) {
            *why = "correspondences are collinear; move the flange through "
                   "non-coplanar orientations or use non-collinear TCP points";
        }
        return true;
    }
    return false;
}

/**
 * Kabsch–Umeyama：求 R, t 使 q_i ≈ R p_i + t。
 * src 的列是 p_i（相机系），dst 的列是 q_i（法兰系）。
 */
Eigen::Isometry3d EstimateRigidKabsch(
    const Eigen::Matrix3Xd& src,
    const Eigen::Matrix3Xd& dst)
{
    const Eigen::Vector3d p_mean = src.rowwise().mean();
    const Eigen::Vector3d q_mean = dst.rowwise().mean();
    const Eigen::Matrix3Xd p = src.colwise() - p_mean;
    const Eigen::Matrix3Xd q = dst.colwise() - q_mean;

    const Eigen::Matrix3d H = p * q.transpose();
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
    const Eigen::Matrix3d U = svd.matrixU();
    const Eigen::Matrix3d V = svd.matrixV();

    Eigen::Matrix3d S = Eigen::Matrix3d::Identity();
    S(2, 2) = (V * U.transpose()).determinant();
    const Eigen::Matrix3d R = V * S * U.transpose();

    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.linear() = R;
    T.translation() = q_mean - R * p_mean;
    return T;
}

EyeInHandCalibResult Fail(const std::string& message)
{
    EyeInHandCalibResult result;
    result.success = false;
    result.message = message;
    return result;
}

} // namespace

Eigen::Isometry3d FlangePoseFromXyzQuat(
    double x, double y, double z,
    double qw, double qx, double qy, double qz)
{
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation() = Eigen::Vector3d(x, y, z);
    pose.linear() = Eigen::Quaterniond(qw, qx, qy, qz).normalized().toRotationMatrix();
    return pose;
}

Eigen::Isometry3d FlangePoseFromXyzRpyZYX(
    double x, double y, double z,
    double roll, double pitch, double yaw)
{
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation() = Eigen::Vector3d(x, y, z);
    pose.linear() =
        (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())
         * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
         * Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()))
            .toRotationMatrix();
    return pose;
}

EyeInHandCalibResult CalibrateEyeInHand(
    const std::vector<Eigen::Vector3d>& points_in_camera,
    const Isometry3dVector& flanges_in_base,
    const std::vector<Eigen::Vector3d>& tcps_in_base)
{
    const std::size_t n = points_in_camera.size();
    if (n != flanges_in_base.size() || n != tcps_in_base.size()) {
        return Fail("camera points, flange poses and TCP points must have the same length");
    }
    if (n < static_cast<std::size_t>(kMinSamples)) {
        return Fail("eye-in-hand point calibration needs at least 3 samples");
    }

    Eigen::Matrix3Xd src(3, static_cast<int>(n));
    Eigen::Matrix3Xd dst(3, static_cast<int>(n));
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_camera(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_flange(new pcl::PointCloud<pcl::PointXYZ>);
    cloud_camera->resize(n);
    cloud_flange->resize(n);

    for (std::size_t i = 0; i < n; ++i) {
        const Eigen::Vector3d p_cam = points_in_camera[i];
        const Eigen::Vector3d p_flange = flanges_in_base[i].inverse() * tcps_in_base[i];
        if (!p_cam.allFinite() || !p_flange.allFinite()) {
            return Fail("sample " + std::to_string(i) + " contains non-finite coordinates");
        }
        src.col(static_cast<int>(i)) = p_cam;
        dst.col(static_cast<int>(i)) = p_flange;
        (*cloud_camera)[i] = ToPcl(p_cam);
        (*cloud_flange)[i] = ToPcl(p_flange);
    }

    std::string degenerate;
    if (PointsAreCollinear(src, &degenerate) || PointsAreCollinear(dst, &degenerate)) {
        return Fail(degenerate);
    }

    // 双精度闭式解（与 PCL TransformationEstimationSVD / Eigen::umeyama 同一算法）。
    const Eigen::Isometry3d T_flange_camera = EstimateRigidKabsch(src, dst);

    // PCL 路径：同一组对应点再估一次，算法等价，PointXYZ 为 float。
    pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ, double>
        estimator(/*use_umeyama=*/true);
    Eigen::Matrix4d T_pcl = Eigen::Matrix4d::Identity();
    estimator.estimateRigidTransformation(*cloud_camera, *cloud_flange, T_pcl);
    const double pcl_trans_delta =
        (T_pcl.block<3, 1>(0, 3) - T_flange_camera.translation()).norm();

    if (!T_flange_camera.matrix().allFinite()) {
        return Fail("SVD produced a non-finite transform");
    }
    if (T_flange_camera.linear().determinant() < 0.0) {
        return Fail("estimated rotation has negative determinant (reflection)");
    }

    EyeInHandCalibResult result;
    result.success = true;
    result.T_flange_camera = T_flange_camera;
    result.per_sample_error.resize(n);

    double sum_sq = 0.0;
    for (std::size_t i = 0; i < n; ++i) {
        const Eigen::Vector3d predicted =
            flanges_in_base[i] * (T_flange_camera * points_in_camera[i]);
        const double err = (predicted - tcps_in_base[i]).norm();
        result.per_sample_error[i] = err;
        sum_sq += err * err;
    }
    result.rmse = std::sqrt(sum_sq / static_cast<double>(n));

    std::ostringstream oss;
    oss << "calibrated ^{F}T_{C} from " << n << " samples, RMSE = " << result.rmse
        << ", PCL-vs-Eigen translation delta = " << pcl_trans_delta;
    result.message = oss.str();
    return result;
}
