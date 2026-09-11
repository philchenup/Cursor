#include "HandEyeCalibration.h"

#include <Eigen/SVD>
#include <cmath>
#include <sstream>

namespace {

constexpr int kMinSamples = 3;

bool Collinear(const Eigen::Matrix3Xd& pts)
{
    const Eigen::Matrix3Xd c = pts.colwise() - pts.rowwise().mean();
    Eigen::JacobiSVD<Eigen::Matrix3Xd> svd(c, Eigen::ComputeThinU);
    return svd.singularValues().size() < 2 ||
           svd.singularValues()(1) < 1e-6 * svd.singularValues()(0);
}

// q ≈ R p + t  （src 列向量 p，dst 列向量 q）
bool EstimateRigid(const Eigen::Matrix3Xd& src, const Eigen::Matrix3Xd& dst,
                   Eigen::Isometry3d& T, std::string& err)
{
    if (src.cols() < kMinSamples) {
        err = "need at least 3 samples";
        return false;
    }
    if (Collinear(src) || Collinear(dst)) {
        err = "points are collinear";
        return false;
    }

    const Eigen::Vector3d p = src.rowwise().mean();
    const Eigen::Vector3d q = dst.rowwise().mean();
    const Eigen::Matrix3d H = (src.colwise() - p) * (dst.colwise() - q).transpose();
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d S = Eigen::Matrix3d::Identity();
    S(2, 2) = (svd.matrixV() * svd.matrixU().transpose()).determinant();
    const Eigen::Matrix3d R = svd.matrixV() * S * svd.matrixU().transpose();

    T = Eigen::Isometry3d::Identity();
    T.linear() = R;
    T.translation() = q - R * p;
    if (!T.matrix().allFinite() || T.linear().determinant() < 0.0) {
        err = "invalid rigid transform";
        return false;
    }
    return true;
}

double FillErrors(const std::vector<Eigen::Vector3d>& pred,
                  const std::vector<Eigen::Vector3d>& gt,
                  std::vector<double>& errors)
{
    errors.resize(pred.size());
    double ss = 0.0;
    for (std::size_t i = 0; i < pred.size(); ++i) {
        errors[i] = (pred[i] - gt[i]).norm();
        ss += errors[i] * errors[i];
    }
    return std::sqrt(ss / static_cast<double>(pred.size()));
}

} // namespace

Eigen::Isometry3d FlangePoseFromXyzQuat(
    double x, double y, double z, double qw, double qx, double qy, double qz)
{
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.translation() = Eigen::Vector3d(x, y, z);
    T.linear() = Eigen::Quaterniond(qw, qx, qy, qz).normalized().toRotationMatrix();
    return T;
}

Eigen::Isometry3d FlangePoseFromXyzRpyZYX(
    double x, double y, double z, double roll, double pitch, double yaw)
{
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.translation() = Eigen::Vector3d(x, y, z);
    T.linear() = (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                  Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                  Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()))
                     .toRotationMatrix();
    return T;
}

EyeInHandCalibResult CalibrateEyeInHand(
    const std::vector<Eigen::Vector3d>& points_in_camera,
    const Isometry3dVector& flanges_in_base,
    const std::vector<Eigen::Vector3d>& tcps_in_base)
{
    EyeInHandCalibResult out;
    const std::size_t n = points_in_camera.size();
    if (n != flanges_in_base.size() || n != tcps_in_base.size()) {
        out.message = "size mismatch";
        return out;
    }

    Eigen::Matrix3Xd src(3, static_cast<int>(n)), dst(3, static_cast<int>(n));
    for (std::size_t i = 0; i < n; ++i) {
        src.col(static_cast<int>(i)) = points_in_camera[i];
        dst.col(static_cast<int>(i)) = flanges_in_base[i].inverse() * tcps_in_base[i];
    }

    if (!EstimateRigid(src, dst, out.T_flange_camera, out.message)) {
        return out;
    }

    std::vector<Eigen::Vector3d> pred(n);
    for (std::size_t i = 0; i < n; ++i) {
        pred[i] = flanges_in_base[i] * (out.T_flange_camera * points_in_camera[i]);
    }
    out.rmse = FillErrors(pred, tcps_in_base, out.per_sample_error);
    out.success = true;
    std::ostringstream oss;
    oss << "eye-in-hand ^{F}T_{C}, N=" << n << ", RMSE=" << out.rmse;
    out.message = oss.str();
    return out;
}

EyeOnHandCalibResult CalibrateEyeOnHand(
    const std::vector<Eigen::Vector3d>& points_in_camera,
    const std::vector<Eigen::Vector3d>& tcps_in_base)
{
    EyeOnHandCalibResult out;
    const std::size_t n = points_in_camera.size();
    if (n != tcps_in_base.size()) {
        out.message = "size mismatch";
        return out;
    }

    Eigen::Matrix3Xd src(3, static_cast<int>(n)), dst(3, static_cast<int>(n));
    for (std::size_t i = 0; i < n; ++i) {
        src.col(static_cast<int>(i)) = points_in_camera[i];
        dst.col(static_cast<int>(i)) = tcps_in_base[i];
    }

    if (!EstimateRigid(src, dst, out.T_base_camera, out.message)) {
        return out;
    }

    std::vector<Eigen::Vector3d> pred(n);
    for (std::size_t i = 0; i < n; ++i) {
        pred[i] = out.T_base_camera * points_in_camera[i];
    }
    out.rmse = FillErrors(pred, tcps_in_base, out.per_sample_error);
    out.success = true;
    std::ostringstream oss;
    oss << "eye-on-hand ^{B}T_{C}, N=" << n << ", RMSE=" << out.rmse;
    out.message = oss.str();
    return out;
}
