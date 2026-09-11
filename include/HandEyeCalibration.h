#ifndef HAND_EYE_CALIBRATION_H
#define HAND_EYE_CALIBRATION_H

#include <Eigen/Geometry>
#include <string>
#include <vector>

using Isometry3dVector =
    std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>>;

struct EyeInHandCalibResult {
    bool success = false;
    std::string message;
    Eigen::Isometry3d T_flange_camera = Eigen::Isometry3d::Identity();  // ^{F}T_{C}
    double rmse = 0.0;
    std::vector<double> per_sample_error;
};

struct EyeOnHandCalibResult {
    bool success = false;
    std::string message;
    Eigen::Isometry3d T_base_camera = Eigen::Isometry3d::Identity();  // ^{B}T_{C}
    double rmse = 0.0;
    std::vector<double> per_sample_error;
};

/// 眼在手上：相机固连法兰。^{B}P = ^{B}T_{F} · ^{F}T_{C} · ^{C}P
EyeInHandCalibResult CalibrateEyeInHand(
    const std::vector<Eigen::Vector3d>& points_in_camera,
    const Isometry3dVector& flanges_in_base,
    const std::vector<Eigen::Vector3d>& tcps_in_base);

/// 眼在手外：相机固定。^{B}P = ^{B}T_{C} · ^{C}P，不需要法兰位姿。
EyeOnHandCalibResult CalibrateEyeOnHand(
    const std::vector<Eigen::Vector3d>& points_in_camera,
    const std::vector<Eigen::Vector3d>& tcps_in_base);

Eigen::Isometry3d FlangePoseFromXyzQuat(
    double x, double y, double z, double qw, double qx, double qy, double qz);

Eigen::Isometry3d FlangePoseFromXyzRpyZYX(
    double x, double y, double z, double roll, double pitch, double yaw);

#endif
