#include "HandEyeCalibration.h"

#include <Eigen/Geometry>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <random>
#include <vector>

namespace {

Eigen::Isometry3d MakePose(const Eigen::Vector3d& t, const Eigen::Vector3d& rpy)
{
    return FlangePoseFromXyzRpyZYX(t.x(), t.y(), t.z(), rpy.x(), rpy.y(), rpy.z());
}

double RotationGeodesicDeg(const Eigen::Matrix3d& a, const Eigen::Matrix3d& b)
{
    const Eigen::Matrix3d d = a.transpose() * b;
    const double c = std::max(-1.0, std::min(1.0, 0.5 * (d.trace() - 1.0)));
    return std::acos(c) * 180.0 / 3.14159265358979323846;
}

bool RunSyntheticTrial(const char* name, int n, unsigned seed, bool noisy)
{
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> trans(-200.0, 200.0);
    std::uniform_real_distribution<double> ang(-0.8, 0.8);
    std::uniform_real_distribution<double> tcp_xy(300.0, 700.0);
    std::uniform_real_distribution<double> tcp_z(50.0, 250.0);
    std::normal_distribution<double> noise(0.0, noisy ? 0.05 : 0.0);

    const Eigen::Isometry3d T_gt = MakePose(
        Eigen::Vector3d(42.0, -18.5, 95.0),
        Eigen::Vector3d(0.12, -0.35, 1.10));

    std::vector<Eigen::Vector3d> points_in_camera;
    Isometry3dVector flanges_in_base;
    std::vector<Eigen::Vector3d> tcps_in_base;
    points_in_camera.reserve(static_cast<std::size_t>(n));
    flanges_in_base.reserve(static_cast<std::size_t>(n));
    tcps_in_base.reserve(static_cast<std::size_t>(n));

    for (int i = 0; i < n; ++i) {
        const Eigen::Vector3d tcp(tcp_xy(rng), tcp_xy(rng), tcp_z(rng));
        const Eigen::Isometry3d flange = MakePose(
            Eigen::Vector3d(trans(rng), trans(rng), 400.0 + 0.2 * trans(rng)),
            Eigen::Vector3d(ang(rng), ang(rng), ang(rng)));
        Eigen::Vector3d p_cam = T_gt.inverse() * (flange.inverse() * tcp);
        if (noisy) {
            p_cam += Eigen::Vector3d(noise(rng), noise(rng), noise(rng));
        }
        points_in_camera.push_back(p_cam);
        flanges_in_base.push_back(flange);
        tcps_in_base.push_back(tcp);
    }

    const EyeInHandCalibResult result =
        CalibrateEyeInHand(points_in_camera, flanges_in_base, tcps_in_base);

    const double rot_err = RotationGeodesicDeg(result.T_flange_camera.linear(), T_gt.linear());
    const double t_err = (result.T_flange_camera.translation() - T_gt.translation()).norm();

    std::cout << "\n=== " << name << " ===\n";
    std::cout << std::fixed << std::setprecision(6);
    std::cout << result.message << "\n";
    std::cout << "T_flange_camera =\n" << result.T_flange_camera.matrix() << "\n";
    std::cout << "rotation error (deg) = " << rot_err << "\n";
    std::cout << "translation error    = " << t_err << "\n";

    if (!result.success) {
        std::cerr << "FAIL: " << result.message << "\n";
        return false;
    }

    const double rot_tol = noisy ? 0.05 : 1e-5;
    const double t_tol = noisy ? 0.2 : 1e-6;
    const double rmse_tol = noisy ? 0.2 : 1e-6;
    if (rot_err > rot_tol || t_err > t_tol || result.rmse > rmse_tol) {
        std::cerr << "FAIL: recovered X is too far from ground truth\n";
        return false;
    }
    std::cout << "PASS\n";
    return true;
}

bool RunTooFewSamples()
{
    std::vector<Eigen::Vector3d> pts(2, Eigen::Vector3d::Zero());
    Isometry3dVector poses(2, Eigen::Isometry3d::Identity());
    std::vector<Eigen::Vector3d> tcps(2, Eigen::Vector3d::UnitX());
    const EyeInHandCalibResult result = CalibrateEyeInHand(pts, poses, tcps);
    std::cout << "\n=== too few samples ===\n" << result.message << "\n";
    if (result.success) {
        std::cerr << "FAIL: expected rejection of N < 3\n";
        return false;
    }
    std::cout << "PASS\n";
    return true;
}

bool RunSingleFixedTcp()
{
    const Eigen::Isometry3d T_gt = MakePose(
        Eigen::Vector3d(-12.0, 30.0, 80.0),
        Eigen::Vector3d(0.4, -0.2, 0.7));
    const Eigen::Vector3d tcp(500.0, 120.0, 80.0);

    const Eigen::Isometry3d poses_src[] = {
        MakePose(Eigen::Vector3d(100, 0, 400), Eigen::Vector3d(0.1, 0.2, 0.3)),
        MakePose(Eigen::Vector3d(80, 40, 420), Eigen::Vector3d(-0.4, 0.15, 0.6)),
        MakePose(Eigen::Vector3d(60, -30, 380), Eigen::Vector3d(0.35, -0.5, -0.2)),
        MakePose(Eigen::Vector3d(120, 20, 450), Eigen::Vector3d(-0.2, 0.45, -0.4)),
    };

    std::vector<Eigen::Vector3d> points_in_camera;
    Isometry3dVector flanges_in_base;
    std::vector<Eigen::Vector3d> tcps_in_base;
    for (const Eigen::Isometry3d& flange : poses_src) {
        points_in_camera.push_back(T_gt.inverse() * (flange.inverse() * tcp));
        flanges_in_base.push_back(flange);
        tcps_in_base.push_back(tcp);
    }

    const EyeInHandCalibResult result =
        CalibrateEyeInHand(points_in_camera, flanges_in_base, tcps_in_base);
    const double rot_err = RotationGeodesicDeg(result.T_flange_camera.linear(), T_gt.linear());
    const double t_err = (result.T_flange_camera.translation() - T_gt.translation()).norm();

    std::cout << "\n=== single fixed TCP, 4 flange poses ===\n";
    std::cout << result.message << "\n";
    std::cout << "rotation error (deg) = " << rot_err << "\n";
    std::cout << "translation error    = " << t_err << "\n";
    if (!result.success || rot_err > 1e-5 || t_err > 1e-6) {
        std::cerr << "FAIL: single-TCP multi-pose case\n";
        return false;
    }
    std::cout << "PASS\n";
    return true;
}

} // namespace

int main()
{
    bool ok = true;
    ok = RunTooFewSamples() && ok;
    ok = RunSingleFixedTcp() && ok;
    ok = RunSyntheticTrial("3 samples, noiseless", 3, 1, false) && ok;
    ok = RunSyntheticTrial("8 samples, noiseless", 8, 2, false) && ok;
    ok = RunSyntheticTrial("8 samples, 0.05 noise", 8, 3, true) && ok;
    if (!ok) {
        std::cerr << "\nHand-eye calibration tests failed.\n";
        return 1;
    }
    std::cout << "\nAll hand-eye calibration tests passed.\n";
    return 0;
}
