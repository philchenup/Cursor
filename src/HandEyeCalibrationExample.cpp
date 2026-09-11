#include "HandEyeCalibration.h"

#include <Eigen/Geometry>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#ifndef HAND_EYE_DATA_DIR
#define HAND_EYE_DATA_DIR "data"
#endif

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg2Rad = kPi / 180.0;
constexpr double kRad2Deg = 180.0 / kPi;

std::string Trim(const std::string& s)
{
    const auto begin = s.find_first_not_of(" \t\r\n");
    if (begin == std::string::npos) {
        return {};
    }
    const auto end = s.find_last_not_of(" \t\r\n");
    return s.substr(begin, end - begin + 1);
}

std::vector<double> SplitCsvDoubles(const std::string& line)
{
    std::vector<double> values;
    std::stringstream ss(line);
    std::string token;
    while (std::getline(ss, token, ',')) {
        const std::string item = Trim(token);
        if (item.empty()) {
            continue;
        }
        values.push_back(std::stod(item));
    }
    return values;
}

std::vector<std::vector<double>> LoadCsvRows(const std::string& path, std::size_t expected_cols)
{
    std::ifstream in(path);
    if (!in) {
        throw std::runtime_error("failed to open " + path);
    }

    std::vector<std::vector<double>> rows;
    std::string line;
    std::size_t line_no = 0;
    while (std::getline(in, line)) {
        ++line_no;
        const std::string trimmed = Trim(line);
        if (trimmed.empty()) {
            continue;
        }
        const std::vector<double> values = SplitCsvDoubles(trimmed);
        if (values.size() != expected_cols) {
            throw std::runtime_error(
                path + " line " + std::to_string(line_no) + " has " +
                std::to_string(values.size()) + " values, expected " +
                std::to_string(expected_cols));
        }
        rows.push_back(values);
    }
    if (rows.empty()) {
        throw std::runtime_error(path + " contains no data");
    }
    return rows;
}

Eigen::Vector3d RotationMatrixToRpyZYXDeg(const Eigen::Matrix3d& R)
{
    // ZYX: R = Rz(yaw) * Ry(pitch) * Rx(roll)
    const double pitch = std::asin(std::max(-1.0, std::min(1.0, -R(2, 0))));
    const double cp = std::cos(pitch);
    double roll = 0.0;
    double yaw = 0.0;
    if (std::abs(cp) > 1e-8) {
        roll = std::atan2(R(2, 1), R(2, 2));
        yaw = std::atan2(R(1, 0), R(0, 0));
    } else {
        roll = 0.0;
        yaw = std::atan2(-R(0, 1), R(1, 1));
    }
    return Eigen::Vector3d(roll * kRad2Deg, pitch * kRad2Deg, yaw * kRad2Deg);
}

void PrintMatrix(const Eigen::Matrix4d& T)
{
    std::cout << std::fixed << std::setprecision(9);
    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) {
            std::cout << std::setw(16) << T(r, c);
        }
        std::cout << "\n";
    }
}

} // namespace

int main(int argc, char** argv)
{
    const std::string data_dir = HAND_EYE_DATA_DIR;
    const std::string camera_path = argc > 1 ? argv[1] : data_dir + "/camera_point.txt";
    const std::string flange_path = argc > 2 ? argv[2] : data_dir + "/robot_flange_pose.txt";
    const std::string tcp_path = argc > 3 ? argv[3] : data_dir + "/tcp_pose.txt";

    try {
        const auto camera_rows = LoadCsvRows(camera_path, 3);
        const auto flange_rows = LoadCsvRows(flange_path, 6);
        const auto tcp_rows = LoadCsvRows(tcp_path, 3);

        if (camera_rows.size() != flange_rows.size() || camera_rows.size() != tcp_rows.size()) {
            std::cerr << "sample counts differ: camera=" << camera_rows.size()
                      << " flange=" << flange_rows.size()
                      << " tcp=" << tcp_rows.size() << "\n";
            return 1;
        }

        std::vector<Eigen::Vector3d> points_in_camera;
        Isometry3dVector flanges_in_base;
        std::vector<Eigen::Vector3d> tcps_in_base;
        points_in_camera.reserve(camera_rows.size());
        flanges_in_base.reserve(flange_rows.size());
        tcps_in_base.reserve(tcp_rows.size());

        std::cout << "Loaded " << camera_rows.size() << " samples\n";
        std::cout << "camera: " << camera_path << "\n";
        std::cout << "flange: " << flange_path << "  (xyz mm + ZYX Euler deg -> rad)\n";
        std::cout << "tcp:    " << tcp_path << "\n\n";

        for (std::size_t i = 0; i < camera_rows.size(); ++i) {
            points_in_camera.emplace_back(camera_rows[i][0], camera_rows[i][1], camera_rows[i][2]);
            tcps_in_base.emplace_back(tcp_rows[i][0], tcp_rows[i][1], tcp_rows[i][2]);
            flanges_in_base.push_back(FlangePoseFromXyzRpyZYX(
                flange_rows[i][0], flange_rows[i][1], flange_rows[i][2],
                flange_rows[i][3] * kDeg2Rad,
                flange_rows[i][4] * kDeg2Rad,
                flange_rows[i][5] * kDeg2Rad));
        }

        const EyeInHandCalibResult result =
            CalibrateEyeInHand(points_in_camera, flanges_in_base, tcps_in_base);

        std::cout << result.message << "\n";
        if (!result.success) {
            std::cerr << "calibration failed\n";
            return 1;
        }

        const Eigen::Vector3d t = result.T_flange_camera.translation();
        const Eigen::Vector3d rpy = RotationMatrixToRpyZYXDeg(result.T_flange_camera.linear());

        std::cout << "\nT_flange_camera (^{F}T_{C}) =\n";
        PrintMatrix(result.T_flange_camera.matrix());

        std::cout << std::fixed << std::setprecision(6);
        std::cout << "\ntranslation xyz = " << t.x() << ", " << t.y() << ", " << t.z() << "\n";
        std::cout << "rotation rpy ZYX deg = " << rpy.x() << ", " << rpy.y() << ", " << rpy.z() << "\n";
        std::cout << "RMSE = " << result.rmse << "\n";
        std::cout << "per-sample error:\n";
        for (std::size_t i = 0; i < result.per_sample_error.size(); ++i) {
            std::cout << "  [" << i << "] " << result.per_sample_error[i] << "\n";
        }
        return 0;
    } catch (const std::exception& ex) {
        std::cerr << ex.what() << "\n";
        return 1;
    }
}
