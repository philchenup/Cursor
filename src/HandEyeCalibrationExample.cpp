#include "HandEyeCalibration.h"

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

constexpr double kDeg2Rad = 3.14159265358979323846 / 180.0;

std::string Trim(std::string s)
{
    const auto a = s.find_first_not_of(" \t\r\n");
    const auto b = s.find_last_not_of(" \t\r\n");
    return a == std::string::npos ? std::string() : s.substr(a, b - a + 1);
}

std::vector<std::vector<double>> LoadCsv(const std::string& path, std::size_t cols)
{
    std::ifstream in(path);
    if (!in) {
        throw std::runtime_error("failed to open " + path);
    }
    std::vector<std::vector<double>> rows;
    std::string line;
    while (std::getline(in, line)) {
        line = Trim(line);
        if (line.empty()) {
            continue;
        }
        std::vector<double> v;
        std::stringstream ss(line);
        std::string tok;
        while (std::getline(ss, tok, ',')) {
            tok = Trim(tok);
            if (!tok.empty()) {
                v.push_back(std::stod(tok));
            }
        }
        if (v.size() != cols) {
            throw std::runtime_error(path + " expected " + std::to_string(cols) + " columns");
        }
        rows.push_back(v);
    }
    return rows;
}

void PrintT(const char* title, const Eigen::Isometry3d& T, double rmse,
            const std::vector<double>& errors)
{
    std::cout << title << "\n" << std::fixed << std::setprecision(9) << T.matrix() << "\n";
    std::cout << std::setprecision(6) << "RMSE = " << rmse << "\n";
    for (std::size_t i = 0; i < errors.size(); ++i) {
        std::cout << "  [" << i << "] " << errors[i] << "\n";
    }
}

} // namespace

int main(int argc, char** argv)
{
    const std::string dir = HAND_EYE_DATA_DIR;
    const std::string cam_path = argc > 1 ? argv[1] : dir + "/camera_point.txt";
    const std::string flg_path = argc > 2 ? argv[2] : dir + "/robot_flange_pose.txt";
    const std::string tcp_path = argc > 3 ? argv[3] : dir + "/tcp_pose.txt";

    try {
        const auto cam_rows = LoadCsv(cam_path, 3);
        const auto flg_rows = LoadCsv(flg_path, 6);
        const auto tcp_rows = LoadCsv(tcp_path, 3);
        if (cam_rows.size() != flg_rows.size() || cam_rows.size() != tcp_rows.size()) {
            std::cerr << "sample count mismatch\n";
            return 1;
        }

        std::vector<Eigen::Vector3d> cam, tcp;
        Isometry3dVector flange;
        for (std::size_t i = 0; i < cam_rows.size(); ++i) {
            cam.emplace_back(cam_rows[i][0], cam_rows[i][1], cam_rows[i][2]);
            tcp.emplace_back(tcp_rows[i][0], tcp_rows[i][1], tcp_rows[i][2]);
            flange.push_back(FlangePoseFromXyzRpyZYX(
                flg_rows[i][0], flg_rows[i][1], flg_rows[i][2],
                flg_rows[i][3] * kDeg2Rad, flg_rows[i][4] * kDeg2Rad, flg_rows[i][5] * kDeg2Rad));
        }

        const auto eih = CalibrateEyeInHand(cam, flange, tcp);
        std::cout << eih.message << "\n";
        if (!eih.success) {
            return 1;
        }
        PrintT("T_flange_camera (^{F}T_{C}) =", eih.T_flange_camera, eih.rmse, eih.per_sample_error);

        // 眼在手外：相机固定，^{B}P = ^{B}T_{C} · ^{C}P。用合成点验证接口。
        const Eigen::Isometry3d T_bc = FlangePoseFromXyzRpyZYX(120.0, -80.0, 650.0, 0.15, -0.25, 0.4);
        std::vector<Eigen::Vector3d> cam_eoh, tcp_eoh;
        const Eigen::Vector3d pts[] = {
            {400, -50, 30}, {480, 40, 55}, {350, 80, 20}, {520, -30, 70}};
        for (const auto& p : pts) {
            tcp_eoh.push_back(p);
            cam_eoh.push_back(T_bc.inverse() * p);
        }
        const auto eoh = CalibrateEyeOnHand(cam_eoh, tcp_eoh);
        std::cout << "\n" << eoh.message << "\n";
        if (!eoh.success) {
            return 1;
        }
        PrintT("T_base_camera (^{B}T_{C}) =", eoh.T_base_camera, eoh.rmse, eoh.per_sample_error);
        return 0;
    } catch (const std::exception& e) {
        std::cerr << e.what() << "\n";
        return 1;
    }
}
