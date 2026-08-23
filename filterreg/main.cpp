#include "filterreg.h"

#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Geometry>
#include <chrono>
#include <iostream>
#include <string>

namespace {

constexpr float kMmPerM = 1000.f;

void PrintUsage() {
    std::cerr << "Usage:\n"
              << "  filterreg_demo --test\n"
              << "  filterreg_demo pt2pt <source.ply> <target.ply>\n"
              << "  filterreg_demo pt2pl <source.ply> <target.ply>\n"
              << "PLY coordinates must be in millimeters.\n";
}

Eigen::Matrix3f AngleAxis(float angle, Eigen::Vector3f axis) {
    axis.normalize();
    return Eigen::AngleAxisf(angle, axis).toRotationMatrix();
}

float MeanPointError(const Eigen::MatrixX3f& a, const Eigen::MatrixX3f& b) {
    return (a - b).rowwise().norm().mean();
}

bool LoadPlyPoints(const std::string& path, Eigen::MatrixX3f& points) {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    if (pcl::io::loadPLYFile(path, cloud) < 0 || cloud.empty()) return false;
    points.resize(static_cast<int>(cloud.size()), 3);
    for (std::size_t i = 0; i < cloud.size(); ++i) {
        points(static_cast<int>(i), 0) = cloud[i].x;
        points(static_cast<int>(i), 1) = cloud[i].y;
        points(static_cast<int>(i), 2) = cloud[i].z;
    }
    return true;
}

bool LoadPlyPointsAndNormals(const std::string& path,
                             Eigen::MatrixX3f& points,
                             Eigen::MatrixX3f& normals) {
    pcl::PointCloud<pcl::PointNormal> cloud;
    if (pcl::io::loadPLYFile(path, cloud) < 0 || cloud.empty()) return false;
    points.resize(static_cast<int>(cloud.size()), 3);
    normals.resize(static_cast<int>(cloud.size()), 3);
    for (std::size_t i = 0; i < cloud.size(); ++i) {
        points(static_cast<int>(i), 0) = cloud[i].x;
        points(static_cast<int>(i), 1) = cloud[i].y;
        points(static_cast<int>(i), 2) = cloud[i].z;
        normals(static_cast<int>(i), 0) = cloud[i].normal_x;
        normals(static_cast<int>(i), 1) = cloud[i].normal_y;
        normals(static_cast<int>(i), 2) = cloud[i].normal_z;
    }
    return true;
}

void SavePlyPoints(const std::string& path, const Eigen::MatrixX3f& points) {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.resize(static_cast<std::size_t>(points.rows()));
    for (int i = 0; i < points.rows(); ++i) {
        cloud[i].x = points(i, 0);
        cloud[i].y = points(i, 1);
        cloud[i].z = points(i, 2);
    }
    pcl::io::savePLYFileASCII(path, cloud);
}

void PrintResult(const char* name, const filterreg::Result& res) {
    std::cout << name << "\n"
              << "  iterations: " << res.iterations << "\n"
              << "  sigma2 (mm^2): " << res.sigma2 << "\n"
              << "  q: " << res.q << "\n"
              << "  R:\n" << res.transformation.rot << "\n"
              << "  t (mm): " << res.transformation.t.transpose() << "\n";
}

// FilterReg internally uses meters. Convert mm clouds, then write t / sigma2 back to mm.
filterreg::Result RegisterMm(const Eigen::MatrixX3f& source_mm,
                             const Eigen::MatrixX3f& target_mm,
                             const Eigen::MatrixX3f& target_normals,
                             const filterreg::Options& opt_m) {
    filterreg::Result res = filterreg::registration(
        source_mm / kMmPerM, target_mm / kMmPerM, target_normals, opt_m);
    res.transformation.t *= kMmPerM;
    res.sigma2 *= kMmPerM * kMmPerM;
    return res;
}

// source_mm / target_mm: N x 3 and M x 3, coordinates in millimeters.
filterreg::Result RunPt2Pt(const Eigen::MatrixX3f& source_mm,
                           const Eigen::MatrixX3f& target_mm) {
    filterreg::Options opt;
    opt.objective = filterreg::Objective::PointToPoint;
    opt.update_sigma2 = true;
    opt.max_iter = 30;
    return RegisterMm(source_mm, target_mm, {}, opt);
}

// source_mm / target_mm: millimeters. target_normals: unit vectors, one per target point.
filterreg::Result RunPt2Pl(const Eigen::MatrixX3f& source_mm,
                           const Eigen::MatrixX3f& target_mm,
                           const Eigen::MatrixX3f& target_normals) {
    filterreg::Options opt;
    opt.objective = filterreg::Objective::PointToPlane;
    opt.sigma2 = 0.08f * 0.08f;  // 80 mm, stored in meters for the solver
    opt.update_sigma2 = false;
    opt.max_iter = 15;
    return RegisterMm(source_mm, target_mm, target_normals, opt);
}

int RunSyntheticTest() {
    const int n_side = 8;
    Eigen::MatrixX3f source_mm(n_side * n_side * n_side, 3);
    int k = 0;
    for (int z = 0; z < n_side; ++z)
        for (int y = 0; y < n_side; ++y)
            for (int x = 0; x < n_side; ++x, ++k)
                source_mm.row(k) << (x / float(n_side - 1) - 0.5f) * 1000.f,
                    (y / float(n_side - 1) - 0.5f) * 1000.f,
                    (z / float(n_side - 1) - 0.5f) * 1000.f;

    filterreg::RigidTransform gt;
    gt.rot = AngleAxis(0.35f, Eigen::Vector3f(0.2f, 0.8f, 0.3f));
    gt.t << 80.f, -50.f, 40.f;  // mm
    const Eigen::MatrixX3f target_mm = gt.apply(source_mm);

    const auto t0 = std::chrono::high_resolution_clock::now();
    const filterreg::Result res = RunPt2Pt(source_mm, target_mm);
    const auto t1 = std::chrono::high_resolution_clock::now();

    const Eigen::MatrixX3f aligned = res.transformation.apply(source_mm);
    const float err = MeanPointError(aligned, target_mm);
    const float rot_err = Eigen::AngleAxisf(res.transformation.rot.transpose() * gt.rot).angle();
    const float t_err = (res.transformation.t - gt.t).norm();

    PrintResult("synthetic pt2pt (mm)", res);
    std::cout << "  mean point error (mm): " << err << "\n"
              << "  rotation error (rad): " << rot_err << "\n"
              << "  translation error (mm): " << t_err << "\n"
              << "  time_ms: "
              << std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count() << "\n";

    if (err > 0.05f || rot_err > 0.05f || t_err > 0.05f) {
        std::cerr << "synthetic test failed\n";
        return 1;
    }
    std::cout << "synthetic test passed\n";
    return 0;
}

int RunPt2PtFiles(const std::string& src_path, const std::string& dst_path) {
    Eigen::MatrixX3f source_mm, target_mm;
    if (!LoadPlyPoints(src_path, source_mm) || !LoadPlyPoints(dst_path, target_mm)) {
        std::cerr << "failed to load PLY (mm): " << src_path << " / " << dst_path << "\n";
        return 1;
    }

    const auto t0 = std::chrono::high_resolution_clock::now();
    const filterreg::Result res = RunPt2Pt(source_mm, target_mm);
    const auto t1 = std::chrono::high_resolution_clock::now();

    const Eigen::MatrixX3f aligned_mm = res.transformation.apply(source_mm);
    std::cout << "pt2pt " << src_path << " -> " << dst_path << "\n"
              << "  source points: " << source_mm.rows() << "\n"
              << "  target points: " << target_mm.rows() << "\n";
    PrintResult("  result", res);
    if (aligned_mm.rows() == target_mm.rows())
        std::cout << "  mean alignment error (mm): " << MeanPointError(aligned_mm, target_mm) << "\n";
    std::cout << "  time_ms: "
              << std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count() << "\n";
    SavePlyPoints("matched_source.ply", aligned_mm);
    SavePlyPoints("matched_target.ply", target_mm);
    return 0;
}

int RunPt2PlFiles(const std::string& src_path, const std::string& dst_path) {
    Eigen::MatrixX3f source_mm, target_mm, target_n, source_n;
    if (!LoadPlyPointsAndNormals(src_path, source_mm, source_n) ||
        !LoadPlyPointsAndNormals(dst_path, target_mm, target_n)) {
        std::cerr << "failed to load PLY (mm): " << src_path << " / " << dst_path << "\n";
        return 1;
    }

    const auto t0 = std::chrono::high_resolution_clock::now();
    const filterreg::Result res = RunPt2Pl(source_mm, target_mm, target_n);
    const auto t1 = std::chrono::high_resolution_clock::now();

    const Eigen::MatrixX3f aligned_mm = res.transformation.apply(source_mm);
    std::cout << "pt2pl " << src_path << " -> " << dst_path << "\n"
              << "  source points: " << source_mm.rows() << "\n"
              << "  target points: " << target_mm.rows() << "\n";
    PrintResult("  result", res);
    std::cout << "  time_ms: "
              << std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count() << "\n";
    (void)aligned_mm;
    SavePlyPoints("matched_source.ply", aligned_mm);
    SavePlyPoints("matched_target.ply", target_mm);
    return 0;
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        PrintUsage();
        return 1;
    }
    const std::string mode = argv[1];
    if (mode == "--test") return RunSyntheticTest();
    if (mode == "pt2pt" && argc == 4) return RunPt2PtFiles(argv[2], argv[3]);
    if (mode == "pt2pl" && argc == 4) return RunPt2PlFiles(argv[2], argv[3]);
    PrintUsage();
    return 1;
}
