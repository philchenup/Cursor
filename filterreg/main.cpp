#include "filterreg.h"

#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Geometry>
#include <chrono>
#include <iostream>
#include <string>

namespace {

void PrintUsage() {
    std::cerr << "Usage:\n"
              << "  filterreg_demo --test\n"
              << "  filterreg_demo pt2pt <cloud.ply>\n"
              << "  filterreg_demo pt2pl <source.ply> <target.ply>\n";
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

int RunSyntheticTest() {
    const int n_side = 8;
    Eigen::MatrixX3f source(n_side * n_side * n_side, 3);
    int k = 0;
    for (int z = 0; z < n_side; ++z)
        for (int y = 0; y < n_side; ++y)
            for (int x = 0; x < n_side; ++x, ++k)
                source.row(k) << (x / float(n_side - 1) - 0.5f),
                    (y / float(n_side - 1) - 0.5f),
                    (z / float(n_side - 1) - 0.5f);

    filterreg::RigidTransform gt;
    gt.rot = AngleAxis(0.35f, Eigen::Vector3f(0.2f, 0.8f, 0.3f));
    gt.t << 0.08f, -0.05f, 0.04f;
    const Eigen::MatrixX3f target = gt.apply(source);

    filterreg::Options opt;
    opt.update_sigma2 = true;
    opt.max_iter = 40;
    const auto t0 = std::chrono::high_resolution_clock::now();
    const filterreg::Result res = filterreg::registration(source, target, {}, opt);
    const auto t1 = std::chrono::high_resolution_clock::now();

    const Eigen::MatrixX3f aligned = res.transformation.apply(source);
    const float err = MeanPointError(aligned, target);
    const float rot_err = Eigen::AngleAxisf(res.transformation.rot.transpose() * gt.rot).angle();
    const float t_err = (res.transformation.t - gt.t).norm();

    std::cout << "synthetic pt2pt\n"
              << "  iterations: " << res.iterations << "\n"
              << "  sigma2: " << res.sigma2 << "\n"
              << "  q: " << res.q << "\n"
              << "  mean point error: " << err << "\n"
              << "  rotation error (rad): " << rot_err << "\n"
              << "  translation error: " << t_err << "\n"
              << "  time_ms: "
              << std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count() << "\n";

    if (err > 0.02f || rot_err > 0.05f || t_err > 0.02f) {
        std::cerr << "synthetic test failed\n";
        return 1;
    }
    std::cout << "synthetic test passed\n";
    return 0;
}

int RunPt2Pt(const std::string& path) {
    Eigen::MatrixX3f cloud;
    if (!LoadPlyPoints(path, cloud)) {
        std::cerr << "failed to load PLY: " << path << "\n";
        return 1;
    }

    filterreg::RigidTransform init;
    init.rot = AngleAxis(0.87f, Eigen::Vector3f(0.3f, 0.5f, 0.8f));
    init.t << 0.06f, -0.04f, 0.05f;

    filterreg::Options opt;
    opt.init = init;
    opt.update_sigma2 = true;
    opt.max_iter = 30;

    const auto t0 = std::chrono::high_resolution_clock::now();
    const filterreg::Result res = filterreg::registration(cloud, cloud, {}, opt);
    const auto t1 = std::chrono::high_resolution_clock::now();

    const Eigen::MatrixX3f aligned = res.transformation.apply(cloud);
    const float err = MeanPointError(aligned, cloud);
    std::cout << "pt2pt " << path << "\n"
              << "  points: " << cloud.rows() << "\n"
              << "  iterations: " << res.iterations << "\n"
              << "  sigma2: " << res.sigma2 << "\n"
              << "  mean alignment error: " << err << "\n"
              << "  R:\n" << res.transformation.rot << "\n"
              << "  t: " << res.transformation.t.transpose() << "\n"
              << "  time_ms: "
              << std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count() << "\n";
    SavePlyPoints("matched_source.ply", aligned);
    SavePlyPoints("matched_target.ply", cloud);
    return 0;
}

int RunPt2Pl(const std::string& src_path, const std::string& dst_path) {
    Eigen::MatrixX3f source, target, target_n, source_n;
    if (!LoadPlyPointsAndNormals(src_path, source, source_n) ||
        !LoadPlyPointsAndNormals(dst_path, target, target_n)) {
        std::cerr << "failed to load PLY files\n";
        return 1;
    }

    filterreg::Options opt;
    opt.objective = filterreg::Objective::PointToPlane;
    opt.sigma2 = 0.08f * 0.08f;
    opt.update_sigma2 = false;
    opt.max_iter = 15;

    const auto t0 = std::chrono::high_resolution_clock::now();
    const filterreg::Result res = filterreg::registration(source, target, target_n, opt);
    const auto t1 = std::chrono::high_resolution_clock::now();

    const Eigen::MatrixX3f aligned = res.transformation.apply(source);
    std::cout << "pt2pl\n"
              << "  source points: " << source.rows() << "\n"
              << "  target points: " << target.rows() << "\n"
              << "  iterations: " << res.iterations << "\n"
              << "  sigma2: " << res.sigma2 << "\n"
              << "  q: " << res.q << "\n"
              << "  R:\n" << res.transformation.rot << "\n"
              << "  t: " << res.transformation.t.transpose() << "\n"
              << "  time_ms: "
              << std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count() << "\n";
    SavePlyPoints("matched_source.ply", aligned);
    SavePlyPoints("matched_target.ply", target);
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
    if (mode == "pt2pt" && argc == 3) return RunPt2Pt(argv[2]);
    if (mode == "pt2pl" && argc == 4) return RunPt2Pl(argv[2], argv[3]);
    PrintUsage();
    return 1;
}
