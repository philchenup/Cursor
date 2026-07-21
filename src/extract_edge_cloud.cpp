/**
 * @file extract_edge_cloud.cpp
 * @brief 基于 PCA 特征值曲率 Sigma 的点云边缘提取
 *
 * 算法：对每个点取 K 近邻 → 协方差矩阵 → 特征值 λs≤λm≤λl
 *       Sigma = λs / (λs+λm+λl)
 *       Sigma 较大的点标为边缘（红色）
 *
 * 适配：PCL 1.12 + C++14
 *
 * 用法：
 *   ./extract_edge_cloud <input.ply|pcd> [K邻域] [sigma倍数]
 *   默认：K=10，倍数=6（阈值 = MinSigma + 倍数 * step）
 */

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>

#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

namespace {

bool loadCloud(const std::string& path,
               pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud) {
  const auto dot = path.find_last_of('.');
  if (dot == std::string::npos) {
    std::cerr << "无法识别文件后缀: " << path << std::endl;
    return false;
  }
  const std::string ext = path.substr(dot);
  int ret = -1;
  if (ext == ".ply" || ext == ".PLY") {
    ret = pcl::io::loadPLYFile(path, *cloud);
  } else if (ext == ".pcd" || ext == ".PCD") {
    ret = pcl::io::loadPCDFile(path, *cloud);
  } else {
    std::cerr << "仅支持 .ply / .pcd" << std::endl;
    return false;
  }
  if (ret < 0 || cloud->empty()) {
    std::cerr << "读取失败或点云为空: " << path << std::endl;
    return false;
  }
  return true;
}

/**
 * 对邻域点计算 3x3 协方差，返回升序特征值 (λs, λm, λl)。
 * SelfAdjointEigenSolver 本身按升序给出特征值，无需手工排序。
 */
bool computeEigenValues(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
                        const std::vector<int>& nn_indices,
                        int nn_count,
                        Eigen::Vector3f& evals) {
  if (nn_count < 3) {
    return false;
  }

  Eigen::Vector3f mean = Eigen::Vector3f::Zero();
  for (int j = 0; j < nn_count; ++j) {
    const auto& p = cloud->points[static_cast<std::size_t>(nn_indices[j])];
    mean += Eigen::Vector3f(p.x, p.y, p.z);
  }
  mean /= static_cast<float>(nn_count);

  // 无偏估计：/(n-1)，与原代码一致
  Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();
  for (int j = 0; j < nn_count; ++j) {
    const auto& p = cloud->points[static_cast<std::size_t>(nn_indices[j])];
    Eigen::Vector3f d(p.x - mean.x(), p.y - mean.y(), p.z - mean.z());
    cov += d * d.transpose();
  }
  cov /= static_cast<float>(nn_count - 1);

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(cov);
  if (solver.info() != Eigen::Success) {
    return false;
  }
  // eigenvalues(): λ0 ≤ λ1 ≤ λ2
  evals = solver.eigenvalues();
  return true;
}

}  // namespace

int main(int argc, char* argv[]) {
  if (argc < 2) {
    std::cerr << "用法: " << argv[0]
              << " <input.ply|pcd> [K邻域] [sigma倍数]\n"
              << "示例: " << argv[0] << " data/Edge.ply 10 6\n";
    return 1;
  }

  const std::string input_path = argv[1];
  const int k_neighbors = (argc >= 3) ? std::atoi(argv[2]) : 10;
  const int sigma_mult = (argc >= 4) ? std::atoi(argv[3]) : 6;
  constexpr int kNumColors = 256;

  if (k_neighbors < 3) {
    std::cerr << "K 邻域至少为 3" << std::endl;
    return 1;
  }

  // ---------- 读取点云（XYZRGB，便于着色）----------
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  if (!loadCloud(input_path, cloud)) {
    return -1;
  }
  std::cout << "输入点数: " << cloud->size() << std::endl;

  // ---------- KdTree ----------
  pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
  kdtree.setInputCloud(cloud);

  const std::size_t n_points = cloud->size();
  std::vector<double> sigma(n_points, 0.0);
  std::vector<int> nn_indices(static_cast<std::size_t>(k_neighbors));
  std::vector<float> nn_dists(static_cast<std::size_t>(k_neighbors));

  // ---------- 逐点：KNN → 协方差 → 特征值 → Sigma ----------
  for (std::size_t i = 0; i < n_points; ++i) {
    const auto& search_point = cloud->points[i];

    const int found = kdtree.nearestKSearch(
        search_point, k_neighbors, nn_indices, nn_dists);
    if (found < 3) {
      sigma[i] = 0.0;
      continue;
    }

    Eigen::Vector3f evals;
    if (!computeEigenValues(cloud, nn_indices, found, evals)) {
      sigma[i] = 0.0;
      continue;
    }

    const double smallest = static_cast<double>(evals[0]);
    const double middle = static_cast<double>(evals[1]);
    const double largest = static_cast<double>(evals[2]);
    const double sum = smallest + middle + largest;

    // 原代码还计算了 DLS/DLM/DMS，边缘判定实际只用 Sigma
    (void)middle;
    (void)largest;
    sigma[i] = (sum > 1e-12) ? (smallest / sum) : 0.0;
  }

  std::cout << "Computing Sigma is Done!" << std::endl;

  // ---------- Sigma 范围 ----------
  double min_sigma = std::numeric_limits<double>::max();
  double max_sigma = std::numeric_limits<double>::lowest();
  for (double s : sigma) {
    min_sigma = std::min(min_sigma, s);
    max_sigma = std::max(max_sigma, s);
  }
  std::cout << "Minimum Sigma: " << min_sigma << std::endl;
  std::cout << "Maximum Sigma: " << max_sigma << std::endl;

  // ---------- 着色：khaki 底色，高 Sigma → 红色边缘 ----------
  for (auto& p : cloud->points) {
    p.r = 240;
    p.g = 230;
    p.b = 140;
  }

  const double step = (max_sigma - min_sigma) / static_cast<double>(kNumColors);
  const double edge_threshold = min_sigma + static_cast<double>(sigma_mult) * step;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr edge_cloud(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  edge_cloud->reserve(n_points / 20);

  int edge_count = 0;
  for (std::size_t i = 0; i < n_points; ++i) {
    if (sigma[i] > edge_threshold) {
      cloud->points[i].r = 255;
      cloud->points[i].g = 0;
      cloud->points[i].b = 0;
      edge_cloud->push_back(cloud->points[i]);
      ++edge_count;
    }
  }
  edge_cloud->width = static_cast<std::uint32_t>(edge_cloud->size());
  edge_cloud->height = 1;
  edge_cloud->is_dense = true;

  std::cout << "Number of Edge points: " << edge_count << std::endl;
  std::cout << "Edge threshold (Min + " << sigma_mult << "*step): "
            << edge_threshold << std::endl;

  // 保存边缘点与着色全图
  pcl::io::savePLYFileBinary("edge_cloud.ply", *edge_cloud);
  pcl::io::savePLYFileBinary("cloud_colored.ply", *cloud);
  std::cout << "已保存: edge_cloud.ply, cloud_colored.ply" << std::endl;

  // ---------- 可视化（PCLVisualizer，PCL 1.12 推荐）----------
  pcl::visualization::PCLVisualizer::Ptr viewer(
      new pcl::visualization::PCLVisualizer("PCA Sigma Edge Viewer"));
  viewer->setBackgroundColor(0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(
      cloud);
  viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "cloud");
  viewer->setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
  viewer->addCoordinateSystem(0.1);
  viewer->initCameraParameters();

  std::cout << "khaki=非边缘，红=边缘。关闭窗口或按 q 退出。" << std::endl;
  while (!viewer->wasStopped()) {
    viewer->spinOnce(100);
  }

  return 0;
}
