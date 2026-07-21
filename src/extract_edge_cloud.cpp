/**
 * @file extract_edge_cloud.cpp
 * @brief 基于点云的边缘提取算法（C++ / PCL）
 *
 * 实现两种常用算法：
 *   1) boundary  —— 切平面角度空隙边界提取（外轮廓 / 孔洞边界）
 *   2) curvature —— PCA 曲率阈值提取（棱边 / 尖锐特征）
 *
 * 用法：
 *   ./extract_edge_cloud <input.pcd|ply> [method] [k] [threshold] [voxel] [threads]
 *
 *   method:    boundary | curvature          默认 boundary
 *   k:         邻域点数                      默认 30
 *   threshold: boundary=角度阈值(度,默认90)
 *              curvature=曲率阈值(默认0.1)
 *   voxel:     体素边长，>0 则先降采样        默认 0
 *   threads:   OpenMP 线程数                 默认硬件并发
 *
 * 示例：
 *   ./extract_edge_cloud cloud.pcd boundary 30 90 0.005 8
 *   ./extract_edge_cloud cloud.pcd curvature 30 0.1 0.005 8
 */

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <numeric>
#include <string>
#include <thread>
#include <vector>

#include <pcl/common/common.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/visualization/pcl_visualizer.h>

namespace {

using CloudT = pcl::PointCloud<pcl::PointXYZ>;
using NormalCloudT = pcl::PointCloud<pcl::Normal>;
using Clock = std::chrono::steady_clock;

double msSince(const Clock::time_point& t0) {
  return std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
}

// ---------------------------------------------------------------------------
// I/O & 预处理
// ---------------------------------------------------------------------------

bool loadCloud(const std::string& path, CloudT::Ptr& cloud) {
  const auto pos = path.find_last_of('.');
  if (pos == std::string::npos) {
    std::cerr << "错误：无法识别文件格式: " << path << std::endl;
    return false;
  }
  const std::string ext = path.substr(pos);
  int ret = -1;
  if (ext == ".pcd" || ext == ".PCD") {
    ret = pcl::io::loadPCDFile<pcl::PointXYZ>(path, *cloud);
  } else if (ext == ".ply" || ext == ".PLY") {
    ret = pcl::io::loadPLYFile<pcl::PointXYZ>(path, *cloud);
  } else {
    std::cerr << "错误：仅支持 .pcd / .ply" << std::endl;
    return false;
  }
  if (ret < 0 || cloud->empty()) {
    std::cerr << "错误：读取失败或点云为空" << std::endl;
    return false;
  }
  return true;
}

CloudT::Ptr downsampleVoxel(const CloudT::Ptr& cloud, float leaf) {
  if (leaf <= 0.0f) {
    return cloud;
  }
  CloudT::Ptr out(new CloudT);
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  vg.setInputCloud(cloud);
  vg.setLeafSize(leaf, leaf, leaf);
  vg.filter(*out);
  return out;
}

NormalCloudT::Ptr estimateNormals(const CloudT::Ptr& cloud,
                                  const pcl::search::KdTree<pcl::PointXYZ>::Ptr& tree,
                                  int k,
                                  unsigned int threads) {
  NormalCloudT::Ptr normals(new NormalCloudT);
  pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
  ne.setNumberOfThreads(threads);
  ne.setInputCloud(cloud);
  ne.setSearchMethod(tree);
  ne.setKSearch(k);
  ne.compute(*normals);
  return normals;
}

// ---------------------------------------------------------------------------
// 算法 1：Boundary Edge（切平面角度空隙）
//
// 对每个查询点 p：
//   1. 取 k 近邻，用法向量 n 将邻域投影到切平面
//   2. 计算投影向量相对某参考方向的方位角，排序
//   3. 找相邻方位角的最大间隙 Δθ_max
//   4. 若 Δθ_max > 阈值，则 p 为边界点（外轮廓或孔洞边缘）
// ---------------------------------------------------------------------------

CloudT::Ptr extractBoundaryEdges(
    const CloudT::Ptr& cloud,
    const NormalCloudT::Ptr& normals,
    const pcl::search::KdTree<pcl::PointXYZ>::Ptr& tree,
    int k,
    float angle_threshold_rad,
    unsigned int /*threads*/) {
  CloudT::Ptr edges(new CloudT);
  edges->reserve(cloud->size() / 10);

  std::vector<int> nn_idx(static_cast<std::size_t>(k));
  std::vector<float> nn_dist(static_cast<std::size_t>(k));
  std::vector<float> angles;
  angles.reserve(static_cast<std::size_t>(k));

  for (std::size_t i = 0; i < cloud->size(); ++i) {
    const auto& p = cloud->points[i];
    const auto& nrm = normals->points[i];
    if (!pcl::isFinite(p) || !std::isfinite(nrm.normal_x)) {
      continue;
    }

    Eigen::Vector3f n(nrm.normal_x, nrm.normal_y, nrm.normal_z);
    const float nlen = n.norm();
    if (nlen < 1e-8f) {
      continue;
    }
    n /= nlen;

    if (tree->nearestKSearch(p, k, nn_idx, nn_dist) < 3) {
      continue;
    }

    // 切平面正交基 u, v
    Eigen::Vector3f u = n.unitOrthogonal();
    Eigen::Vector3f v = n.cross(u);

    angles.clear();
    for (int j = 0; j < static_cast<int>(nn_idx.size()); ++j) {
      if (static_cast<std::size_t>(nn_idx[j]) == i) {
        continue;
      }
      const auto& q = cloud->points[nn_idx[j]];
      Eigen::Vector3f d(q.x - p.x, q.y - p.y, q.z - p.z);
      // 投影到切平面
      const float x = d.dot(u);
      const float y = d.dot(v);
      if (x * x + y * y < 1e-12f) {
        continue;
      }
      angles.push_back(std::atan2(y, x));
    }

    if (angles.size() < 2) {
      continue;
    }

    std::sort(angles.begin(), angles.end());

    float max_gap = 0.0f;
    for (std::size_t a = 1; a < angles.size(); ++a) {
      max_gap = std::max(max_gap, angles[a] - angles[a - 1]);
    }
    // 首尾跨越 ±π
    const float wrap_gap =
        (angles.front() + static_cast<float>(2.0 * M_PI)) - angles.back();
    max_gap = std::max(max_gap, wrap_gap);

    if (max_gap > angle_threshold_rad) {
      edges->push_back(p);
    }
  }

  edges->width = static_cast<uint32_t>(edges->size());
  edges->height = 1;
  edges->is_dense = true;
  return edges;
}

// ---------------------------------------------------------------------------
// 算法 2：Curvature Edge（PCA 表面曲率）
//
// 对每个点取 k 邻域做 PCA：
//   协方差矩阵特征值 λ0 ≤ λ1 ≤ λ2
//   曲率 σ = λ0 / (λ0+λ1+λ2)
//   σ 大 → 局部弯曲/棱边显著 → 判为边缘点
// ---------------------------------------------------------------------------

CloudT::Ptr extractCurvatureEdges(
    const CloudT::Ptr& cloud,
    const pcl::search::KdTree<pcl::PointXYZ>::Ptr& tree,
    int k,
    float curvature_threshold,
    unsigned int /*threads*/) {
  CloudT::Ptr edges(new CloudT);
  edges->reserve(cloud->size() / 10);

  std::vector<int> nn_idx(static_cast<std::size_t>(k));
  std::vector<float> nn_dist(static_cast<std::size_t>(k));

  for (std::size_t i = 0; i < cloud->size(); ++i) {
    const auto& p = cloud->points[i];
    if (!pcl::isFinite(p)) {
      continue;
    }
    if (tree->nearestKSearch(p, k, nn_idx, nn_dist) < 3) {
      continue;
    }

    // 邻域质心
    Eigen::Vector3f centroid(0.f, 0.f, 0.f);
    const int n_nn = static_cast<int>(nn_idx.size());
    for (int j = 0; j < n_nn; ++j) {
      const auto& q = cloud->points[nn_idx[j]];
      centroid += Eigen::Vector3f(q.x, q.y, q.z);
    }
    centroid /= static_cast<float>(n_nn);

    // 协方差
    Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();
    for (int j = 0; j < n_nn; ++j) {
      const auto& q = cloud->points[nn_idx[j]];
      Eigen::Vector3f d(q.x - centroid.x(), q.y - centroid.y(),
                        q.z - centroid.z());
      cov += d * d.transpose();
    }
    cov /= static_cast<float>(n_nn);

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(cov);
    if (solver.info() != Eigen::Success) {
      continue;
    }
    // SelfAdjointEigenSolver：特征值升序 λ0 ≤ λ1 ≤ λ2
    const Eigen::Vector3f evals = solver.eigenvalues();
    const float sum = evals.sum();
    if (sum < 1e-12f) {
      continue;
    }
    const float curvature = evals[0] / sum;

    if (curvature >= curvature_threshold) {
      edges->push_back(p);
    }
  }

  edges->width = static_cast<uint32_t>(edges->size());
  edges->height = 1;
  edges->is_dense = true;
  return edges;
}

// ---------------------------------------------------------------------------
// 可视化
// ---------------------------------------------------------------------------

void visualize(const CloudT::Ptr& cloud, const CloudT::Ptr& edges,
               const std::string& title) {
  pcl::visualization::PCLVisualizer::Ptr viewer(
      new pcl::visualization::PCLVisualizer(title));
  viewer->setBackgroundColor(0.0, 0.0, 0.0);

  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> gray(
      cloud, 120, 120, 120);
  viewer->addPointCloud<pcl::PointXYZ>(cloud, gray, "cloud");
  viewer->setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");

  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red(
      edges, 255, 40, 40);
  viewer->addPointCloud<pcl::PointXYZ>(edges, red, "edges");
  viewer->setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "edges");

  pcl::PointXYZ mn, mx;
  pcl::getMinMax3D(*cloud, mn, mx);
  const float diag = (mx.getVector3fMap() - mn.getVector3fMap()).norm();
  viewer->addCoordinateSystem(std::max(diag * 0.15f, 0.01f));
  viewer->initCameraParameters();
  viewer->resetCamera();

  std::cout << "灰=点云，红=边缘。关闭窗口或按 q 退出。" << std::endl;
  while (!viewer->wasStopped()) {
    viewer->spinOnce(100);
  }
}

void printUsage(const char* prog) {
  std::cerr
      << "用法: " << prog
      << " <input.pcd|ply> [method] [k] [threshold] [voxel] [threads]\n"
      << "  method: boundary | curvature\n"
      << "  boundary  threshold = 角度阈值(度), 默认 90\n"
      << "  curvature threshold = 曲率阈值,     默认 0.1\n"
      << "示例:\n"
      << "  " << prog << " cloud.pcd boundary 30 90 0.005 8\n"
      << "  " << prog << " cloud.pcd curvature 30 0.1 0.005 8\n";
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 2) {
    printUsage(argv[0]);
    return 1;
  }

  const std::string input_path = argv[1];
  const std::string method = (argc >= 3) ? argv[2] : "boundary";
  const int k = (argc >= 4) ? std::atoi(argv[3]) : 30;
  const float threshold = (argc >= 5)
                              ? static_cast<float>(std::atof(argv[4]))
                              : ((method == "curvature") ? 0.1f : 90.0f);
  const float voxel = (argc >= 6) ? static_cast<float>(std::atof(argv[5])) : 0.0f;
  const unsigned int hw = std::max(1u, std::thread::hardware_concurrency());
  const unsigned int threads =
      (argc >= 7) ? static_cast<unsigned int>(std::atoi(argv[6])) : hw;

  if (k < 3) {
    std::cerr << "错误：k 至少为 3" << std::endl;
    return 1;
  }
  if (method != "boundary" && method != "curvature") {
    std::cerr << "错误：method 应为 boundary 或 curvature" << std::endl;
    printUsage(argv[0]);
    return 1;
  }

  const auto t_all = Clock::now();

  // 1. 读取
  CloudT::Ptr cloud(new CloudT);
  std::cout << "[1] 读取: " << input_path << std::endl;
  auto t0 = Clock::now();
  if (!loadCloud(input_path, cloud)) {
    return -1;
  }
  std::cout << "    点数=" << cloud->size() << " (" << msSince(t0) << " ms)\n";

  // 2. 降采样
  t0 = Clock::now();
  auto work = downsampleVoxel(cloud, voxel);
  if (voxel > 0.0f) {
    std::cout << "[2] VoxelGrid leaf=" << voxel
              << " → " << work->size() << " (" << msSince(t0) << " ms)\n";
  } else {
    std::cout << "[2] 跳过降采样\n";
  }

  // 3. KdTree
  t0 = Clock::now();
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
      new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(work);
  std::cout << "[3] KdTree (" << msSince(t0) << " ms)\n";

  CloudT::Ptr edges;

  if (method == "boundary") {
    // 4a. 法向量 + 边界算法
    const float angle_rad = threshold * static_cast<float>(M_PI) / 180.0f;
    std::cout << "[4] 法向量估计 (OMP, k=" << k << ", threads=" << threads
              << ")\n";
    t0 = Clock::now();
    auto normals = estimateNormals(work, tree, k, threads);
    std::cout << "    (" << msSince(t0) << " ms)\n";

    std::cout << "[5] Boundary 边缘提取 (角度>" << threshold << "°)\n";
    t0 = Clock::now();
    edges = extractBoundaryEdges(work, normals, tree, k, angle_rad, threads);
    std::cout << "    边缘点数=" << edges->size() << " (" << msSince(t0)
              << " ms)\n";
  } else {
    // 4b. 曲率算法（内部做 PCA，无需单独法向量）
    std::cout << "[4] Curvature 边缘提取 (σ>=" << threshold << ", k=" << k
              << ")\n";
    t0 = Clock::now();
    edges = extractCurvatureEdges(work, tree, k, threshold, threads);
    std::cout << "    边缘点数=" << edges->size() << " (" << msSince(t0)
              << " ms)\n";
  }

  if (edges->empty()) {
    std::cerr << "警告：未检测到边缘点，请调整 threshold / k / voxel\n";
  }

  std::cout << "总耗时(不含可视化): " << msSince(t_all) << " ms\n";

  pcl::io::savePCDFileBinary("edge_cloud.pcd", *edges);
  std::cout << "已保存: edge_cloud.pcd\n";

  visualize(work, edges, "Edge Extraction [" + method + "]");
  return 0;
}
