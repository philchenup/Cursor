/**
 * @file extract_edge_cloud.cpp
 * @brief 使用 PCL 读取点云并提取边缘点云（含加速策略）
 *
 * 流程：
 *   1. 读取点云（PCD / PLY）
 *   2. [加速] VoxelGrid 降采样，减小 N
 *   3. [加速] OpenMP 并行法向量估计
 *   4. [加速] 多线程边界估计
 *   5. 提取边缘点云并可视化、保存
 *
 * 用法：
 *   ./extract_edge_cloud <input.pcd|ply> [k邻域] [角度阈值(度)] [体素边长] [线程数]
 *
 *   默认：k=30，角度=90°，体素=0（不降采样），线程=硬件并发数
 *
 * 加速要点（按收益排序）：
 *   A. 先 VoxelGrid 降采样 —— 复杂度近似 O(N·k·logN)，N 下降最有效
 *   B. NormalEstimationOMP / Feature 多线程 —— 吃满多核
 *   C. 复用同一棵 KdTree，避免重复建树
 *   D. 适当减小 k —— 邻域查询更便宜
 *   E. 有序点云（深度图）可改用 OrganizedEdge*，比 KdTree 快一个量级
 */

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>

#include <pcl/common/common.h>
#include <pcl/features/boundary.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/visualization/pcl_visualizer.h>

namespace {

using Clock = std::chrono::steady_clock;

double msSince(const Clock::time_point& t0) {
  return std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
}

bool loadCloud(const std::string& path,
               pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
  const auto ext_pos = path.find_last_of('.');
  if (ext_pos == std::string::npos) {
    std::cerr << "错误：无法识别文件格式: " << path << std::endl;
    return false;
  }
  const std::string ext = path.substr(ext_pos);

  int ret = -1;
  if (ext == ".pcd" || ext == ".PCD") {
    ret = pcl::io::loadPCDFile<pcl::PointXYZ>(path, *cloud);
  } else if (ext == ".ply" || ext == ".PLY") {
    ret = pcl::io::loadPLYFile<pcl::PointXYZ>(path, *cloud);
  } else {
    std::cerr << "错误：仅支持 .pcd / .ply，当前: " << ext << std::endl;
    return false;
  }

  if (ret < 0 || cloud->empty()) {
    std::cerr << "错误：读取点云失败或点云为空: " << path << std::endl;
    return false;
  }
  return true;
}

/** 加速 A：体素降采样，优先砍掉点数 N */
pcl::PointCloud<pcl::PointXYZ>::Ptr downsampleVoxel(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    float leaf) {
  if (leaf <= 0.0f) {
    return cloud;
  }
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  vg.setInputCloud(cloud);
  vg.setLeafSize(leaf, leaf, leaf);
  vg.filter(*filtered);
  return filtered;
}

/**
 * 加速 B：OpenMP 并行法向量估计。
 * 加速 C：外部传入并复用 KdTree，避免重复建树。
 */
pcl::PointCloud<pcl::Normal>::Ptr estimateNormalsOMP(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    const pcl::search::KdTree<pcl::PointXYZ>::Ptr& tree,
    int k_neighbors,
    unsigned int num_threads) {
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
  pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
  ne.setNumberOfThreads(num_threads);
  ne.setInputCloud(cloud);
  ne.setSearchMethod(tree);
  ne.setKSearch(k_neighbors);
  ne.compute(*normals);
  return normals;
}

/**
 * 边界估计：Feature 基类在 PCL 编译启用 OpenMP 时支持 setNumberOfThreads。
 * 与法向量共用同一棵 KdTree。
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr extractBoundaryCloud(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    const pcl::PointCloud<pcl::Normal>::Ptr& normals,
    const pcl::search::KdTree<pcl::PointXYZ>::Ptr& tree,
    int k_neighbors,
    float angle_threshold_rad,
    unsigned int num_threads) {
  pcl::PointCloud<pcl::Boundary> boundaries;
  pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary> be;
  be.setNumberOfThreads(num_threads);
  be.setInputCloud(cloud);
  be.setInputNormals(normals);
  be.setSearchMethod(tree);
  be.setKSearch(k_neighbors);
  be.setAngleThreshold(angle_threshold_rad);
  be.compute(boundaries);

  pcl::PointCloud<pcl::PointXYZ>::Ptr edge_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  edge_cloud->reserve(cloud->size() / 10);

  for (std::size_t i = 0; i < cloud->size(); ++i) {
    if (boundaries.points[i].boundary_point != 0) {
      edge_cloud->push_back(cloud->points[i]);
    }
  }

  edge_cloud->width = static_cast<uint32_t>(edge_cloud->size());
  edge_cloud->height = 1;
  edge_cloud->is_dense = true;
  return edge_cloud;
}

void visualize(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
               const pcl::PointCloud<pcl::PointXYZ>::Ptr& edge_cloud) {
  pcl::visualization::PCLVisualizer::Ptr viewer(
      new pcl::visualization::PCLVisualizer("Edge Point Cloud"));
  viewer->setBackgroundColor(0.0, 0.0, 0.0);

  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> gray(
      cloud, 120, 120, 120);
  viewer->addPointCloud<pcl::PointXYZ>(cloud, gray, "cloud");
  viewer->setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");

  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red(
      edge_cloud, 255, 40, 40);
  viewer->addPointCloud<pcl::PointXYZ>(edge_cloud, red, "edge");
  viewer->setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "edge");

  pcl::PointXYZ min_pt, max_pt;
  pcl::getMinMax3D(*cloud, min_pt, max_pt);
  const float diag = std::sqrt(std::pow(max_pt.x - min_pt.x, 2) +
                               std::pow(max_pt.y - min_pt.y, 2) +
                               std::pow(max_pt.z - min_pt.z, 2));
  viewer->addCoordinateSystem(std::max(diag * 0.15f, 0.01f));
  viewer->initCameraParameters();
  viewer->resetCamera();

  std::cout << "可视化：灰=工作点云，红=边缘点云。关闭窗口或按 q 退出。"
            << std::endl;
  while (!viewer->wasStopped()) {
    viewer->spinOnce(100);
  }
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr
        << "用法: " << argv[0]
        << " <input.pcd|ply> [k邻域] [角度阈值(度)] [体素边长] [线程数]\n"
        << "示例: " << argv[0] << " cloud.pcd 20 90 0.005 8\n"
        << "  体素边长>0 时先降采样；线程数默认=硬件并发。\n";
    return 1;
  }

  const std::string input_path = argv[1];
  const int k_neighbors = (argc >= 3) ? std::atoi(argv[2]) : 30;
  const float angle_deg = (argc >= 4) ? static_cast<float>(std::atof(argv[3]))
                                      : 90.0f;
  const float voxel_leaf = (argc >= 5) ? static_cast<float>(std::atof(argv[4]))
                                       : 0.0f;
  const unsigned int hw = std::max(1u, std::thread::hardware_concurrency());
  const unsigned int num_threads =
      (argc >= 6) ? static_cast<unsigned int>(std::atoi(argv[5])) : hw;
  const float angle_rad = angle_deg * static_cast<float>(M_PI) / 180.0f;

  if (k_neighbors < 3) {
    std::cerr << "错误：k 邻域至少为 3。" << std::endl;
    return 1;
  }

  const auto t_all = Clock::now();

  // ---------- 1. 读取 ----------
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  std::cout << "读取点云: " << input_path << " ..." << std::endl;
  auto t0 = Clock::now();
  if (!loadCloud(input_path, cloud)) {
    return -1;
  }
  std::cout << "  原始点数: " << cloud->size()
            << "  (" << msSince(t0) << " ms)" << std::endl;

  // ---------- 2. 降采样（加速 A）----------
  t0 = Clock::now();
  auto work_cloud = downsampleVoxel(cloud, voxel_leaf);
  if (voxel_leaf > 0.0f) {
    std::cout << "VoxelGrid leaf=" << voxel_leaf
              << " → 工作点数: " << work_cloud->size()
              << "  (" << msSince(t0) << " ms)" << std::endl;
  } else {
    std::cout << "未降采样（体素边长=0）" << std::endl;
  }

  // ---------- 3. 建树一次并复用（加速 C）----------
  t0 = Clock::now();
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
      new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(work_cloud);
  std::cout << "KdTree 构建: " << msSince(t0) << " ms" << std::endl;

  // ---------- 4. 并行法向量（加速 B）----------
  std::cout << "估计法向量 (k=" << k_neighbors << ", threads=" << num_threads
            << ") ..." << std::endl;
  t0 = Clock::now();
  auto normals =
      estimateNormalsOMP(work_cloud, tree, k_neighbors, num_threads);
  std::cout << "  法向量: " << msSince(t0) << " ms" << std::endl;

  // ---------- 5. 并行边界估计 ----------
  std::cout << "提取边缘点 (角度=" << angle_deg << "°) ..." << std::endl;
  t0 = Clock::now();
  auto edge_cloud = extractBoundaryCloud(
      work_cloud, normals, tree, k_neighbors, angle_rad, num_threads);
  std::cout << "  边缘点数: " << edge_cloud->size() << " / "
            << work_cloud->size() << "  (" << msSince(t0) << " ms)"
            << std::endl;

  if (edge_cloud->empty()) {
    std::cerr << "警告：未检测到边缘点。可尝试减小角度阈值或调整 k / 体素。"
              << std::endl;
  }

  std::cout << "总耗时(不含可视化): " << msSince(t_all) << " ms" << std::endl;

  // ---------- 6. 保存 ----------
  const std::string out_path = "edge_cloud.pcd";
  pcl::io::savePCDFileBinary(out_path, *edge_cloud);
  std::cout << "边缘点云已保存: " << out_path << std::endl;

  // ---------- 7. 可视化 ----------
  visualize(work_cloud, edge_cloud);
  return 0;
}
