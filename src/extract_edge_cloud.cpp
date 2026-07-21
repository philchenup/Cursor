/**
 * @file extract_edge_cloud.cpp
 * @brief 使用 PCL 读取点云并提取边缘点云
 *
 * 流程：
 *   1. 读取点云（PCD / PLY）
 *   2. 估计法向量
 *   3. 用 BoundaryEstimation 检测边界/边缘点
 *   4. 提取边缘点云并可视化、保存
 *
 * 用法：
 *   ./extract_edge_cloud <input.pcd|ply> [k邻域] [角度阈值(度)]
 *   默认：k=30，角度阈值=90°
 */

#include <cmath>
#include <iostream>
#include <string>

#include <pcl/common/common.h>
#include <pcl/features/boundary.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/visualization/pcl_visualizer.h>

namespace {

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

/**
 * 估计法向量（k 邻域）。
 * 边缘检测依赖法向量方向，法向量质量直接影响边缘提取效果。
 */
pcl::PointCloud<pcl::Normal>::Ptr estimateNormals(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    const pcl::search::KdTree<pcl::PointXYZ>::Ptr& tree,
    int k_neighbors) {
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud(cloud);
  ne.setSearchMethod(tree);
  ne.setKSearch(k_neighbors);
  ne.compute(*normals);
  return normals;
}

/**
 * 基于法向量夹角的边界估计，提取边缘点云。
 *
 * 原理：对每个点，考察其邻域投影到切平面后的角度分布；
 * 若存在大于 angle_threshold 的空隙，则判定为边界点。
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr extractBoundaryCloud(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    const pcl::PointCloud<pcl::Normal>::Ptr& normals,
    const pcl::search::KdTree<pcl::PointXYZ>::Ptr& tree,
    int k_neighbors,
    float angle_threshold_rad) {
  pcl::PointCloud<pcl::Boundary> boundaries;
  pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary> be;
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

  // 原始点云：灰色
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> gray(
      cloud, 120, 120, 120);
  viewer->addPointCloud<pcl::PointXYZ>(cloud, gray, "cloud");
  viewer->setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");

  // 边缘点云：红色高亮
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

  std::cout << "可视化：灰=原始点云，红=边缘点云。关闭窗口或按 q 退出。"
            << std::endl;
  while (!viewer->wasStopped()) {
    viewer->spinOnce(100);
  }
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "用法: " << argv[0]
              << " <input.pcd|ply> [k邻域] [角度阈值(度)]\n"
              << "示例: " << argv[0] << " cloud.pcd 30 90\n"
              << "  k邻域越大越平滑，角度阈值越小边缘越敏感。\n";
    return 1;
  }

  const std::string input_path = argv[1];
  const int k_neighbors = (argc >= 3) ? std::atoi(argv[2]) : 30;
  const float angle_deg = (argc >= 4) ? std::atof(argv[3]) : 90.0f;
  const float angle_rad = angle_deg * static_cast<float>(M_PI) / 180.0f;

  if (k_neighbors < 3) {
    std::cerr << "错误：k 邻域至少为 3。" << std::endl;
    return 1;
  }

  // ---------- 1. 读取点云 ----------
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  std::cout << "读取点云: " << input_path << " ..." << std::endl;
  if (!loadCloud(input_path, cloud)) {
    return -1;
  }
  std::cout << "  点数: " << cloud->size() << std::endl;

  // ---------- 2. 估计法向量 ----------
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
      new pcl::search::KdTree<pcl::PointXYZ>);
  std::cout << "估计法向量 (k=" << k_neighbors << ") ..." << std::endl;
  auto normals = estimateNormals(cloud, tree, k_neighbors);

  // ---------- 3. 提取边缘点 ----------
  std::cout << "提取边缘点 (角度阈值=" << angle_deg << "°) ..." << std::endl;
  auto edge_cloud =
      extractBoundaryCloud(cloud, normals, tree, k_neighbors, angle_rad);
  std::cout << "  边缘点数: " << edge_cloud->size() << " / " << cloud->size()
            << std::endl;

  if (edge_cloud->empty()) {
    std::cerr << "警告：未检测到边缘点。可尝试减小角度阈值或调整 k 邻域。"
              << std::endl;
  }

  // ---------- 4. 保存 ----------
  const std::string out_path = "edge_cloud.pcd";
  pcl::io::savePCDFileBinary(out_path, *edge_cloud);
  std::cout << "边缘点云已保存: " << out_path << std::endl;

  // ---------- 5. 可视化 ----------
  visualize(cloud, edge_cloud);
  return 0;
}
