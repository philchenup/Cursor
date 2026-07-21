/**
 * @file stl_to_pointcloud.cpp
 * @brief 使用 PCL 读取 STL 模型，采样为点云并可视化
 *
 * 功能：
 *   1. 读取 STL 三角网格
 *   2. 在网格表面均匀采样生成点云
 *   3. 按 Z 轴着色可视化（黑底 + 坐标轴）
 *
 * 用法：
 *   ./stl_to_pointcloud <model.stl> [采样点数]
 *   默认采样点数：50000
 */

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <random>
#include <string>
#include <vector>

#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

namespace {

/** 三角形面积（海伦公式，向量叉积取模 / 2） */
float triangleArea(const pcl::PointXYZ& a,
                   const pcl::PointXYZ& b,
                   const pcl::PointXYZ& c) {
  Eigen::Vector3f ab(b.x - a.x, b.y - a.y, b.z - a.z);
  Eigen::Vector3f ac(c.x - a.x, c.y - a.y, c.z - a.z);
  return 0.5f * ab.cross(ac).norm();
}

/**
 * 在三角网格表面按面积加权随机采样，生成点云。
 * 比仅取顶点更能还原 STL 曲面外形（如图所示的稠密点云效果）。
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr sampleMeshSurface(
    const pcl::PolygonMesh& mesh,
    std::size_t num_samples) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr vertices(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(mesh.cloud, *vertices);

  if (vertices->empty() || mesh.polygons.empty()) {
    std::cerr << "错误：网格为空，无法采样。" << std::endl;
    return vertices;
  }

  // 计算每个三角形面积，构建累积分布用于按面积采样
  std::vector<float> areas;
  areas.reserve(mesh.polygons.size());
  float total_area = 0.0f;

  for (const auto& poly : mesh.polygons) {
    if (poly.vertices.size() < 3) {
      areas.push_back(0.0f);
      continue;
    }
    const auto& a = vertices->points[poly.vertices[0]];
    const auto& b = vertices->points[poly.vertices[1]];
    const auto& c = vertices->points[poly.vertices[2]];
    float area = triangleArea(a, b, c);
    areas.push_back(area);
    total_area += area;
  }

  if (total_area <= 0.0f) {
    std::cerr << "错误：网格总面积为 0。" << std::endl;
    return vertices;
  }

  std::vector<float> cdf(areas.size());
  float accum = 0.0f;
  for (std::size_t i = 0; i < areas.size(); ++i) {
    accum += areas[i] / total_area;
    cdf[i] = accum;
  }
  cdf.back() = 1.0f;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  cloud->reserve(num_samples);

  std::mt19937 rng(42);
  std::uniform_real_distribution<float> uniform(0.0f, 1.0f);

  for (std::size_t i = 0; i < num_samples; ++i) {
    // 按面积选中三角形
    float r = uniform(rng);
    auto it = std::lower_bound(cdf.begin(), cdf.end(), r);
    std::size_t tri_idx =
        static_cast<std::size_t>(std::distance(cdf.begin(), it));
    if (tri_idx >= mesh.polygons.size()) {
      tri_idx = mesh.polygons.size() - 1;
    }

    const auto& poly = mesh.polygons[tri_idx];
    if (poly.vertices.size() < 3) {
      continue;
    }

    const auto& A = vertices->points[poly.vertices[0]];
    const auto& B = vertices->points[poly.vertices[1]];
    const auto& C = vertices->points[poly.vertices[2]];

    // 在三角形内均匀采样（重心坐标）
    float u = uniform(rng);
    float v = uniform(rng);
    if (u + v > 1.0f) {
      u = 1.0f - u;
      v = 1.0f - v;
    }
    float w = 1.0f - u - v;

    pcl::PointXYZ p;
    p.x = w * A.x + u * B.x + v * C.x;
    p.y = w * A.y + u * B.y + v * C.y;
    p.z = w * A.z + u * B.z + v * C.z;
    cloud->push_back(p);
  }

  cloud->width = static_cast<uint32_t>(cloud->size());
  cloud->height = 1;
  cloud->is_dense = true;
  return cloud;
}

void visualize(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
  pcl::visualization::PCLVisualizer::Ptr viewer(
      new pcl::visualization::PCLVisualizer("STL Point Cloud"));
  viewer->setBackgroundColor(0.0, 0.0, 0.0);

  // 按 Z 轴字段着色：低处蓝紫 → 高处黄橙（与图示一致）
  pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ>
      color_handler(cloud, "z");
  viewer->addPointCloud<pcl::PointXYZ>(cloud, color_handler, "cloud");
  viewer->setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");

  // 坐标轴：红=X，绿=Y，蓝=Z
  pcl::PointXYZ min_pt, max_pt;
  pcl::getMinMax3D(*cloud, min_pt, max_pt);
  float diag = std::sqrt(std::pow(max_pt.x - min_pt.x, 2) +
                         std::pow(max_pt.y - min_pt.y, 2) +
                         std::pow(max_pt.z - min_pt.z, 2));
  float axis_len = std::max(diag * 0.15f, 0.01f);
  viewer->addCoordinateSystem(axis_len);

  viewer->initCameraParameters();
  viewer->resetCamera();

  std::cout << "可视化窗口已打开，关闭窗口或按 q 退出。" << std::endl;
  while (!viewer->wasStopped()) {
    viewer->spinOnce(100);
  }
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "用法: " << argv[0] << " <model.stl> [采样点数]\n"
              << "示例: " << argv[0] << " models/bunny.stl 50000\n";
    return 1;
  }

  const std::string stl_path = argv[1];
  const std::size_t num_samples =
      (argc >= 3) ? static_cast<std::size_t>(std::atoi(argv[2])) : 50000;

  // ---------- 1. 读取 STL ----------
  pcl::PolygonMesh mesh;
  std::cout << "正在读取 STL: " << stl_path << " ..." << std::endl;
  if (pcl::io::loadPolygonFileSTL(stl_path, mesh) == -1) {
    std::cerr << "无法读取 STL 文件: " << stl_path << std::endl;
    return -1;
  }
  std::cout << "  多边形数: " << mesh.polygons.size() << std::endl;

  // ---------- 2. 表面采样 → 点云 ----------
  std::cout << "正在采样 " << num_samples << " 个点 ..." << std::endl;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud =
      sampleMeshSurface(mesh, num_samples);
  std::cout << "  点云点数: " << cloud->size() << std::endl;

  // 可选：保存为 PCD，便于后续处理
  const std::string pcd_path = "output_cloud.pcd";
  pcl::io::savePCDFileBinary(pcd_path, *cloud);
  std::cout << "点云已保存: " << pcd_path << std::endl;

  // ---------- 3. 可视化 ----------
  visualize(cloud);
  return 0;
}
