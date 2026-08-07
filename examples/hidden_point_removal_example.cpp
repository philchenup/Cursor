#include "hidden_point_removal.h"

#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <cmath>
#include <iostream>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

int main(int argc, char** argv) {
  using PointT = pcl::PointXYZ;

  pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);

  if (argc >= 2) {
    if (pcl::io::loadPCDFile<PointT>(argv[1], *cloud) < 0) {
      std::cerr << "Failed to read PCD: " << argv[1] << '\n';
      return 1;
    }
  } else {
    // Fallback demo cloud: points on a sphere + a few occluded interior points.
    cloud->reserve(220);
    for (int i = 0; i < 200; ++i) {
      const double theta = 2.0 * M_PI * i / 200.0;
      const double phi = M_PI * ((i % 20) + 0.5) / 20.0;
      PointT p;
      p.x = static_cast<float>(std::sin(phi) * std::cos(theta));
      p.y = static_cast<float>(std::sin(phi) * std::sin(theta));
      p.z = static_cast<float>(std::cos(phi));
      cloud->push_back(p);
    }
    for (int i = 0; i < 20; ++i) {
      PointT p;
      p.x = 0.05f * static_cast<float>(i);
      p.y = 0.0f;
      p.z = 0.0f;
      cloud->push_back(p);
    }
    cloud->width = static_cast<uint32_t>(cloud->size());
    cloud->height = 1;
    cloud->is_dense = true;
  }

  Eigen::Vector4f min_pt, max_pt;
  pcl::getMinMax3D(*cloud, min_pt, max_pt);
  const double diameter = (max_pt.head<3>() - min_pt.head<3>()).norm();
  const Eigen::Vector3d camera(0.0, 0.0, diameter);
  const double radius = diameter * 100.0;

  std::cout << "input points: " << cloud->size() << '\n';
  std::cout << "camera: [" << camera.transpose() << "], radius: " << radius
            << '\n';

  const auto result =
      pcl_utils::HiddenPointRemoval<PointT>(cloud, camera, radius);
  auto visible =
      pcl_utils::ExtractVisiblePoints<PointT>(cloud, result.visible_indices);

  std::cout << "visible points: " << result.visible_indices.size() << '\n';
  std::cout << "visible triangles: " << result.mesh.polygons.size() << '\n';

  if (argc >= 3) {
    pcl::io::savePCDFileBinary(argv[2], *visible);
    std::cout << "wrote visible cloud to " << argv[2] << '\n';
  }

  return 0;
}
