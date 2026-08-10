// Stub that pulls FLANN/KdTree headers the same way many pcd_utils.cpp files do.
// Used only to document the LZ4 link requirement; replace with your real sources.
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>

namespace pcd_utils_lz4_link_stub {

void touch_kdtree(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud) {
  pcl::search::KdTree<pcl::PointXYZ> tree;
  tree.setInputCloud(cloud);
}

}  // namespace pcd_utils_lz4_link_stub
