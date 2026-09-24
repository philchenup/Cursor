#ifndef SYNTHETIC_WELD_CLOUD_H_
#define SYNTHETIC_WELD_CLOUD_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cmath>

namespace weld_demo
{
  inline void
  appendPlaneXY (pcl::PointCloud<pcl::PointXYZ> &cloud,
                 float x0, float x1,
                 float y0, float y1,
                 float z,
                 float spacing)
  {
    for (float x = x0; x <= x1 + 0.5f * spacing; x += spacing)
    {
      for (float y = y0; y <= y1 + 0.5f * spacing; y += spacing)
      {
        pcl::PointXYZ p;
        p.x = x;
        p.y = y;
        p.z = z;
        cloud.push_back (p);
      }
    }
  }

  inline void
  appendPlaneZY (pcl::PointCloud<pcl::PointXYZ> &cloud,
                 float x,
                 float y0, float y1,
                 float z0, float z1,
                 float spacing)
  {
    for (float z = z0; z <= z1 + 0.5f * spacing; z += spacing)
    {
      for (float y = y0; y <= y1 + 0.5f * spacing; y += spacing)
      {
        pcl::PointXYZ p;
        p.x = x;
        p.y = y;
        p.z = z;
        cloud.push_back (p);
      }
    }
  }

  /** 单块矩形平板：只有自由边，没有两面交线。 */
  inline pcl::PointCloud<pcl::PointXYZ>::Ptr
  makeSinglePlane (float spacing = 0.015f)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    appendPlaneXY (*cloud, 0.0f, 1.0f, 0.0f, 1.0f, 0.0f, spacing);
    cloud->width = static_cast<std::uint32_t> (cloud->size ());
    cloud->height = 1;
    cloud->is_dense = true;
    return cloud;
  }

  /** L 形对接：z=0 与 x=0 两面沿 y 轴相交，交线即焊缝。 */
  inline pcl::PointCloud<pcl::PointXYZ>::Ptr
  makeLJoint (float spacing = 0.015f)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    appendPlaneXY (*cloud, 0.0f, 1.0f, 0.0f, 1.0f, 0.0f, spacing);
    appendPlaneZY (*cloud, 0.0f, 0.0f, 1.0f, spacing, 1.0f, spacing);
    cloud->width = static_cast<std::uint32_t> (cloud->size ());
    cloud->height = 1;
    cloud->is_dense = true;
    return cloud;
  }

  /** T 形接头：底板跨越 x∈[-1,1]，立板在 x=0。 */
  inline pcl::PointCloud<pcl::PointXYZ>::Ptr
  makeTJoint (float spacing = 0.015f)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    appendPlaneXY (*cloud, -1.0f, 1.0f, 0.0f, 1.0f, 0.0f, spacing);
    appendPlaneZY (*cloud, 0.0f, 0.0f, 1.0f, spacing, 1.0f, spacing);
    cloud->width = static_cast<std::uint32_t> (cloud->size ());
    cloud->height = 1;
    cloud->is_dense = true;
    return cloud;
  }

  /** 两块分离的共面平板：没有相交焊缝。 */
  inline pcl::PointCloud<pcl::PointXYZ>::Ptr
  makeDisconnectedPlanes (float spacing = 0.015f)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    appendPlaneXY (*cloud, 0.0f, 1.0f, 0.0f, 1.0f, 0.0f, spacing);
    appendPlaneXY (*cloud, 2.0f, 3.0f, 0.0f, 1.0f, 0.0f, spacing);
    cloud->width = static_cast<std::uint32_t> (cloud->size ());
    cloud->height = 1;
    cloud->is_dense = true;
    return cloud;
  }

  inline float
  distanceToYAxis (const pcl::PointXYZ &p)
  {
    return std::sqrt (p.x * p.x + p.z * p.z);
  }
}  // namespace weld_demo

#endif  // SYNTHETIC_WELD_CLOUD_H_
