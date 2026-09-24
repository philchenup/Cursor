#ifndef GENG_DEMO_MODELS_H_
#define GENG_DEMO_MODELS_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

/** 耿玉森 RCIM 2022 三类中厚板结构件的毫米单位合成点云。 */
namespace geng_demo
{
inline void
appendXY(pcl::PointCloud<pcl::PointXYZ>& cloud,
         float x0, float x1, float y0, float y1, float z, float s)
{
  for (float x = x0; x <= x1 + 0.5f * s; x += s)
    for (float y = y0; y <= y1 + 0.5f * s; y += s)
      cloud.push_back(pcl::PointXYZ(x, y, z));
}

inline void
appendZY(pcl::PointCloud<pcl::PointXYZ>& cloud,
         float x, float y0, float y1, float z0, float z1, float s)
{
  for (float z = z0; z <= z1 + 0.5f * s; z += s)
    for (float y = y0; y <= y1 + 0.5f * s; y += s)
      cloud.push_back(pcl::PointXYZ(x, y, z));
}

inline void
appendZX(pcl::PointCloud<pcl::PointXYZ>& cloud,
         float y, float x0, float x1, float z0, float z1, float s)
{
  for (float x = x0; x <= x1 + 0.5f * s; x += s)
    for (float z = z0; z <= z1 + 0.5f * s; z += s)
      cloud.push_back(pcl::PointXYZ(x, y, z));
}

inline pcl::PointCloud<pcl::PointXYZ>::Ptr
finish(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
  cloud->width = static_cast<std::uint32_t>(cloud->size());
  cloud->height = 1;
  cloud->is_dense = true;
  return cloud;
}

/** 模型 I：L 形角接，焊缝沿 Y 轴，单位 mm。 */
inline pcl::PointCloud<pcl::PointXYZ>::Ptr
makeLJointMm(float size = 200.0f, float spacing = 4.0f)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  appendXY(*cloud, 0.0f, size, 0.0f, size, 0.0f, spacing);
  appendZY(*cloud, 0.0f, 0.0f, size, spacing, size, spacing);
  return finish(cloud);
}

/** 模型 II：T 形角接，焊缝沿 Y 轴，单位 mm。 */
inline pcl::PointCloud<pcl::PointXYZ>::Ptr
makeTJointMm(float size = 200.0f, float spacing = 4.0f)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  appendXY(*cloud, -size, size, 0.0f, size, 0.0f, spacing);
  appendZY(*cloud, 0.0f, 0.0f, size, spacing, size, spacing);
  return finish(cloud);
}

/** 模型 III：箱角三面相交，三条焊缝，单位 mm。 */
inline pcl::PointCloud<pcl::PointXYZ>::Ptr
makeBoxCornerMm(float size = 200.0f, float spacing = 4.0f)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  appendXY(*cloud, 0.0f, size, 0.0f, size, 0.0f, spacing);
  appendZY(*cloud, 0.0f, 0.0f, size, spacing, size, spacing);
  appendZX(*cloud, 0.0f, spacing, size, spacing, size, spacing);
  return finish(cloud);
}
}  // namespace geng_demo

#endif  // GENG_DEMO_MODELS_H_
