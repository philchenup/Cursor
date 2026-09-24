#include "geng_demo_models.h"
#include "geng_rcim2022.h"

#include <cmath>
#include <iostream>
#include <string>

namespace
{
int g_failures = 0;

void
expect(bool cond, const std::string& msg)
{
  if (cond)
    std::cout << "  PASS  " << msg << "\n";
  else
  {
    std::cerr << "  FAIL  " << msg << "\n";
    ++g_failures;
  }
}

float
meanDistToY(const pcl::PointCloud<pcl::PointNormal>& traj)
{
  if (traj.empty())
    return 1e9f;
  float s = 0.0f;
  for (const auto& p : traj)
    s += std::sqrt(p.x * p.x + p.z * p.z);
  return s / static_cast<float>(traj.size());
}
}  // namespace

int
main()
{
  std::cout << "Geng RCIM 2022 tests (mm)\n";

  {
    std::cout << "[L-joint mm]\n";
    GengRcim2022 det;
    det.setInputCloud(geng_demo::makeLJointMm());
    det.compute();
    const auto traj = det.trajectoryCloud();
    std::cout << "  planes=" << det.planes().size()
              << " seams=" << det.seams().size()
              << " seam_pts=" << det.seamCloud()->size()
              << " traj=" << traj->size() << "\n";
    expect(det.segmentedCloud() && !det.segmentedCloud()->empty(),
           "L-joint builds a segmented-face cloud");
    expect(det.planes().size() >= 2, "L-joint finds two plates");
    expect(det.seams().size() == 1, "L-joint has one weld");
    expect(!det.seamCloud()->empty(), "L-joint outputs a weld point cloud");
    expect(traj->size() >= 80, "L-joint trajectory is densely sampled");
    expect(meanDistToY(*traj) < 6.0f, "L-joint trajectory stays on the Y-axis seam");
    if (!det.seams().empty())
    {
      expect(std::abs(det.seams()[0].length_mm - 200.0f) < 20.0f, "L-joint uses full overlapping support");
      const auto& s = det.seams()[0];
      const Eigen::Vector3f dir = (s.end - s.start).normalized();
      const Eigen::Vector3f z0(traj->front().normal_x, traj->front().normal_y, traj->front().normal_z);
      const Eigen::Vector3f z1(traj->back().normal_x, traj->back().normal_y, traj->back().normal_z);
      const float a0 = std::acos(std::min(1.0f, std::max(-1.0f, z0.normalized().dot(s.torch_z)))) *
                       180.0f / 3.14159265f;
      const float a1 = std::acos(std::min(1.0f, std::max(-1.0f, z1.normalized().dot(s.torch_z)))) *
                       180.0f / 3.14159265f;
      expect(std::abs(a0 - 45.0f) < 2.0f && z0.dot(dir) > 0.0f,
             "start torch tilts 45 deg into the seam");
      expect(std::abs(a1 - 45.0f) < 2.0f && z1.dot(-dir) > 0.0f,
             "end torch tilts 45 deg into the seam");
    }
  }

  {
    std::cout << "[T-joint mm]\n";
    GengRcim2022 det;
    det.setInputCloud(geng_demo::makeTJointMm());
    det.compute();
    const auto traj = det.trajectoryCloud();
    std::cout << "  planes=" << det.planes().size()
              << " seams=" << det.seams().size()
              << " traj=" << traj->size() << "\n";
    expect(det.seams().size() == 1, "T-joint has one weld");
    expect(meanDistToY(*traj) < 6.0f, "T-joint trajectory stays on the Y-axis seam");
  }

  {
    std::cout << "[box corner mm]\n";
    GengRcim2022 det;
    det.setInputCloud(geng_demo::makeBoxCornerMm());
    det.compute();
    std::cout << "  planes=" << det.planes().size()
              << " seams=" << det.seams().size()
              << " traj=" << det.trajectoryCloud()->size() << "\n";
    expect(det.planes().size() >= 3, "box corner finds three plates");
    expect(det.seams().size() == 3, "box corner has three welds");
    expect(det.trajectoryCloud()->size() >= 200, "box corner outputs three trajectories");
    int reach_corner = 0;
    for (const auto& s : det.seams())
    {
      const float d = std::min(s.start.norm(), s.end.norm());
      if (d < 8.0f)
        ++reach_corner;
    }
    expect(reach_corner == 3, "each box-corner weld reaches the triple point");
  }

  {
    std::cout << "[reject coplanar island]\n";
    auto cloud = geng_demo::makeLJointMm();
    geng_demo::appendXY(*cloud, 400.0f, 440.0f, 0.0f, 40.0f, 0.0f, 4.0f);
    cloud->width = static_cast<std::uint32_t>(cloud->size());
    cloud->height = 1;
    GengRcim2022 det;
    det.setInputCloud(cloud);
    det.compute();
    bool leaked = false;
    for (const auto& plane : det.planes())
    {
      for (const auto& p : plane.points)
      {
        if (p.x > 300.0f)
          leaked = true;
      }
    }
    expect(!leaked, "distant coplanar island is stripped from the main face");
    expect(det.seams().size() == 1, "L-joint weld still found after plane cleanup");
  }

  {
    std::cout << "[single plate mm]\n";
    pcl::PointCloud<pcl::PointXYZ>::Ptr plate(new pcl::PointCloud<pcl::PointXYZ>);
    geng_demo::appendXY(*plate, 0.0f, 200.0f, 0.0f, 200.0f, 0.0f, 4.0f);
    plate->width = static_cast<std::uint32_t>(plate->size());
    plate->height = 1;
    GengRcim2022 det;
    det.setInputCloud(plate);
    det.compute();
    expect(det.seams().empty(), "single plate has no weld trajectory");
  }

  if (g_failures == 0)
  {
    std::cout << "All tests passed.\n";
    return 0;
  }
  std::cerr << g_failures << " test(s) failed.\n";
  return 1;
}
