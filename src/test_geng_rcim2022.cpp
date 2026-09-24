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
    expect(det.planes().size() >= 2, "L-joint finds two plates");
    expect(det.seams().size() == 1, "L-joint has one weld");
    expect(!det.seamCloud()->empty(), "L-joint outputs a weld point cloud");
    expect(traj->size() >= 80, "L-joint trajectory is densely sampled");
    expect(meanDistToY(*traj) < 6.0f, "L-joint trajectory stays on the Y-axis seam");
    if (!det.seams().empty())
      expect(std::abs(det.seams()[0].length_mm - 200.0f) < 20.0f, "L-joint length ~200 mm");
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
