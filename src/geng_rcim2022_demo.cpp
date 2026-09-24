#include "geng_demo_models.h"
#include "geng_rcim2022.h"

#include <pcl/io/pcd_io.h>

#include <iostream>
#include <string>

int
main(int argc, char** argv)
{
  if (argc < 2)
  {
    std::cout << "Usage:\n"
              << "  " << argv[0] << " --demo l|t|box [seam.pcd] [traj.pcd]\n"
              << "  " << argv[0] << " <input.pcd> [seam.pcd] [traj.pcd]\n"
              << "Coordinates and all length parameters are in mm.\n";
    return 0;
  }

  const std::string a1 = argv[1];
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  std::string seam_path = "geng_seam_mm.pcd";
  std::string traj_path = "geng_traj_mm.pcd";
  int path_arg = 2;

  if (a1 == "--demo")
  {
    const std::string model = (argc >= 3) ? argv[2] : "l";
    path_arg = 3;
    if (model == "t")
      cloud = geng_demo::makeTJointMm();
    else if (model == "box")
      cloud = geng_demo::makeBoxCornerMm();
    else
      cloud = geng_demo::makeLJointMm();
    std::cout << "Demo model '" << model << "': " << cloud->size() << " points (mm)\n";
  }
  else if (pcl::io::loadPCDFile(a1, *cloud) < 0)
  {
    std::cerr << "Failed to load " << a1 << "\n";
    return 1;
  }
  else
  {
    std::cout << "Loaded " << cloud->size() << " points from " << a1 << "\n";
  }

  if (argc > path_arg)
    seam_path = argv[path_arg];
  if (argc > path_arg + 1)
    traj_path = argv[path_arg + 1];

  GengRcim2022 detector;
  detector.setInputCloud(cloud);
  const bool ok = detector.compute();

  auto seam = detector.seamCloud();
  auto traj = detector.trajectoryCloud();
  const auto segmented = detector.segmentedCloud();
  std::cout << "Planes: " << detector.planes().size()
            << "  segmented: " << (segmented ? segmented->size() : 0)
            << "  seams: " << detector.seams().size()
            << "  seam points: " << seam->size()
            << "  trajectory points: " << traj->size() << "\n";
  for (std::size_t i = 0; i < detector.seams().size(); ++i)
  {
    const auto& s = detector.seams()[i];
    std::cout << "  seam " << i << " length=" << s.length_mm << " mm"
              << " start=(" << s.start.x() << "," << s.start.y() << "," << s.start.z() << ")"
              << " end=(" << s.end.x() << "," << s.end.y() << "," << s.end.z() << ")\n";
  }

  if (pcl::io::savePCDFileBinary(seam_path, *seam) < 0 ||
      pcl::io::savePCDFileBinary(traj_path, *traj) < 0)
  {
    std::cerr << "Failed to write output clouds\n";
    return 1;
  }
  std::cout << "Wrote weld cloud (mm) to " << seam_path << "\n";
  std::cout << "Wrote trajectory (mm, normal=torch Z) to " << traj_path << "\n";
  return ok ? 0 : 2;
}
