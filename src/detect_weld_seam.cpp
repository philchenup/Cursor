#include "ced_3d.h"
#include "ced_weld_seam.h"
#include "synthetic_weld_cloud.h"

#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <cstdlib>
#include <iostream>
#include <string>

namespace
{
struct Options
{
  std::string input_pcd;
  std::string output_pcd = "weld_seam.pcd";
  std::string ced_pcd;
  std::string demo_pcd;
  double radius = 0.05;
  double centroid = 0.10;
  double support_radius = 0.0;
  double voxel = 0.0;
  bool run_demo = false;
  bool radius_set = false;
  bool unit_mm = false;
};

void
printUsage (const char *exe)
{
  std::cout
      << "Usage:\n"
      << "  " << exe << " <input.pcd> [output_seam.pcd] [options]\n"
      << "  " << exe << " --demo [output_seam.pcd] [options]\n"
      << "\nOptions:\n"
      << "  --unit mm|m            cloud coordinate unit; mm changes default radius to 50\n"
      << "  --radius <value>       CED neighborhood radius in the same unit as the cloud\n"
      << "                         (default 0.05 m, or 50 mm with --unit mm)\n"
      << "  --centroid <ratio>     CED saliency threshold in [0,1] (default 0.10)\n"
      << "  --support-radius <v>   two-plane analysis radius (default 1.3*radius)\n"
      << "  --voxel <value>        optional voxel downsample leaf size, same unit as cloud\n"
      << "  --save-ced <file.pcd>  also write raw CED-3D keypoints for comparison\n"
      << "  --save-demo <file.pcd> write the generated demo cloud\n";
}

bool
parseOptions (int argc, char **argv, Options &opt)
{
  if (argc < 2)
    return false;

  int positional = 0;
  for (int i = 1; i < argc; ++i)
  {
    const std::string arg = argv[i];
    auto needValue = [&] (double &dst) -> bool {
      if (i + 1 >= argc)
        return false;
      dst = std::atof (argv[++i]);
      return true;
    };

    if (arg == "--demo")
    {
      opt.run_demo = true;
    }
    else if (arg == "--unit")
    {
      if (i + 1 >= argc)
        return false;
      const std::string unit = argv[++i];
      if (unit == "mm")
        opt.unit_mm = true;
      else if (unit == "m")
        opt.unit_mm = false;
      else
      {
        std::cerr << "Unknown unit: " << unit << " (use mm or m)\n";
        return false;
      }
    }
    else if (arg == "--radius")
    {
      if (!needValue (opt.radius))
        return false;
      opt.radius_set = true;
    }
    else if (arg == "--centroid")
    {
      if (!needValue (opt.centroid))
        return false;
    }
    else if (arg == "--support-radius")
    {
      if (!needValue (opt.support_radius))
        return false;
    }
    else if (arg == "--voxel")
    {
      if (!needValue (opt.voxel))
        return false;
    }
    else if (arg == "--save-ced")
    {
      if (i + 1 >= argc)
        return false;
      opt.ced_pcd = argv[++i];
    }
    else if (arg == "--save-demo")
    {
      if (i + 1 >= argc)
        return false;
      opt.demo_pcd = argv[++i];
    }
    else if (arg == "-h" || arg == "--help")
    {
      return false;
    }
    else if (!arg.empty () && arg[0] == '-')
    {
      std::cerr << "Unknown option: " << arg << "\n";
      return false;
    }
    else if (positional == 0)
    {
      if (opt.run_demo)
        opt.output_pcd = arg;
      else
        opt.input_pcd = arg;
      ++positional;
    }
    else if (positional == 1 && !opt.run_demo)
    {
      opt.output_pcd = arg;
      ++positional;
    }
    else
    {
      return false;
    }
  }

  if (!opt.run_demo && opt.input_pcd.empty ())
    return false;
  if (opt.unit_mm && !opt.radius_set && !opt.run_demo)
    opt.radius = 50.0;
  return true;
}

void
removeNaN (pcl::PointCloud<pcl::PointXYZ> &cloud)
{
  std::vector<int> index;
  pcl::removeNaNFromPointCloud (cloud, cloud, index);
}

void
downsample (pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, float leaf)
{
  if (leaf <= 0.0f || !cloud)
    return;
  pcl::VoxelGrid<pcl::PointXYZ> grid;
  grid.setLeafSize (leaf, leaf, leaf);
  grid.setInputCloud (cloud);
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered (new pcl::PointCloud<pcl::PointXYZ>);
  grid.filter (*filtered);
  cloud = filtered;
}
}  // namespace

int
main (int argc, char **argv)
{
  Options opt;
  if (!parseOptions (argc, argv, opt))
  {
    printUsage (argv[0]);
    return (argc < 2) ? 0 : 1;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  if (opt.run_demo)
  {
    cloud = weld_demo::makeLJoint ();
    std::cout << "Generated L-joint demo cloud with " << cloud->size () << " points\n";
    if (!opt.demo_pcd.empty () && pcl::io::savePCDFileBinary (opt.demo_pcd, *cloud) == 0)
      std::cout << "Wrote demo cloud to " << opt.demo_pcd << "\n";
  }
  else if (pcl::io::loadPCDFile (opt.input_pcd, *cloud) < 0)
  {
    std::cerr << "Failed to load " << opt.input_pcd << "\n";
    return 1;
  }
  else
  {
    std::cout << "Loaded " << cloud->size () << " points from " << opt.input_pcd << "\n";
  }

  removeNaN (*cloud);
  downsample (cloud, static_cast<float> (opt.voxel));
  std::cout << "Using " << cloud->size () << " points after cleanup\n";

  pcl::PointCloud<pcl::PointXYZ>::Ptr seam (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::CEDWeldSeamDetector<pcl::PointXYZ, pcl::PointXYZ> detector;
  detector.setRadiusSearch (opt.radius);
  detector.setCentroidThreshold (opt.centroid);
  if (opt.support_radius > 0.0)
    detector.setSupportRadius (opt.support_radius);
  detector.setInputCloud (cloud);
  detector.compute (*seam);

  std::cout << "Search radius: " << opt.radius
            << (opt.unit_mm ? " mm" : " (cloud units)") << "\n";
  std::cout << "Weld seam points: " << seam->size ()
            << " (two-surface junctions only)\n";

  if (!opt.ced_pcd.empty ())
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr ced_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::CEDKeypoint3D<pcl::PointXYZ, pcl::PointXYZ> ced;
    ced.setRadiusSearch (opt.radius);
    ced.setNonMaxRadius (opt.radius);
    ced.setCentroidThreshold (std::max (0.20, opt.centroid));
    ced.setMinNeighbors (5);
    ced.setInputCloud (cloud);
    ced.compute (*ced_cloud);
    std::cout << "Raw CED-3D keypoints: " << ced_cloud->size ()
              << " (includes isolated edges)\n";
    if (pcl::io::savePCDFileBinary (opt.ced_pcd, *ced_cloud) == 0)
      std::cout << "Wrote CED-3D keypoints to " << opt.ced_pcd << "\n";
  }

  if (pcl::io::savePCDFileBinary (opt.output_pcd, *seam) < 0)
  {
    std::cerr << "Failed to write " << opt.output_pcd << "\n";
    return 1;
  }
  std::cout << "Wrote weld seam to " << opt.output_pcd << "\n";
  return 0;
}
