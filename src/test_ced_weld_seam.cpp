#include "ced_3d.h"
#include "ced_weld_seam.h"
#include "synthetic_weld_cloud.h"

#include <iostream>
#include <string>

namespace
{
int g_failures = 0;

void
configureWeldDetector (pcl::CEDWeldSeamDetector<pcl::PointXYZ, pcl::PointXYZ> &detector)
{
  detector.setRadiusSearch (0.05);
  detector.setCentroidThreshold (0.10);
  detector.setSupportRadius (0.06);
  detector.setNormalRadius (0.03);
  detector.setPlaneDistanceThreshold (0.008);
  detector.setSeamBandWidth (0.025);
  detector.setDihedralAngleRange (25.0, 155.0);
  detector.setMinNeighbors (8);
  detector.setMinPlaneInliers (8);
  detector.setMinPlaneInlierRatio (0.20);
  detector.setMinSeamClusterSize (10);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr
detectWeld (const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr seam (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::CEDWeldSeamDetector<pcl::PointXYZ, pcl::PointXYZ> detector;
  configureWeldDetector (detector);
  detector.setInputCloud (cloud);
  detector.compute (*seam);
  return seam;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr
detectCed3d (const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::CEDKeypoint3D<pcl::PointXYZ, pcl::PointXYZ> ced;
  ced.setRadiusSearch (0.05);
  ced.setNonMaxRadius (0.05);
  ced.setCentroidThreshold (0.20);
  ced.setMinNeighbors (5);
  ced.setInputCloud (cloud);
  ced.compute (*keypoints);
  return keypoints;
}

void
expect (bool cond, const std::string &message)
{
  if (cond)
  {
    std::cout << "  PASS  " << message << "\n";
    return;
  }
  std::cerr << "  FAIL  " << message << "\n";
  ++g_failures;
}

float
meanDistanceToYAxis (const pcl::PointCloud<pcl::PointXYZ> &cloud)
{
  if (cloud.empty ())
    return 0.0f;
  float sum = 0.0f;
  for (const auto &p : cloud)
    sum += weld_demo::distanceToYAxis (p);
  return sum / static_cast<float> (cloud.size ());
}

int
countNearYAxis (const pcl::PointCloud<pcl::PointXYZ> &cloud, float max_dist)
{
  int count = 0;
  for (const auto &p : cloud)
  {
    if (weld_demo::distanceToYAxis (p) <= max_dist)
      ++count;
  }
  return count;
}
}  // namespace

int
main ()
{
  std::cout << "CED weld-seam detector tests\n";

  {
    std::cout << "[L-joint] two perpendicular faces sharing the Y axis\n";
    auto cloud = weld_demo::makeLJoint ();
    auto seam = detectWeld (cloud);
    auto ced = detectCed3d (cloud);
    const float mean_dist = meanDistanceToYAxis (*seam);
    const int near_axis = countNearYAxis (*seam, 0.03f);
    std::cout << "  cloud=" << cloud->size ()
              << " ced3d=" << ced->size ()
              << " weld=" << seam->size ()
              << " mean_dist=" << mean_dist << "\n";
    expect (seam->size () >= 20, "L-joint yields a dense weld cloud");
    expect (mean_dist <= 0.02f, "L-joint weld points stay on the intersection");
    expect (near_axis * 10 >= static_cast<int> (seam->size ()) * 8,
            "at least 80% of L-joint weld points are near the intersection");
    const int ced_off_axis = static_cast<int> (ced->size ()) - countNearYAxis (*ced, 0.05f);
    expect (ced_off_axis >= 10,
            "raw CED-3D also marks isolated plate edges away from the weld");
  }

  {
    std::cout << "[T-joint] upright face meeting the interior of a base plate\n";
    auto cloud = weld_demo::makeTJoint ();
    auto seam = detectWeld (cloud);
    const float mean_dist = meanDistanceToYAxis (*seam);
    std::cout << "  cloud=" << cloud->size ()
              << " weld=" << seam->size ()
              << " mean_dist=" << mean_dist << "\n";
    expect (seam->size () >= 20, "T-joint yields a dense weld cloud");
    expect (mean_dist <= 0.02f, "T-joint weld points stay on the intersection");
  }

  {
    std::cout << "[single plane] only free edges, no second face\n";
    auto cloud = weld_demo::makeSinglePlane ();
    auto seam = detectWeld (cloud);
    auto ced = detectCed3d (cloud);
    std::cout << "  cloud=" << cloud->size ()
              << " ced3d=" << ced->size ()
              << " weld=" << seam->size () << "\n";
    expect (seam->empty (), "single plane must not produce a weld cloud");
    expect (!ced->empty (), "raw CED-3D still reports isolated plate edges");
  }

  {
    std::cout << "[disconnected planes] two coplanar plates with a gap\n";
    auto cloud = weld_demo::makeDisconnectedPlanes ();
    auto seam = detectWeld (cloud);
    std::cout << "  cloud=" << cloud->size () << " weld=" << seam->size () << "\n";
    expect (seam->empty (), "disconnected coplanar plates must not produce a weld cloud");
  }

  if (g_failures == 0)
  {
    std::cout << "All tests passed.\n";
    return 0;
  }
  std::cerr << g_failures << " test(s) failed.\n";
  return 1;
}
