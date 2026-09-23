#include <pcl/ModelCoefficients.h>
#include <pcl/PointIndices.h>
#include <pcl/common/io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

// Point cloud unit: millimetre (mm).
using Cloud = pcl::PointCloud<pcl::PointXYZ>;
using CloudRGB = pcl::PointCloud<pcl::PointXYZRGB>;

struct Options {
  std::string input;
  std::string prefix = "sac_plane";
  int max_planes = 3;
  int max_iterations = 1000;
  double distance_threshold = 1.2;  // mm
  double voxel_leaf = 1.0;          // mm
  int min_inliers = 200;
  int sor_mean_k = 20;
  double sor_stddev = 1.8;
  double z_min = -1e9;  // mm
  double z_max = 1e9;   // mm
  bool show = false;
};

bool loadCloud(const std::string& path, Cloud::Ptr cloud) {
  const auto ext = path.substr(path.find_last_of('.') + 1);
  if (ext == "pcd" || ext == "PCD") {
    return pcl::io::loadPCDFile(path, *cloud) >= 0;
  }
  return pcl::io::loadPLYFile(path, *cloud) >= 0;
}

Cloud::Ptr preprocess(const Cloud::ConstPtr& in, const Options& opt) {
  Cloud::Ptr z_filtered(new Cloud);
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(in);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(static_cast<float>(opt.z_min), static_cast<float>(opt.z_max));
  pass.filter(*z_filtered);

  Cloud::Ptr down(new Cloud);
  if (opt.voxel_leaf > 0.0) {
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(z_filtered);
    vg.setLeafSize(static_cast<float>(opt.voxel_leaf),
                   static_cast<float>(opt.voxel_leaf),
                   static_cast<float>(opt.voxel_leaf));
    vg.filter(*down);
  } else {
    *down = *z_filtered;
  }

  Cloud::Ptr clean(new Cloud);
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud(down);
  sor.setMeanK(opt.sor_mean_k);
  sor.setStddevMulThresh(opt.sor_stddev);
  sor.filter(*clean);
  return clean;
}

bool extractOnePlane(const Cloud::ConstPtr& cloud,
                     const Options& opt,
                     pcl::ModelCoefficients& coeff,
                     pcl::PointIndices& inliers) {
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(opt.max_iterations);
  seg.setDistanceThreshold(opt.distance_threshold);  // mm
  seg.setInputCloud(cloud);
  seg.segment(inliers, coeff);
  return !inliers.indices.empty() &&
         static_cast<int>(inliers.indices.size()) >= opt.min_inliers;
}

void splitCloud(const Cloud::ConstPtr& cloud,
                const pcl::PointIndices& inliers,
                Cloud::Ptr inlier_cloud,
                Cloud::Ptr outlier_cloud) {
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud(cloud);
  pcl::PointIndices::Ptr inliers_ptr(new pcl::PointIndices(inliers));
  extract.setIndices(inliers_ptr);
  extract.setNegative(false);
  extract.filter(*inlier_cloud);
  extract.setNegative(true);
  extract.filter(*outlier_cloud);
}

// plane0 red, plane1 green, plane2 blue, then yellow / magenta; rest gray
const std::uint8_t kPlaneColors[][3] = {
    {230, 40, 40}, {40, 190, 50}, {50, 110, 240}, {230, 190, 40}, {180, 70, 210}};
const std::uint8_t kRestColor[3] = {150, 150, 150};

void appendColored(const Cloud::ConstPtr& xyz, std::uint8_t r, std::uint8_t g, std::uint8_t b, CloudRGB& out) {
  out.reserve(out.size() + xyz->size());
  for (const auto& p : xyz->points) {
    pcl::PointXYZRGB q;
    q.x = p.x;
    q.y = p.y;
    q.z = p.z;
    q.r = r;
    q.g = g;
    q.b = b;
    out.push_back(q);
  }
}

CloudRGB::Ptr toColoredCloud(const Cloud::ConstPtr& xyz, std::uint8_t r, std::uint8_t g, std::uint8_t b) {
  CloudRGB::Ptr out(new CloudRGB);
  out->header = xyz->header;
  appendColored(xyz, r, g, b, *out);
  return out;
}

void showColoredResult(const std::vector<Cloud::Ptr>& plane_clouds,
                       const Cloud::ConstPtr& rest,
                       const Eigen::Vector3d* corner) {
  pcl::visualization::PCLVisualizer vis("SAC_RANSAC colored planes (mm)");
  vis.setBackgroundColor(0.08, 0.08, 0.10);
  vis.addCoordinateSystem(50.0);

  for (std::size_t i = 0; i < plane_clouds.size(); ++i) {
    const auto& rgb = kPlaneColors[i % 5];
    CloudRGB::Ptr colored = toColoredCloud(plane_clouds[i], rgb[0], rgb[1], rgb[2]);
    const std::string id = "plane" + std::to_string(i);
    vis.addPointCloud<pcl::PointXYZRGB>(colored, id);
    vis.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, id);
    vis.addText("plane" + std::to_string(i), 12, static_cast<int>(80 - 18 * i), 14, rgb[0] / 255.0,
                rgb[1] / 255.0, rgb[2] / 255.0, "legend" + std::to_string(i));
  }

  if (rest && !rest->empty()) {
    CloudRGB::Ptr rest_rgb = toColoredCloud(rest, kRestColor[0], kRestColor[1], kRestColor[2]);
    vis.addPointCloud<pcl::PointXYZRGB>(rest_rgb, "rest");
    vis.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "rest");
    vis.addText("rest", 12, static_cast<int>(80 - 18 * plane_clouds.size()), 14, 0.6, 0.6, 0.6, "legend_rest");
  }

  if (corner) {
    pcl::PointXYZ p;
    p.x = static_cast<float>(corner->x());
    p.y = static_cast<float>(corner->y());
    p.z = static_cast<float>(corner->z());
    vis.addSphere(p, 3.0, 1.0, 1.0, 0.0, "corner");
    vis.addText3D("corner", p, 6.0, 1.0, 1.0, 0.0, "corner_txt");
  }

  vis.resetCamera();
  vis.spin();
}

Eigen::Vector3d planeIntersectionPoint(const pcl::ModelCoefficients& a,
                                       const pcl::ModelCoefficients& b,
                                       const pcl::ModelCoefficients& c) {
  Eigen::Matrix3d A;
  A << a.values[0], a.values[1], a.values[2],
       b.values[0], b.values[1], b.values[2],
       c.values[0], c.values[1], c.values[2];
  Eigen::Vector3d rhs(-a.values[3], -b.values[3], -c.values[3]);
  return A.colPivHouseholderQr().solve(rhs);
}

void printPlane(int id, const pcl::ModelCoefficients& coeff, std::size_t n) {
  const double nx = coeff.values[0];
  const double ny = coeff.values[1];
  const double nz = coeff.values[2];
  const double d = coeff.values[3];
  const double norm = std::sqrt(nx * nx + ny * ny + nz * nz);
  std::cout << "plane[" << id << "]  " << nx << " x + " << ny << " y + " << nz
            << " z + " << d << " = 0   |n|=" << norm << "  inliers=" << n
            << "  (xyz in mm)\n";
}

int run(const Options& opt) {
  Cloud::Ptr raw(new Cloud);
  if (!loadCloud(opt.input, raw) || raw->empty()) {
    std::cerr << "failed to load " << opt.input << "\n";
    return 1;
  }
  std::cout << "loaded " << raw->size() << " points (unit: mm)\n";

  Cloud::Ptr remaining = preprocess(raw, opt);
  std::cout << "after preprocess " << remaining->size() << " points\n";

  CloudRGB labeled;
  labeled.header = remaining->header;

  std::vector<pcl::ModelCoefficients> planes;
  std::vector<Cloud::Ptr> plane_clouds;

  for (int i = 0; i < opt.max_planes; ++i) {
    if (remaining->size() < static_cast<std::size_t>(opt.min_inliers)) {
      break;
    }
    pcl::ModelCoefficients coeff;
    pcl::PointIndices inliers;
    if (!extractOnePlane(remaining, opt, coeff, inliers)) {
      std::cerr << "SAC_RANSAC stopped at plane " << i << "\n";
      break;
    }

    Cloud::Ptr inlier_cloud(new Cloud);
    Cloud::Ptr outlier_cloud(new Cloud);
    splitCloud(remaining, inliers, inlier_cloud, outlier_cloud);

    printPlane(i, coeff, inlier_cloud->size());
    planes.push_back(coeff);
    plane_clouds.push_back(inlier_cloud);

    const auto& rgb = kPlaneColors[i % 5];
    appendColored(inlier_cloud, rgb[0], rgb[1], rgb[2], labeled);
    CloudRGB::Ptr plane_rgb = toColoredCloud(inlier_cloud, rgb[0], rgb[1], rgb[2]);

    const std::string ply = opt.prefix + "_plane" + std::to_string(i) + ".ply";
    const std::string pcd = opt.prefix + "_plane" + std::to_string(i) + ".pcd";
    pcl::io::savePLYFileBinary(ply, *plane_rgb);
    pcl::io::savePCDFileBinary(pcd, *inlier_cloud);
    std::cout << "  color RGB=(" << static_cast<int>(rgb[0]) << "," << static_cast<int>(rgb[1])
              << "," << static_cast<int>(rgb[2]) << ")  -> " << ply << "\n";

    remaining.swap(outlier_cloud);
  }

  appendColored(remaining, kRestColor[0], kRestColor[1], kRestColor[2], labeled);
  pcl::io::savePLYFileBinary(opt.prefix + "_labeled.ply", labeled);
  pcl::io::savePCDFileBinary(opt.prefix + "_rest.pcd", *remaining);
  std::cout << "rest points=" << remaining->size() << " (gray)  labeled -> "
            << opt.prefix << "_labeled.ply\n";

  if (planes.size() >= 2) {
    for (std::size_t i = 0; i < planes.size(); ++i) {
      for (std::size_t j = i + 1; j < planes.size(); ++j) {
        Eigen::Vector3d n1(planes[i].values[0], planes[i].values[1], planes[i].values[2]);
        Eigen::Vector3d n2(planes[j].values[0], planes[j].values[1], planes[j].values[2]);
        n1.normalize();
        n2.normalize();
        const double ang =
            std::acos(std::min(1.0, std::max(-1.0, std::abs(n1.dot(n2))))) * 180.0 / 3.141592653589793;
        const Eigen::Vector3d dir = n1.cross(n2);
        std::cout << "dihedral " << i << "-" << j << " = " << ang << " deg  line_dir=["
                  << dir.transpose() << "]\n";
      }
    }
  }
  Eigen::Vector3d corner = Eigen::Vector3d::Zero();
  const Eigen::Vector3d* corner_ptr = nullptr;
  if (planes.size() >= 3) {
    corner = planeIntersectionPoint(planes[0], planes[1], planes[2]);
    corner_ptr = &corner;
    std::cout << "three-plane corner (mm) = [" << corner.transpose() << "]\n";
  }

  if (opt.show) {
    showColoredResult(plane_clouds, remaining, corner_ptr);
  }
  return 0;
}

void usage() {
  std::cout
      << "usage: pcl_sac_ransac_planes --in cloud.ply [options]\n"
      << "  Point cloud unit MUST be millimetre (mm).\n"
      << "  --prefix NAME          output prefix (default sac_plane)\n"
      << "  --planes N             number of planes (default 3)\n"
      << "  --dist MM              RANSAC distance threshold in mm (default 1.2)\n"
      << "  --iters N              max iterations (default 1000)\n"
      << "  --min-inliers N        reject smaller planes (default 200)\n"
      << "  --voxel MM             voxel leaf in mm, 0 to disable (default 1.0)\n"
      << "  --zmin MM --zmax MM    height ROI in mm\n"
      << "  --show                 open PCL visualizer\n";
}

int main(int argc, char** argv) {
  Options opt;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    auto next = [&](double& dst) {
      if (i + 1 < argc) {
        dst = std::stod(argv[++i]);
      }
    };
    auto nexti = [&](int& dst) {
      if (i + 1 < argc) {
        dst = std::stoi(argv[++i]);
      }
    };
    if (a == "--in" && i + 1 < argc) {
      opt.input = argv[++i];
    } else if (a == "--prefix" && i + 1 < argc) {
      opt.prefix = argv[++i];
    } else if (a == "--planes") {
      nexti(opt.max_planes);
    } else if (a == "--dist") {
      next(opt.distance_threshold);
    } else if (a == "--iters") {
      nexti(opt.max_iterations);
    } else if (a == "--min-inliers") {
      nexti(opt.min_inliers);
    } else if (a == "--voxel") {
      next(opt.voxel_leaf);
    } else if (a == "--zmin") {
      next(opt.z_min);
    } else if (a == "--zmax") {
      next(opt.z_max);
    } else if (a == "--show") {
      opt.show = true;
    } else if (a == "-h" || a == "--help") {
      usage();
      return 0;
    }
  }
  if (opt.input.empty()) {
    usage();
    return 1;
  }
  return run(opt);
}
