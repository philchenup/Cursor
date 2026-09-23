#include <pcl/ModelCoefficients.h>
#include <pcl/PointIndices.h>
#include <pcl/common/centroid.h>
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

using Cloud = pcl::PointCloud<pcl::PointXYZ>;
using CloudRGB = pcl::PointCloud<pcl::PointXYZRGB>;

struct Options {
  std::string input;
  std::string prefix = "sac_plane";
  int max_planes = 3;
  int max_iterations = 1000;
  double distance_threshold = 0.0012;  // metres
  double voxel_leaf = 0.001;
  int min_inliers = 200;
  int sor_mean_k = 20;
  double sor_stddev = 1.8;
  double z_min = -1e9;
  double z_max = 1e9;
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
  seg.setDistanceThreshold(opt.distance_threshold);
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

void paint(const Cloud::ConstPtr& xyz, std::uint8_t r, std::uint8_t g, std::uint8_t b, CloudRGB& out) {
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
            << " z + " << d << " = 0   |n|=" << norm << "  inliers=" << n << "\n";
}

int run(const Options& opt) {
  Cloud::Ptr raw(new Cloud);
  if (!loadCloud(opt.input, raw) || raw->empty()) {
    std::cerr << "failed to load " << opt.input << "\n";
    return 1;
  }
  std::cout << "loaded " << raw->size() << " points\n";

  Cloud::Ptr remaining = preprocess(raw, opt);
  std::cout << "after preprocess " << remaining->size() << " points\n";

  const std::uint8_t colors[][3] = {
      {230, 40, 40}, {40, 190, 50}, {40, 90, 230}, {220, 180, 40}, {160, 80, 200}};
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

    const auto& c = colors[i % 5];
    paint(inlier_cloud, c[0], c[1], c[2], labeled);

    const std::string ply = opt.prefix + "_plane" + std::to_string(i) + ".ply";
    const std::string pcd = opt.prefix + "_plane" + std::to_string(i) + ".pcd";
    pcl::io::savePLYFileBinary(ply, *inlier_cloud);
    pcl::io::savePCDFileBinary(pcd, *inlier_cloud);

    remaining.swap(outlier_cloud);
  }

  paint(remaining, 140, 140, 140, labeled);
  pcl::io::savePLYFileBinary(opt.prefix + "_labeled.ply", labeled);
  pcl::io::savePCDFileBinary(opt.prefix + "_rest.pcd", *remaining);
  std::cout << "rest points=" << remaining->size() << "\n";

  if (planes.size() >= 2) {
    for (std::size_t i = 0; i < planes.size(); ++i) {
      for (std::size_t j = i + 1; j < planes.size(); ++j) {
        Eigen::Vector3d n1(planes[i].values[0], planes[i].values[1], planes[i].values[2]);
        Eigen::Vector3d n2(planes[j].values[0], planes[j].values[1], planes[j].values[2]);
        n1.normalize();
        n2.normalize();
                const double ang = std::acos(std::min(1.0, std::max(-1.0, std::abs(n1.dot(n2))))) * 180.0 / 3.141592653589793;
        const Eigen::Vector3d dir = n1.cross(n2);
        std::cout << "dihedral " << i << "-" << j << " = " << ang << " deg  line_dir=["
                  << dir.transpose() << "]\n";
      }
    }
  }
  if (planes.size() >= 3) {
    const Eigen::Vector3d corner = planeIntersectionPoint(planes[0], planes[1], planes[2]);
    std::cout << "three-plane corner = [" << corner.transpose() << "]\n";
  }

  if (opt.show) {
    pcl::visualization::PCLVisualizer vis("SAC_RANSAC planes");
    CloudRGB::Ptr labeled_ptr(new CloudRGB(labeled));
    vis.addPointCloud<pcl::PointXYZRGB>(labeled_ptr, "labeled");
    vis.addCoordinateSystem(0.05);
    vis.spin();
  }
  return 0;
}

void usage() {
  std::cout
      << "usage: pcl_sac_ransac_planes --in cloud.ply [options]\n"
      << "  --prefix NAME          output prefix (default sac_plane)\n"
      << "  --planes N             number of planes (default 3)\n"
      << "  --dist M               RANSAC distance threshold in metres (default 0.0012)\n"
      << "  --iters N              max iterations (default 1000)\n"
      << "  --min-inliers N        reject smaller planes (default 200)\n"
      << "  --voxel M              voxel leaf, 0 to disable (default 0.001)\n"
      << "  --zmin M --zmax M      height ROI\n"
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
