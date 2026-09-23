#include <pcl/ModelCoefficients.h>
#include <pcl/PointIndices.h>
#include <pcl/common/centroid.h>
#include <pcl/common/io.h>
#include <pcl/features/normal_3d.h>
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
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <random>
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
  double normal_th = 0.88;   // min |n·n_plane|, 0.88 ≈ 28 deg
  double band = 1.0;         // mm, points near two planes stay unlabeled
  double cluster_tol = 3.0;  // mm, drop isolated strips
  bool show = false;
  bool self_test = false;
};

struct SegResult {
  Cloud::Ptr full;
  std::vector<pcl::ModelCoefficients> planes;
  std::vector<Cloud::Ptr> plane_clouds;
  Cloud::Ptr rest;
};

bool loadCloud(const std::string& path, Cloud::Ptr cloud) {
  const auto ext = path.substr(path.find_last_of('.') + 1);
  if (ext == "pcd" || ext == "PCD") {
    return pcl::io::loadPCDFile(path, *cloud) >= 0;
  }
  return pcl::io::loadPLYFile(path, *cloud) >= 0;
}

void normalizePlane(pcl::ModelCoefficients& coeff);

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
  if (inliers.indices.empty() || static_cast<int>(inliers.indices.size()) < opt.min_inliers) {
    return false;
  }
  normalizePlane(coeff);
  return true;
}

void normalizePlane(pcl::ModelCoefficients& coeff) {
  Eigen::Vector3d n(coeff.values[0], coeff.values[1], coeff.values[2]);
  const double len = n.norm();
  if (len < 1e-12) {
    return;
  }
  coeff.values[0] = n.x() / len;
  coeff.values[1] = n.y() / len;
  coeff.values[2] = n.z() / len;
  coeff.values[3] /= len;
}

double pointPlaneAbsDist(const pcl::PointXYZ& p, const pcl::ModelCoefficients& c) {
  return std::abs(c.values[0] * p.x + c.values[1] * p.y + c.values[2] * p.z + c.values[3]);
}

void estimateNormals(const Cloud::ConstPtr& cloud, double radius_mm, pcl::PointCloud<pcl::Normal>::Ptr normals) {
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud(cloud);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  ne.setSearchMethod(tree);
  ne.setRadiusSearch(radius_mm);
  ne.compute(*normals);
}

void refinePlaneSvd(const Cloud::ConstPtr& cloud, pcl::ModelCoefficients& coeff) {
  if (cloud->size() < 3) {
    return;
  }
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(*cloud, centroid);
  Eigen::Matrix3f cov;
  pcl::computeCovarianceMatrixNormalized(*cloud, centroid, cov);
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(cov);
  const Eigen::Vector3f n = solver.eigenvectors().col(0);
  coeff.values[0] = n.x();
  coeff.values[1] = n.y();
  coeff.values[2] = n.z();
  coeff.values[3] = -n.dot(centroid.head<3>());
  normalizePlane(coeff);
}

int pickPlaneForPoint(const pcl::PointXYZ& p,
                      const Eigen::Vector3d& pn,
                      bool has_n,
                      const std::vector<pcl::ModelCoefficients>& planes,
                      const Options& opt) {
  int best = -1;
  double best_align = -1.0;
  double best_d = 1e9;
  int second = -1;
  double second_align = -1.0;
  double second_d = 1e9;
  for (std::size_t k = 0; k < planes.size(); ++k) {
    const auto& c = planes[k];
    const double d = pointPlaneAbsDist(p, c);
    if (d > opt.distance_threshold) {
      continue;
    }
    const Eigen::Vector3d npl(c.values[0], c.values[1], c.values[2]);
    const double align = (has_n && pn.squaredNorm() > 0.0) ? std::abs(pn.dot(npl)) : 1.0;
    if (has_n && align < opt.normal_th) {
      continue;
    }
    const bool better =
        (align > best_align + 0.04) || (std::abs(align - best_align) <= 0.04 && d < best_d);
    if (better) {
      second = best;
      second_align = best_align;
      second_d = best_d;
      best = static_cast<int>(k);
      best_align = align;
      best_d = d;
    } else if (align > second_align || (std::abs(align - second_align) <= 0.04 && d < second_d)) {
      second = static_cast<int>(k);
      second_align = align;
      second_d = d;
    }
  }
  // Only the exact crease stays unlabeled: both faces fit equally well.
  if (best >= 0 && second >= 0 && std::abs(best_align - second_align) < 0.08 && best_d < opt.band &&
      second_d < opt.band) {
    return -1;
  }
  return best;
}

void smoothLabels(const Cloud::ConstPtr& cloud, std::vector<int>& labels, int n_planes, int k) {
  if (cloud->empty() || n_planes <= 0) {
    return;
  }
  pcl::KdTreeFLANN<pcl::PointXYZ> tree;
  tree.setInputCloud(cloud);
  std::vector<int> next = labels;
  for (std::size_t i = 0; i < cloud->size(); ++i) {
    std::vector<int> nn;
    std::vector<float> dist;
    if (tree.nearestKSearch(cloud->points[i], k, nn, dist) < 3) {
      continue;
    }
    std::vector<int> hist(n_planes, 0);
    int valid = 0;
    for (int j : nn) {
      if (j >= 0 && labels[static_cast<std::size_t>(j)] >= 0) {
        hist[labels[static_cast<std::size_t>(j)]]++;
        valid++;
      }
    }
    if (valid < 3) {
      continue;
    }
    int maj = 0;
    for (int p = 1; p < n_planes; ++p) {
      if (hist[p] > hist[maj]) {
        maj = p;
      }
    }
    if (hist[maj] * 2 > valid) {
      next[i] = maj;
    }
  }
  labels.swap(next);
}

void keepLargestClusterLabels(const Cloud::ConstPtr& cloud,
                              std::vector<int>& labels,
                              int plane_id,
                              double tol_mm) {
  Cloud::Ptr subset(new Cloud);
  std::vector<int> map;
  for (std::size_t i = 0; i < labels.size(); ++i) {
    if (labels[i] == plane_id) {
      subset->push_back(cloud->points[i]);
      map.push_back(static_cast<int>(i));
    }
  }
  if (subset->size() < 20) {
    return;
  }
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(subset);
  std::vector<pcl::PointIndices> clusters;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(tol_mm);
  ec.setMinClusterSize(20);
  ec.setSearchMethod(tree);
  ec.setInputCloud(subset);
  ec.extract(clusters);
  if (clusters.empty()) {
    return;
  }
  std::size_t best = 0;
  for (std::size_t i = 1; i < clusters.size(); ++i) {
    if (clusters[i].indices.size() > clusters[best].indices.size()) {
      best = i;
    }
  }
  std::vector<char> keep(subset->size(), 0);
  for (int idx : clusters[best].indices) {
    keep[static_cast<std::size_t>(idx)] = 1;
  }
  for (std::size_t i = 0; i < map.size(); ++i) {
    if (!keep[i]) {
      labels[static_cast<std::size_t>(map[i])] = -1;
    }
  }
}

// Sequential RANSAC lets the first plane keep a ridge strip that belongs on
// the next face. Re-label every point, then majority-vote the neighborhood so
// a thin red band sitting on the blue face flips back to blue.
void reassignPoints(const Cloud::ConstPtr& cloud,
                    const pcl::PointCloud<pcl::Normal>::ConstPtr& normals,
                    std::vector<pcl::ModelCoefficients>& planes,
                    const Options& opt,
                    std::vector<Cloud::Ptr>& plane_clouds,
                    Cloud::Ptr rest) {
  std::vector<int> labels(cloud->size(), -1);
  for (std::size_t i = 0; i < cloud->size(); ++i) {
    Eigen::Vector3d pn(0, 0, 0);
    bool has_n = normals && i < normals->size() && std::isfinite(normals->points[i].normal_x);
    if (has_n) {
      pn = Eigen::Vector3d(normals->points[i].normal_x, normals->points[i].normal_y,
                           normals->points[i].normal_z);
      if (pn.norm() > 1e-6) {
        pn.normalize();
      } else {
        has_n = false;
      }
    }
    labels[i] = pickPlaneForPoint(cloud->points[i], pn, has_n, planes, opt);
  }

  smoothLabels(cloud, labels, static_cast<int>(planes.size()), 24);
  for (int k = 0; k < static_cast<int>(planes.size()); ++k) {
    keepLargestClusterLabels(cloud, labels, k, opt.cluster_tol);
  }
  smoothLabels(cloud, labels, static_cast<int>(planes.size()), 24);

  plane_clouds.assign(planes.size(), Cloud::Ptr());
  for (auto& pc : plane_clouds) {
    pc.reset(new Cloud);
    pc->header = cloud->header;
  }
  rest->clear();
  rest->header = cloud->header;
  for (std::size_t i = 0; i < cloud->size(); ++i) {
    if (labels[i] >= 0) {
      plane_clouds[static_cast<std::size_t>(labels[i])]->push_back(cloud->points[i]);
    } else {
      rest->push_back(cloud->points[i]);
    }
  }
  for (std::size_t k = 0; k < planes.size(); ++k) {
    refinePlaneSvd(plane_clouds[k], planes[k]);
  }
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

int segmentCloud(const Cloud::ConstPtr& raw, const Options& opt, SegResult& out, bool verbose) {
  Cloud::Ptr remaining = preprocess(raw, opt);
  if (verbose) {
    std::cout << "after preprocess " << remaining->size() << " points\n";
  }

  out.full.reset(new Cloud(*remaining));
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
  const double n_radius = std::max(3.0 * std::max(opt.voxel_leaf, 0.5), 3.0);
  estimateNormals(out.full, n_radius, normals);
  if (verbose) {
    std::cout << "normals radius=" << n_radius << " mm  normal_th=" << opt.normal_th
              << "  band=" << opt.band << " mm\n";
  }

  out.planes.clear();
  for (int i = 0; i < opt.max_planes; ++i) {
    if (remaining->size() < static_cast<std::size_t>(opt.min_inliers)) {
      break;
    }
    pcl::ModelCoefficients coeff;
    pcl::PointIndices inliers;
    if (!extractOnePlane(remaining, opt, coeff, inliers)) {
      if (verbose) {
        std::cerr << "SAC_RANSAC stopped at plane " << i << "\n";
      }
      break;
    }
    Cloud::Ptr inlier_cloud(new Cloud);
    Cloud::Ptr outlier_cloud(new Cloud);
    splitCloud(remaining, inliers, inlier_cloud, outlier_cloud);
    if (verbose) {
      printPlane(i, coeff, inlier_cloud->size());
    }
    out.planes.push_back(coeff);
    remaining.swap(outlier_cloud);
  }

  out.rest.reset(new Cloud);
  reassignPoints(out.full, normals, out.planes, opt, out.plane_clouds, out.rest);
  reassignPoints(out.full, normals, out.planes, opt, out.plane_clouds, out.rest);
  return out.planes.empty() ? 1 : 0;
}

Cloud::Ptr makeLeakScene() {
  Cloud::Ptr cloud(new Cloud);
  std::mt19937 rng(1);
  std::uniform_real_distribution<float> ux(0.0f, 80.0f);
  std::uniform_real_distribution<float> uy(0.0f, 60.0f);
  std::uniform_real_distribution<float> uz(-50.0f, 0.0f);
  std::normal_distribution<float> n01(0.0f, 0.12f);
  std::uniform_real_distribution<float> leak_y(4.0f, 56.0f);
  std::uniform_real_distribution<float> leak_z(-0.70f, -0.15f);
  auto add = [&](float x, float y, float z) {
    cloud->push_back(pcl::PointXYZ{x, y, z});
  };
  for (int i = 0; i < 5000; ++i) {
    add(ux(rng), uy(rng), n01(rng));  // red z=0
  }
  for (int i = 0; i < 4200; ++i) {
    add(ux(rng), n01(rng), uz(rng));  // green y=0
  }
  for (int i = 0; i < 4200; ++i) {
    add(80.0f + n01(rng), uy(rng), uz(rng));  // blue x=80
  }
  // Strip on the blue face, still within the red-plane distance threshold.
  for (int i = 0; i < 300; ++i) {
    add(80.0f + n01(rng) * 0.6f, leak_y(rng), leak_z(rng));
  }
  return cloud;
}

bool isLeakPoint(const pcl::PointXYZ& p) {
  return std::abs(p.x - 80.0f) < 1.0f && p.z < -0.12f && p.z > -0.85f;
}

int planeAxis(const pcl::ModelCoefficients& c) {
  const double ax = std::abs(c.values[0]);
  const double ay = std::abs(c.values[1]);
  const double az = std::abs(c.values[2]);
  if (az >= ax && az >= ay) {
    return 2;  // red / z
  }
  if (ax >= ay) {
    return 0;  // blue / x
  }
  return 1;  // green / y
}

int selfTest() {
  Options opt;
  opt.max_planes = 3;
  opt.distance_threshold = 1.2;
  opt.voxel_leaf = 1.0;
  opt.min_inliers = 200;
  opt.normal_th = 0.88;
  opt.band = 1.0;
  opt.cluster_tol = 3.0;

  Cloud::Ptr raw = makeLeakScene();
  SegResult seg;
  if (segmentCloud(raw, opt, seg, false) != 0 || seg.plane_clouds.size() < 3) {
    std::cerr << "self-test: failed to extract 3 planes\n";
    return 1;
  }

  int red_i = -1;
  int blue_i = -1;
  for (std::size_t i = 0; i < seg.planes.size(); ++i) {
    const int axis = planeAxis(seg.planes[i]);
    if (axis == 2) {
      red_i = static_cast<int>(i);
    }
    if (axis == 0) {
      blue_i = static_cast<int>(i);
    }
  }
  if (red_i < 0 || blue_i < 0) {
    std::cerr << "self-test: could not identify red/blue planes\n";
    return 1;
  }

  int leak_red = 0;
  int leak_blue = 0;
  for (const auto& p : seg.plane_clouds[static_cast<std::size_t>(red_i)]->points) {
    leak_red += isLeakPoint(p) ? 1 : 0;
  }
  for (const auto& p : seg.plane_clouds[static_cast<std::size_t>(blue_i)]->points) {
    leak_blue += isLeakPoint(p) ? 1 : 0;
  }
  std::cout << "self-test leak-like points  red=" << leak_red << "  blue=" << leak_blue << "\n";
  if (leak_blue < 50 || leak_red * 2 > leak_blue) {
    std::cerr << "self-test: red still owns the strip that should be blue\n";
    return 1;
  }
  std::cout << "self-test ok\n";
  return 0;
}

int run(const Options& opt) {
  Cloud::Ptr raw(new Cloud);
  if (!loadCloud(opt.input, raw) || raw->empty()) {
    std::cerr << "failed to load " << opt.input << "\n";
    return 1;
  }
  std::cout << "loaded " << raw->size() << " points (unit: mm)\n";

  SegResult seg;
  if (segmentCloud(raw, opt, seg, true) != 0) {
    return 1;
  }
  const auto& planes = seg.planes;
  const auto& plane_clouds = seg.plane_clouds;
  const auto& rest = seg.rest;

  CloudRGB labeled;
  labeled.header = seg.full->header;
  for (std::size_t i = 0; i < plane_clouds.size(); ++i) {
    printPlane(static_cast<int>(i), planes[i], plane_clouds[i]->size());
    const auto& rgb = kPlaneColors[i % 5];
    appendColored(plane_clouds[i], rgb[0], rgb[1], rgb[2], labeled);
    CloudRGB::Ptr plane_rgb = toColoredCloud(plane_clouds[i], rgb[0], rgb[1], rgb[2]);
    const std::string ply = opt.prefix + "_plane" + std::to_string(i) + ".ply";
    const std::string pcd = opt.prefix + "_plane" + std::to_string(i) + ".pcd";
    pcl::io::savePLYFileBinary(ply, *plane_rgb);
    pcl::io::savePCDFileBinary(pcd, *plane_clouds[i]);
    std::cout << "  after reassign color RGB=(" << static_cast<int>(rgb[0]) << ","
              << static_cast<int>(rgb[1]) << "," << static_cast<int>(rgb[2]) << ")  -> " << ply
              << "\n";
  }

  appendColored(rest, kRestColor[0], kRestColor[1], kRestColor[2], labeled);
  pcl::io::savePLYFileBinary(opt.prefix + "_labeled.ply", labeled);
  pcl::io::savePCDFileBinary(opt.prefix + "_rest.pcd", *rest);
  std::cout << "rest points=" << rest->size() << " (gray)  labeled -> " << opt.prefix
            << "_labeled.ply\n";

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
    showColoredResult(plane_clouds, rest, corner_ptr);
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
      << "  --normal T             min |n·n_plane| for assignment (default 0.88)\n"
      << "  --band MM              intersection dead-band in mm (default 1.0)\n"
      << "  --cluster MM           drop isolated strips, mm (default 3.0)\n"
      << "  --show                 open PCL visualizer\n"
      << "  --self-test            C++ leak-strip regression, no input file\n";
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
    } else if (a == "--normal") {
      next(opt.normal_th);
    } else if (a == "--band") {
      next(opt.band);
    } else if (a == "--cluster") {
      next(opt.cluster_tol);
    } else if (a == "--show") {
      opt.show = true;
    } else if (a == "--self-test") {
      opt.self_test = true;
    } else if (a == "-h" || a == "--help") {
      usage();
      return 0;
    }
  }
  if (opt.self_test) {
    return selfTest();
  }
  if (opt.input.empty()) {
    usage();
    return 1;
  }
  return run(opt);
}
