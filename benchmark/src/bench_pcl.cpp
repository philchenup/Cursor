// PCL timing benchmark for the requested algorithm catalog.
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <numeric>
#include <random>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/principal_curvatures.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/normal_space.h>
#include <pcl/features/boundary.h>
#include <pcl/common/pca.h>

#include <nlohmann/json.hpp>

using json = nlohmann::json;
using Cloud = pcl::PointCloud<pcl::PointXYZ>;
using CloudN = pcl::PointCloud<pcl::PointNormal>;
using CloudRGB = pcl::PointCloud<pcl::PointXYZRGB>;
using Clock = std::chrono::steady_clock;

static constexpr int WARMUP = 1;
static constexpr int REPEATS = 5;

template <typename Fn>
json timed(Fn&& fn) {
  for (int i = 0; i < WARMUP; ++i) fn();
  std::vector<double> samples;
  samples.reserve(REPEATS);
  for (int i = 0; i < REPEATS; ++i) {
    auto t0 = Clock::now();
    fn();
    auto t1 = Clock::now();
    samples.push_back(std::chrono::duration<double, std::milli>(t1 - t0).count());
  }
  double sum = std::accumulate(samples.begin(), samples.end(), 0.0);
  double mean = sum / samples.size();
  double var = 0.0;
  for (double s : samples) var += (s - mean) * (s - mean);
  var /= samples.size();
  json j;
  j["supported"] = true;
  j["mean_ms"] = mean;
  j["std_ms"] = std::sqrt(var);
  j["min_ms"] = *std::min_element(samples.begin(), samples.end());
  j["max_ms"] = *std::max_element(samples.begin(), samples.end());
  j["samples_ms"] = samples;
  return j;
}

json unsupported(const std::string& reason) {
  return json{{"supported", false}, {"reason", reason}};
}

Cloud::Ptr load_cloud_xyz(const std::string& path) {
  auto cloud = Cloud::Ptr(new Cloud);
  if (path.size() >= 4 && path.substr(path.size() - 4) == ".ply") {
    if (pcl::io::loadPLYFile(path, *cloud) < 0) throw std::runtime_error("PLY load failed: " + path);
  } else {
    if (pcl::io::loadPCDFile(path, *cloud) < 0) throw std::runtime_error("PCD load failed: " + path);
  }
  return cloud;
}

Cloud::Ptr txt_to_cloud(const std::string& path) {
  auto cloud = Cloud::Ptr(new Cloud);
  std::ifstream ifs(path);
  std::string line;
  while (std::getline(ifs, line)) {
    if (line.empty()) continue;
    std::istringstream iss(line);
    pcl::PointXYZ p;
    if (!(iss >> p.x >> p.y >> p.z)) continue;
    cloud->push_back(p);
  }
  cloud->width = cloud->size();
  cloud->height = 1;
  cloud->is_dense = true;
  return cloud;
}

void cloud_to_txt(const Cloud::Ptr& cloud, const std::string& path) {
  std::ofstream ofs(path);
  ofs.setf(std::ios::fixed);
  ofs.precision(6);
  for (const auto& p : cloud->points) {
    ofs << p.x << ' ' << p.y << ' ' << p.z << '\n';
  }
}

int main(int argc, char** argv) {
  std::string data_dir = (argc > 1) ? argv[1] : "data";
  std::string out_path = (argc > 2) ? argv[2] : "results/pcl_timings.json";

  auto cloud = load_cloud_xyz(data_dir + "/cloud.pcd");
  auto src = load_cloud_xyz(data_dir + "/cloud_source.pcd");
  auto tgt = load_cloud_xyz(data_dir + "/cloud_target.pcd");
  if (cloud->empty() || src->empty() || tgt->empty()) {
    std::cerr << "Failed to load required clouds\n";
    return 1;
  }

  json results;
  results["library"] = "PCL";
  results["version"] = PCL_VERSION;
  results["n_points"] = static_cast<int>(cloud->size());

  json io;
  io["load_cloud"] = timed([&]() { auto c = load_cloud_xyz(data_dir + "/cloud.pcd"); (void)c; });
  io["save_cloud"] = timed([&]() { pcl::io::savePCDFileBinary(data_dir + "/_tmp_pcl.pcd", *cloud); });
  io["txt_to_cloud"] = timed([&]() { auto c = txt_to_cloud(data_dir + "/cloud.txt"); (void)c; });
  io["cloud_to_txt"] = timed([&]() { cloud_to_txt(cloud, data_dir + "/_tmp_pcl.txt"); });
  io["rescale_units"] = timed([&]() {
    Cloud out;
    out.reserve(cloud->size());
    for (const auto& p : *cloud) out.push_back(pcl::PointXYZ(p.x * 1000.f, p.y * 1000.f, p.z * 1000.f));
  });
  io["transform_cloud"] = timed([&]() {
    Eigen::Affine3f T = Eigen::Affine3f::Identity();
    T.translation() << 0.01f, -0.02f, 0.03f;
    T.rotate(Eigen::AngleAxisf(0.1f, Eigen::Vector3f::UnitX()));
    Cloud out;
    pcl::transformPointCloud(*cloud, out, T);
  });
  io["depth_to_cloud"] = unsupported("Requires organized depth + camera model; not in core one-liner API used here");
  io["rgbd_to_cloud"] = unsupported("Requires organized RGB-D conversion pipeline");
  io["extract_cloud_by_mask"] = unsupported("Needs organized cloud + image mask pipeline");
  io["concat_clouds"] = timed([&]() {
    Cloud out = *cloud;
    out += *cloud;
  });
  results["categories"]["io_conversion"] = io;

  json pre;
  pre["downsample_voxel"] = timed([&]() {
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(0.05f, 0.05f, 0.05f);
    Cloud out;
    vg.filter(out);
  });
  pre["downsample_uniform"] = timed([&]() {
    pcl::UniformSampling<pcl::PointXYZ> us;
    us.setInputCloud(cloud);
    us.setRadiusSearch(0.05);
    Cloud out;
    us.filter(out);
  });
  pre["downsample_random"] = timed([&]() {
    pcl::RandomSample<pcl::PointXYZ> rs;
    rs.setInputCloud(cloud);
    rs.setSample(static_cast<unsigned>(cloud->size() * 0.2));
    Cloud out;
    rs.filter(out);
  });
  pre["downsample_fps"] = unsupported("PCL has no built-in Farthest Point Sampling filter");
  pre["downsample_to_count"] = timed([&]() {
    pcl::RandomSample<pcl::PointXYZ> rs;
    rs.setInputCloud(cloud);
    rs.setSample(std::min<unsigned>(8000u, static_cast<unsigned>(cloud->size())));
    Cloud out;
    rs.filter(out);
  });

  // Normal-space sampling needs normals
  auto normals_for_nss = pcl::PointCloud<pcl::Normal>::Ptr(new pcl::PointCloud<pcl::Normal>);
  {
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(cloud);
    auto tree = pcl::search::KdTree<pcl::PointXYZ>::Ptr(new pcl::search::KdTree<pcl::PointXYZ>);
    ne.setSearchMethod(tree);
    ne.setRadiusSearch(0.08);
    ne.compute(*normals_for_nss);
  }
  pre["downsample_normal_space"] = timed([&]() {
    pcl::NormalSpaceSampling<pcl::PointXYZ, pcl::Normal> nss;
    nss.setInputCloud(cloud);
    nss.setNormals(normals_for_nss);
    nss.setBins(4, 4, 4);
    nss.setSample(std::min<unsigned>(8000u, static_cast<unsigned>(cloud->size())));
    Cloud out;
    nss.filter(out);
  });

  pcl::PointXYZ min_pt, max_pt;
  pcl::getMinMax3D(*cloud, min_pt, max_pt);
  float z_mid = 0.5f * (min_pt.z + max_pt.z);

  pre["filter_axis_range"] = timed([&]() {
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(min_pt.z, z_mid);
    Cloud out;
    pass.filter(out);
  });
  pre["filter_axis_threshold"] = timed([&]() {
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(z_mid, max_pt.z);
    Cloud out;
    pass.filter(out);
  });
  pre["filter_by_direction"] = unsupported("No dedicated direction filter in PCL core filters");
  pre["remove_statistical_outliers"] = timed([&]() {
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(20);
    sor.setStddevMulThresh(2.0);
    Cloud out;
    sor.filter(out);
  });
  pre["remove_radius_outliers"] = timed([&]() {
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> ror;
    ror.setInputCloud(cloud);
    ror.setRadiusSearch(0.05);
    ror.setMinNeighborsInRadius(16);
    Cloud out;
    ror.filter(out);
  });
  pre["estimate_normals"] = timed([&]() {
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(cloud);
    auto tree = pcl::search::KdTree<pcl::PointXYZ>::Ptr(new pcl::search::KdTree<pcl::PointXYZ>);
    ne.setSearchMethod(tree);
    ne.setRadiusSearch(0.08);
    pcl::PointCloud<pcl::Normal> n;
    ne.compute(n);
  });
  pre["orient_normals"] = timed([&]() {
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(cloud);
    auto tree = pcl::search::KdTree<pcl::PointXYZ>::Ptr(new pcl::search::KdTree<pcl::PointXYZ>);
    ne.setSearchMethod(tree);
    ne.setRadiusSearch(0.08);
    ne.setViewPoint(0.f, 0.f, 5.f);
    pcl::PointCloud<pcl::Normal> n;
    ne.compute(n);
  });
  pre["flip_normals"] = timed([&]() {
    pcl::PointCloud<pcl::Normal> n = *normals_for_nss;
    for (auto& p : n.points) {
      p.normal_x = -p.normal_x;
      p.normal_y = -p.normal_y;
      p.normal_z = -p.normal_z;
    }
  });
  pre["compute_fpfh"] = timed([&]() {
    Cloud down;
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(0.05f, 0.05f, 0.05f);
    vg.filter(down);
    auto down_ptr = down.makeShared();
    pcl::PointCloud<pcl::Normal> n;
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(down_ptr);
    auto tree = pcl::search::KdTree<pcl::PointXYZ>::Ptr(new pcl::search::KdTree<pcl::PointXYZ>);
    ne.setSearchMethod(tree);
    ne.setRadiusSearch(0.1);
    ne.compute(n);
    pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh;
    fpfh.setInputCloud(down_ptr);
    fpfh.setInputNormals(n.makeShared());
    fpfh.setSearchMethod(tree);
    fpfh.setRadiusSearch(0.25);
    pcl::PointCloud<pcl::FPFHSignature33> desc;
    fpfh.compute(desc);
  });
  pre["compute_curvature"] = timed([&]() {
    Cloud down;
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(0.04f, 0.04f, 0.04f);
    vg.filter(down);
    auto down_ptr = down.makeShared();
    pcl::PointCloud<pcl::Normal> n;
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(down_ptr);
    auto tree = pcl::search::KdTree<pcl::PointXYZ>::Ptr(new pcl::search::KdTree<pcl::PointXYZ>);
    ne.setSearchMethod(tree);
    ne.setRadiusSearch(0.08);
    ne.compute(n);
    pcl::PrincipalCurvaturesEstimation<pcl::PointXYZ, pcl::Normal, pcl::PrincipalCurvatures> pc;
    pc.setInputCloud(down_ptr);
    pc.setInputNormals(n.makeShared());
    pc.setSearchMethod(tree);
    pc.setRadiusSearch(0.08);
    pcl::PointCloud<pcl::PrincipalCurvatures> curv;
    pc.compute(curv);
  });
  results["categories"]["preprocess"] = pre;

  json geo;
  geo["compute_obb"] = timed([&]() {
    // PCA-based OBB approximation
    pcl::PCA<pcl::PointXYZ> pca;
    pca.setInputCloud(cloud);
    Eigen::Vector4f centroid = pca.getMean();
    Eigen::Matrix3f ev = pca.getEigenVectors();
    (void)centroid;
    (void)ev;
  });
  geo["compute_convex_hull"] = timed([&]() {
    pcl::ConvexHull<pcl::PointXYZ> ch;
    ch.setInputCloud(cloud);
    Cloud hull;
    ch.reconstruct(hull);
  });
  geo["compute_centroid"] = timed([&]() {
    Eigen::Vector4f c;
    pcl::compute3DCentroid(*cloud, c);
  });
  geo["compute_aabb"] = timed([&]() {
    pcl::PointXYZ mn, mx;
    pcl::getMinMax3D(*cloud, mn, mx);
  });
  geo["select_by_mask"] = timed([&]() {
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    inliers->indices.reserve(cloud->size() / 2);
    for (int i = 0; i < static_cast<int>(cloud->size()); i += 2) inliers->indices.push_back(i);
    pcl::ExtractIndices<pcl::PointXYZ> ex;
    ex.setInputCloud(cloud);
    ex.setIndices(inliers);
    Cloud out;
    ex.filter(out);
  });
  geo["select_by_index"] = timed([&]() {
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    for (int i = 0; i < static_cast<int>(cloud->size()); i += 5) inliers->indices.push_back(i);
    pcl::ExtractIndices<pcl::PointXYZ> ex;
    ex.setInputCloud(cloud);
    ex.setIndices(inliers);
    Cloud out;
    ex.filter(out);
  });
  geo["segment_plane"] = timed([&]() {
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.02);
    seg.setMaxIterations(500);
    seg.setInputCloud(cloud);
    pcl::PointIndices inliers;
    pcl::ModelCoefficients coef;
    seg.segment(inliers, coef);
  });
  geo["cluster_dbscan"] = timed([&]() {
    auto tree = pcl::search::KdTree<pcl::PointXYZ>::Ptr(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);
    std::vector<pcl::PointIndices> clusters;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.05);
    ec.setMinClusterSize(10);
    ec.setMaxClusterSize(250000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(clusters);
  });
  geo["split_by_labels"] = timed([&]() {
    auto tree = pcl::search::KdTree<pcl::PointXYZ>::Ptr(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);
    std::vector<pcl::PointIndices> clusters;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.05);
    ec.setMinClusterSize(10);
    ec.setMaxClusterSize(250000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(clusters);
    std::vector<Cloud> parts;
    parts.reserve(clusters.size());
    for (const auto& cl : clusters) {
      Cloud part;
      for (int idx : cl.indices) part.push_back((*cloud)[idx]);
      parts.push_back(std::move(part));
    }
  });
  geo["fit_line"] = timed([&]() {
    auto model = pcl::SampleConsensusModelLine<pcl::PointXYZ>::Ptr(
        new pcl::SampleConsensusModelLine<pcl::PointXYZ>(cloud));
    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model);
    ransac.setDistanceThreshold(0.02);
    ransac.computeModel();
  });
  geo["fit_sphere"] = timed([&]() {
    auto model = pcl::SampleConsensusModelSphere<pcl::PointXYZ>::Ptr(
        new pcl::SampleConsensusModelSphere<pcl::PointXYZ>(cloud));
    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model);
    ransac.setDistanceThreshold(0.02);
    ransac.computeModel();
  });
  geo["fit_plane"] = timed([&]() {
    auto model = pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr(
        new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(cloud));
    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model);
    ransac.setDistanceThreshold(0.02);
    ransac.computeModel();
  });
  geo["plane_from_points"] = timed([&]() {
    const auto& a = (*cloud)[0];
    const auto& b = (*cloud)[1];
    const auto& c = (*cloud)[2];
    Eigen::Vector3f v1(b.x - a.x, b.y - a.y, b.z - a.z);
    Eigen::Vector3f v2(c.x - a.x, c.y - a.y, c.z - a.z);
    Eigen::Vector3f n = v1.cross(v2).normalized();
    float d = -n.dot(Eigen::Vector3f(a.x, a.y, a.z));
    (void)d;
  });
  geo["plane_normal"] = timed([&]() {
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.02);
    seg.setMaxIterations(200);
    seg.setInputCloud(cloud);
    pcl::PointIndices inliers;
    pcl::ModelCoefficients coef;
    seg.segment(inliers, coef);
  });
  geo["split_by_plane"] = timed([&]() {
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.02);
    seg.setMaxIterations(300);
    seg.setInputCloud(cloud);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients coef;
    seg.segment(*inliers, coef);
    pcl::ExtractIndices<pcl::PointXYZ> ex;
    ex.setInputCloud(cloud);
    ex.setIndices(inliers);
    Cloud plane_cloud, rest;
    ex.setNegative(false);
    ex.filter(plane_cloud);
    ex.setNegative(true);
    ex.filter(rest);
  });
  geo["extract_boundary_points"] = timed([&]() {
    Cloud down;
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(0.03f, 0.03f, 0.03f);
    vg.filter(down);
    auto down_ptr = down.makeShared();
    pcl::PointCloud<pcl::Normal> n;
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(down_ptr);
    auto tree = pcl::search::KdTree<pcl::PointXYZ>::Ptr(new pcl::search::KdTree<pcl::PointXYZ>);
    ne.setSearchMethod(tree);
    ne.setRadiusSearch(0.08);
    ne.compute(n);
    pcl::PointCloud<pcl::Boundary> boundaries;
    pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary> be;
    be.setInputCloud(down_ptr);
    be.setInputNormals(n.makeShared());
    be.setRadiusSearch(0.06);
    be.setSearchMethod(tree);
    be.compute(boundaries);
  });
  results["categories"]["geometry_segmentation"] = geo;

  // Registration prep
  Cloud src_d, tgt_d;
  {
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setLeafSize(0.05f, 0.05f, 0.05f);
    vg.setInputCloud(src);
    vg.filter(src_d);
    vg.setInputCloud(tgt);
    vg.filter(tgt_d);
  }
  auto src_d_ptr = src_d.makeShared();
  auto tgt_d_ptr = tgt_d.makeShared();
  pcl::PointCloud<pcl::Normal> src_n, tgt_n;
  {
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
    auto tree = pcl::search::KdTree<pcl::PointXYZ>::Ptr(new pcl::search::KdTree<pcl::PointXYZ>);
    ne.setSearchMethod(tree);
    ne.setRadiusSearch(0.1);
    ne.setInputCloud(src_d_ptr);
    ne.compute(src_n);
    ne.setInputCloud(tgt_d_ptr);
    ne.compute(tgt_n);
  }
  pcl::PointCloud<pcl::FPFHSignature33> src_fpfh, tgt_fpfh;
  {
    auto tree = pcl::search::KdTree<pcl::PointXYZ>::Ptr(new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh;
    fpfh.setSearchMethod(tree);
    fpfh.setRadiusSearch(0.25);
    fpfh.setInputCloud(src_d_ptr);
    fpfh.setInputNormals(src_n.makeShared());
    fpfh.compute(src_fpfh);
    fpfh.setInputCloud(tgt_d_ptr);
    fpfh.setInputNormals(tgt_n.makeShared());
    fpfh.compute(tgt_fpfh);
  }

  json reg;
  reg["make_init_pose"] = timed([&]() {
    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
    (void)T;
  });
  reg["register_fpfh_ransac"] = timed([&]() {
    pcl::SampleConsensusPrerejective<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> align;
    align.setInputSource(src_d_ptr);
    align.setSourceFeatures(src_fpfh.makeShared());
    align.setInputTarget(tgt_d_ptr);
    align.setTargetFeatures(tgt_fpfh.makeShared());
    align.setMaximumIterations(50000);
    align.setNumberOfSamples(3);
    align.setCorrespondenceRandomness(5);
    align.setSimilarityThreshold(0.9f);
    align.setMaxCorrespondenceDistance(0.075);
    align.setInlierFraction(0.25f);
    Cloud aligned;
    align.align(aligned);
  });
  reg["register_fast_global"] = unsupported("Fast Global Registration not in PCL");
  reg["register_ppf"] = unsupported("PPF registration module not exercised / may be unavailable in packaged build");
  reg["relative_sampling_step"] = unsupported("Utility parameter, not a PCL algorithm");
  reg["register_icp_point_to_point"] = timed([&]() {
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(src_d_ptr);
    icp.setInputTarget(tgt_d_ptr);
    icp.setMaxCorrespondenceDistance(0.1);
    icp.setMaximumIterations(30);
    Cloud aligned;
    icp.align(aligned);
  });
  reg["register_icp_point_to_plane"] = timed([&]() {
    // Use PointNormal ICP (point-to-plane style via Nonlinear ICP with normals)
    pcl::PointCloud<pcl::PointNormal> src_pn, tgt_pn;
    pcl::copyPointCloud(*src_d_ptr, src_pn);
    pcl::copyPointCloud(*tgt_d_ptr, tgt_pn);
    for (size_t i = 0; i < src_pn.size(); ++i) {
      src_pn[i].normal_x = src_n[i].normal_x;
      src_pn[i].normal_y = src_n[i].normal_y;
      src_pn[i].normal_z = src_n[i].normal_z;
    }
    for (size_t i = 0; i < tgt_pn.size(); ++i) {
      tgt_pn[i].normal_x = tgt_n[i].normal_x;
      tgt_pn[i].normal_y = tgt_n[i].normal_y;
      tgt_pn[i].normal_z = tgt_n[i].normal_z;
    }
    pcl::IterativeClosestPointWithNormals<pcl::PointNormal, pcl::PointNormal> icp;
    icp.setInputSource(src_pn.makeShared());
    icp.setInputTarget(tgt_pn.makeShared());
    icp.setMaxCorrespondenceDistance(0.1);
    icp.setMaximumIterations(30);
    pcl::PointCloud<pcl::PointNormal> aligned;
    icp.align(aligned);
  });
  reg["register_icp_generalized"] = timed([&]() {
    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;
    gicp.setInputSource(src_d_ptr);
    gicp.setInputTarget(tgt_d_ptr);
    gicp.setMaxCorrespondenceDistance(0.1);
    gicp.setMaximumIterations(30);
    Cloud aligned;
    gicp.align(aligned);
  });
  reg["register_icp_colored"] = unsupported("Colored ICP not in standard PCL");
  reg["register_filterreg"] = unsupported("FilterReg not in PCL");
  reg["register_cpd"] = unsupported("CPD not in PCL");
  reg["register_gmmtree"] = unsupported("GMMTree not in PCL");
  reg["evaluate_registration"] = timed([&]() {
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(src_d_ptr);
    icp.setInputTarget(tgt_d_ptr);
    icp.setMaxCorrespondenceDistance(0.1);
    icp.setMaximumIterations(1);
    Cloud aligned;
    icp.align(aligned);
    double fitness = icp.getFitnessScore();
    (void)fitness;
  });
  reg["is_registration_valid"] = timed([&]() {
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(src_d_ptr);
    icp.setInputTarget(tgt_d_ptr);
    icp.setMaxCorrespondenceDistance(0.1);
    icp.setMaximumIterations(5);
    Cloud aligned;
    icp.align(aligned);
    bool ok = icp.hasConverged() && icp.getFitnessScore() < 0.01;
    (void)ok;
  });
  results["categories"]["registration"] = reg;

  std::ofstream ofs(out_path);
  ofs << results.dump(2) << std::endl;
  std::cout << results.dump(2) << std::endl;
  return 0;
}
