#include "geng_rcim2022.h"

#include <Eigen/Dense>
#include <pcl/common/centroid.h>
#include <pcl/common/io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <random>

namespace
{
constexpr float kPi = 3.14159265358979323846f;

pcl::PointXYZ
toXyz(const Eigen::Vector3f& v)
{
  pcl::PointXYZ p;
  p.x = v.x();
  p.y = v.y();
  p.z = v.z();
  return p;
}
}  // namespace

void
GengRcim2022::setInputCloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud)
{
  input_ = cloud;
}

bool
GengRcim2022::compute()
{
  planes_.clear();
  seams_.clear();
  cloud_.reset();
  segmented_cloud_.reset();
  normals_.reset();
  tree_.reset();

  if (!input_ || input_->empty())
    return false;

  downsample();
  if (!cloud_ || static_cast<int>(cloud_->size()) < params_.min_plane_inliers)
    return false;

  tree_.reset(new pcl::search::KdTree<pcl::PointXYZ>);
  tree_->setInputCloud(cloud_);

  extractPlanes();
  mergeSimilarPlanes();
  cleanSegmentedPlanes();
  materializeSegmentedPlanes();
  extractSeamsAndTrajectories();
  return !seams_.empty();
}

pcl::PointCloud<pcl::PointXYZ>::Ptr
GengRcim2022::seamCloud() const
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr out(new pcl::PointCloud<pcl::PointXYZ>);
  for (const auto& seam : seams_)
    *out += seam.seam_cloud;
  out->width = static_cast<std::uint32_t>(out->size());
  out->height = 1;
  out->is_dense = true;
  return out;
}

pcl::PointCloud<pcl::PointNormal>::Ptr
GengRcim2022::trajectoryCloud() const
{
  pcl::PointCloud<pcl::PointNormal>::Ptr out(new pcl::PointCloud<pcl::PointNormal>);
  for (const auto& seam : seams_)
    *out += seam.trajectory;
  out->width = static_cast<std::uint32_t>(out->size());
  out->height = 1;
  out->is_dense = true;
  return out;
}

void
GengRcim2022::downsample()
{
  if (params_.voxel_leaf_mm <= 0.0f)
  {
    cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>(*input_));
    return;
  }

  pcl::VoxelGrid<pcl::PointXYZ> grid;
  grid.setInputCloud(input_);
  grid.setLeafSize(params_.voxel_leaf_mm, params_.voxel_leaf_mm, params_.voxel_leaf_mm);
  cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
  grid.filter(*cloud_);
}

bool
GengRcim2022::fitThreePoints(const Eigen::Vector3f& p0,
                             const Eigen::Vector3f& p1,
                             const Eigen::Vector3f& p2,
                             Eigen::Vector3f& normal,
                             float& d)
{
  normal = (p1 - p0).cross(p2 - p0);
  if (normal.norm() < 1e-8f)
    return false;
  normal.normalize();
  d = -normal.dot(p0);
  return true;
}

float
GengRcim2022::unsignedAngleDeg(const Eigen::Vector3f& a, const Eigen::Vector3f& b)
{
  const float na = a.norm();
  const float nb = b.norm();
  if (na < 1e-8f || nb < 1e-8f)
    return 0.0f;
  float c = std::abs(a.dot(b)) / (na * nb);
  c = std::min(1.0f, std::max(0.0f, c));
  return std::acos(c) * 180.0f / kPi;
}

Eigen::Vector3f
GengRcim2022::pointOnIntersection(const FittedPlane& a, const FittedPlane& b)
{
  const float n01 = a.normal.dot(b.normal);
  const float det = 1.0f - n01 * n01;
  if (std::abs(det) < 1e-6f)
    return 0.5f * (a.centroid + b.centroid);

  const float rhs0 = -a.d;
  const float rhs1 = -b.d;
  const float alpha = (rhs0 - rhs1 * n01) / det;
  const float beta = (rhs1 - rhs0 * n01) / det;
  return alpha * a.normal + beta * b.normal;
}

float
GengRcim2022::pointToLineDistance(const Eigen::Vector3f& p,
                                  const Eigen::Vector3f& origin,
                                  const Eigen::Vector3f& dir)
{
  return (p - origin).cross(dir).norm();
}

void
GengRcim2022::refinePlane(FittedPlane& plane) const
{
  if (plane.inliers.size() < 3)
    return;

  Eigen::Vector4f centroid_h;
  pcl::compute3DCentroid(*cloud_, plane.inliers, centroid_h);
  plane.centroid = centroid_h.head<3>();

  Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();
  for (const int idx : plane.inliers)
  {
    const Eigen::Vector3f d = (*cloud_)[idx].getVector3fMap() - plane.centroid;
    cov += d * d.transpose();
  }
  cov /= static_cast<float>(plane.inliers.size());

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(cov, Eigen::ComputeEigenvectors);
  if (solver.info() != Eigen::Success)
    return;

  plane.normal = solver.eigenvectors().col(0).normalized();
  plane.d = -plane.normal.dot(plane.centroid);
}

bool
GengRcim2022::ransacOnePlane(const std::vector<int>& remaining, FittedPlane& plane) const
{
  if (static_cast<int>(remaining.size()) < params_.min_plane_inliers)
    return false;

  std::mt19937 rng(params_.rng_seed + static_cast<std::uint32_t>(remaining.size()));
  std::uniform_int_distribution<int> pick(0, static_cast<int>(remaining.size()) - 1);

  std::vector<char> is_remaining(cloud_->size(), 0);
  for (const int idx : remaining)
    is_remaining[static_cast<std::size_t>(idx)] = 1;

  int best_count = 0;
  Eigen::Vector3f best_n = Eigen::Vector3f::UnitZ();
  float best_d = 0.0f;

  for (int iter = 0; iter < params_.ransac_iters; ++iter)
  {
    const int seed = remaining[static_cast<std::size_t>(pick(rng))];
    pcl::Indices nn;
    std::vector<float> nn_dist;
    tree_->radiusSearch(seed, params_.local_sample_radius_mm, nn, nn_dist);

    std::vector<int> local;
    local.reserve(nn.size());
    for (const auto idx : nn)
    {
      if (is_remaining[static_cast<std::size_t>(idx)])
        local.push_back(static_cast<int>(idx));
    }
    if (local.size() < 3)
      continue;

    std::uniform_int_distribution<int> pick_local(0, static_cast<int>(local.size()) - 1);
    const int i0 = local[static_cast<std::size_t>(pick_local(rng))];
    const int i1 = local[static_cast<std::size_t>(pick_local(rng))];
    const int i2 = local[static_cast<std::size_t>(pick_local(rng))];
    if (i0 == i1 || i0 == i2 || i1 == i2)
      continue;

    Eigen::Vector3f n;
    float d = 0.0f;
    if (!fitThreePoints((*cloud_)[i0].getVector3fMap(),
                        (*cloud_)[i1].getVector3fMap(),
                        (*cloud_)[i2].getVector3fMap(),
                        n,
                        d))
      continue;

    int count = 0;
    for (const int idx : remaining)
    {
      const float dist = std::abs(n.dot((*cloud_)[idx].getVector3fMap()) + d);
      if (dist <= params_.plane_dist_mm)
        ++count;
    }
    if (count > best_count)
    {
      best_count = count;
      best_n = n;
      best_d = d;
    }
  }

  if (best_count < params_.min_plane_inliers)
    return false;

  plane.normal = best_n;
  plane.d = best_d;
  plane.inliers.clear();
  for (const int idx : remaining)
  {
    const float dist = std::abs(plane.normal.dot((*cloud_)[idx].getVector3fMap()) + plane.d);
    if (dist <= params_.plane_dist_mm)
      plane.inliers.push_back(idx);
  }

  refinePlane(plane);
  plane.inliers.clear();
  for (const int idx : remaining)
  {
    const float dist = std::abs(plane.normal.dot((*cloud_)[idx].getVector3fMap()) + plane.d);
    if (dist <= params_.plane_dist_mm)
      plane.inliers.push_back(idx);
  }
  return static_cast<int>(plane.inliers.size()) >= params_.min_plane_inliers;
}

void
GengRcim2022::estimateNormals()
{
  normals_.reset();
  if (!cloud_ || cloud_->empty() || params_.max_normal_dev_deg <= 0.0f)
    return;

  float radius = params_.local_sample_radius_mm;
  if (radius <= 0.0f)
    radius = std::max(6.0f, 3.0f * params_.voxel_leaf_mm);

  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud(cloud_);
  ne.setSearchMethod(tree_);
  ne.setRadiusSearch(radius);
  normals_.reset(new pcl::PointCloud<pcl::Normal>);
  ne.compute(*normals_);
}

void
GengRcim2022::reassignPointsToNearestPlane()
{
  if (!cloud_ || planes_.empty())
    return;

  std::vector<std::vector<int>> assigned(planes_.size());
  for (std::size_t i = 0; i < cloud_->size(); ++i)
  {
    const Eigen::Vector3f p = (*cloud_)[static_cast<std::size_t>(i)].getVector3fMap();
    float best = std::numeric_limits<float>::infinity();
    int best_j = -1;
    for (std::size_t j = 0; j < planes_.size(); ++j)
    {
      const float dist = pointToPlaneDistance(p, planes_[j]);
      if (dist < best)
      {
        best = dist;
        best_j = static_cast<int>(j);
      }
    }
    if (best_j >= 0 && best <= params_.plane_dist_mm)
      assigned[static_cast<std::size_t>(best_j)].push_back(static_cast<int>(i));
  }

  for (std::size_t j = 0; j < planes_.size(); ++j)
  {
    planes_[j].inliers.swap(assigned[j]);
    refinePlane(planes_[j]);
  }
}

void
GengRcim2022::filterPlaneByNormal(FittedPlane& plane) const
{
  if (!normals_ || normals_->size() != cloud_->size() || params_.max_normal_dev_deg <= 0.0f)
    return;
  if (plane.inliers.size() < 3)
    return;

  const float min_dot = std::cos(params_.max_normal_dev_deg * kPi / 180.0f);
  std::vector<int> kept;
  kept.reserve(plane.inliers.size());
  for (const int idx : plane.inliers)
  {
    if (idx < 0 || idx >= static_cast<int>(normals_->size()))
      continue;
    const pcl::Normal& n = (*normals_)[static_cast<std::size_t>(idx)];
    if (!std::isfinite(n.normal_x) || !std::isfinite(n.normal_y) || !std::isfinite(n.normal_z))
      continue;
    Eigen::Vector3f ln(n.normal_x, n.normal_y, n.normal_z);
    if (ln.norm() < 1e-6f)
      continue;
    ln.normalize();
    if (std::abs(ln.dot(plane.normal)) >= min_dot)
      kept.push_back(idx);
  }
  if (static_cast<int>(kept.size()) >= params_.min_plane_inliers)
    plane.inliers.swap(kept);
}

void
GengRcim2022::keepLargestInlierCluster(FittedPlane& plane) const
{
  if (plane.inliers.size() < 2 || params_.plane_cluster_tol_mm <= 0.0f)
    return;

  pcl::PointCloud<pcl::PointXYZ>::Ptr subset(new pcl::PointCloud<pcl::PointXYZ>);
  subset->reserve(plane.inliers.size());
  for (const int idx : plane.inliers)
    subset->push_back((*cloud_)[static_cast<std::size_t>(idx)]);

  pcl::search::KdTree<pcl::PointXYZ> local;
  local.setInputCloud(subset);

  const float tol = params_.plane_cluster_tol_mm;
  std::vector<int> label(subset->size(), -1);
  int best_label = -1;
  int best_size = 0;
  int nlab = 0;
  std::vector<int> queue;
  queue.reserve(subset->size());

  for (std::size_t seed = 0; seed < subset->size(); ++seed)
  {
    if (label[seed] >= 0)
      continue;
    queue.clear();
    queue.push_back(static_cast<int>(seed));
    label[seed] = nlab;
    int sz = 0;
    for (std::size_t head = 0; head < queue.size(); ++head)
    {
      ++sz;
      pcl::Indices nn;
      std::vector<float> nn_dist;
      local.radiusSearch(queue[head], tol, nn, nn_dist);
      for (const auto j : nn)
      {
        if (label[static_cast<std::size_t>(j)] < 0)
        {
          label[static_cast<std::size_t>(j)] = nlab;
          queue.push_back(static_cast<int>(j));
        }
      }
    }
    if (sz > best_size)
    {
      best_size = sz;
      best_label = nlab;
    }
    ++nlab;
  }

  if (best_label < 0)
    return;

  std::vector<int> kept;
  kept.reserve(static_cast<std::size_t>(best_size));
  for (std::size_t i = 0; i < label.size(); ++i)
  {
    if (label[i] == best_label)
      kept.push_back(plane.inliers[i]);
  }
  plane.inliers.swap(kept);
}

void
GengRcim2022::cleanSegmentedPlanes()
{
  if (planes_.empty() || !cloud_)
    return;

  estimateNormals();
  reassignPointsToNearestPlane();
  for (auto& plane : planes_)
  {
    filterPlaneByNormal(plane);
    keepLargestInlierCluster(plane);
    refinePlane(plane);
  }

  planes_.erase(std::remove_if(planes_.begin(),
                               planes_.end(),
                               [&](const FittedPlane& plane) {
                                 return static_cast<int>(plane.inliers.size()) <
                                        params_.min_plane_inliers;
                               }),
                planes_.end());
}

void
GengRcim2022::materializeSegmentedPlanes()
{
  segmented_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
  std::vector<char> taken(cloud_ ? cloud_->size() : 0, 0);

  for (auto& plane : planes_)
  {
    plane.points.clear();
    plane.points.reserve(plane.inliers.size());
    for (const int idx : plane.inliers)
    {
      if (idx < 0 || idx >= static_cast<int>(taken.size()) || taken[static_cast<std::size_t>(idx)])
        continue;
      taken[static_cast<std::size_t>(idx)] = 1;
      plane.points.push_back((*cloud_)[idx]);
    }
    plane.points.width = static_cast<std::uint32_t>(plane.points.size());
    plane.points.height = 1;
    plane.points.is_dense = true;
    *segmented_cloud_ += plane.points;
  }
  segmented_cloud_->width = static_cast<std::uint32_t>(segmented_cloud_->size());
  segmented_cloud_->height = 1;
  segmented_cloud_->is_dense = true;
}

void
GengRcim2022::extractPlanes()
{
  planes_.clear();
  std::vector<int> remaining(cloud_->size());
  std::iota(remaining.begin(), remaining.end(), 0);

  while (static_cast<int>(planes_.size()) < params_.max_planes &&
         static_cast<int>(remaining.size()) >= params_.min_plane_inliers)
  {
    FittedPlane plane;
    if (!ransacOnePlane(remaining, plane))
      break;

    std::vector<char> used(cloud_->size(), 0);
    for (const int idx : plane.inliers)
      used[static_cast<std::size_t>(idx)] = 1;

    std::vector<int> next;
    next.reserve(remaining.size() - plane.inliers.size());
    for (const int idx : remaining)
    {
      if (!used[static_cast<std::size_t>(idx)])
        next.push_back(idx);
    }
    remaining.swap(next);
    planes_.push_back(std::move(plane));
  }
}

void
GengRcim2022::mergeSimilarPlanes()
{
  const float min_dot = std::cos(params_.merge_normal_deg * kPi / 180.0f);
  bool merged = true;
  while (merged)
  {
    merged = false;
    for (std::size_t i = 0; i < planes_.size(); ++i)
    {
      for (std::size_t j = i + 1; j < planes_.size(); ++j)
      {
        Eigen::Vector3f n2 = planes_[j].normal;
        float d2 = planes_[j].d;
        if (planes_[i].normal.dot(n2) < 0.0f)
        {
          n2 = -n2;
          d2 = -d2;
        }
        if (planes_[i].normal.dot(n2) < min_dot)
          continue;
        if (std::abs(planes_[i].d - d2) > params_.merge_offset_mm)
          continue;

        planes_[i].inliers.insert(
            planes_[i].inliers.end(), planes_[j].inliers.begin(), planes_[j].inliers.end());
        std::sort(planes_[i].inliers.begin(), planes_[i].inliers.end());
        planes_[i].inliers.erase(
            std::unique(planes_[i].inliers.begin(), planes_[i].inliers.end()),
            planes_[i].inliers.end());
        refinePlane(planes_[i]);
        planes_.erase(planes_.begin() + static_cast<std::ptrdiff_t>(j));
        merged = true;
        break;
      }
      if (merged)
        break;
    }
  }
}

Eigen::Vector3f
GengRcim2022::tiltTorchInward(const Eigen::Vector3f& torch_z,
                              const Eigen::Vector3f& inward,
                              float tilt_deg)
{
  Eigen::Vector3f z = torch_z;
  Eigen::Vector3f in = inward;
  if (z.norm() < 1e-8f || in.norm() < 1e-8f)
    return z.norm() < 1e-8f ? Eigen::Vector3f::UnitZ() : z.normalized();
  z.normalize();
  in.normalize();

  Eigen::Vector3f side = in - in.dot(z) * z;
  if (side.norm() < 1e-6f)
    return z;
  side.normalize();

  const float rad = tilt_deg * kPi / 180.0f;
  return (std::cos(rad) * z + std::sin(rad) * side).normalized();
}

void
GengRcim2022::updateTrajectoryNormals(WeldSeam& seam) const
{
  if (seam.trajectory.empty())
    return;

  Eigen::Vector3f dir = seam.end - seam.start;
  if (dir.norm() < 1e-8f)
    dir = seam.line_dir;
  if (dir.norm() < 1e-8f)
    dir = Eigen::Vector3f::UnitY();
  dir.normalize();

  const Eigen::Vector3f n_mid = seam.torch_z.normalized();
  const Eigen::Vector3f n_start = tiltTorchInward(n_mid, dir, params_.end_tilt_deg);
  const Eigen::Vector3f n_end = tiltTorchInward(n_mid, -dir, params_.end_tilt_deg);
  const int last = static_cast<int>(seam.trajectory.size()) - 1;

  for (int i = 0; i <= last; ++i)
  {
    Eigen::Vector3f n = n_mid;
    if (i == 0)
      n = n_start;
    else if (i == last)
      n = n_end;
    seam.trajectory[static_cast<std::size_t>(i)].getNormalVector3fMap() = n;
    seam.trajectory[static_cast<std::size_t>(i)].curvature = 0.0f;
  }
}

void
GengRcim2022::buildTrajectory(WeldSeam& seam) const
{
  seam.trajectory.clear();
  const Eigen::Vector3f vec = seam.end - seam.start;
  const float length = vec.norm();
  if (length < 1e-4f)
    return;

  const int steps = std::max(1, static_cast<int>(std::ceil(length / params_.trajectory_step_mm)));
  for (int i = 0; i <= steps; ++i)
  {
    const float t = static_cast<float>(i) / static_cast<float>(steps);
    const Eigen::Vector3f p = seam.start + t * vec;
    pcl::PointNormal q;
    q.getVector3fMap() = p;
    q.getNormalVector3fMap() = Eigen::Vector3f::Zero();
    q.curvature = 0.0f;
    seam.trajectory.push_back(q);
  }
  seam.trajectory.width = static_cast<std::uint32_t>(seam.trajectory.size());
  seam.trajectory.height = 1;
  seam.trajectory.is_dense = true;
  updateTrajectoryNormals(seam);
}

float
GengRcim2022::pointToPlaneDistance(const Eigen::Vector3f& p, const FittedPlane& plane)
{
  return std::abs(plane.normal.dot(p) + plane.d);
}

bool
GengRcim2022::liesOnOtherPlane(const Eigen::Vector3f& p,
                               const FittedPlane& a,
                               const FittedPlane& b) const
{
  const float tol = params_.plane_dist_mm;
  for (const auto& plane : planes_)
  {
    if (&plane == &a || &plane == &b)
      continue;
    if (pointToPlaneDistance(p, plane) <= tol)
      return true;
  }
  return false;
}

void
GengRcim2022::collectTwoPlaneSeamPoints(const FittedPlane& a,
                                        const FittedPlane& b,
                                        const Eigen::Vector3f& origin,
                                        const Eigen::Vector3f& dir,
                                        std::vector<float>& t_a,
                                        std::vector<float>& t_b,
                                        pcl::PointCloud<pcl::PointXYZ>& seam_cloud) const
{
  t_a.clear();
  t_b.clear();
  seam_cloud.clear();

  auto collect = [&](const FittedPlane& plane, std::vector<float>& ts) {
    for (const auto& pt : plane.points)
    {
      const Eigen::Vector3f p = pt.getVector3fMap();
      if (pointToPlaneDistance(p, plane) > params_.plane_dist_mm)
        continue;
      if (pointToLineDistance(p, origin, dir) > params_.seam_band_mm)
        continue;
      if (liesOnOtherPlane(p, a, b))
        continue;
      ts.push_back((p - origin).dot(dir));
      seam_cloud.push_back(pt);
    }
  };
  collect(a, t_a);
  collect(b, t_b);
}

bool
GengRcim2022::buildSeam(const FittedPlane& a, const FittedPlane& b, WeldSeam& seam) const
{
  const float dihedral = unsignedAngleDeg(a.normal, b.normal);
  if (dihedral < params_.min_dihedral_deg || dihedral > params_.max_dihedral_deg)
    return false;

  Eigen::Vector3f dir = a.normal.cross(b.normal);
  if (dir.norm() < 1e-5f)
    return false;
  dir.normalize();

  const Eigen::Vector3f origin = pointOnIntersection(a, b);
  std::vector<float> t_a;
  std::vector<float> t_b;
  collectTwoPlaneSeamPoints(a, b, origin, dir, t_a, t_b, seam.seam_cloud);

  if (static_cast<int>(t_a.size()) < params_.min_plane_support ||
      static_cast<int>(t_b.size()) < params_.min_plane_support)
    return false;

  const auto span = [](const std::vector<float>& ts) {
    const auto mm = std::minmax_element(ts.begin(), ts.end());
    return std::make_pair(*mm.first, *mm.second);
  };
  const auto sa = span(t_a);
  const auto sb = span(t_b);
  const float t0 = std::max(sa.first, sb.first);
  const float t1 = std::min(sa.second, sb.second);
  const float t_lo = t0;
  const float t_hi = t1;
  if (t_hi - t_lo < params_.min_seam_length_mm)
    return false;

  pcl::PointCloud<pcl::PointXYZ> cropped;
  cropped.reserve(seam.seam_cloud.size());
  for (const auto& p : seam.seam_cloud)
  {
    const float t = (p.getVector3fMap() - origin).dot(dir);
    if (t >= t_lo && t <= t_hi)
      cropped.push_back(p);
  }
  seam.seam_cloud.swap(cropped);

  seam.start = origin + t_lo * dir;
  seam.end = origin + t_hi * dir;
  seam.line_dir = dir;
  seam.length_mm = t_hi - t_lo;
  seam.seam_cloud.width = static_cast<std::uint32_t>(seam.seam_cloud.size());
  seam.seam_cloud.height = 1;
  seam.seam_cloud.is_dense = true;

  Eigen::Vector3f n1 = a.normal;
  Eigen::Vector3f n2 = b.normal;
  const Eigen::Vector3f mid = 0.5f * (seam.start + seam.end);
  auto toward = [&](Eigen::Vector3f& n, const FittedPlane& other) {
    Eigen::Vector3f v = other.centroid - mid;
    v -= v.dot(dir) * dir;
    if (v.norm() < 1e-4f)
      return;
    if (n.dot(v) < 0.0f)
      n = -n;
  };
  toward(n1, b);
  toward(n2, a);
  seam.n1 = n1;
  seam.n2 = n2;
  seam.torch_z = (n1 + n2);
  if (seam.torch_z.norm() < 1e-6f)
    seam.torch_z = n1;
  else
    seam.torch_z.normalize();

  buildTrajectory(seam);
  return !seam.trajectory.empty();
}

void
GengRcim2022::extractSeamsAndTrajectories()
{
  seams_.clear();
  for (std::size_t i = 0; i < planes_.size(); ++i)
  {
    for (std::size_t j = i + 1; j < planes_.size(); ++j)
    {
      WeldSeam seam;
      if (buildSeam(planes_[i], planes_[j], seam))
        seams_.push_back(std::move(seam));
    }
  }
}
