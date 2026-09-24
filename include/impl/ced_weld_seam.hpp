#ifndef CED_WELD_SEAM_HPP_
#define CED_WELD_SEAM_HPP_

#include <Eigen/Dense>

#include <cmath>
#include <limits>
#include <queue>

namespace
{
constexpr float kPi = 3.14159265358979323846f;
}

template <typename PointInT, typename PointOutT> bool
pcl::CEDWeldSeamDetector<PointInT, PointOutT>::initCompute ()
{
  if (!Keypoint<PointInT, PointOutT>::initCompute ())
  {
    PCL_ERROR ("[pcl::%s::initCompute] init failed!\n", name_.c_str ());
    return (false);
  }
  if (search_radius_ <= 0.0)
  {
    PCL_ERROR ("[pcl::%s::initCompute] search radius (%f) must be positive!\n",
               name_.c_str (), search_radius_);
    return (false);
  }
  if (min_neighbors_ <= 0)
  {
    PCL_ERROR ("[pcl::%s::initCompute] min neighbors (%d) must be positive!\n",
               name_.c_str (), min_neighbors_);
    return (false);
  }
  if (min_dihedral_deg_ < 0.0 || max_dihedral_deg_ > 180.0 || min_dihedral_deg_ >= max_dihedral_deg_)
  {
    PCL_ERROR ("[pcl::%s::initCompute] invalid dihedral range [%f, %f]\n",
               name_.c_str (), min_dihedral_deg_, max_dihedral_deg_);
    return (false);
  }

  resolved_support_radius_ = (support_radius_ > 0.0) ? support_radius_ : 1.3 * search_radius_;
  resolved_plane_distance_ = (plane_distance_threshold_ > 0.0)
                                 ? plane_distance_threshold_
                                 : 0.18 * search_radius_;
  resolved_seam_band_ = (seam_band_width_ > 0.0) ? seam_band_width_ : 0.35 * search_radius_;
  resolved_cluster_gap_ = (cluster_gap_radius_ > 0.0) ? cluster_gap_radius_ : search_radius_;

  if (input_normals_ && static_cast<int> (input_normals_->size ()) != static_cast<int> (input_->size ()))
  {
    PCL_ERROR ("[pcl::%s::initCompute] input normals size (%zu) != cloud size (%zu)\n",
               name_.c_str (),
               static_cast<std::size_t> (input_normals_->size ()),
               static_cast<std::size_t> (input_->size ()));
    return (false);
  }

  estimateNormalsIfNeeded ();
  if (!normals_ || static_cast<int> (normals_->size ()) != static_cast<int> (input_->size ()))
  {
    PCL_ERROR ("[pcl::%s::initCompute] failed to prepare normals!\n", name_.c_str ());
    return (false);
  }
  return (true);
}


template <typename PointInT, typename PointOutT> void
pcl::CEDWeldSeamDetector<PointInT, PointOutT>::estimateNormalsIfNeeded ()
{
  if (input_normals_)
  {
    normals_ = input_normals_;
    return;
  }

  computed_normals_.reset (new NormalCloud);
  pcl::NormalEstimation<PointInT, pcl::Normal> ne;
  ne.setInputCloud (input_);
  ne.setSearchMethod (tree_);
  const double radius = (normal_radius_ > 0.0) ? normal_radius_ : 0.6 * search_radius_;
  ne.setRadiusSearch (radius);
  ne.compute (*computed_normals_);
  normals_ = computed_normals_;
}


template <typename PointInT, typename PointOutT> float
pcl::CEDWeldSeamDetector<PointInT, PointOutT>::unsignedAngleDeg (const Eigen::Vector3f &a,
                                                                 const Eigen::Vector3f &b)
{
  const float na = a.norm ();
  const float nb = b.norm ();
  if (na < 1e-8f || nb < 1e-8f)
    return 0.0f;
  float c = std::abs (a.dot (b)) / (na * nb);
  c = std::min (1.0f, std::max (0.0f, c));
  return std::acos (c) * 180.0f / kPi;
}


template <typename PointInT, typename PointOutT> float
pcl::CEDWeldSeamDetector<PointInT, PointOutT>::distanceToIntersectionLine (
    const Eigen::Vector3f &query,
    const Eigen::Vector3f &c0,
    const Eigen::Vector3f &n0,
    const Eigen::Vector3f &c1,
    const Eigen::Vector3f &n1)
{
  Eigen::Vector3f dir = n0.cross (n1);
  const float dir_norm = dir.norm ();
  if (dir_norm < 1e-6f)
    return std::numeric_limits<float>::infinity ();
  dir /= dir_norm;

  const float n01 = n0.dot (n1);
  const float det = 1.0f - n01 * n01;
  if (std::abs (det) < 1e-6f)
    return std::numeric_limits<float>::infinity ();

  const float rhs0 = n0.dot (c0);
  const float rhs1 = n1.dot (c1);
  const float a = (rhs0 - rhs1 * n01) / det;
  const float b = (rhs1 - rhs0 * n01) / det;
  const Eigen::Vector3f x0 = a * n0 + b * n1;
  return (query - x0).cross (dir).norm ();
}


template <typename PointInT, typename PointOutT> bool
pcl::CEDWeldSeamDetector<PointInT, PointOutT>::fitPcaPlane (const std::vector<int> &indices,
                                                           Eigen::Vector3f &centroid,
                                                           Eigen::Vector3f &normal,
                                                           float &rms) const
{
  if (indices.size () < 3)
    return false;

  centroid = Eigen::Vector3f::Zero ();
  for (const int idx : indices)
    centroid += (*input_)[idx].getVector3fMap ();
  centroid /= static_cast<float> (indices.size ());

  Eigen::Matrix3f cov = Eigen::Matrix3f::Zero ();
  for (const int idx : indices)
  {
    const Eigen::Vector3f d = (*input_)[idx].getVector3fMap () - centroid;
    cov += d * d.transpose ();
  }
  cov /= static_cast<float> (indices.size ());

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver (cov, Eigen::ComputeEigenvectors);
  if (solver.info () != Eigen::Success)
    return false;

  normal = solver.eigenvectors ().col (0);
  if (normal.norm () < 1e-8f)
    return false;
  normal.normalize ();

  double sse = 0.0;
  for (const int idx : indices)
  {
    const float dist = std::abs (normal.dot ((*input_)[idx].getVector3fMap () - centroid));
    sse += static_cast<double> (dist) * static_cast<double> (dist);
  }
  rms = static_cast<float> (std::sqrt (sse / static_cast<double> (indices.size ())));
  return true;
}


template <typename PointInT, typename PointOutT> bool
pcl::CEDWeldSeamDetector<PointInT, PointOutT>::isTwoSurfaceJunction (
    int idx, const pcl::Indices &nn_indices) const
{
  struct Sample
  {
    int index;
    Eigen::Vector3f normal;
  };

  std::vector<Sample> samples;
  samples.reserve (nn_indices.size ());
  for (const auto nn_idx : nn_indices)
  {
    const pcl::Normal &n = (*normals_)[nn_idx];
    if (!std::isfinite (n.normal_x) || !std::isfinite (n.normal_y) || !std::isfinite (n.normal_z))
      continue;
    Eigen::Vector3f nv (n.normal_x, n.normal_y, n.normal_z);
    if (nv.norm () < 1e-6f)
      continue;
    nv.normalize ();
    samples.push_back ({static_cast<int> (nn_idx), nv});
  }

  const int sample_count = static_cast<int> (samples.size ());
  const int min_inliers = std::max (
      min_plane_inliers_,
      static_cast<int> (std::ceil (min_plane_inlier_ratio_ * static_cast<double> (sample_count))));
  if (sample_count < 2 * min_inliers)
    return false;

  int seed0 = 0;
  int seed1 = 1;
  float best_angle = -1.0f;
  for (int i = 1; i < sample_count; ++i)
  {
    const float angle = unsignedAngleDeg (samples.front ().normal,
                                          samples[static_cast<std::size_t> (i)].normal);
    if (angle > best_angle)
    {
      best_angle = angle;
      seed1 = i;
    }
  }
  best_angle = -1.0f;
  for (int i = 0; i < sample_count; ++i)
  {
    const float angle = unsignedAngleDeg (samples[static_cast<std::size_t> (seed1)].normal,
                                          samples[static_cast<std::size_t> (i)].normal);
    if (angle > best_angle)
    {
      best_angle = angle;
      seed0 = i;
    }
  }

  // 所有邻域法向彼此接近：这是单面（含自由边），不是两面焊缝。
  if (best_angle < static_cast<float> (min_dihedral_deg_))
    return false;

  Eigen::Vector3f mean0 = samples[static_cast<std::size_t> (seed0)].normal;
  Eigen::Vector3f mean1 = samples[static_cast<std::size_t> (seed1)].normal;
  std::vector<int> cluster0;
  std::vector<int> cluster1;
  cluster0.reserve (static_cast<std::size_t> (sample_count));
  cluster1.reserve (static_cast<std::size_t> (sample_count));

  for (int iter = 0; iter < 4; ++iter)
  {
    cluster0.clear ();
    cluster1.clear ();
    Eigen::Vector3f sum0 = Eigen::Vector3f::Zero ();
    Eigen::Vector3f sum1 = Eigen::Vector3f::Zero ();

    for (const auto &sample : samples)
    {
      const float d0 = std::abs (sample.normal.dot (mean0));
      const float d1 = std::abs (sample.normal.dot (mean1));
      if (d0 >= d1)
      {
        cluster0.push_back (sample.index);
        sum0 += (sample.normal.dot (mean0) >= 0.0f) ? sample.normal : -sample.normal;
      }
      else
      {
        cluster1.push_back (sample.index);
        sum1 += (sample.normal.dot (mean1) >= 0.0f) ? sample.normal : -sample.normal;
      }
    }

    if (cluster0.empty () || cluster1.empty ())
      return false;
    mean0 = sum0.normalized ();
    mean1 = sum1.normalized ();
  }

  if (static_cast<int> (cluster0.size ()) < min_inliers ||
      static_cast<int> (cluster1.size ()) < min_inliers)
    return false;

  const float dihedral = unsignedAngleDeg (mean0, mean1);
  if (dihedral < static_cast<float> (min_dihedral_deg_) ||
      dihedral > static_cast<float> (max_dihedral_deg_))
    return false;

  Eigen::Vector3f c0, n0, c1, n1;
  float rms0 = 0.0f;
  float rms1 = 0.0f;
  if (!fitPcaPlane (cluster0, c0, n0, rms0) || !fitPcaPlane (cluster1, c1, n1, rms1))
    return false;

  const float max_rms = static_cast<float> (max_plane_rms_scale_ * resolved_plane_distance_);
  if (rms0 > max_rms || rms1 > max_rms)
    return false;

  const Eigen::Vector3f query = (*input_)[idx].getVector3fMap ();
  const float line_dist = distanceToIntersectionLine (query, c0, n0, c1, n1);
  return line_dist <= static_cast<float> (resolved_seam_band_);
}


template <typename PointInT, typename PointOutT> void
pcl::CEDWeldSeamDetector<PointInT, PointOutT>::removeSmallSeamClusters (
    std::vector<char> &is_seam) const
{
  if (min_seam_cluster_size_ <= 1)
    return;

  const int n = static_cast<int> (is_seam.size ());
  std::vector<int> labels (static_cast<std::size_t> (n), -1);
  std::vector<int> cluster_sizes;
  int current = 0;

  for (int i = 0; i < n; ++i)
  {
    if (!is_seam[static_cast<std::size_t> (i)] || labels[static_cast<std::size_t> (i)] >= 0)
      continue;

    std::queue<int> q;
    q.push (i);
    labels[static_cast<std::size_t> (i)] = current;
    int size = 0;

    while (!q.empty ())
    {
      const int u = q.front ();
      q.pop ();
      ++size;

      pcl::Indices nn_indices;
      std::vector<float> nn_dists;
      tree_->radiusSearch (u, resolved_cluster_gap_, nn_indices, nn_dists);
      for (const auto v : nn_indices)
      {
        if (is_seam[static_cast<std::size_t> (v)] && labels[static_cast<std::size_t> (v)] < 0)
        {
          labels[static_cast<std::size_t> (v)] = current;
          q.push (static_cast<int> (v));
        }
      }
    }

    cluster_sizes.push_back (size);
    ++current;
  }

  for (int i = 0; i < n; ++i)
  {
    if (!is_seam[static_cast<std::size_t> (i)])
      continue;
    const int label = labels[static_cast<std::size_t> (i)];
    if (label < 0 || cluster_sizes[static_cast<std::size_t> (label)] < min_seam_cluster_size_)
      is_seam[static_cast<std::size_t> (i)] = 0;
  }
}


template <typename PointInT, typename PointOutT> void
pcl::CEDWeldSeamDetector<PointInT, PointOutT>::detectKeypoints (PointCloudOut &output)
{
  output.clear ();
  keypoints_indices_->indices.clear ();

  const int input_size = static_cast<int> (input_->size ());
  centroid_distances_.assign (static_cast<std::size_t> (input_size), 0.0f);
  std::vector<char> is_seam (static_cast<std::size_t> (input_size), 0);

  for (int idx = 0; idx < input_size; ++idx)
  {
    pcl::Indices nn_indices;
    std::vector<float> nn_dists;
    tree_->radiusSearch (idx, search_radius_, nn_indices, nn_dists);
    if (static_cast<int> (nn_indices.size ()) < min_neighbors_)
      continue;

    Eigen::Vector3f geo_centroid = Eigen::Vector3f::Zero ();
    for (const auto nn_idx : nn_indices)
      geo_centroid += (*input_)[nn_idx].getVector3fMap ();
    geo_centroid /= static_cast<float> (nn_indices.size ());
    const float ced = (geo_centroid - (*input_)[idx].getVector3fMap ()).norm ();
    centroid_distances_[static_cast<std::size_t> (idx)] = ced;

    if (ced < static_cast<float> (centroid_threshold_ * search_radius_))
      continue;

    pcl::Indices support_indices;
    std::vector<float> support_dists;
    if (std::abs (resolved_support_radius_ - search_radius_) < 1e-12)
    {
      support_indices = nn_indices;
    }
    else
    {
      tree_->radiusSearch (idx, resolved_support_radius_, support_indices, support_dists);
    }

    if (isTwoSurfaceJunction (idx, support_indices))
      is_seam[static_cast<std::size_t> (idx)] = 1;
  }

  removeSmallSeamClusters (is_seam);

  if (apply_non_max_suppression_)
  {
    const double nms_radius = (non_max_radius_ > 0.0) ? non_max_radius_ : search_radius_;
    for (int idx = 0; idx < input_size; ++idx)
    {
      if (!is_seam[static_cast<std::size_t> (idx)])
        continue;

      pcl::Indices nn_indices;
      std::vector<float> nn_dists;
      tree_->radiusSearch (idx, nms_radius, nn_indices, nn_dists);
      bool maximum = true;
      for (const auto nn_idx : nn_indices)
      {
        if (!is_seam[static_cast<std::size_t> (nn_idx)])
          continue;
        if (centroid_distances_[static_cast<std::size_t> (idx)] <
            centroid_distances_[static_cast<std::size_t> (nn_idx)])
        {
          maximum = false;
          break;
        }
      }
      if (!maximum)
        is_seam[static_cast<std::size_t> (idx)] = 0;
    }
  }

  output.reserve (static_cast<std::size_t> (input_size) / 20 + 8);
  for (int idx = 0; idx < input_size; ++idx)
  {
    if (!is_seam[static_cast<std::size_t> (idx)])
      continue;
    PointOutT p;
    p.getVector3fMap () = (*input_)[idx].getVector3fMap ();
    output.push_back (p);
    keypoints_indices_->indices.push_back (idx);
  }

  output.height = 1;
  output.width = static_cast<std::uint32_t> (output.size ());
  output.is_dense = true;
}

#endif  // CED_WELD_SEAM_HPP_
