#pragma once

// Requires linking Qhull at the final binary (MSVC LNK2001 on qh_* if missing):
//   qhull_r.lib  or  qhullstatic_r.lib   (ONLY the reentrant *_r lib)
// pcl::ConvexHull only references these symbols; they are defined in Qhull.
// Mixing qhullstatic.lib + qhullstatic_r.lib causes LNK2005 and can crash at runtime
// inside hull.reconstruct() / qh_new_qhull().

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>
#include <pcl/conversions.h>
#include <pcl/surface/convex_hull.h>

#include <Eigen/Core>

#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

namespace pcl_utils {

/// Result of Hidden Point Removal (Katz et al., 2007), matching Open3D's
/// `PointCloud::HiddenPointRemoval` return value:
/// - `mesh`: triangle mesh of the visible surface (origin/camera vertex removed)
/// - `visible_indices`: indices of visible points in the *original* input cloud
struct HiddenPointRemovalResult {
  pcl::PolygonMesh mesh;
  std::vector<int> visible_indices;
};

namespace detail {

inline void triangulatePolygon(const pcl::Indices& verts,
                               std::vector<pcl::Vertices>& triangles) {
  if (verts.size() < 3) {
    return;
  }
  // Fan triangulation around verts[0], same practical outcome as qhull "Qt".
  for (std::size_t i = 1; i + 1 < verts.size(); ++i) {
    pcl::Vertices tri;
    tri.vertices = {verts[0], verts[i], verts[i + 1]};
    triangles.push_back(std::move(tri));
  }
}

template <typename PointT>
inline Eigen::Vector3d toVector3d(const PointT& p) {
  return Eigen::Vector3d(static_cast<double>(p.x), static_cast<double>(p.y),
                         static_cast<double>(p.z));
}

template <typename PointT>
inline PointT fromVector3d(const Eigen::Vector3d& v) {
  PointT p;
  p.x = static_cast<decltype(p.x)>(v.x());
  p.y = static_cast<decltype(p.y)>(v.y());
  p.z = static_cast<decltype(p.z)>(v.z());
  return p;
}

inline bool isFiniteVec3(const Eigen::Vector3d& v) {
  return std::isfinite(v.x()) && std::isfinite(v.y()) && std::isfinite(v.z());
}

/// PointT is usually float; Qhull crashes/aborts on NaN/Inf projected coords.
template <typename PointT>
inline bool fitsPointT(const Eigen::Vector3d& v) {
  using Scalar = decltype(std::declval<PointT>().x);
  const double lo = static_cast<double>(std::numeric_limits<Scalar>::lowest());
  const double hi = static_cast<double>(std::numeric_limits<Scalar>::max());
  return isFiniteVec3(v) && v.x() >= lo && v.x() <= hi && v.y() >= lo &&
         v.y() <= hi && v.z() >= lo && v.z() <= hi;
}

}  // namespace detail

/// Open3D-compatible Hidden Point Removal for PCL (>= 1.12).
///
/// Implements Katz et al. "Direct Visibility of Point Sets", 2007:
/// 1) spherical projection from \p camera_location with radius \p radius
/// 2) append the projection-space origin
/// 3) compute the 3D convex hull
/// 4) map hull vertices back to original points and drop the origin vertex
///
/// \param cloud Input point cloud (XYZ fields required).
/// \param camera_location Viewpoint; points not visible from here are removed.
/// \param radius Spherical projection radius; must be > 0 (Open3D constraint).
///        Typical choice: ~100 * scene diameter. Too large with float PointT
///        overflows projection coords and makes Qhull crash inside reconstruct().
/// \return Visible mesh + original-cloud indices of visible points.
///
/// Requires PCL built with Qhull support (`pcl::ConvexHull`).
///
/// Why `hull.reconstruct(...)` may "crash":
/// - It calls Qhull (`qh_new_qhull`). Bad geometry / NaN / Inf often abort there.
/// - Wrong Qhull link (non-`_r` lib, or mixed static libs) → access violation.
/// - Fewer than 4 valid projected points for a 3D hull.
template <typename PointT>
HiddenPointRemovalResult HiddenPointRemoval(
    const typename pcl::PointCloud<PointT>::ConstPtr& cloud,
    const Eigen::Vector3d& camera_location,
    double radius) {
  if (!cloud) {
    throw std::invalid_argument("HiddenPointRemoval: cloud is null.");
  }
  if (radius <= 0.0 || !std::isfinite(radius)) {
    throw std::invalid_argument(
        "HiddenPointRemoval: radius must be finite and larger than zero.");
  }
  if (!detail::isFiniteVec3(camera_location)) {
    throw std::invalid_argument(
        "HiddenPointRemoval: camera_location contains NaN/Inf.");
  }
  if (cloud->empty()) {
    return {};
  }

  using CloudT = pcl::PointCloud<PointT>;

  // Map projected-cloud index -> original cloud index (-1 = skipped invalid).
  // We build a dense projected cloud of only valid points, plus origin.
  std::vector<int> projected_to_original;
  projected_to_original.reserve(cloud->size());

  // 1) Spherical projection (identical formula to Open3D).
  typename CloudT::Ptr projected(new CloudT);
  projected->reserve(cloud->size() + 1);

  for (std::size_t i = 0; i < cloud->size(); ++i) {
    const auto& pt = (*cloud)[i];
    const Eigen::Vector3d p = detail::toVector3d(pt);
    if (!detail::isFiniteVec3(p)) {
      continue;  // NaN/Inf input → Qhull crash if kept
    }

    const Eigen::Vector3d projected_point = p - camera_location;
    const double norm = projected_point.norm();
    if (!(norm > 0.0) || !std::isfinite(norm)) {
      // coincident with camera, or non-finite
      continue;
    }

    const Eigen::Vector3d sph =
        projected_point +
        2.0 * (radius - norm) * projected_point / norm;

    // float PointXYZ overflow is a very common cause of reconstruct() crash
    if (!detail::fitsPointT<PointT>(sph)) {
      throw std::runtime_error(
          "HiddenPointRemoval: spherical projection overflowed PointT range. "
          "Reduce radius (e.g. ~100 * scene diameter) or use smaller coordinates.");
    }

    projected->push_back(detail::fromVector3d<PointT>(sph));
    projected_to_original.push_back(static_cast<int>(i));
  }

  // 3D convex hull needs >= 4 points; origin adds one.
  if (projected->size() < 3) {
    throw std::runtime_error(
        "HiddenPointRemoval: need at least 3 finite points not coincident "
        "with the camera (got " +
        std::to_string(projected->size()) + ").");
  }

  // 2) Add projection-space origin (camera). Marked with original index -1.
  projected->push_back(detail::fromVector3d<PointT>(Eigen::Vector3d::Zero()));
  projected_to_original.push_back(-1);
  projected->width = static_cast<uint32_t>(projected->size());
  projected->height = 1;
  projected->is_dense = true;

  // 3) Convex hull of spherical projection (Open3D uses Qhull; PCL does too).
  // NOTE: reconstruct() itself only allocates outputs; the crash is almost
  // always inside Qhull when input is invalid or the wrong qhull*_r is linked.
  pcl::ConvexHull<PointT> hull;
  hull.setInputCloud(projected);
  hull.setDimension(3);
  // Avoid area/volume mode: it forces console output and extra Qhull options.
  hull.setComputeAreaVolume(false);

  typename CloudT::Ptr hull_points(new CloudT);
  std::vector<pcl::Vertices> polygons;
  try {
    hull.reconstruct(*hull_points, polygons);
  } catch (const std::exception& e) {
    throw std::runtime_error(
        std::string("HiddenPointRemoval: ConvexHull::reconstruct failed: ") +
        e.what());
  } catch (...) {
    throw std::runtime_error(
        "HiddenPointRemoval: ConvexHull::reconstruct failed (unknown). "
        "Check Qhull link (use only qhullstatic_r / qhull_r) and input NaNs.");
  }

  if (hull_points->empty() || polygons.empty()) {
    // PCL returns empty on Qhull failure instead of throwing; treat as error.
    throw std::runtime_error(
        "HiddenPointRemoval: Qhull failed to compute a 3D convex hull. "
        "Check degenerate geometry, radius, and that only qhull*_r is linked.");
  }

  pcl::PointIndices hull_point_indices;
  hull.getHullPointIndices(hull_point_indices);
  std::vector<int> pt_map = hull_point_indices.indices;

  if (pt_map.size() != hull_points->size()) {
    throw std::runtime_error(
        "HiddenPointRemoval: hull point / index map size mismatch. "
        "Ensure PCL was built with a matching reentrant Qhull (qhull*_r).");
  }

  // Remap hull indices from projected-cloud space to original-cloud indices.
  // pt_map[i] is an index into `projected` (including origin at origin_pidx).
  std::vector<int> original_pt_map;
  original_pt_map.reserve(pt_map.size());
  for (int pidx : pt_map) {
    if (pidx < 0 || static_cast<std::size_t>(pidx) >= projected_to_original.size()) {
      throw std::runtime_error(
          "HiddenPointRemoval: invalid hull point index from ConvexHull.");
    }
    original_pt_map.push_back(projected_to_original[static_cast<std::size_t>(pidx)]);
  }

  // 4) Reassign original points onto hull vertices.
  int origin_vidx = static_cast<int>(original_pt_map.size());
  for (std::size_t vidx = 0; vidx < original_pt_map.size(); ++vidx) {
    const int orig_idx = original_pt_map[vidx];
    if (orig_idx >= 0) {
      (*hull_points)[vidx] = (*cloud)[static_cast<std::size_t>(orig_idx)];
    } else {
      // synthetic origin
      origin_vidx = static_cast<int>(vidx);
      (*hull_points)[vidx] = detail::fromVector3d<PointT>(camera_location);
    }
  }

  // Triangulate polygons so the mesh matches Open3D's TriangleMesh.
  std::vector<pcl::Vertices> triangles;
  triangles.reserve(polygons.size());
  for (const auto& poly : polygons) {
    detail::triangulatePolygon(poly.vertices, triangles);
  }

  // 5) Erase origin if it is part of the hull (same bookkeeping as Open3D).
  if (origin_vidx < static_cast<int>(hull_points->size())) {
    hull_points->points.erase(hull_points->points.begin() + origin_vidx);
    original_pt_map.erase(original_pt_map.begin() +
                          static_cast<std::size_t>(origin_vidx));

    for (std::size_t tidx = triangles.size(); tidx-- > 0;) {
      auto& verts = triangles[tidx].vertices;
      const bool touches_origin = verts[0] == origin_vidx ||
                                  verts[1] == origin_vidx ||
                                  verts[2] == origin_vidx;
      if (touches_origin) {
        triangles.erase(triangles.begin() + static_cast<std::ptrdiff_t>(tidx));
        continue;
      }
      for (auto& v : verts) {
        if (v > origin_vidx) {
          --v;
        }
      }
    }
  }

  hull_points->width = static_cast<uint32_t>(hull_points->size());
  hull_points->height = 1;
  hull_points->is_dense = true;

  std::vector<int> visible_indices;
  visible_indices.reserve(original_pt_map.size());
  for (int idx : original_pt_map) {
    if (idx >= 0) {
      visible_indices.push_back(idx);
    }
  }

  HiddenPointRemovalResult result;
  result.visible_indices = std::move(visible_indices);
  result.mesh.polygons = std::move(triangles);
  pcl::toPCLPointCloud2(*hull_points, result.mesh.cloud);
  return result;
}

/// Convenience overload with Eigen::Vector3f camera (common in PCL code).
template <typename PointT>
HiddenPointRemovalResult HiddenPointRemoval(
    const typename pcl::PointCloud<PointT>::ConstPtr& cloud,
    const Eigen::Vector3f& camera_location,
    double radius) {
  return HiddenPointRemoval<PointT>(
      cloud, camera_location.cast<double>(), radius);
}

/// Extract only the visible points into a new cloud.
template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr ExtractVisiblePoints(
    const typename pcl::PointCloud<PointT>::ConstPtr& cloud,
    const std::vector<int>& visible_indices) {
  typename pcl::PointCloud<PointT>::Ptr out(new pcl::PointCloud<PointT>);
  if (!cloud) {
    return out;
  }
  out->reserve(visible_indices.size());
  for (int idx : visible_indices) {
    if (idx >= 0 && static_cast<std::size_t>(idx) < cloud->size()) {
      out->push_back((*cloud)[static_cast<std::size_t>(idx)]);
    }
  }
  out->width = static_cast<uint32_t>(out->size());
  out->height = 1;
  out->is_dense = cloud->is_dense;
  return out;
}

}  // namespace pcl_utils
