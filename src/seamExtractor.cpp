#include "seamExtractor.h"

#include <pcl/common/centroid.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <random>
#include <utility>

namespace
{
    constexpr float kPi = 3.14159265358979323846f;
}  // namespace

void
SeamExtra::setInputCloud(const ct::Cloud::Ptr& cloud)
{
    input_ = cloud;
}

float
SeamExtra::planeDist(const Eigen::Vector3f& p, const FittedPlane& plane)
{
    return std::abs(plane.normal.dot(p) + plane.d);
}

float
SeamExtra::lineDist(const Eigen::Vector3f& p,
    const Eigen::Vector3f& origin,
    const Eigen::Vector3f& dir)
{
    return (p - origin).cross(dir).norm();
}

float
SeamExtra::angleDeg(const Eigen::Vector3f& a, const Eigen::Vector3f& b)
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
SeamExtra::intersectionPoint(const FittedPlane& a, const FittedPlane& b)
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

void
SeamExtra::downsample()
{
    if (params_.voxel_leaf_mm <= 0.0f)
    {
        cloud_.reset(new ct::Cloud(*input_));
        return;
    }
    pcl::VoxelGrid<pcl::PointXYZRGBNormal> grid;
    grid.setInputCloud(input_);
    grid.setLeafSize(params_.voxel_leaf_mm, params_.voxel_leaf_mm, params_.voxel_leaf_mm);
    cloud_.reset(new ct::Cloud);
    grid.filter(*cloud_);
}

void
SeamExtra::refinePlane(FittedPlane& plane) const
{
    if (plane.inliers.size() < 3)
        return;
    Eigen::Vector4f c;
    pcl::compute3DCentroid(*cloud_, plane.inliers, c);
    plane.centroid = c.head<3>();

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

void
SeamExtra::keepLargestCluster(FittedPlane& plane) const
{
    if (plane.inliers.size() < 2 || params_.plane_cluster_tol_mm <= 0.0f)
        return;

    ct::Cloud::Ptr subset(new ct::Cloud);
    for (const int idx : plane.inliers)
        subset->push_back((*cloud_)[static_cast<std::size_t>(idx)]);

    pcl::search::KdTree<pcl::PointXYZRGBNormal> local;
    local.setInputCloud(subset);

    std::vector<int> label(subset->size(), -1);
    int best_lab = -1;
    int best_sz = 0;
    int nlab = 0;
    std::vector<int> q;
    q.reserve(subset->size());

    for (std::size_t s = 0; s < subset->size(); ++s)
    {
        if (label[s] >= 0)
            continue;
        q.clear();
        q.push_back(static_cast<int>(s));
        label[s] = nlab;
        int sz = 0;
        for (std::size_t h = 0; h < q.size(); ++h)
        {
            ++sz;
            pcl::Indices nn;
            std::vector<float> dist;
            local.radiusSearch(q[h], params_.plane_cluster_tol_mm, nn, dist);
            for (const auto j : nn)
            {
                if (label[static_cast<std::size_t>(j)] < 0)
                {
                    label[static_cast<std::size_t>(j)] = nlab;
                    q.push_back(static_cast<int>(j));
                }
            }
        }
        if (sz > best_sz)
        {
            best_sz = sz;
            best_lab = nlab;
        }
        ++nlab;
    }

    std::vector<int> kept;
    kept.reserve(static_cast<std::size_t>(best_sz));
    for (std::size_t i = 0; i < label.size(); ++i)
    {
        if (label[i] == best_lab)
            kept.push_back(plane.inliers[i]);
    }
    plane.inliers.swap(kept);
}

bool
SeamExtra::ransacOnePlane(const std::vector<int>& remaining, FittedPlane& plane) const
{
    if (static_cast<int>(remaining.size()) < params_.min_plane_inliers)
        return false;

    std::mt19937 rng(params_.rng_seed + static_cast<std::uint32_t>(remaining.size()));
    std::uniform_int_distribution<int> pick(0, static_cast<int>(remaining.size()) - 1);
    std::vector<char> alive(cloud_->size(), 0);
    for (const int idx : remaining)
        alive[static_cast<std::size_t>(idx)] = 1;

    int best_n = 0;
    Eigen::Vector3f best_normal = Eigen::Vector3f::UnitZ();
    float best_d = 0.0f;

    for (int it = 0; it < params_.ransac_iters; ++it)
    {
        const int seed = remaining[static_cast<std::size_t>(pick(rng))];
        pcl::Indices nn;
        std::vector<float> nn_dist;
        tree_->radiusSearch(seed, params_.local_sample_radius_mm, nn, nn_dist);
        std::vector<int> local;
        for (const auto idx : nn)
        {
            if (alive[static_cast<std::size_t>(idx)])
                local.push_back(static_cast<int>(idx));
        }
        if (local.size() < 3)
            continue;

        std::uniform_int_distribution<int> pl(0, static_cast<int>(local.size()) - 1);
        const int i0 = local[static_cast<std::size_t>(pl(rng))];
        const int i1 = local[static_cast<std::size_t>(pl(rng))];
        const int i2 = local[static_cast<std::size_t>(pl(rng))];
        if (i0 == i1 || i0 == i2 || i1 == i2)
            continue;

        const Eigen::Vector3f p0 = (*cloud_)[i0].getVector3fMap();
        const Eigen::Vector3f p1 = (*cloud_)[i1].getVector3fMap();
        const Eigen::Vector3f p2 = (*cloud_)[i2].getVector3fMap();
        Eigen::Vector3f n = (p1 - p0).cross(p2 - p0);
        if (n.norm() < 1e-8f)
            continue;
        n.normalize();
        const float d = -n.dot(p0);

        int count = 0;
        for (const int idx : remaining)
        {
            if (std::abs(n.dot((*cloud_)[idx].getVector3fMap()) + d) <= params_.plane_dist_mm)
                ++count;
        }
        if (count > best_n)
        {
            best_n = count;
            best_normal = n;
            best_d = d;
        }
    }
    if (best_n < params_.min_plane_inliers)
        return false;

    plane.normal = best_normal;
    plane.d = best_d;
    plane.inliers.clear();
    for (const int idx : remaining)
    {
        if (std::abs(plane.normal.dot((*cloud_)[idx].getVector3fMap()) + plane.d) <=
            params_.plane_dist_mm)
            plane.inliers.push_back(idx);
    }
    refinePlane(plane);
    plane.inliers.clear();
    for (const int idx : remaining)
    {
        if (std::abs(plane.normal.dot((*cloud_)[idx].getVector3fMap()) + plane.d) <=
            params_.plane_dist_mm)
            plane.inliers.push_back(idx);
    }
    return static_cast<int>(plane.inliers.size()) >= params_.min_plane_inliers;
}

void
SeamExtra::extractPlanes()
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
        for (const int idx : remaining)
        {
            if (!used[static_cast<std::size_t>(idx)])
                next.push_back(idx);
        }
        remaining.swap(next);
        planes_.push_back(std::move(plane));
    }

    estimateNormals();

    std::vector<std::vector<int>> assigned(planes_.size());
    for (std::size_t i = 0; i < cloud_->size(); ++i)
    {
        const Eigen::Vector3f p = (*cloud_)[i].getVector3fMap();
        float best = std::numeric_limits<float>::infinity();
        int best_j = -1;
        for (std::size_t j = 0; j < planes_.size(); ++j)
        {
            const float dist = planeDist(p, planes_[j]);
            if (dist < best)
            {
                best = dist;
                best_j = static_cast<int>(j);
            }
        }
        if (best_j >= 0 && best <= params_.plane_dist_mm)
            assigned[static_cast<std::size_t>(best_j)].push_back(static_cast<int>(i));
    }

    segmented_cloud_.reset(new ct::Cloud);
    std::vector<FittedPlane> kept;
    for (std::size_t j = 0; j < planes_.size(); ++j)
    {
        planes_[j].inliers.swap(assigned[j]);
        refinePlane(planes_[j]);
        filterPlaneByNormal(planes_[j]);
        keepLargestCluster(planes_[j]);
        refinePlane(planes_[j]);
        if (static_cast<int>(planes_[j].inliers.size()) < params_.min_plane_inliers)
            continue;

        planes_[j].points.clear();
        for (const int idx : planes_[j].inliers)
            planes_[j].points.push_back((*cloud_)[idx]);
        planes_[j].points.width = static_cast<std::uint32_t>(planes_[j].points.size());
        planes_[j].points.height = 1;
        planes_[j].points.is_dense = true;
        *segmented_cloud_ += planes_[j].points;
        kept.push_back(std::move(planes_[j]));
    }
    planes_.swap(kept);
    segmented_cloud_->width = static_cast<std::uint32_t>(segmented_cloud_->size());
    segmented_cloud_->height = 1;
    segmented_cloud_->is_dense = true;
}

void
SeamExtra::estimateNormals()
{
    normals_.reset();
    if (!cloud_ || cloud_->empty() || params_.max_normal_dev_deg <= 0.0f)
        return;

    float radius = params_.local_sample_radius_mm;
    if (radius <= 0.0f)
        radius = std::max(6.0f, 3.0f * params_.voxel_leaf_mm);

    pcl::NormalEstimation<pcl::PointXYZRGBNormal, pcl::Normal> ne;
    ne.setInputCloud(cloud_);
    ne.setSearchMethod(tree_);
    ne.setRadiusSearch(radius);
    normals_.reset(new pcl::PointCloud<pcl::Normal>);
    ne.compute(*normals_);
}

bool
SeamExtra::nearOtherPlane(const Eigen::Vector3f& p, const FittedPlane& self, float tol) const
{
    for (const auto& plane : planes_)
    {
        if (&plane == &self)
            continue;
        if (planeDist(p, plane) <= tol)
            return true;
    }
    return false;
}

void
SeamExtra::filterPlaneByNormal(FittedPlane& plane) const
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
        const bool normal_ok = std::abs(ln.dot(plane.normal)) >= min_dot;
        const Eigen::Vector3f p = (*cloud_)[static_cast<std::size_t>(idx)].getVector3fMap();
        const float edge_tol = std::max(params_.plane_dist_mm, params_.local_sample_radius_mm);
        if (normal_ok || nearOtherPlane(p, plane, edge_tol))
            kept.push_back(idx);
    }
    if (static_cast<int>(kept.size()) >= params_.min_plane_inliers)
        plane.inliers.swap(kept);
}

bool
SeamExtra::closerToOtherPlane(const Eigen::Vector3f& p,
    const FittedPlane& a,
    const FittedPlane& b) const
{
    const float own = std::min(planeDist(p, a), planeDist(p, b));
    const float margin = 0.25f * params_.plane_dist_mm;
    for (const auto& c : planes_)
    {
        if (&c == &a || &c == &b)
            continue;
        if (planeDist(p, c) + margin < own)
            return true;
    }
    return false;
}

bool
SeamExtra::fillSeamPoses(WeldSeam& seam) const
{
    Eigen::Affine3f start = Eigen::Affine3f::Identity();
    start.translation() = seam.start;
    start.linear().col(2) = seam.torch_z;

    Eigen::Affine3f end = Eigen::Affine3f::Identity();
    end.translation() = seam.end;
    end.linear().col(2) = seam.torch_z;

    if (!computeWeldTcpStartEnd(start, end, params_.inward_deg))
        return false;

    seam.start_pose = start;
    seam.end_pose = end;
    seam.start = start.translation();
    seam.end = end.translation();

    seam.trajectory.clear();
    pcl::PointNormal pn_start;
    pn_start.getVector3fMap() = seam.start;
    pn_start.getNormalVector3fMap() = start.linear().col(2);
    pn_start.curvature = 0.0f;
    seam.trajectory.push_back(pn_start);

    pcl::PointNormal pn_end;
    pn_end.getVector3fMap() = seam.end;
    pn_end.getNormalVector3fMap() = end.linear().col(2);
    pn_end.curvature = 0.0f;
    seam.trajectory.push_back(pn_end);

    seam.trajectory.width = static_cast<std::uint32_t>(seam.trajectory.size());
    seam.trajectory.height = 1;
    seam.trajectory.is_dense = true;
    return true;
}

bool
SeamExtra::buildSeam(const FittedPlane& a, const FittedPlane& b, WeldSeam& seam) const
{
    const float dihedral = angleDeg(a.normal, b.normal);
    if (dihedral < params_.min_dihedral_deg || dihedral > params_.max_dihedral_deg)
        return false;

    Eigen::Vector3f dir = a.normal.cross(b.normal);
    if (dir.norm() < 1e-5f)
        return false;
    dir.normalize();
    const Eigen::Vector3f origin = intersectionPoint(a, b);

    std::vector<float> t_a;
    std::vector<float> t_b;
    seam.seam_cloud.clear();

    auto collect = [&](const FittedPlane& plane, std::vector<float>& ts) {
        for (const auto& pt : plane.points)
        {
            const Eigen::Vector3f p = pt.getVector3fMap();
            if (planeDist(p, plane) > params_.plane_dist_mm)
                continue;
            if (lineDist(p, origin, dir) > params_.seam_band_mm)
                continue;
            if (closerToOtherPlane(p, a, b))
                continue;
            ts.push_back((p - origin).dot(dir));
            seam.seam_cloud.push_back(pt);
        }
        };
    collect(a, t_a);
    collect(b, t_b);
    if (static_cast<int>(t_a.size()) < params_.min_plane_support ||
        static_cast<int>(t_b.size()) < params_.min_plane_support)
        return false;

    auto span = [](const std::vector<float>& ts) {
        const auto mm = std::minmax_element(ts.begin(), ts.end());
        return std::make_pair(*mm.first, *mm.second);
        };
    const auto sa = span(t_a);
    const auto sb = span(t_b);
    float t0 = std::max(sa.first, sb.first);
    float t1 = std::min(sa.second, sb.second);

    const float mid = 0.5f * (t0 + t1);
    for (const auto& c : planes_)
    {
        if (&c == &a || &c == &b)
            continue;
        const float denom = c.normal.dot(dir);
        if (std::abs(denom) < 1e-5f)
            continue;
        const float tc = -(c.normal.dot(origin) + c.d) / denom;
        if (tc > mid && tc < t1)
            t1 = tc;
        else if (tc < mid && tc > t0)
            t0 = tc;
    }
    if (t1 - t0 < params_.min_seam_length_mm)
        return false;

    ct::Cloud cropped;
    for (const auto& pt : seam.seam_cloud)
    {
        const float t = (pt.getVector3fMap() - origin).dot(dir);
        if (t >= t0 && t <= t1)
            cropped.push_back(pt);
    }
    seam.seam_cloud.swap(cropped);
    if (seam.seam_cloud.size() < 2)
        return false;

    t0 = std::numeric_limits<float>::infinity();
    t1 = -std::numeric_limits<float>::infinity();
    for (const auto& pt : seam.seam_cloud)
    {
        const float t = (pt.getVector3fMap() - origin).dot(dir);
        t0 = std::min(t0, t);
        t1 = std::max(t1, t);
    }
    if (t1 - t0 < params_.min_seam_length_mm)
        return false;

    seam.start = origin + t0 * dir;
    seam.end = origin + t1 * dir;
    seam.length_mm = t1 - t0;
    seam.seam_cloud.width = static_cast<std::uint32_t>(seam.seam_cloud.size());
    seam.seam_cloud.height = 1;
    seam.seam_cloud.is_dense = true;

    Eigen::Vector3f n1 = a.normal;
    Eigen::Vector3f n2 = b.normal;
    const Eigen::Vector3f center = 0.5f * (seam.start + seam.end);
    auto toward = [&](Eigen::Vector3f& n, const FittedPlane& other) {
        Eigen::Vector3f v = other.centroid - center;
        v -= v.dot(dir) * dir;
        if (v.norm() < 1e-4f)
            return;
        if (n.dot(v) < 0.0f)
            n = -n;
        };
    toward(n1, b);
    toward(n2, a);
    seam.torch_z = n1 + n2;
    if (seam.torch_z.norm() < 1e-6f)
        seam.torch_z = n1;
    else
        seam.torch_z.normalize();

    return fillSeamPoses(seam);
}

bool
SeamExtra::compute()
{
    planes_.clear();
    seams_.clear();
    cloud_.reset();
    segmented_cloud_.reset();
    tree_.reset();
    if (!input_ || input_->empty())
        return false;

    downsample();
    if (!cloud_ || static_cast<int>(cloud_->size()) < params_.min_plane_inliers)
        return false;

    tree_.reset(new pcl::search::KdTree<pcl::PointXYZRGBNormal>);
    tree_->setInputCloud(cloud_);
    extractPlanes();

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
    return !seams_.empty();
}

std::vector<std::pair<Eigen::Affine3f, Eigen::Affine3f>>
SeamExtra::trajectoryCloud() const
{
    std::vector<std::pair<Eigen::Affine3f, Eigen::Affine3f>> out;
    out.reserve(seams_.size());
    for (const auto& s : seams_)
        out.emplace_back(s.start_pose, s.end_pose);
    return out;
}
