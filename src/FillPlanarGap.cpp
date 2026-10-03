#include "FillPlanarGap.h"

#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <map>
#include <vector>

namespace {

pcl::PointCloud<pcl::PointXYZ>::Ptr take(
    const pcl::PointCloud<pcl::PointXYZ>& cloud,
    const pcl::PointIndices& indices)
{
    auto out = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    out->reserve(indices.indices.size());
    for (int index : indices.indices)
        out->push_back(cloud[index]);
    return out;
}

Eigen::Vector3f centroidOf(const pcl::PointCloud<pcl::PointXYZ>& cloud)
{
    Eigen::Vector3f sum = Eigen::Vector3f::Zero();
    for (std::size_t i = 0; i < cloud.size(); ++i)
        sum += Eigen::Vector3f(cloud[i].x, cloud[i].y, cloud[i].z);
    return sum / static_cast<float>(cloud.size());
}

float medianOf(std::vector<float> values)
{
    const auto mid = values.begin() + static_cast<std::ptrdiff_t>(values.size() / 2);
    std::nth_element(values.begin(), mid, values.end());
    return *mid;
}

} // namespace

pcl::PointCloud<pcl::PointXYZ>::Ptr fillPlanarGap(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud,
    float spacing,
    float cluster_tolerance)
{
    auto filled = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    if (!cloud || cloud->size() < 20 || spacing <= 0.f || cluster_tolerance <= spacing)
        return filled;

    auto tree = pcl::make_shared<pcl::search::KdTree<pcl::PointXYZ>>();
    tree->setInputCloud(cloud);
    std::vector<pcl::PointIndices> clusters;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> extractor;
    extractor.setClusterTolerance(cluster_tolerance);
    extractor.setMinClusterSize(100);
    extractor.setMaxClusterSize(static_cast<int>(cloud->size()));
    extractor.setSearchMethod(tree);
    extractor.setInputCloud(cloud);
    extractor.extract(clusters);
    if (clusters.size() < 2)
        return filled;
    std::sort(clusters.begin(), clusters.end(),
              [](const pcl::PointIndices& a, const pcl::PointIndices& b) {
                  return a.indices.size() > b.indices.size();
              });

    const auto first = take(*cloud, clusters[0]);
    const auto second = take(*cloud, clusters[1]);
    const Eigen::Vector3f center_first = centroidOf(*first);
    const Eigen::Vector3f center_second = centroidOf(*second);
    Eigen::Vector3f across = center_second - center_first;
    if (across.norm() <= spacing)
        return filled;
    across.normalize();
    const Eigen::Vector3f helper = std::fabs(across.z()) < 0.9f ? Eigen::Vector3f::UnitZ()
                                                                 : Eigen::Vector3f::UnitX();
    const Eigen::Vector3f along = across.cross(helper).normalized();
    const Eigen::Vector3f origin = 0.5f * (center_first + center_second);

    struct Bin {
        std::array<std::vector<float>, 2> across;
        std::vector<float> along;
    };
    std::map<int, Bin> bins;
    auto accumulate = [&](const pcl::PointCloud<pcl::PointXYZ>& cluster, int side) {
        for (std::size_t i = 0; i < cluster.size(); ++i) {
            const Eigen::Vector3f delta(cluster[i].x - origin.x(),
                                        cluster[i].y - origin.y(),
                                        cluster[i].z - origin.z());
            const float along_coord = delta.dot(along);
            const float across_coord = delta.dot(across);
            Bin& bin = bins[static_cast<int>(std::floor(along_coord / spacing))];
            bin.across[side].push_back(across_coord);
            bin.along.push_back(along_coord);
        }
    };
    accumulate(*first, 0);
    accumulate(*second, 1);

    struct Span {
        float along;
        float near;
        float far;
    };
    std::vector<Span> spans;
    std::vector<float> widths;
    for (const auto& item : bins) {
        const Bin& bin = item.second;
        if (bin.across[0].size() < 8 || bin.across[1].size() < 8)
            continue;
        const float near = *std::max_element(bin.across[0].begin(), bin.across[0].end());
        const float far = *std::min_element(bin.across[1].begin(), bin.across[1].end());
        if (far - near <= spacing)
            continue;
        spans.push_back({medianOf(bin.along), near, far});
        widths.push_back(far - near);
    }
    if (widths.size() < 5)
        return filled;

    const float typical = medianOf(widths);
    const float limit = std::max(3.f * spacing, 0.25f * typical);
    for (std::size_t i = 0; i < spans.size(); ++i) {
        if (std::fabs(widths[i] - typical) > limit)
            continue;
        const Span& span = spans[i];
        for (float offset = span.near + spacing; offset < span.far - spacing * 0.25f; offset += spacing) {
            const Eigen::Vector3f point = origin + span.along * along + offset * across;
            filled->emplace_back(point.x(), point.y(), point.z());
        }
    }
    filled->width = static_cast<std::uint32_t>(filled->size());
    filled->height = 1;
    filled->is_dense = true;
    return filled;
}
