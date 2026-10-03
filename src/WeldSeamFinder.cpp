#include "WeldSeamFinder.h"

#include <pcl/ModelCoefficients.h>
#include <pcl/common/pca.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/convex_hull.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <queue>
#include <stdexcept>
#include <utility>
#include <vector>

namespace {

using Cloud = pcl::PointCloud<pcl::PointXYZ>;
using CloudPtr = Cloud::Ptr;

void Pack(Cloud& cloud)
{
    cloud.width = static_cast<std::uint32_t>(cloud.size());
    cloud.height = 1;
    cloud.is_dense = true;
}

CloudPtr RemoveInvalidPoints(const Cloud::ConstPtr& cloud, float zeroEpsilon)
{
    CloudPtr valid(new Cloud);
    valid->reserve(cloud->size());
    for (const pcl::PointXYZ& point : cloud->points) {
        const float x = point.x;
        const float y = point.y;
        const float z = point.z;
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
            continue;
        }
        if (std::fabs(x) <= zeroEpsilon && std::fabs(y) <= zeroEpsilon && std::fabs(z) <= zeroEpsilon) {
            continue;
        }
        valid->push_back(point);
    }
    Pack(*valid);
    return valid;
}

CloudPtr RemoveOutliers(const CloudPtr& cloud, int meanK, float stddevMul)
{
    if (cloud->size() <= static_cast<std::size_t>(std::max(meanK, 2))) {
        return cloud;
    }

    CloudPtr filtered(new Cloud);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> filter;
    filter.setInputCloud(cloud);
    filter.setMeanK(meanK);
    filter.setStddevMulThresh(stddevMul);
    filter.filter(*filtered);
    Pack(*filtered);
    return filtered;
}

CloudPtr RemoveSmallClusters(const CloudPtr& cloud, float tolerance, int minClusterSize)
{
    if (cloud->size() < static_cast<std::size_t>(std::max(minClusterSize, 1))) {
        return CloudPtr(new Cloud);
    }

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> clusters;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> extractor;
    extractor.setClusterTolerance(tolerance);
    extractor.setMinClusterSize(std::max(minClusterSize, 1));
    extractor.setMaxClusterSize(static_cast<int>(cloud->size()));
    extractor.setSearchMethod(tree);
    extractor.setInputCloud(cloud);
    extractor.extract(clusters);

    CloudPtr kept(new Cloud);
    kept->reserve(cloud->size());
    for (const pcl::PointIndices& cluster : clusters) {
        for (int index : cluster.indices) {
            kept->push_back((*cloud)[index]);
        }
    }
    Pack(*kept);
    return kept;
}

bool FindLargestPlane(const CloudPtr& cloud, float distanceThreshold, Eigen::Vector4f& plane)
{
    CloudPtr remaining(new Cloud(*cloud));
    pcl::SACSegmentation<pcl::PointXYZ> segmentation;
    segmentation.setOptimizeCoefficients(true);
    segmentation.setModelType(pcl::SACMODEL_PLANE);
    segmentation.setMethodType(pcl::SAC_RANSAC);
    segmentation.setDistanceThreshold(distanceThreshold);
    segmentation.setMaxIterations(1000);
    segmentation.setProbability(0.99);

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setNegative(true);

    int bestCount = 0;
    Eigen::Vector4f best = Eigen::Vector4f::Zero();
    constexpr int kMaxPlanes = 6;

    for (int i = 0; i < kMaxPlanes && remaining->size() >= 3; ++i) {
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        segmentation.setInputCloud(remaining);
        segmentation.segment(*inliers, *coefficients);
        if (coefficients->values.size() < 4 || inliers->indices.size() < 3) {
            break;
        }

        if (static_cast<int>(inliers->indices.size()) > bestCount) {
            bestCount = static_cast<int>(inliers->indices.size());
            best << coefficients->values[0], coefficients->values[1], coefficients->values[2],
                coefficients->values[3];
        }

        CloudPtr next(new Cloud);
        extract.setInputCloud(remaining);
        extract.setIndices(inliers);
        extract.filter(*next);
        if (next->size() >= remaining->size()) {
            break;
        }
        remaining.swap(next);
    }

    const float norm = best.head<3>().norm();
    if (bestCount < 3 || norm < 1e-6f) {
        return false;
    }
    best.head<3>() /= norm;
    best[3] /= norm;
    plane = best;
    return true;
}

void SplitByHeight(const Cloud& cloud,
                   const Eigen::Vector4f& plane,
                   float planeDistance,
                   float heightThreshold,
                   Cloud& onPlane,
                   Cloud& outside)
{
    const Eigen::Vector3f normal = plane.head<3>();
    const float offset = plane[3];
    const float groundBand = std::max(planeDistance, 0.0f);
    const float outsideHeight = std::max(heightThreshold, groundBand);

    onPlane.reserve(cloud.size());
    outside.reserve(cloud.size());
    for (const pcl::PointXYZ& point : cloud.points) {
        const float distance = std::fabs(normal.dot(point.getVector3fMap()) + offset);
        if (distance <= groundBand) {
            onPlane.push_back(point);
        } else if (distance > outsideHeight) {
            outside.push_back(point);
        }
    }
    Pack(onPlane);
    Pack(outside);
}

CloudPtr ProjectToPlane(const CloudPtr& cloud, const Eigen::Vector4f& plane)
{
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    coefficients->values = {plane[0], plane[1], plane[2], plane[3]};

    CloudPtr projected(new Cloud);
    pcl::ProjectInliers<pcl::PointXYZ> projector;
    projector.setModelType(pcl::SACMODEL_PLANE);
    projector.setInputCloud(cloud);
    projector.setModelCoefficients(coefficients);
    projector.filter(*projected);
    Pack(*projected);
    return projected;
}

Eigen::Vector2f ToPlane(const pcl::PointXYZ& point, const Eigen::Vector3f& axisU, const Eigen::Vector3f& axisV)
{
    const Eigen::Vector3f position = point.getVector3fMap();
    return Eigen::Vector2f(position.dot(axisU), position.dot(axisV));
}

bool PointInPolygon(const Eigen::Vector2f& point, const Cloud& polygon)
{
    bool inside = false;
    const int count = static_cast<int>(polygon.size());
    for (int i = 0, j = count - 1; i < count; j = i++) {
        const float yi = polygon[i].y;
        const float yj = polygon[j].y;
        const bool crosses = (yi > point.y()) != (yj > point.y());
        if (!crosses) {
            continue;
        }
        const float xi = polygon[i].x;
        const float xj = polygon[j].x;
        const float xCross = (xj - xi) * (point.y() - yi) / (yj - yi) + xi;
        if (point.x() < xCross) {
            inside = !inside;
        }
    }
    return inside;
}

int RadiusInCells(float radius, float resolution)
{
    if (radius <= 0.0f) {
        return 0;
    }
    return std::max(0, static_cast<int>(std::ceil(radius / resolution)));
}

void MarkDisk(std::vector<std::uint8_t>& grid, int cols, int rows, int col, int row, int radius)
{
    const long long radius2 = static_cast<long long>(radius) * radius;
    for (int dy = -radius; dy <= radius; ++dy) {
        const int rr = row + dy;
        if (rr < 0 || rr >= rows) {
            continue;
        }
        for (int dx = -radius; dx <= radius; ++dx) {
            if (static_cast<long long>(dx) * dx + static_cast<long long>(dy) * dy > radius2) {
                continue;
            }
            const int cc = col + dx;
            if (cc < 0 || cc >= cols) {
                continue;
            }
            grid[static_cast<std::size_t>(rr) * static_cast<std::size_t>(cols) + static_cast<std::size_t>(cc)] = 1;
        }
    }
}

CloudPtr ExtractHollow(const Cloud& onPlane,
                       const Cloud& projected,
                       const Eigen::Vector4f& plane,
                       const WeldSeamParams& params)
{
    CloudPtr hollow(new Cloud);
    if (onPlane.size() < 3 || projected.empty()) {
        return hollow;
    }

    const Eigen::Vector3f normal = plane.head<3>();
    const Eigen::Vector3f helper = std::fabs(normal.z()) < 0.9f ? Eigen::Vector3f::UnitZ() : Eigen::Vector3f::UnitX();
    const Eigen::Vector3f axisU = normal.cross(helper).normalized();
    const Eigen::Vector3f axisV = normal.cross(axisU);

    CloudPtr plane2d(new Cloud);
    plane2d->reserve(onPlane.size());
    Eigen::Vector2f minPoint(std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
    Eigen::Vector2f maxPoint = -minPoint;

    auto include = [&](const Eigen::Vector2f& point) {
        minPoint = minPoint.cwiseMin(point);
        maxPoint = maxPoint.cwiseMax(point);
    };

    for (const pcl::PointXYZ& point : onPlane.points) {
        const Eigen::Vector2f uv = ToPlane(point, axisU, axisV);
        plane2d->push_back(pcl::PointXYZ(uv.x(), uv.y(), 0.0f));
        include(uv);
    }
    Pack(*plane2d);

    std::vector<Eigen::Vector2f> projectedUv;
    projectedUv.reserve(projected.size());
    for (const pcl::PointXYZ& point : projected.points) {
        const Eigen::Vector2f uv = ToPlane(point, axisU, axisV);
        projectedUv.push_back(uv);
        include(uv);
    }

    CloudPtr polygon(new Cloud);
    try {
        pcl::ConvexHull<pcl::PointXYZ> hull;
        hull.setDimension(2);
        hull.setInputCloud(plane2d);
        hull.reconstruct(*polygon);
    } catch (const std::exception&) {
        return hollow;
    }
    if (polygon->size() < 3) {
        return hollow;
    }

    float resolution = std::max(params.gridResolution, 0.5f);
    minPoint.array() -= resolution;
    maxPoint.array() += resolution;

    auto gridSize = [&](float step) {
        const int cols = std::max(1, static_cast<int>(std::ceil((maxPoint.x() - minPoint.x()) / step)));
        const int rows = std::max(1, static_cast<int>(std::ceil((maxPoint.y() - minPoint.y()) / step)));
        return std::make_pair(cols, rows);
    };

    std::pair<int, int> size = gridSize(resolution);
    while (static_cast<long long>(size.first) * size.second > 2500000LL && resolution < 50.0f) {
        resolution *= 2.0f;
        size = gridSize(resolution);
    }
    const int cols = size.first;
    const int rows = size.second;

    auto toCell = [&](const Eigen::Vector2f& uv, int& col, int& row) {
        col = static_cast<int>(std::floor((uv.x() - minPoint.x()) / resolution));
        row = static_cast<int>(std::floor((uv.y() - minPoint.y()) / resolution));
        return col >= 0 && row >= 0 && col < cols && row < rows;
    };

    const std::size_t cellCount = static_cast<std::size_t>(cols) * static_cast<std::size_t>(rows);
    std::vector<std::uint8_t> occupied(cellCount, 0);
    std::vector<std::uint8_t> band(cellCount, 0);
    const int occupyRadius = RadiusInCells(params.occupyRadius, resolution);
    const int bandRadius = RadiusInCells(params.projectionBand, resolution);

    for (const pcl::PointXYZ& point : plane2d->points) {
        int col = 0;
        int row = 0;
        if (toCell(Eigen::Vector2f(point.x, point.y), col, row)) {
            MarkDisk(occupied, cols, rows, col, row, occupyRadius);
        }
    }
    for (const Eigen::Vector2f& uv : projectedUv) {
        int col = 0;
        int row = 0;
        if (toCell(uv, col, row)) {
            MarkDisk(band, cols, rows, col, row, bandRadius);
        }
    }

    std::vector<std::uint8_t> hollowMask(cellCount, 0);
    for (int row = 0; row < rows; ++row) {
        for (int col = 0; col < cols; ++col) {
            const std::size_t id = static_cast<std::size_t>(row * cols + col);
            if (occupied[id] || !band[id]) {
                continue;
            }
            const Eigen::Vector2f center(minPoint.x() + (static_cast<float>(col) + 0.5f) * resolution,
                                         minPoint.y() + (static_cast<float>(row) + 0.5f) * resolution);
            if (PointInPolygon(center, *polygon)) {
                hollowMask[id] = 1;
            }
        }
    }

    std::vector<int> labels(cellCount, 0);
    std::vector<int> componentSize;
    const int stepCol[4] = {1, -1, 0, 0};
    const int stepRow[4] = {0, 0, 1, -1};
    int label = 0;

    for (int row = 0; row < rows; ++row) {
        for (int col = 0; col < cols; ++col) {
            const int id = row * cols + col;
            if (!hollowMask[static_cast<std::size_t>(id)] || labels[static_cast<std::size_t>(id)] != 0) {
                continue;
            }
            ++label;
            int count = 0;
            std::queue<int> seeds;
            seeds.push(id);
            labels[static_cast<std::size_t>(id)] = label;
            while (!seeds.empty()) {
                const int current = seeds.front();
                seeds.pop();
                ++count;
                const int currentRow = current / cols;
                const int currentCol = current % cols;
                for (int k = 0; k < 4; ++k) {
                    const int nextCol = currentCol + stepCol[k];
                    const int nextRow = currentRow + stepRow[k];
                    if (nextCol < 0 || nextRow < 0 || nextCol >= cols || nextRow >= rows) {
                        continue;
                    }
                    const int next = nextRow * cols + nextCol;
                    if (!hollowMask[static_cast<std::size_t>(next)] || labels[static_cast<std::size_t>(next)] != 0) {
                        continue;
                    }
                    labels[static_cast<std::size_t>(next)] = label;
                    seeds.push(next);
                }
            }
            componentSize.push_back(count);
        }
    }

    int largest = 0;
    bool hasQualified = false;
    for (int count : componentSize) {
        largest = std::max(largest, count);
        if (count >= params.minHollowCells) {
            hasQualified = true;
        }
    }
    const int minCells = hasQualified ? std::max(params.minHollowCells, 1) : largest;
    if (minCells <= 0) {
        return hollow;
    }

    hollow->reserve(static_cast<std::size_t>(largest));
    for (int row = 0; row < rows; ++row) {
        for (int col = 0; col < cols; ++col) {
            const int id = row * cols + col;
            const int component = labels[static_cast<std::size_t>(id)];
            if (component == 0 || componentSize[static_cast<std::size_t>(component - 1)] < minCells) {
                continue;
            }
            const float u = minPoint.x() + (static_cast<float>(col) + 0.5f) * resolution;
            const float v = minPoint.y() + (static_cast<float>(row) + 0.5f) * resolution;
            const Eigen::Vector3f position = u * axisU + v * axisV - plane[3] * normal;
            hollow->push_back(pcl::PointXYZ(position.x(), position.y(), position.z()));
        }
    }
    Pack(*hollow);
    return hollow;
}

bool ComputePrincipalDirection(const CloudPtr& hollow,
                               const Eigen::Vector3f& normal,
                               Eigen::Vector3f& direction,
                               Eigen::Vector3f& centroid)
{
    if (!hollow || hollow->size() < 3) {
        return false;
    }

    pcl::PCA<pcl::PointXYZ> pca;
    pca.setInputCloud(hollow);
    const Eigen::Vector3f values = pca.getEigenValues();
    const Eigen::Matrix3f vectors = pca.getEigenVectors();
    int axis = 0;
    if (!(values.maxCoeff(&axis) > 0.0f) || axis < 0 || axis > 2) {
        return false;
    }

    direction = vectors.col(axis);
    direction -= direction.dot(normal) * normal;
    if (direction.norm() < 1e-6f) {
        return false;
    }
    direction.normalize();

    int dominant = 0;
    direction.cwiseAbs().maxCoeff(&dominant);
    if (direction[dominant] < 0.0f) {
        direction = -direction;
    }

    const Eigen::Vector4f mean = pca.getMean();
    centroid = mean.head<3>();
    return std::isfinite(direction.x()) && std::isfinite(direction.y()) && std::isfinite(direction.z())
        && std::isfinite(centroid.x()) && std::isfinite(centroid.y()) && std::isfinite(centroid.z());
}

WeldSeamResult FindWeldSeamImpl(const Cloud::ConstPtr& cloud, const WeldSeamParams& params)
{
    WeldSeamResult result;
    if (!cloud || cloud->empty()) {
        return result;
    }

    CloudPtr valid = RemoveInvalidPoints(cloud, params.zeroEpsilon);
    CloudPtr denoised = RemoveOutliers(valid, params.sorMeanK, params.sorStddevMul);
    CloudPtr cleaned = RemoveSmallClusters(denoised, params.clusterTolerance, params.minClusterSize);
    if (cleaned->size() < 3) {
        return result;
    }

    Eigen::Vector4f plane = Eigen::Vector4f::Zero();
    if (!FindLargestPlane(cleaned, std::max(params.planeDistance, 1e-4f), plane)) {
        return result;
    }

    Cloud onPlane;
    Cloud outside;
    SplitByHeight(*cleaned, plane, params.planeDistance, params.heightThreshold, onPlane, outside);
    if (onPlane.size() < 3 || outside.empty()) {
        return result;
    }

    const CloudPtr projected = ProjectToPlane(outside.makeShared(), plane);
    result.hollow = ExtractHollow(onPlane, *projected, plane, params);
    if (!ComputePrincipalDirection(result.hollow, plane.head<3>(), result.direction, result.centroid)) {
        result.hollow->clear();
        return result;
    }

    result.ground = onPlane.makeShared();
    result.projected = projected;
    result.plane = plane;
    result.success = true;
    return result;
}

} // namespace

WeldSeamResult FindWeldSeam(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud, const WeldSeamParams& params)
{
    try {
        return FindWeldSeamImpl(cloud, params);
    } catch (const std::exception&) {
        return WeldSeamResult();
    }
}
