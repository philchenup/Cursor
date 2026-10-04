#include "SceneSeamExtractor.h"

#include <pcl/filters/voxel_grid.h>

#include <Eigen/Eigenvalues>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <queue>
#include <stdexcept>
#include <utility>

namespace {

using Cloud = pcl::PointCloud<pcl::PointXYZ>;
using CloudPtr = Cloud::Ptr;

constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();
constexpr float kPi = 3.14159265358979323846f;

float Deg2Rad(float deg)
{
    return deg * kPi / 180.0f;
}

Eigen::Vector2f Perp(const Eigen::Vector2f& d)
{
    return Eigen::Vector2f(-d.y(), d.x());
}

float Percentile(std::vector<float> values, float fraction)
{
    if (values.empty()) {
        return 0.0f;
    }
    const std::size_t index = static_cast<std::size_t>(
        std::min<float>(static_cast<float>(values.size() - 1), std::floor(fraction * static_cast<float>(values.size()))));
    std::nth_element(values.begin(), values.begin() + static_cast<std::ptrdiff_t>(index), values.end());
    return values[index];
}

// ---------------------------------------------------------------------------
// 高度图
// ---------------------------------------------------------------------------

struct HeightGrid {
    int cols = 0;
    int rows = 0;
    float res = 1.0f;
    Eigen::Vector2f origin = Eigen::Vector2f::Zero();
    std::vector<float> z; ///< 每格最高点，无数据为 NaN

    std::size_t index(int c, int r) const
    {
        return static_cast<std::size_t>(r) * static_cast<std::size_t>(cols) + static_cast<std::size_t>(c);
    }

    bool inside(int c, int r) const
    {
        return c >= 0 && r >= 0 && c < cols && r < rows;
    }

    bool cellOf(float x, float y, int& c, int& r) const
    {
        c = static_cast<int>(std::floor((x - origin.x()) / res));
        r = static_cast<int>(std::floor((y - origin.y()) / res));
        return inside(c, r);
    }

    Eigen::Vector2f center(int c, int r) const
    {
        return origin + Eigen::Vector2f((static_cast<float>(c) + 0.5f) * res, (static_cast<float>(r) + 0.5f) * res);
    }

    bool hasData(int c, int r) const
    {
        return inside(c, r) && std::isfinite(z[index(c, r)]);
    }
};

bool BuildHeightGrid(const Cloud& cloud, const std::vector<int>* indices, float res, HeightGrid& grid)
{
    Eigen::Vector2f minXY(std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
    Eigen::Vector2f maxXY = -minXY;

    auto forEach = [&](auto&& fn) {
        if (indices) {
            for (int i : *indices) {
                fn(cloud[static_cast<std::size_t>(i)]);
            }
        } else {
            for (const pcl::PointXYZ& p : cloud.points) {
                fn(p);
            }
        }
    };

    std::size_t count = 0;
    forEach([&](const pcl::PointXYZ& p) {
        minXY = minXY.cwiseMin(Eigen::Vector2f(p.x, p.y));
        maxXY = maxXY.cwiseMax(Eigen::Vector2f(p.x, p.y));
        ++count;
    });
    if (count == 0) {
        return false;
    }

    grid.res = res;
    grid.origin = minXY - Eigen::Vector2f(res, res);
    const Eigen::Vector2f span = maxXY - grid.origin + Eigen::Vector2f(res, res);
    grid.cols = std::max(1, static_cast<int>(std::ceil(span.x() / res)));
    grid.rows = std::max(1, static_cast<int>(std::ceil(span.y() / res)));
    if (static_cast<long long>(grid.cols) * grid.rows > 60000000LL) {
        return false;
    }
    grid.z.assign(static_cast<std::size_t>(grid.cols) * static_cast<std::size_t>(grid.rows), kNaN);

    forEach([&](const pcl::PointXYZ& p) {
        int c = 0;
        int r = 0;
        if (!grid.cellOf(p.x, p.y, c, r)) {
            return;
        }
        float& cell = grid.z[grid.index(c, r)];
        if (!std::isfinite(cell) || p.z > cell) {
            cell = p.z;
        }
    });
    return true;
}

// ---------------------------------------------------------------------------
// 二值图形态学与连通域
// ---------------------------------------------------------------------------

using Mask = std::vector<std::uint8_t>;

std::vector<std::pair<int, int>> DiskOffsets(int radius)
{
    std::vector<std::pair<int, int>> offsets;
    for (int dy = -radius; dy <= radius; ++dy) {
        for (int dx = -radius; dx <= radius; ++dx) {
            if (dx * dx + dy * dy <= radius * radius) {
                offsets.emplace_back(dx, dy);
            }
        }
    }
    return offsets;
}

Mask Dilate(const Mask& src, int cols, int rows, int radius)
{
    if (radius <= 0) {
        return src;
    }
    const std::vector<std::pair<int, int>> offsets = DiskOffsets(radius);
    Mask dst(src.size(), 0);
    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c) {
            if (!src[static_cast<std::size_t>(r) * cols + c]) {
                continue;
            }
            for (const auto& [dx, dy] : offsets) {
                const int cc = c + dx;
                const int rr = r + dy;
                if (cc >= 0 && rr >= 0 && cc < cols && rr < rows) {
                    dst[static_cast<std::size_t>(rr) * cols + cc] = 1;
                }
            }
        }
    }
    return dst;
}

Mask Erode(const Mask& src, int cols, int rows, int radius)
{
    if (radius <= 0) {
        return src;
    }
    const std::vector<std::pair<int, int>> offsets = DiskOffsets(radius);
    Mask dst(src.size(), 0);
    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c) {
            if (!src[static_cast<std::size_t>(r) * cols + c]) {
                continue;
            }
            bool keep = true;
            for (const auto& [dx, dy] : offsets) {
                const int cc = c + dx;
                const int rr = r + dy;
                if (cc < 0 || rr < 0 || cc >= cols || rr >= rows || !src[static_cast<std::size_t>(rr) * cols + cc]) {
                    keep = false;
                    break;
                }
            }
            dst[static_cast<std::size_t>(r) * cols + c] = keep ? 1 : 0;
        }
    }
    return dst;
}

Mask Close(const Mask& src, int cols, int rows, int radius)
{
    return Erode(Dilate(src, cols, rows, radius), cols, rows, radius);
}

// 填充被前景包围的空洞：从栅格边界沿 4 邻域淹没背景，淹不到的背景就是内部空洞。
Mask FillEnclosedHoles(const Mask& src, int cols, int rows)
{
    Mask outside(src.size(), 0);
    std::vector<int> stack;
    auto push = [&](int c, int r) {
        if (c < 0 || r < 0 || c >= cols || r >= rows) {
            return;
        }
        const std::size_t id = static_cast<std::size_t>(r) * cols + c;
        if (src[id] || outside[id]) {
            return;
        }
        outside[id] = 1;
        stack.push_back(static_cast<int>(id));
    };
    for (int c = 0; c < cols; ++c) {
        push(c, 0);
        push(c, rows - 1);
    }
    for (int r = 0; r < rows; ++r) {
        push(0, r);
        push(cols - 1, r);
    }
    while (!stack.empty()) {
        const int id = stack.back();
        stack.pop_back();
        const int c = id % cols;
        const int r = id / cols;
        push(c + 1, r);
        push(c - 1, r);
        push(c, r + 1);
        push(c, r - 1);
    }
    Mask filled(src.size(), 1);
    for (std::size_t i = 0; i < src.size(); ++i) {
        if (outside[i]) {
            filled[i] = 0;
        }
    }
    return filled;
}

struct Component {
    int label = 0;
    int count = 0;
    int minC = 0;
    int minR = 0;
    int maxC = 0;
    int maxR = 0;
};

std::vector<Component> LabelComponents(const Mask& mask, int cols, int rows, std::vector<int>& labels)
{
    labels.assign(mask.size(), 0);
    std::vector<Component> components;
    const int stepC[8] = {1, -1, 0, 0, 1, 1, -1, -1};
    const int stepR[8] = {0, 0, 1, -1, 1, -1, 1, -1};

    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c) {
            const std::size_t id = static_cast<std::size_t>(r) * cols + c;
            if (!mask[id] || labels[id] != 0) {
                continue;
            }
            Component comp;
            comp.label = static_cast<int>(components.size()) + 1;
            comp.minC = comp.maxC = c;
            comp.minR = comp.maxR = r;
            std::queue<std::pair<int, int>> queue;
            queue.emplace(c, r);
            labels[id] = comp.label;
            while (!queue.empty()) {
                const auto [cc, cr] = queue.front();
                queue.pop();
                ++comp.count;
                comp.minC = std::min(comp.minC, cc);
                comp.maxC = std::max(comp.maxC, cc);
                comp.minR = std::min(comp.minR, cr);
                comp.maxR = std::max(comp.maxR, cr);
                for (int k = 0; k < 8; ++k) {
                    const int nc = cc + stepC[k];
                    const int nr = cr + stepR[k];
                    if (nc < 0 || nr < 0 || nc >= cols || nr >= rows) {
                        continue;
                    }
                    const std::size_t nid = static_cast<std::size_t>(nr) * cols + nc;
                    if (!mask[nid] || labels[nid] != 0) {
                        continue;
                    }
                    labels[nid] = comp.label;
                    queue.emplace(nc, nr);
                }
            }
            components.push_back(comp);
        }
    }
    return components;
}

// ---------------------------------------------------------------------------
// 平面拟合
// ---------------------------------------------------------------------------

Eigen::Vector4f HorizontalPlane(float z)
{
    return Eigen::Vector4f(0.0f, 0.0f, 1.0f, -z);
}

// 最小二乘拟合 z = a x + b y + c，返回法向朝上的归一化平面。
bool FitPlaneLeastSquares(const std::vector<Eigen::Vector3f>& points, Eigen::Vector4f& plane)
{
    if (points.size() < 3) {
        return false;
    }
    Eigen::Vector3d mean = Eigen::Vector3d::Zero();
    for (const Eigen::Vector3f& p : points) {
        mean += p.cast<double>();
    }
    mean /= static_cast<double>(points.size());

    Eigen::Matrix2d ata = Eigen::Matrix2d::Zero();
    Eigen::Vector2d atb = Eigen::Vector2d::Zero();
    for (const Eigen::Vector3f& p : points) {
        const Eigen::Vector3d q = p.cast<double>() - mean;
        const Eigen::Vector2d xy(q.x(), q.y());
        ata += xy * xy.transpose();
        atb += xy * q.z();
    }
    if (std::fabs(ata.determinant()) < 1e-9) {
        plane = HorizontalPlane(static_cast<float>(mean.z()));
        return true;
    }
    const Eigen::Vector2d ab = ata.ldlt().solve(atb);
    Eigen::Vector3d normal(-ab.x(), -ab.y(), 1.0);
    const double norm = normal.norm();
    normal /= norm;
    const double d = -normal.dot(mean);
    plane << static_cast<float>(normal.x()), static_cast<float>(normal.y()), static_cast<float>(normal.z()),
        static_cast<float>(d);
    return true;
}

float PlaneTiltDeg(const Eigen::Vector4f& plane)
{
    const float cosTilt = std::min(1.0f, std::fabs(plane[2]) / plane.head<3>().norm());
    return std::acos(cosTilt) * 180.0f / kPi;
}

// 高度直方图中最低的一层显著高度。倾斜地面的高度会铺开成一段，
// 其中最低的区段只含地面，不会混入底板；工件内部最低的一层则是底板上表面。
bool HistogramBaseLevel(const std::vector<float>& values, float bin, float& level)
{
    if (values.empty()) {
        return false;
    }
    float minV = std::numeric_limits<float>::max();
    float maxV = std::numeric_limits<float>::lowest();
    for (float v : values) {
        minV = std::min(minV, v);
        maxV = std::max(maxV, v);
    }
    const long long binCount = static_cast<long long>((maxV - minV) / bin) + 1;
    if (binCount > 2000000LL) {
        return false;
    }
    std::vector<int> histogram(static_cast<std::size_t>(binCount), 0);
    for (float v : values) {
        ++histogram[static_cast<std::size_t>((v - minV) / bin)];
    }
    const int peak = *std::max_element(histogram.begin(), histogram.end());
    const int significant = std::max(1, static_cast<int>(0.3f * static_cast<float>(peak)));
    for (std::size_t i = 0; i < histogram.size(); ++i) {
        if (histogram[i] >= significant) {
            level = minV + (static_cast<float>(i) + 0.5f) * bin;
            return true;
        }
    }
    return false;
}

// 用最低显著高度做初值，再迭代最小二乘拟合主平面。
bool FitDominantPlane(const HeightGrid& grid,
                      const Mask* candidate,
                      float tolerance,
                      float maxTiltDeg,
                      Eigen::Vector4f& plane)
{
    std::vector<float> heights;
    heights.reserve(grid.z.size() / 4);
    for (int r = 0; r < grid.rows; ++r) {
        for (int c = 0; c < grid.cols; ++c) {
            const std::size_t id = grid.index(c, r);
            if (!std::isfinite(grid.z[id]) || (candidate && !(*candidate)[id])) {
                continue;
            }
            heights.push_back(grid.z[id]);
        }
    }

    float mode = 0.0f;
    if (!HistogramBaseLevel(heights, std::max(tolerance * 0.5f, 0.5f), mode)) {
        return false;
    }
    plane = HorizontalPlane(mode);

    std::vector<Eigen::Vector3f> inliers;
    for (int iteration = 0; iteration < 4; ++iteration) {
        inliers.clear();
        for (int r = 0; r < grid.rows; ++r) {
            for (int c = 0; c < grid.cols; ++c) {
                const std::size_t id = grid.index(c, r);
                if (!std::isfinite(grid.z[id]) || (candidate && !(*candidate)[id])) {
                    continue;
                }
                const Eigen::Vector2f xy = grid.center(c, r);
                if (std::fabs(grid.z[id] - PlaneZ(plane, xy.x(), xy.y())) <= tolerance) {
                    inliers.emplace_back(xy.x(), xy.y(), grid.z[id]);
                }
            }
        }
        Eigen::Vector4f fitted;
        if (!FitPlaneLeastSquares(inliers, fitted)) {
            break;
        }
        if (PlaneTiltDeg(fitted) > maxTiltDeg) {
            plane = HorizontalPlane(mode);
            break;
        }
        plane = fitted;
    }
    return true;
}

// ---------------------------------------------------------------------------
// 立板提取
// ---------------------------------------------------------------------------

struct RibCell {
    Eigen::Vector2f p; ///< 相对工件中心的平面坐标
    float h;           ///< 高出底板
};

struct RibCandidate {
    Eigen::Vector2f center;
    Eigen::Vector2f dir;
    float tMin = 0.0f;
    float tMax = 0.0f;
    float thickness = 0.0f;
    float height = 0.0f;
    float confidence = 0.0f;
};

void SplitRuns(const std::vector<std::pair<float, int>>& sorted, float gap, std::vector<std::pair<int, int>>& runs)
{
    runs.clear();
    if (sorted.empty()) {
        return;
    }
    int begin = 0;
    for (int i = 1; i < static_cast<int>(sorted.size()); ++i) {
        if (sorted[static_cast<std::size_t>(i)].first - sorted[static_cast<std::size_t>(i - 1)].first > gap) {
            runs.emplace_back(begin, i);
            begin = i;
        }
    }
    runs.emplace_back(begin, static_cast<int>(sorted.size()));
}

// 把一段两端栅格稀疏的部分收掉，避免零星高点把立板拉长。
void TrimRun(const std::vector<std::pair<float, int>>& sorted, float res, int& begin, int& end)
{
    const float sliceWidth = 3.0f * res;
    const float t0 = sorted[static_cast<std::size_t>(begin)].first;
    const int sliceCount = static_cast<int>((sorted[static_cast<std::size_t>(end - 1)].first - t0) / sliceWidth) + 1;
    std::vector<int> counts(static_cast<std::size_t>(sliceCount), 0);
    for (int i = begin; i < end; ++i) {
        ++counts[static_cast<std::size_t>((sorted[static_cast<std::size_t>(i)].first - t0) / sliceWidth)];
    }
    std::vector<int> occupied;
    for (int count : counts) {
        if (count > 0) {
            occupied.push_back(count);
        }
    }
    std::nth_element(occupied.begin(), occupied.begin() + static_cast<std::ptrdiff_t>(occupied.size() / 2), occupied.end());
    const int median = occupied[occupied.size() / 2];
    const int minCellsPerSlice = std::max(2, static_cast<int>(std::round(0.3f * static_cast<float>(median))));

    int firstDense = 0;
    while (firstDense < sliceCount && counts[static_cast<std::size_t>(firstDense)] < minCellsPerSlice) {
        ++firstDense;
    }
    int lastDense = sliceCount - 1;
    while (lastDense > firstDense && counts[static_cast<std::size_t>(lastDense)] < minCellsPerSlice) {
        --lastDense;
    }
    if (firstDense >= sliceCount) {
        end = begin;
        return;
    }
    const float tLow = t0 + static_cast<float>(firstDense) * sliceWidth;
    const float tHigh = t0 + static_cast<float>(lastDense + 1) * sliceWidth;
    while (begin < end && sorted[static_cast<std::size_t>(begin)].first < tLow) {
        ++begin;
    }
    while (end > begin && sorted[static_cast<std::size_t>(end - 1)].first > tHigh) {
        --end;
    }
}

bool PrincipalDirection(const std::vector<RibCell>& cells, const std::vector<int>& ids, Eigen::Vector2f& center,
                        Eigen::Vector2f& dir)
{
    if (ids.size() < 2) {
        return false;
    }
    Eigen::Vector2d mean = Eigen::Vector2d::Zero();
    for (int id : ids) {
        mean += cells[static_cast<std::size_t>(id)].p.cast<double>();
    }
    mean /= static_cast<double>(ids.size());
    Eigen::Matrix2d cov = Eigen::Matrix2d::Zero();
    for (int id : ids) {
        const Eigen::Vector2d q = cells[static_cast<std::size_t>(id)].p.cast<double>() - mean;
        cov += q * q.transpose();
    }
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(cov);
    if (solver.info() != Eigen::Success) {
        return false;
    }
    const Eigen::Vector2d major = solver.eigenvectors().col(1);
    center = mean.cast<float>();
    dir = major.cast<float>().normalized();
    return true;
}

// 对一组共线栅格估计板厚、高度、置信度，并把直线横向重新居中。
RibCandidate MakeCandidate(const std::vector<RibCell>& cells,
                           const std::vector<std::pair<float, int>>& sorted,
                           int begin,
                           int end,
                           const Eigen::Vector2f& center,
                           const Eigen::Vector2f& dir,
                           float res,
                           const SceneSeamParams& params)
{
    const Eigen::Vector2f normal = Perp(dir);
    std::vector<float> lateral;
    std::vector<float> absLateral;
    std::vector<float> heights;
    lateral.reserve(static_cast<std::size_t>(end - begin));
    for (int i = begin; i < end; ++i) {
        const RibCell& cell = cells[static_cast<std::size_t>(sorted[static_cast<std::size_t>(i)].second)];
        const float d = normal.dot(cell.p - center);
        lateral.push_back(d);
        heights.push_back(cell.h);
    }
    const float median = Percentile(lateral, 0.5f);
    for (float d : lateral) {
        absLateral.push_back(std::fabs(d - median));
    }

    RibCandidate candidate;
    candidate.center = center + normal * median;
    candidate.dir = dir;
    candidate.tMin = sorted[static_cast<std::size_t>(begin)].first - res * 0.5f;
    candidate.tMax = sorted[static_cast<std::size_t>(end - 1)].first + res * 0.5f;
    candidate.thickness = std::clamp(2.0f * Percentile(absLateral, 0.9f) + res, params.ribMinThickness,
                                     params.ribMaxThickness);
    candidate.height = Percentile(heights, 0.95f);

    // 置信度：沿线有数据支撑的比例 x 直线度
    const float sliceWidth = 4.0f * res;
    const int sliceCount = std::max(1, static_cast<int>(std::ceil((candidate.tMax - candidate.tMin) / sliceWidth)));
    std::vector<double> sliceSum(static_cast<std::size_t>(sliceCount), 0.0);
    std::vector<int> sliceCountPerBin(static_cast<std::size_t>(sliceCount), 0);
    for (int i = begin; i < end; ++i) {
        const float t = sorted[static_cast<std::size_t>(i)].first;
        int slice = static_cast<int>((t - candidate.tMin) / sliceWidth);
        slice = std::clamp(slice, 0, sliceCount - 1);
        sliceSum[static_cast<std::size_t>(slice)] += lateral[static_cast<std::size_t>(i - begin)] - median;
        ++sliceCountPerBin[static_cast<std::size_t>(slice)];
    }
    int supported = 0;
    double straightness = 0.0;
    for (int s = 0; s < sliceCount; ++s) {
        if (sliceCountPerBin[static_cast<std::size_t>(s)] == 0) {
            continue;
        }
        ++supported;
        const double mean = sliceSum[static_cast<std::size_t>(s)] / sliceCountPerBin[static_cast<std::size_t>(s)];
        straightness += mean * mean;
    }
    const float support = static_cast<float>(supported) / static_cast<float>(sliceCount);
    const float rms = supported > 0 ? static_cast<float>(std::sqrt(straightness / supported)) : 0.0f;
    candidate.confidence = support * std::clamp(1.0f - rms / (2.0f * res), 0.0f, 1.0f);
    return candidate;
}

// Hough 直线迭代提取立板落地轮廓。确定性，不依赖随机采样。
std::vector<RibCandidate> ExtractRibs(const std::vector<RibCell>& cells, float res, const SceneSeamParams& params)
{
    std::vector<RibCandidate> ribs;
    if (cells.size() < 4) {
        return ribs;
    }

    const int thetaBins = std::max(1, static_cast<int>(std::round(180.0f / params.houghAngleStepDeg)));
    std::vector<float> cosTable(static_cast<std::size_t>(thetaBins));
    std::vector<float> sinTable(static_cast<std::size_t>(thetaBins));
    for (int t = 0; t < thetaBins; ++t) {
        const float theta = Deg2Rad(static_cast<float>(t) * params.houghAngleStepDeg);
        cosTable[static_cast<std::size_t>(t)] = std::cos(theta);
        sinTable[static_cast<std::size_t>(t)] = std::sin(theta);
    }

    float rhoMax = 0.0f;
    for (const RibCell& cell : cells) {
        rhoMax = std::max(rhoMax, cell.p.norm());
    }
    const float rhoStep = res;
    rhoMax += rhoStep;
    const int rhoBins = static_cast<int>(std::ceil(2.0f * rhoMax / rhoStep)) + 1;
    if (static_cast<long long>(thetaBins) * rhoBins > 50000000LL) {
        return ribs;
    }

    const float bandHalf = params.ribMaxThickness * 0.5f + res;
    const int minVotes = std::max(4, static_cast<int>(0.6f * params.ribMinLength / res));
    std::vector<std::uint8_t> used(cells.size(), 0);
    std::vector<int> accumulator(static_cast<std::size_t>(thetaBins) * static_cast<std::size_t>(rhoBins));
    std::vector<int> band;
    std::vector<std::pair<float, int>> sorted;
    std::vector<std::pair<int, int>> runs;

    for (int iteration = 0; iteration < 64; ++iteration) {
        std::fill(accumulator.begin(), accumulator.end(), 0);
        int remaining = 0;
        for (std::size_t i = 0; i < cells.size(); ++i) {
            if (used[i]) {
                continue;
            }
            ++remaining;
            const Eigen::Vector2f& p = cells[i].p;
            for (int t = 0; t < thetaBins; ++t) {
                const float rho = p.x() * cosTable[static_cast<std::size_t>(t)] + p.y() * sinTable[static_cast<std::size_t>(t)];
                const int bin = static_cast<int>((rho + rhoMax) / rhoStep);
                if (bin >= 0 && bin < rhoBins) {
                    ++accumulator[static_cast<std::size_t>(t) * rhoBins + bin];
                }
            }
        }
        if (remaining < minVotes) {
            break;
        }

        const auto peak = std::max_element(accumulator.begin(), accumulator.end());
        if (*peak < minVotes) {
            break;
        }
        const int peakIndex = static_cast<int>(peak - accumulator.begin());
        const int thetaIndex = peakIndex / rhoBins;
        const float rho = static_cast<float>(peakIndex % rhoBins) * rhoStep - rhoMax + rhoStep * 0.5f;
        const Eigen::Vector2f normal(cosTable[static_cast<std::size_t>(thetaIndex)], sinTable[static_cast<std::size_t>(thetaIndex)]);

        band.clear();
        for (std::size_t i = 0; i < cells.size(); ++i) {
            if (!used[i] && std::fabs(normal.dot(cells[i].p) - rho) <= bandHalf) {
                band.push_back(static_cast<int>(i));
            }
        }

        Eigen::Vector2f center;
        Eigen::Vector2f dir;
        if (band.size() < static_cast<std::size_t>(minVotes) || !PrincipalDirection(cells, band, center, dir)) {
            for (int id : band) {
                used[static_cast<std::size_t>(id)] = 1;
            }
            continue;
        }

        // 用 PCA 方向重新取带内栅格，减少 Hough 角度量化误差
        const Eigen::Vector2f refinedNormal = Perp(dir);
        band.clear();
        for (std::size_t i = 0; i < cells.size(); ++i) {
            if (!used[i] && std::fabs(refinedNormal.dot(cells[i].p - center)) <= bandHalf) {
                band.push_back(static_cast<int>(i));
            }
        }

        sorted.clear();
        for (int id : band) {
            sorted.emplace_back(dir.dot(cells[static_cast<std::size_t>(id)].p - center), id);
        }
        std::sort(sorted.begin(), sorted.end());
        SplitRuns(sorted, params.ribGapTolerance, runs);

        for (auto [begin, end] : runs) {
            TrimRun(sorted, res, begin, end);
            if (end - begin < minVotes) {
                continue;
            }
            const float length = sorted[static_cast<std::size_t>(end - 1)].first - sorted[static_cast<std::size_t>(begin)].first + res;
            if (length < params.ribMinLength) {
                continue;
            }
            ribs.push_back(MakeCandidate(cells, sorted, begin, end, center, dir, res, params));
        }
        for (int id : band) {
            used[static_cast<std::size_t>(id)] = 1;
        }
    }
    return ribs;
}

// 把与最长立板平行或垂直的立板吸附到该方向。
void SnapDirections(std::vector<RibCandidate>& ribs, float snapDeg)
{
    if (ribs.empty()) {
        return;
    }
    const auto longest = std::max_element(ribs.begin(), ribs.end(), [](const RibCandidate& a, const RibCandidate& b) {
        return (a.tMax - a.tMin) < (b.tMax - b.tMin);
    });
    const float theta0 = std::atan2(longest->dir.y(), longest->dir.x());
    const float snap = Deg2Rad(snapDeg);

    for (RibCandidate& rib : ribs) {
        const float theta = std::atan2(rib.dir.y(), rib.dir.x());
        float delta = theta - theta0;
        const float quarter = kPi * 0.5f;
        const float k = std::round(delta / quarter);
        const float residual = delta - k * quarter;
        if (std::fabs(residual) > snap) {
            continue;
        }
        const float snapped = theta0 + k * quarter;
        const Eigen::Vector2f newDir(std::cos(snapped), std::sin(snapped));
        const Eigen::Vector2f mid = rib.center + rib.dir * 0.5f * (rib.tMin + rib.tMax);
        const float half = 0.5f * (rib.tMax - rib.tMin);
        rib.center = mid;
        rib.dir = newDir;
        rib.tMin = -half;
        rib.tMax = half;
    }
}

bool IntersectLines(const RibCandidate& a, const RibCandidate& b, float& ta, float& tb)
{
    const float cross = a.dir.x() * b.dir.y() - a.dir.y() * b.dir.x();
    if (std::fabs(cross) < std::sin(Deg2Rad(10.0f))) {
        return false;
    }
    const Eigen::Vector2f delta = b.center - a.center;
    ta = (delta.x() * b.dir.y() - delta.y() * b.dir.x()) / cross;
    tb = (delta.x() * a.dir.y() - delta.y() * a.dir.x()) / cross;
    return true;
}

// T 形搭接：立板端部靠近另一块立板时，把端部修到对方板面。
void SnapJunctions(std::vector<RibCandidate>& ribs, const SceneSeamParams& params)
{
    for (RibCandidate& rib : ribs) {
        for (const RibCandidate& other : ribs) {
            if (&rib == &other) {
                continue;
            }
            float tRib = 0.0f;
            float tOther = 0.0f;
            if (!IntersectLines(rib, other, tRib, tOther)) {
                continue;
            }
            if (tOther < other.tMin - params.junctionTolerance || tOther > other.tMax + params.junctionTolerance) {
                continue;
            }
            const float face = other.thickness * 0.5f;
            const float reach = params.junctionTolerance + face;
            const float minLength = params.ribMinLength;
            if (std::fabs(tRib - rib.tMin) <= reach && tRib + face < rib.tMax - minLength) {
                rib.tMin = tRib + face;
            } else if (std::fabs(tRib - rib.tMax) <= reach && tRib - face > rib.tMin + minLength) {
                rib.tMax = tRib - face;
            }
        }
    }
}

// ---------------------------------------------------------------------------
// 工件处理
// ---------------------------------------------------------------------------

struct Junction {
    int ribA = 0;
    int ribB = 0;
    float tA = 0.0f;
    float tB = 0.0f;
};

/// 立板越过交点至少这么长才算有一条臂，用于判断 T 形 / 十字接头的象限
constexpr float kMinArm = 15.0f;

// 底板视图：区分“有底板”“确认没有底板（地面）”和“无数据（阴影）”三种状态
struct BaseMaskView {
    const HeightGrid* grid = nullptr;
    const Mask* footprint = nullptr;          ///< 底板轮廓：底板栅格 + 被底板包围的空洞（阴影、立板根部）
    const std::vector<float>* aboveBase = nullptr;
    float tolerance = 0.0f;

    bool hasBase(const Eigen::Vector2f& xy) const
    {
        int c = 0;
        int r = 0;
        return grid->cellOf(xy.x(), xy.y(), c, r) && (*footprint)[grid->index(c, r)];
    }

    /// 栅格有数据且明显低于底板平面（地面）时确认没有底板；阴影等无数据栅格视为未知
    bool confirmedNoBase(const Eigen::Vector2f& xy) const
    {
        int c = 0;
        int r = 0;
        if (!grid->cellOf(xy.x(), xy.y(), c, r)) {
            return true;
        }
        const std::size_t id = grid->index(c, r);
        if ((*footprint)[id] || !std::isfinite(grid->z[id])) {
            return false;
        }
        return (*aboveBase)[id] < -tolerance;
    }
};

Eigen::Vector3f Lift(const Eigen::Vector4f& plane, const Eigen::Vector2f& xy, float above = 0.0f)
{
    return Eigen::Vector3f(xy.x(), xy.y(), PlaneZ(plane, xy.x(), xy.y()) + above);
}

// 该侧是否有底板：沿法向从板根外侧一直探到立板高度量级的距离，碰到底板即算有，碰到地面即算无。
// 阴影宽度不会超过立板高度，所以不依赖固定的探测距离。
bool SideHasBase(const BaseMaskView& base, const RibCandidate& rib, const Eigen::Vector2f& normal, float side, float res)
{
    constexpr int kSamples = 7;
    const float nearDist = rib.thickness * 0.5f + 12.0f;
    const float farDist = rib.thickness * 0.5f + std::max(60.0f, rib.height);
    const float step = std::max(2.0f * res, 4.0f);
    int hits = 0;
    for (int s = 0; s < kSamples; ++s) {
        const float t = rib.tMin + (rib.tMax - rib.tMin) * (static_cast<float>(s) + 0.5f) / kSamples;
        const Eigen::Vector2f foot = rib.center + rib.dir * t;
        for (float d = nearDist; d <= farDist; d += step) {
            const Eigen::Vector2f probe = foot + normal * side * d;
            if (base.hasBase(probe)) {
                ++hits;
                break;
            }
            if (base.confirmedNoBase(probe)) {
                break;
            }
        }
    }
    return hits * 2 >= kSamples;
}

void BuildFlatSeams(const Workpiece& piece,
                    const std::vector<RibCandidate>& ribs,
                    const std::vector<Junction>& junctions,
                    const BaseMaskView& base,
                    const SceneSeamParams& params,
                    std::vector<InitialSeam, Eigen::aligned_allocator<InitialSeam>>& seams)
{
    using Cuts = std::vector<std::pair<float, float>>;
    for (std::size_t i = 0; i < ribs.size(); ++i) {
        const RibCandidate& rib = ribs[i];
        const Eigen::Vector2f normal = Perp(rib.dir);

        // 接头处的切口只加在对方立板实际延伸过来的那一侧；T 形端部的另一侧焊缝保持连续
        Cuts cutsPositive;
        Cuts cutsNegative;
        for (const Junction& junction : junctions) {
            int otherIndex = -1;
            float tSelf = 0.0f;
            float tOther = 0.0f;
            if (junction.ribA == static_cast<int>(i)) {
                otherIndex = junction.ribB;
                tSelf = junction.tA;
                tOther = junction.tB;
            } else if (junction.ribB == static_cast<int>(i)) {
                otherIndex = junction.ribA;
                tSelf = junction.tB;
                tOther = junction.tA;
            } else {
                continue;
            }
            const RibCandidate& other = ribs[static_cast<std::size_t>(otherIndex)];
            const float half = other.thickness * 0.5f + params.seamClearance;
            const float arm = rib.thickness * 0.5f + kMinArm;
            const bool otherForwardIsPositive = normal.dot(other.dir) >= 0.0f;
            if (other.tMax >= tOther + arm) {
                (otherForwardIsPositive ? cutsPositive : cutsNegative).emplace_back(tSelf - half, tSelf + half);
            }
            if (other.tMin <= tOther - arm) {
                (otherForwardIsPositive ? cutsNegative : cutsPositive).emplace_back(tSelf - half, tSelf + half);
            }
        }
        std::sort(cutsPositive.begin(), cutsPositive.end());
        std::sort(cutsNegative.begin(), cutsNegative.end());

        for (float side : {1.0f, -1.0f}) {
            const Eigen::Vector2f offset = normal * side * rib.thickness * 0.5f;

            // 该侧必须有底板，否则是板边立板，不焊
            if (!SideHasBase(base, rib, normal, side, base.grid->res)) {
                continue;
            }
            const Cuts& cuts = side > 0.0f ? cutsPositive : cutsNegative;

            float cursor = rib.tMin;
            auto emit = [&](float t0, float t1) {
                // 焊缝长度以立板顶边为准，只有确认探到地面时才收缩端点；阴影不收缩
                const float step = 2.0f;
                while (t1 - t0 >= params.minSeamLength && base.confirmedNoBase(rib.center + rib.dir * t0 + offset)) {
                    t0 += step;
                }
                while (t1 - t0 >= params.minSeamLength && base.confirmedNoBase(rib.center + rib.dir * t1 + offset)) {
                    t1 -= step;
                }
                if (t1 - t0 < params.minSeamLength) {
                    return;
                }
                InitialSeam seam;
                seam.workpieceId = piece.id;
                seam.type = SeamType::FlatFillet;
                seam.start = Lift(piece.basePlane, rib.center + rib.dir * t0 + offset);
                seam.end = Lift(piece.basePlane, rib.center + rib.dir * t1 + offset);
                seam.approachSide = Eigen::Vector3f(normal.x() * side, normal.y() * side, 0.0f);
                seam.ribHeight = rib.height;
                seam.confidence = rib.confidence;
                seam.ribA = static_cast<int>(i);
                seams.push_back(seam);
            };

            for (const auto& [cutBegin, cutEnd] : cuts) {
                if (cutBegin > cursor) {
                    emit(cursor, std::min(cutBegin, rib.tMax));
                }
                cursor = std::max(cursor, cutEnd);
            }
            if (cursor < rib.tMax) {
                emit(cursor, rib.tMax);
            }
        }
    }
}

void BuildVerticalSeams(const Workpiece& piece,
                        const std::vector<RibCandidate>& ribs,
                        const std::vector<Junction>& junctions,
                        std::vector<InitialSeam, Eigen::aligned_allocator<InitialSeam>>& seams)
{
    for (const Junction& junction : junctions) {
        const RibCandidate& a = ribs[static_cast<std::size_t>(junction.ribA)];
        const RibCandidate& b = ribs[static_cast<std::size_t>(junction.ribB)];
        const Eigen::Vector2f cross = a.center + a.dir * junction.tA;
        const float height = std::min(a.height, b.height);

        for (float sa : {1.0f, -1.0f}) {
            const bool armA = sa > 0.0f ? a.tMax >= junction.tA + b.thickness * 0.5f + kMinArm
                                        : a.tMin <= junction.tA - b.thickness * 0.5f - kMinArm;
            if (!armA) {
                continue;
            }
            for (float sb : {1.0f, -1.0f}) {
                const bool armB = sb > 0.0f ? b.tMax >= junction.tB + a.thickness * 0.5f + kMinArm
                                            : b.tMin <= junction.tB - a.thickness * 0.5f - kMinArm;
                if (!armB) {
                    continue;
                }
                const Eigen::Vector2f corner = cross + a.dir * (sa * b.thickness * 0.5f) + b.dir * (sb * a.thickness * 0.5f);
                const Eigen::Vector2f approach = (a.dir * sa + b.dir * sb).normalized();

                InitialSeam seam;
                seam.workpieceId = piece.id;
                seam.type = SeamType::VerticalFillet;
                seam.start = Lift(piece.basePlane, corner);
                seam.end = Lift(piece.basePlane, corner, height);
                seam.approachSide = Eigen::Vector3f(approach.x(), approach.y(), 0.0f);
                seam.ribHeight = height;
                seam.confidence = std::min(a.confidence, b.confidence);
                seam.ribA = junction.ribA;
                seam.ribB = junction.ribB;
                seams.push_back(seam);
            }
        }
    }
}

bool ProcessWorkpiece(const Cloud& scene,
                      const std::vector<int>& indices,
                      const Eigen::Vector4f& groundPlane,
                      const SceneSeamParams& params,
                      Workpiece& piece)
{
    HeightGrid grid;
    if (!BuildHeightGrid(scene, &indices, params.workpieceResolution, grid)) {
        return false;
    }

    // 底板：只在高出地面的栅格中找主平面
    Mask aboveGround(grid.z.size(), 0);
    for (int r = 0; r < grid.rows; ++r) {
        for (int c = 0; c < grid.cols; ++c) {
            const std::size_t id = grid.index(c, r);
            if (!std::isfinite(grid.z[id])) {
                continue;
            }
            const Eigen::Vector2f xy = grid.center(c, r);
            if (grid.z[id] - PlaneZ(groundPlane, xy.x(), xy.y()) > params.groundThreshold) {
                aboveGround[id] = 1;
            }
        }
    }
    if (!FitDominantPlane(grid, &aboveGround, params.basePlaneTolerance, params.maxGroundTiltDeg, piece.basePlane)) {
        return false;
    }

    Mask baseMask(grid.z.size(), 0);
    Mask ribMask(grid.z.size(), 0);
    std::vector<float> aboveBase(grid.z.size(), 0.0f);
    for (int r = 0; r < grid.rows; ++r) {
        for (int c = 0; c < grid.cols; ++c) {
            const std::size_t id = grid.index(c, r);
            if (!std::isfinite(grid.z[id])) {
                continue;
            }
            const Eigen::Vector2f xy = grid.center(c, r);
            const float above = grid.z[id] - PlaneZ(piece.basePlane, xy.x(), xy.y());
            aboveBase[id] = above;
            if (std::fabs(above) <= params.basePlaneTolerance) {
                baseMask[id] = 1;
            } else if (above > params.ribMinHeight) {
                ribMask[id] = 1;
            }
        }
    }
    const int closeCells = static_cast<int>(std::ceil((params.ribMaxThickness * 0.5f + 10.0f) / grid.res));
    baseMask = Close(baseMask, grid.cols, grid.rows, closeCells);
    // 立板阴影和根部遮挡总是被底板包围，填掉这些内部空洞得到底板轮廓，而真正的板边是开放的
    const Mask footprint = FillEnclosedHoles(baseMask, grid.cols, grid.rows);

    // 立板顶边是一条连续窄带，附近没有其他高点的栅格是离群点或反射假点
    std::vector<RibCell> ribCells;
    const Eigen::Vector2f localCenter = 0.5f * (piece.minXY + piece.maxXY);
    constexpr int kNeighbourRadius = 2;
    constexpr int kMinNeighbours = 3;
    for (int r = 0; r < grid.rows; ++r) {
        for (int c = 0; c < grid.cols; ++c) {
            const std::size_t id = grid.index(c, r);
            if (!ribMask[id]) {
                continue;
            }
            int neighbours = 0;
            for (int dr = -kNeighbourRadius; dr <= kNeighbourRadius; ++dr) {
                for (int dc = -kNeighbourRadius; dc <= kNeighbourRadius; ++dc) {
                    if ((dr != 0 || dc != 0) && grid.inside(c + dc, r + dr) && ribMask[grid.index(c + dc, r + dr)]) {
                        ++neighbours;
                    }
                }
            }
            if (neighbours >= kMinNeighbours) {
                ribCells.push_back({grid.center(c, r) - localCenter, aboveBase[id]});
            }
        }
    }

    piece.baseHeight = PlaneZ(piece.basePlane, localCenter.x(), localCenter.y())
        - PlaneZ(groundPlane, localCenter.x(), localCenter.y());

    std::vector<RibCandidate> ribs = ExtractRibs(ribCells, grid.res, params);
    SnapDirections(ribs, params.ribSnapAngleDeg);
    SnapJunctions(ribs, params);
    for (RibCandidate& rib : ribs) {
        rib.center += localCenter;
    }

    piece.ribs.clear();
    for (std::size_t i = 0; i < ribs.size(); ++i) {
        RibSegment segment;
        segment.id = static_cast<int>(i);
        segment.start = ribs[i].center + ribs[i].dir * ribs[i].tMin;
        segment.end = ribs[i].center + ribs[i].dir * ribs[i].tMax;
        segment.thickness = ribs[i].thickness;
        segment.height = ribs[i].height;
        segment.confidence = ribs[i].confidence;
        piece.ribs.push_back(segment);
    }

    if (!ribs.empty()) {
        const auto longest = std::max_element(ribs.begin(), ribs.end(), [](const RibCandidate& a, const RibCandidate& b) {
            return (a.tMax - a.tMin) < (b.tMax - b.tMin);
        });
        float yaw = std::atan2(longest->dir.y(), longest->dir.x());
        if (yaw < 0.0f) {
            yaw += kPi;
        }
        piece.yawRad = yaw;
    }

    std::vector<Junction> junctions;
    constexpr float kJunctionSlack = 5.0f;
    for (std::size_t i = 0; i < ribs.size(); ++i) {
        for (std::size_t j = i + 1; j < ribs.size(); ++j) {
            float ti = 0.0f;
            float tj = 0.0f;
            if (!IntersectLines(ribs[i], ribs[j], ti, tj)) {
                continue;
            }
            const float reachI = ribs[j].thickness * 0.5f + kJunctionSlack;
            const float reachJ = ribs[i].thickness * 0.5f + kJunctionSlack;
            if (ti < ribs[i].tMin - reachI || ti > ribs[i].tMax + reachI) {
                continue;
            }
            if (tj < ribs[j].tMin - reachJ || tj > ribs[j].tMax + reachJ) {
                continue;
            }
            junctions.push_back({static_cast<int>(i), static_cast<int>(j), ti, tj});
        }
    }

    BaseMaskView base{&grid, &footprint, &aboveBase, params.basePlaneTolerance};
    piece.seams.clear();
    BuildFlatSeams(piece, ribs, junctions, base, params, piece.seams);
    BuildVerticalSeams(piece, ribs, junctions, piece.seams);
    return true;
}

// ---------------------------------------------------------------------------
// 整场流程
// ---------------------------------------------------------------------------

CloudPtr Preprocess(const Cloud::ConstPtr& cloud, const SceneSeamParams& params)
{
    CloudPtr valid(new Cloud);
    valid->reserve(cloud->size());
    for (const pcl::PointXYZ& p : cloud->points) {
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
            continue;
        }
        if (std::fabs(p.x) < 1e-4f && std::fabs(p.y) < 1e-4f && std::fabs(p.z) < 1e-4f) {
            continue;
        }
        valid->push_back(p);
    }
    valid->width = static_cast<std::uint32_t>(valid->size());
    valid->height = 1;
    valid->is_dense = true;

    if (params.voxelSize <= 0.0f || valid->empty()) {
        return valid;
    }
    CloudPtr down(new Cloud);
    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    voxel.setInputCloud(valid);
    voxel.setLeafSize(params.voxelSize, params.voxelSize, params.voxelSize);
    voxel.filter(*down);
    if (down->empty()) {
        return valid;
    }
    return down;
}

// ---------------------------------------------------------------------------
// Z 轴翻转：相机坐标系（Z 指向地面）与计算用的 Z 向上坐标系互换
// ---------------------------------------------------------------------------

void NegateZ(Cloud& cloud)
{
    for (pcl::PointXYZ& p : cloud.points) {
        p.z = -p.z;
    }
}

// ax + by + cz + d = 0 在 z' = -z 下变为 ax + by - cz' + d = 0
Eigen::Vector4f NegateZ(const Eigen::Vector4f& plane)
{
    return Eigen::Vector4f(plane[0], plane[1], -plane[2], plane[3]);
}

Eigen::Vector3f NegateZ(const Eigen::Vector3f& v)
{
    return Eigen::Vector3f(v.x(), v.y(), -v.z());
}

void NegateZ(InitialSeam& seam)
{
    seam.start = NegateZ(seam.start);
    seam.end = NegateZ(seam.end);
    seam.approachSide = NegateZ(seam.approachSide);
}

SceneSeamResult ExtractImpl(const Cloud::ConstPtr& input, const SceneSeamParams& params)
{
    SceneSeamResult result;
    if (!input || input->empty()) {
        result.message = "输入点云为空";
        return result;
    }

    result.cloud = Preprocess(input, params);
    if (params.zAxisDown) {
        NegateZ(*result.cloud);
    }
    const Cloud& cloud = *result.cloud;
    if (cloud.size() < 100) {
        result.message = "有效点过少";
        return result;
    }

    HeightGrid grid;
    if (!BuildHeightGrid(cloud, nullptr, params.sceneResolution, grid)) {
        result.message = "整场高度图过大或为空";
        return result;
    }

    if (std::isfinite(params.groundHeight)) {
        result.groundPlane = HorizontalPlane(params.zAxisDown ? -params.groundHeight : params.groundHeight);
    } else if (!FitDominantPlane(grid, nullptr, params.groundFitTolerance, params.maxGroundTiltDeg, result.groundPlane)) {
        result.message = "地面估计失败";
        return result;
    }

    Mask objectMask(grid.z.size(), 0);
    for (int r = 0; r < grid.rows; ++r) {
        for (int c = 0; c < grid.cols; ++c) {
            const std::size_t id = grid.index(c, r);
            if (!std::isfinite(grid.z[id])) {
                continue;
            }
            const Eigen::Vector2f xy = grid.center(c, r);
            if (grid.z[id] - PlaneZ(result.groundPlane, xy.x(), xy.y()) > params.groundThreshold) {
                objectMask[id] = 1;
            }
        }
    }
    const int closeCells = static_cast<int>(std::ceil(params.closeRadius / grid.res));
    objectMask = Close(objectMask, grid.cols, grid.rows, closeCells);

    std::vector<int> labels;
    const std::vector<Component> components = LabelComponents(objectMask, grid.cols, grid.rows, labels);

    std::vector<int> labelToPiece(components.size() + 1, -1);
    const float cellArea = grid.res * grid.res;
    for (const Component& comp : components) {
        const float area = static_cast<float>(comp.count) * cellArea;
        const float sizeX = static_cast<float>(comp.maxC - comp.minC + 1) * grid.res;
        const float sizeY = static_cast<float>(comp.maxR - comp.minR + 1) * grid.res;
        if (area < params.minWorkpieceArea || std::min(sizeX, sizeY) < params.minWorkpieceSize) {
            continue;
        }
        Workpiece piece;
        piece.id = static_cast<int>(result.workpieces.size());
        piece.minXY = grid.origin + Eigen::Vector2f(static_cast<float>(comp.minC), static_cast<float>(comp.minR)) * grid.res;
        piece.maxXY = grid.origin + Eigen::Vector2f(static_cast<float>(comp.maxC + 1), static_cast<float>(comp.maxR + 1)) * grid.res;
        piece.center = 0.5f * (piece.minXY + piece.maxXY);
        labelToPiece[static_cast<std::size_t>(comp.label)] = piece.id;
        result.workpieces.push_back(piece);
    }

    std::vector<std::vector<int>> pieceIndices(result.workpieces.size());
    for (std::size_t i = 0; i < cloud.size(); ++i) {
        int c = 0;
        int r = 0;
        if (!grid.cellOf(cloud[i].x, cloud[i].y, c, r)) {
            continue;
        }
        const int label = labels[grid.index(c, r)];
        if (label == 0) {
            continue;
        }
        const int pieceId = labelToPiece[static_cast<std::size_t>(label)];
        if (pieceId >= 0) {
            pieceIndices[static_cast<std::size_t>(pieceId)].push_back(static_cast<int>(i));
        }
    }

    std::size_t rejected = 0;
    for (Workpiece& piece : result.workpieces) {
        const std::vector<int>& indices = pieceIndices[static_cast<std::size_t>(piece.id)];
        piece.cloud->reserve(indices.size());
        for (int index : indices) {
            piece.cloud->push_back(cloud[static_cast<std::size_t>(index)]);
        }
        piece.cloud->width = static_cast<std::uint32_t>(piece.cloud->size());
        piece.cloud->height = 1;
        piece.cloud->is_dense = true;

        if (!ProcessWorkpiece(cloud, indices, result.groundPlane, params, piece)) {
            continue;
        }
        for (const InitialSeam& seam : piece.seams) {
            if (seam.confidence >= params.minConfidence) {
                result.seams.push_back(seam);
            } else {
                ++rejected;
            }
        }
    }

    result.success = true;
    result.message = "工件 " + std::to_string(result.workpieces.size()) + " 个，焊缝 " + std::to_string(result.seams.size())
        + " 条，低置信度 " + std::to_string(rejected) + " 条";
    return result;
}

} // namespace

// ---------------------------------------------------------------------------
// 公共接口
// ---------------------------------------------------------------------------

Eigen::Vector2f RibSegment::direction() const
{
    const Eigen::Vector2f d = end - start;
    const float n = d.norm();
    return n > 1e-6f ? Eigen::Vector2f(d / n) : Eigen::Vector2f(1.0f, 0.0f);
}

float RibSegment::length() const
{
    return (end - start).norm();
}

float InitialSeam::length() const
{
    return (end - start).norm();
}

Workpiece::Workpiece()
    : cloud(new pcl::PointCloud<pcl::PointXYZ>)
{
}

SceneSeamResult::SceneSeamResult()
    : cloud(new pcl::PointCloud<pcl::PointXYZ>)
{
}

float PlaneZ(const Eigen::Vector4f& plane, float x, float y)
{
    if (std::fabs(plane[2]) < 1e-6f) {
        return 0.0f;
    }
    return -(plane[0] * x + plane[1] * y + plane[3]) / plane[2];
}

SceneSeamResult ExtractSceneSeams(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud, const SceneSeamParams& params)
{
    try {
        SceneSeamResult result = ExtractImpl(cloud, params);
        if (params.zAxisDown) {
            FlipResultZ(result);
        }
        return result;
    } catch (const std::exception& error) {
        SceneSeamResult failed;
        failed.message = std::string("异常: ") + error.what();
        return failed;
    }
}

// 高度类标量（baseHeight、ribHeight）不随坐标系翻转改变
void FlipResultZ(SceneSeamResult& result)
{
    if (result.cloud) {
        NegateZ(*result.cloud);
    }
    result.groundPlane = NegateZ(result.groundPlane);
    for (Workpiece& piece : result.workpieces) {
        piece.basePlane = NegateZ(piece.basePlane);
        if (piece.cloud) {
            NegateZ(*piece.cloud);
        }
        for (InitialSeam& seam : piece.seams) {
            NegateZ(seam);
        }
    }
    for (InitialSeam& seam : result.seams) {
        NegateZ(seam);
    }
}
