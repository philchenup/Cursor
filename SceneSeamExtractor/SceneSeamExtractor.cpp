#include "SceneSeamExtractor.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <queue>
#include <stdexcept>
#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

float SceneSeamExtractor::Percentile(std::vector<float> values, float fraction)
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


bool SceneSeamExtractor::BuildHeightGrid(const Cloud& cloud, const std::vector<int>* indices, float res, HeightGrid& grid)
{
    Eigen::Vector2f minXY(std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
    Eigen::Vector2f maxXY = -minXY;

    auto forEach = [&](auto&& fn) {
        if (indices) {
            for (int i : *indices) {
                fn(cloud[static_cast<std::size_t>(i)]);
            }
        }
        else {
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


std::vector<std::pair<int, int>> SceneSeamExtractor::DiskOffsets(int radius)
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

SceneSeamExtractor::Mask SceneSeamExtractor::Dilate(const Mask& src, int cols, int rows, int radius)
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

SceneSeamExtractor::Mask SceneSeamExtractor::Erode(const Mask& src, int cols, int rows, int radius)
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

SceneSeamExtractor::Mask SceneSeamExtractor::Close(const Mask& src, int cols, int rows, int radius)
{
    return Erode(Dilate(src, cols, rows, radius), cols, rows, radius);
}


std::vector<SceneSeamExtractor::Component> SceneSeamExtractor::LabelComponents(const Mask& mask, int cols, int rows, std::vector<int>& labels)
{
    labels.assign(mask.size(), 0);
    std::vector<Component> components;
    const int stepC[8] = { 1, -1, 0, 0, 1, 1, -1, -1 };
    const int stepR[8] = { 0, 0, 1, -1, 1, -1, 1, -1 };

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

int SceneSeamExtractor::CellsOf(float length, float res)
{
    return std::max(1, static_cast<int>(std::round(length / std::max(res, 1e-3f))));
}

Eigen::Vector4f SceneSeamExtractor::HorizontalPlane(float z)
{
    return Eigen::Vector4f(0.0f, 0.0f, 1.0f, -z);
}

// PCL 平面拟合，法向朝上。不创建可视化窗口，避免把 VTK 链进 Qt 程序后找不到入口。
bool SceneSeamExtractor::FitPlaneLeastSquares(const std::vector<Eigen::Vector3f>& points, Eigen::Vector4f& plane)
{
    if (points.size() < 3) {
        return false;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->reserve(points.size());
    for (const Eigen::Vector3f& p : points) {
        cloud->push_back(pcl::PointXYZ(p.x(), p.y(), p.z()));
    }

    // 随机种子固定。这些点已是候选内点，距离阈值取得很大，优化时用全部点做最小二乘。
    pcl::SACSegmentation<pcl::PointXYZ> seg(false);
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(1.0e6);
    seg.setMaxIterations(50);
    seg.setInputCloud(cloud);

    pcl::PointIndices inliers;
    pcl::ModelCoefficients coeff;
    seg.segment(inliers, coeff);
    if (inliers.indices.size() < 3 || coeff.values.size() < 4) {
        return false;
    }

    Eigen::Vector3f normal(coeff.values[0], coeff.values[1], coeff.values[2]);
    float d = coeff.values[3];
    if (normal.z() < 0.0f) {
        normal = -normal;
        d = -d;
    }
    const float norm = normal.norm();
    if (norm < 1e-9f) {
        return false;
    }
    normal /= norm;
    d /= norm;
    plane << normal.x(), normal.y(), normal.z(), d;
    return true;
}

float SceneSeamExtractor::PlaneTiltDeg(const Eigen::Vector4f& plane)
{
    const float cosTilt = std::min(1.0f, std::fabs(plane[2]) / plane.head<3>().norm());
    return std::acos(cosTilt) * 180.0f / kPi;
}

// 地面取直方图峰值：整场格子里地板最多，峰值落在地板上。
// 若取最低的显著层，倾斜或噪声会把初值压低，高的一侧地面会越过 groundThreshold。
// 底板仍取最低显著层：高出地面的格子里，最矮的一层是底板上表面，立板更高。
bool SceneSeamExtractor::HistogramBaseLevel(const std::vector<float>& values, float bin, bool lowestSignificant, float& level)
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
    if (!lowestSignificant) {
        const std::size_t peakBin = static_cast<std::size_t>(std::max_element(histogram.begin(), histogram.end()) - histogram.begin());
        level = minV + (static_cast<float>(peakBin) + 0.5f) * bin;
        return peak > 0;
    }
    const int significant = std::max(1, static_cast<int>(0.3f * static_cast<float>(peak)));
    for (std::size_t i = 0; i < histogram.size(); ++i) {
        if (histogram[i] >= significant) {
            level = minV + (static_cast<float>(i) + 0.5f) * bin;
            return true;
        }
    }
    return false;
}

// 用直方图初值，再迭代最小二乘拟合主平面。
// lowestSignificant 为真时从最低显著层起步，倾斜地面不会在第一步就把底板吸进来。
// fallbackToPeak 为真且倾角超限时，水平退回用峰值而不是被压低的那一层。
bool SceneSeamExtractor::FitDominantPlane(const HeightGrid& grid,
    const Mask* candidate,
    float tolerance,
    float maxTiltDeg,
    bool lowestSignificant,
    bool fallbackToPeak,
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
    if (!HistogramBaseLevel(heights, std::max(tolerance * 0.5f, 0.5f), lowestSignificant, mode)) {
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
                if (std::fabs(grid.z[id] - SceneSeamExtractor::PlaneZ(plane, xy.x(), xy.y())) <= tolerance) {
                    inliers.emplace_back(xy.x(), xy.y(), grid.z[id]);
                }
            }
        }
        Eigen::Vector4f fitted;
        if (!FitPlaneLeastSquares(inliers, fitted)) {
            break;
        }
        if (PlaneTiltDeg(fitted) > maxTiltDeg) {
            float fallback = mode;
            if (fallbackToPeak && lowestSignificant) {
                HistogramBaseLevel(heights, std::max(tolerance * 0.5f, 0.5f), false, fallback);
            }
            plane = HorizontalPlane(fallback);
            break;
        }
        plane = fitted;
    }
    return true;
}

Eigen::Vector3f SceneSeamExtractor::Lift(const Eigen::Vector4f& plane, const Eigen::Vector2f& xy, float above)
{
    return Eigen::Vector3f(xy.x(), xy.y(), SceneSeamExtractor::PlaneZ(plane, xy.x(), xy.y()) + above);
}


bool SceneSeamExtractor::MaskAt(const Mask& mask, int cols, int rows, int c, int r)
{
    return c >= 0 && r >= 0 && c < cols && r < rows && mask[static_cast<std::size_t>(r) * cols + c];
}

float SceneSeamExtractor::PointSegmentDistance(const Eigen::Vector2f& p, const Eigen::Vector2f& a, const Eigen::Vector2f& b)
{
    const Eigen::Vector2f ab = b - a;
    const float len2 = ab.squaredNorm();
    if (len2 < 1e-12f) {
        return (p - a).norm();
    }
    const float t = std::clamp((p - a).dot(ab) / len2, 0.0f, 1.0f);
    return (p - (a + ab * t)).norm();
}

void SceneSeamExtractor::DouglasPeucker(const std::vector<Eigen::Vector2f>& pts, int i, int j, float epsilon, std::vector<char>& keep)
{
    if (j <= i + 1) {
        return;
    }
    float best = 0.0f;
    int split = -1;
    for (int t = i + 1; t < j; ++t) {
        const float distance = PointSegmentDistance(pts[static_cast<std::size_t>(t)], pts[static_cast<std::size_t>(i)],
            pts[static_cast<std::size_t>(j)]);
        if (distance > best) {
            best = distance;
            split = t;
        }
    }
    if (split >= 0 && best > epsilon) {
        keep[static_cast<std::size_t>(split)] = 1;
        DouglasPeucker(pts, i, split, epsilon, keep);
        DouglasPeucker(pts, split, j, epsilon, keep);
    }
}

std::vector<Eigen::Vector2f> SceneSeamExtractor::Chain(const std::vector<Eigen::Vector2f>& pts, int from, int to)
{
    std::vector<Eigen::Vector2f> chain;
    const int n = static_cast<int>(pts.size());
    if (n == 0) {
        return chain;
    }
    for (int i = from;; i = (i + 1) % n) {
        chain.push_back(pts[static_cast<std::size_t>(i)]);
        if (i == to || static_cast<int>(chain.size()) > n) {
            break;
        }
    }
    return chain;
}

std::vector<Eigen::Vector2f> SceneSeamExtractor::SimplifyChain(const std::vector<Eigen::Vector2f>& chain, float epsilon)
{
    std::vector<Eigen::Vector2f> simplified;
    if (chain.size() < 2) {
        return chain;
    }
    std::vector<char> keep(chain.size(), 0);
    keep.front() = 1;
    keep.back() = 1;
    DouglasPeucker(chain, 0, static_cast<int>(chain.size()) - 1, epsilon, keep);
    for (std::size_t i = 0; i < chain.size(); ++i) {
        if (keep[i]) {
            simplified.push_back(chain[i]);
        }
    }
    return simplified;
}

// 闭环：取距质心最远点和距该点最远点把环拆成两段再拟合，避免首尾弦长为零。
std::vector<Eigen::Vector2f> SceneSeamExtractor::SimplifyLoop(std::vector<Eigen::Vector2f> pts, float epsilon)
{
    if (pts.size() >= 2 && (pts.front() - pts.back()).norm() < 1e-4f) {
        pts.pop_back();
    }
    if (pts.size() < 4) {
        return {};
    }

    Eigen::Vector2f mean = Eigen::Vector2f::Zero();
    for (const Eigen::Vector2f& p : pts) {
        mean += p;
    }
    mean /= static_cast<float>(pts.size());

    int far = 0;
    float farDist = -1.0f;
    for (int i = 0; i < static_cast<int>(pts.size()); ++i) {
        const float distance = (pts[static_cast<std::size_t>(i)] - mean).squaredNorm();
        if (distance > farDist) {
            farDist = distance;
            far = i;
        }
    }
    int opposite = far;
    float oppositeDist = -1.0f;
    for (int i = 0; i < static_cast<int>(pts.size()); ++i) {
        const float distance = (pts[static_cast<std::size_t>(i)] - pts[static_cast<std::size_t>(far)]).squaredNorm();
        if (distance > oppositeDist) {
            oppositeDist = distance;
            opposite = i;
        }
    }
    if (opposite == far) {
        return {};
    }

    const std::vector<Eigen::Vector2f> first = SimplifyChain(Chain(pts, far, opposite), epsilon);
    const std::vector<Eigen::Vector2f> second = SimplifyChain(Chain(pts, opposite, far), epsilon);
    std::vector<Eigen::Vector2f> loop;
    loop.insert(loop.end(), first.begin(), first.end() - 1);
    loop.insert(loop.end(), second.begin(), second.end() - 1);

    // 沿轮廓向前、向后各走一段，看整体方向还是不是同一条边。
    // 自由端会在一个板厚之内掉头，这个点必须留下，否则两侧焊缝收到尖角。
    auto turnsAround = [](const std::vector<Eigen::Vector2f>& poly, std::size_t index) {
        const std::size_t n = poly.size();
        auto walk = [&](int sign) {
            float walked = 0.0f;
            Eigen::Vector2f pos = poly[index];
            std::size_t cursor = index;
            while (walked < kCornerLookahead) {
                cursor = static_cast<std::size_t>((static_cast<int>(cursor) + sign + static_cast<int>(n)) % static_cast<int>(n));
                if (cursor == index) {
                    break;
                }
                const float step = (poly[cursor] - pos).norm();
                walked += step;
                pos = poly[cursor];
                if (step < 1e-6f) {
                    continue;
                }
            }
            return pos;
            };
        const Eigen::Vector2f incoming = poly[index] - walk(-1);
        const Eigen::Vector2f outgoing = walk(1) - poly[index];
        const float inLen = incoming.norm();
        const float outLen = outgoing.norm();
        if (inLen < 1e-3f || outLen < 1e-3f) {
            return true;
        }
        return incoming.dot(outgoing) < 0.75f * inLen * outLen;
        };

    bool changed = true;
    while (changed && loop.size() > 4) {
        changed = false;
        for (std::size_t i = 0; i < loop.size();) {
            const std::size_t n = loop.size();
            const Eigen::Vector2f prev = loop[(i + n - 1) % n];
            const Eigen::Vector2f cur = loop[i];
            const Eigen::Vector2f next = loop[(i + 1) % n];
            const float lenIn = (cur - prev).norm();
            const float lenOut = (next - cur).norm();
            if (lenIn < 1e-3f || lenOut < 1e-3f) {
                loop.erase(loop.begin() + static_cast<std::ptrdiff_t>(i));
                changed = true;
                continue;
            }
            if (!turnsAround(loop, i) && PointSegmentDistance(cur, prev, next) <= kSameDirectionJog) {
                loop.erase(loop.begin() + static_cast<std::ptrdiff_t>(i));
                changed = true;
                continue;
            }
            ++i;
        }
    }

    // 两块板的直角会被栅格磨成一条十几毫米的短边，立焊缝因此对不上拐角。
    // 短边两端都是长边、转角接近 90° 时，收到两条长边的交点。
    // 自由端是两侧长边掉头，转角接近 180°，中间隔着板厚，不能收到一个点。
    constexpr float kMinArm = 30.0f;
    constexpr float kMaxChamfer = 40.0f;
    bool snapped = true;
    while (snapped && loop.size() > 4) {
        snapped = false;
        const std::size_t n = loop.size();
        for (std::size_t i = 0; i < n; ++i) {
            const std::size_t i1 = (i + 1) % n;
            const Eigen::Vector2f in = loop[i1] - loop[i];
            if (in.norm() < kMinArm) {
                continue;
            }
            std::size_t k = i1;
            float chamfer = 0.0f;
            int steps = 0;
            while (steps < 8) {
                const std::size_t nxt = (k + 1) % n;
                if (nxt == i) {
                    break;
                }
                const float step = (loop[nxt] - loop[k]).norm();
                if (step >= kMinArm || chamfer + step > kMaxChamfer) {
                    break;
                }
                chamfer += step;
                k = nxt;
                ++steps;
            }
            if (steps < 1) {
                continue;
            }
            const std::size_t m = (k + 1) % n;
            const Eigen::Vector2f out = loop[m] - loop[k];
            if (out.norm() < kMinArm || m == i) {
                continue;
            }
            const float cross = in.x() * out.y() - in.y() * out.x();
            const float turnDeg = std::fabs(std::atan2(cross, in.dot(out))) * 180.0f / kPi;
            if (turnDeg < 50.0f || turnDeg > 130.0f || std::fabs(cross) < 1e-4f) {
                continue;
            }
            const Eigen::Vector2f delta = loop[k] - loop[i];
            const float t = (delta.x() * out.y() - delta.y() * out.x()) / cross;
            const Eigen::Vector2f hit = loop[i] + in * t;
            if ((hit - loop[i1]).norm() > kMaxChamfer || (hit - loop[k]).norm() > kMaxChamfer) {
                continue;
            }
            std::vector<Eigen::Vector2f> next;
            next.reserve(n);
            for (std::size_t p = m;; p = (p + 1) % n) {
                next.push_back(loop[p]);
                if (p == i) {
                    break;
                }
            }
            next.push_back(hit);
            if (next.size() >= 4) {
                loop.swap(next);
                snapped = true;
            }
            break;
        }
    }
    return loop;
}

std::vector<std::vector<Eigen::Vector2f>> SceneSeamExtractor::TraceContours(const Mask& mask, const HeightGrid& grid)
{
    const int cols = grid.cols;
    const int rows = grid.rows;
    const int stride = cols + 1;
    std::unordered_map<std::uint64_t, std::uint8_t> outgoing;
    outgoing.reserve(static_cast<std::size_t>(std::min(cols, rows)) * 8);

    auto keyOf = [stride](int x, int y) {
        return static_cast<std::uint64_t>(y) * static_cast<std::uint64_t>(stride) + static_cast<std::uint64_t>(x);
        };
    auto addEdge = [&](int x, int y, int dir) {
        outgoing[keyOf(x, y)] |= static_cast<std::uint8_t>(1u << dir);
        };

    std::vector<std::tuple<int, int, int>> edges;
    edges.reserve(static_cast<std::size_t>(cols + rows) * 4);
    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c) {
            if (!MaskAt(mask, cols, rows, c, r)) {
                continue;
            }
            // 底边向右、右边向上、顶边向左、左边向下，材料保持在左侧
            if (!MaskAt(mask, cols, rows, c, r - 1)) {
                addEdge(c, r, 0);
                edges.emplace_back(c, r, 0);
            }
            if (!MaskAt(mask, cols, rows, c + 1, r)) {
                addEdge(c + 1, r, 1);
                edges.emplace_back(c + 1, r, 1);
            }
            if (!MaskAt(mask, cols, rows, c, r + 1)) {
                addEdge(c + 1, r + 1, 2);
                edges.emplace_back(c + 1, r + 1, 2);
            }
            if (!MaskAt(mask, cols, rows, c - 1, r)) {
                addEdge(c, r + 1, 3);
                edges.emplace_back(c, r + 1, 3);
            }
        }
    }

    auto take = [&](int x, int y, int dir) {
        const auto it = outgoing.find(keyOf(x, y));
        if (it == outgoing.end() || (it->second & static_cast<std::uint8_t>(1u << dir)) == 0) {
            return false;
        }
        it->second = static_cast<std::uint8_t>(it->second & ~static_cast<std::uint8_t>(1u << dir));
        return true;
        };
    auto nextDir = [&](int x, int y, int incoming, int& dir) {
        const auto it = outgoing.find(keyOf(x, y));
        if (it == outgoing.end() || it->second == 0) {
            return false;
        }
        const int order[4] = { (incoming + 1) & 3, incoming & 3, (incoming + 3) & 3, (incoming + 2) & 3 };
        for (int candidate : order) {
            if ((it->second & static_cast<std::uint8_t>(1u << candidate)) == 0) {
                continue;
            }
            it->second = static_cast<std::uint8_t>(it->second & ~static_cast<std::uint8_t>(1u << candidate));
            dir = candidate;
            return true;
        }
        return false;
        };

    std::vector<std::vector<Eigen::Vector2f>> loops;
    const int guardLimit = static_cast<int>(edges.size()) + 2;
    for (const auto& [sx, sy, sdir] : edges) {
        if (!take(sx, sy, sdir)) {
            continue;
        }
        std::vector<Eigen::Vector2f> raw;
        raw.push_back(grid.corner(sx, sy));
        int x = sx + kDirX[sdir];
        int y = sy + kDirY[sdir];
        int incoming = sdir;
        raw.push_back(grid.corner(x, y));
        int guard = 0;
        while ((x != sx || y != sy) && guard++ < guardLimit) {
            int dir = 0;
            if (!nextDir(x, y, incoming, dir)) {
                break;
            }
            x += kDirX[dir];
            y += kDirY[dir];
            incoming = dir;
            raw.push_back(grid.corner(x, y));
        }
        if ((raw.front() - raw.back()).squaredNorm() > grid.res * grid.res) {
            continue;
        }
        std::vector<Eigen::Vector2f> loop = SimplifyLoop(std::move(raw), std::max(kRasterEpsilon, grid.res));
        if (loop.size() >= 4) {
            loops.push_back(std::move(loop));
        }
    }
    return loops;
}

// 沿轮廓边、朝材料内侧采样顶面高度。anchor 为端点，toward 指向边的内部。
float SceneSeamExtractor::SampleEdgeHeight(const HeightGrid& grid,
    const Mask& crest,
    const std::vector<float>& above,
    const Eigen::Vector2f& anchor,
    const Eigen::Vector2f& toward,
    const Eigen::Vector2f& inward)
{
    const float alongReach = 28.0f;
    const float depth = 20.0f;
    const int pad = CellsOf(std::max(alongReach, depth) + grid.res, grid.res) + 1;
    int ac = 0;
    int ar = 0;
    if (!grid.cellOf(anchor.x(), anchor.y(), ac, ar)) {
        ac = static_cast<int>(std::floor((anchor.x() - grid.origin.x()) / grid.res));
        ar = static_cast<int>(std::floor((anchor.y() - grid.origin.y()) / grid.res));
    }

    std::vector<float> heights;
    for (int r = ar - pad; r <= ar + pad; ++r) {
        for (int c = ac - pad; c <= ac + pad; ++c) {
            if (!grid.inside(c, r) || !crest[grid.index(c, r)]) {
                continue;
            }
            const Eigen::Vector2f delta = grid.center(c, r) - anchor;
            const float along = delta.dot(toward);
            const float side = delta.dot(inward);
            if (along < -grid.res || along > alongReach || side < -grid.res || side > depth) {
                continue;
            }
            heights.push_back(above[grid.index(c, r)]);
        }
    }
    if (heights.empty()) {
        for (int r = ar - pad; r <= ar + pad; ++r) {
            for (int c = ac - pad; c <= ac + pad; ++c) {
                if (!grid.inside(c, r) || !crest[grid.index(c, r)]) {
                    continue;
                }
                if ((grid.center(c, r) - anchor).norm() <= 36.0f) {
                    heights.push_back(above[grid.index(c, r)]);
                }
            }
        }
    }
    if (heights.empty()) {
        return 0.0f;
    }
    return Percentile(std::move(heights), 0.6f);
}


bool SceneSeamExtractor::ProcessWorkpiece(const Cloud& scene,
    const std::vector<int>& indices,
    const Eigen::Vector4f& groundPlane,
    const SceneSeamParams& params,
    Workpiece& piece)
{
    struct VerticalCandidate {
        Eigen::Vector2f xy = Eigen::Vector2f::Zero();
        float height = 0.0f;
        float confidence = 0.0f;
        int edgeA = -1;
        int edgeB = -1;
        Eigen::Vector2f approach = Eigen::Vector2f::Zero();
    };

    HeightGrid grid;
    if (!BuildHeightGrid(scene, &indices, params.workpieceResolution, grid)) {
        return false;
    }

    Mask aboveGround(grid.z.size(), 0);
    for (int r = 0; r < grid.rows; ++r) {
        for (int c = 0; c < grid.cols; ++c) {
            const std::size_t id = grid.index(c, r);
            if (!std::isfinite(grid.z[id])) {
                continue;
            }
            const Eigen::Vector2f xy = grid.center(c, r);
            if (grid.z[id] - SceneSeamExtractor::PlaneZ(groundPlane, xy.x(), xy.y()) > params.groundThreshold) {
                aboveGround[id] = 1;
            }
        }
    }
    if (!FitDominantPlane(grid, &aboveGround, kBasePlaneTolerance, kMaxGroundTiltDeg, true, false, piece.basePlane)) {
        return false;
    }

    std::vector<float> above(grid.z.size(), 0.0f);
    Mask crest(grid.z.size(), 0);
    const int topRadius = CellsOf(kTopNeighborhood, grid.res);
    for (int r = 0; r < grid.rows; ++r) {
        for (int c = 0; c < grid.cols; ++c) {
            const std::size_t id = grid.index(c, r);
            if (!std::isfinite(grid.z[id])) {
                continue;
            }
            const Eigen::Vector2f xy = grid.center(c, r);
            const float h = grid.z[id] - SceneSeamExtractor::PlaneZ(piece.basePlane, xy.x(), xy.y());
            above[id] = h;
            if (h < params.ribMinHeight) {
                continue;
            }
            // 飞点沿视线从顶边斜着落到板上，邻域里只高出一截。
            // 另一块更高的立板是台阶，高差更大，矮板自己的顶面要保留，否则接头处轮廓断开。
            bool faceSlope = false;
            for (int dr = -topRadius; dr <= topRadius && !faceSlope; ++dr) {
                for (int dc = -topRadius; dc <= topRadius; ++dc) {
                    if ((dr == 0 && dc == 0) || !grid.inside(c + dc, r + dr)) {
                        continue;
                    }
                    const std::size_t nid = grid.index(c + dc, r + dr);
                    if (!std::isfinite(grid.z[nid])) {
                        continue;
                    }
                    const Eigen::Vector2f nxy = grid.center(c + dc, r + dr);
                    const float rise = grid.z[nid] - SceneSeamExtractor::PlaneZ(piece.basePlane, nxy.x(), nxy.y()) - h;
                    if (rise > kTopTolerance && rise < kPlateStep) {
                        faceSlope = true;
                        break;
                    }
                }
            }
            if (!faceSlope) {
                crest[id] = 1;
            }
        }
    }

    const int supportRadius = CellsOf(kTopSupportRadius, grid.res);
    Mask supported(grid.z.size(), 0);
    for (int r = 0; r < grid.rows; ++r) {
        for (int c = 0; c < grid.cols; ++c) {
            if (!crest[grid.index(c, r)]) {
                continue;
            }
            int neighbours = 0;
            for (int dr = -supportRadius; dr <= supportRadius && neighbours == 0; ++dr) {
                for (int dc = -supportRadius; dc <= supportRadius; ++dc) {
                    if ((dr != 0 || dc != 0) && grid.inside(c + dc, r + dr) && crest[grid.index(c + dc, r + dr)]) {
                        ++neighbours;
                        break;
                    }
                }
            }
            if (neighbours > 0) {
                supported[grid.index(c, r)] = 1;
            }
        }
    }

    const Mask top = Close(supported, grid.cols, grid.rows, CellsOf(kTopCloseRadius, grid.res));
    const std::vector<std::vector<Eigen::Vector2f>> loops = TraceContours(top, grid);

    const Eigen::Vector2f localCenter = 0.5f * (piece.minXY + piece.maxXY);
    piece.baseHeight = SceneSeamExtractor::PlaneZ(piece.basePlane, localCenter.x(), localCenter.y())
        - SceneSeamExtractor::PlaneZ(groundPlane, localCenter.x(), localCenter.y());

    struct EdgeRecord {
        Eigen::Vector2f a;
        Eigen::Vector2f b;
        Eigen::Vector2f inward;
        float height = 0.0f;
        float confidence = 0.0f;
    };
    std::vector<EdgeRecord> edges;
    std::vector<VerticalCandidate> verticals;

    for (const std::vector<Eigen::Vector2f>& loop : loops) {
        const int n = static_cast<int>(loop.size());
        std::vector<int> edgeIndex(static_cast<std::size_t>(n), -1);
        for (int i = 0; i < n; ++i) {
            const Eigen::Vector2f& a = loop[static_cast<std::size_t>(i)];
            const Eigen::Vector2f& b = loop[static_cast<std::size_t>((i + 1) % n)];
            const Eigen::Vector2f delta = b - a;
            const float length = delta.norm();
            if (length < params.minSeamLength) {
                continue;
            }
            const Eigen::Vector2f dir = delta / length;
            const Eigen::Vector2f inward(-dir.y(), dir.x()); // 材料在左侧
            const float h0 = SampleEdgeHeight(grid, supported, above, a, dir, inward);
            const float h1 = SampleEdgeHeight(grid, supported, above, b, -dir, inward);
            EdgeRecord edge;
            edge.a = a;
            edge.b = b;
            edge.inward = inward;
            edge.height = std::max(h0, h1);
            edge.confidence = edge.height >= params.ribMinHeight ? 0.9f : 0.55f;
            edgeIndex[static_cast<std::size_t>(i)] = static_cast<int>(edges.size());
            edges.push_back(edge);
        }

        for (int i = 0; i < n; ++i) {
            const int incoming = edgeIndex[static_cast<std::size_t>((i + n - 1) % n)];
            const int outgoing = edgeIndex[static_cast<std::size_t>(i)];
            if (incoming < 0 || outgoing < 0) {
                continue;
            }
            const EdgeRecord& prev = edges[static_cast<std::size_t>(incoming)];
            const EdgeRecord& next = edges[static_cast<std::size_t>(outgoing)];
            const Eigen::Vector2f inDir = (prev.b - prev.a).normalized();
            const Eigen::Vector2f outDir = (next.b - next.a).normalized();
            const float cross = inDir.x() * outDir.y() - inDir.y() * outDir.x();
            const float dot = inDir.dot(outDir);
            const float turnDeg = std::atan2(cross, dot) * 180.0f / kPi;
            // 材料在左侧时，凹拐角是右转
            if (turnDeg > -kMinCornerTurnDeg) {
                continue;
            }
            const float height = std::min(prev.height > 1.0f ? prev.height : next.height,
                next.height > 1.0f ? next.height : prev.height);
            if (height < params.ribMinHeight) {
                continue;
            }
            const Eigen::Vector2f outwardIn(inDir.y(), -inDir.x());
            const Eigen::Vector2f outwardOut(outDir.y(), -outDir.x());
            Eigen::Vector2f approach = outwardIn + outwardOut;
            if (approach.norm() < 1e-4f) {
                approach = outwardIn;
            }
            approach.normalize();
            verticals.push_back({ loop[static_cast<std::size_t>(i)], height, std::min(prev.confidence, next.confidence),
                                 incoming, outgoing, approach });
        }
    }

    // 圆角上挨得很近的两个凹点并成一个，距离小于最薄板厚，不会把 T 形两侧的立焊缝并掉
    std::vector<char> used(verticals.size(), 0);
    std::vector<VerticalCandidate> merged;
    constexpr float kVerticalMerge = 3.0f;
    for (std::size_t i = 0; i < verticals.size(); ++i) {
        if (used[i]) {
            continue;
        }
        Eigen::Vector2f sum = verticals[i].xy;
        float heightSum = verticals[i].height;
        Eigen::Vector2f approachSum = verticals[i].approach;
        float confidence = verticals[i].confidence;
        int count = 1;
        int edgeA = verticals[i].edgeA;
        int edgeB = verticals[i].edgeB;
        used[i] = 1;
        for (std::size_t j = i + 1; j < verticals.size(); ++j) {
            if (used[j] || (verticals[j].xy - verticals[i].xy).norm() > kVerticalMerge) {
                continue;
            }
            used[j] = 1;
            sum += verticals[j].xy;
            heightSum += verticals[j].height;
            approachSum += verticals[j].approach;
            confidence = std::min(confidence, verticals[j].confidence);
            ++count;
        }
        VerticalCandidate corner;
        corner.xy = sum / static_cast<float>(count);
        corner.height = heightSum / static_cast<float>(count);
        corner.confidence = confidence;
        corner.edgeA = edgeA;
        corner.edgeB = edgeB;
        corner.approach = approachSum.norm() > 1e-4f ? approachSum.normalized() : verticals[i].approach;
        merged.push_back(corner);
    }

    piece.ribs.clear();
    piece.seams.clear();
    float longest = 0.0f;
    for (std::size_t i = 0; i < edges.size(); ++i) {
        const EdgeRecord& edge = edges[i];
        RibSegment rib;
        rib.id = static_cast<int>(i);
        rib.start = edge.a;
        rib.end = edge.b;
        rib.height = edge.height;
        rib.confidence = edge.confidence;
        piece.ribs.push_back(rib);
        if (rib.length() > longest) {
            longest = rib.length();
            float yaw = std::atan2(edge.b.y() - edge.a.y(), edge.b.x() - edge.a.x());
            if (yaw < 0.0f) {
                yaw += kPi;
            }
            piece.yawRad = yaw;
        }

        InitialSeam seam;
        seam.workpieceId = piece.id;
        seam.type = SeamType::FlatFillet;
        seam.start = Lift(piece.basePlane, edge.a);
        seam.end = Lift(piece.basePlane, edge.b);
        seam.approachSide = Eigen::Vector3f(-edge.inward.x(), -edge.inward.y(), 0.0f);
        seam.ribHeight = edge.height;
        seam.confidence = edge.confidence;
        seam.ribA = rib.id;
        piece.seams.push_back(seam);
    }
    for (const VerticalCandidate& corner : merged) {
        InitialSeam seam;
        seam.workpieceId = piece.id;
        seam.type = SeamType::VerticalFillet;
        seam.start = Lift(piece.basePlane, corner.xy);
        seam.end = Lift(piece.basePlane, corner.xy, corner.height);
        seam.approachSide = Eigen::Vector3f(corner.approach.x(), corner.approach.y(), 0.0f);
        seam.ribHeight = corner.height;
        seam.confidence = corner.confidence;
        seam.ribA = corner.edgeA;
        seam.ribB = corner.edgeB;
        piece.seams.push_back(seam);
    }
    return true;
}

// ---------------------------------------------------------------------------
// 整场流程
// ---------------------------------------------------------------------------

SceneSeamExtractor::CloudPtr SceneSeamExtractor::Preprocess(const Cloud::ConstPtr& cloud, const SceneSeamParams& params)
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

void SceneSeamExtractor::NegateZ(Cloud& cloud)
{
    for (pcl::PointXYZ& p : cloud.points) {
        p.z = -p.z;
    }
}

// ax + by + cz + d = 0 在 z' = -z 下变为 ax + by - cz' + d = 0
Eigen::Vector4f SceneSeamExtractor::NegateZ(const Eigen::Vector4f& plane)
{
    return Eigen::Vector4f(plane[0], plane[1], -plane[2], plane[3]);
}

Eigen::Vector3f SceneSeamExtractor::NegateZ(const Eigen::Vector3f& v)
{
    return Eigen::Vector3f(v.x(), v.y(), -v.z());
}

void SceneSeamExtractor::NegateZ(InitialSeam& seam)
{
    seam.start = NegateZ(seam.start);
    seam.end = NegateZ(seam.end);
    seam.approachSide = NegateZ(seam.approachSide);
}


SceneSeamExtractor::Rgb SceneSeamExtractor::FromHue(float hue, float sat, float val)
{
    const float h = hue * 6.0f;
    const int sector = static_cast<int>(h) % 6;
    const float f = h - std::floor(h);
    const float p = val * (1.0f - sat);
    const float q = val * (1.0f - sat * f);
    const float t = val * (1.0f - sat * (1.0f - f));
    float rf = 0.0f;
    float gf = 0.0f;
    float bf = 0.0f;
    switch (sector) {
    case 0: rf = val; gf = t; bf = p; break;
    case 1: rf = q; gf = val; bf = p; break;
    case 2: rf = p; gf = val; bf = t; break;
    case 3: rf = p; gf = q; bf = val; break;
    case 4: rf = t; gf = p; bf = val; break;
    default: rf = val; gf = p; bf = q; break;
    }
    return Rgb{ static_cast<std::uint8_t>(rf * 255.0f), static_cast<std::uint8_t>(gf * 255.0f),
               static_cast<std::uint8_t>(bf * 255.0f) };
}

SceneSeamExtractor::Rgb SceneSeamExtractor::ColorOfLabel(int label, bool kept)
{
    const float hue = std::fmod(static_cast<float>(label) * 0.6180339887f, 1.0f);
    return kept ? FromHue(hue, 0.72f, 0.96f) : FromHue(hue, 0.45f, 0.55f);
}

SceneSeamResult SceneSeamExtractor::ExtractImpl(const Cloud::ConstPtr& input, const SceneSeamParams& params)
{
    SceneSeamResult result;
    if (!input || input->empty()) {
        result.message = "Input cloud is empty";
        return result;
    }
    if (params.sceneResolution <= 0.0f || params.workpieceResolution <= 0.0f) {
        result.message = "resolution must is positive";
        return result;
    }

    result.cloud = Preprocess(input, params);
    if (params.zAxisDown) {
        NegateZ(*result.cloud);
    }
    const Cloud& cloud = *result.cloud;
    if (cloud.size() < 100) {
        result.message = "valid point num is too low";
        return result;
    }

    HeightGrid grid;
    if (!BuildHeightGrid(cloud, nullptr, params.sceneResolution, grid)) {
        result.message = "Height map too large";
        return result;
    }

    if (std::isfinite(params.groundHeight)) {
        result.groundPlane = HorizontalPlane(params.zAxisDown ? -params.groundHeight : params.groundHeight);
    }
    else if (!FitDominantPlane(grid, nullptr, kGroundFitTolerance, kMaxGroundTiltDeg, true, true, result.groundPlane)) {
        result.message = "plane estimate failed";
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
            if (grid.z[id] - SceneSeamExtractor::PlaneZ(result.groundPlane, xy.x(), xy.y()) > params.groundThreshold) {
                objectMask[id] = 1;
            }
        }
    }
    objectMask = Close(objectMask, grid.cols, grid.rows, CellsOf(kWorkpieceCloseRadius, grid.res));

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
        if (pieceId < 0) {
            continue;
        }
        // 掩码按格子最高点判定。同一格里贴着地面的点不能跟着最高点进入工件。
        const float above = cloud[i].z - SceneSeamExtractor::PlaneZ(result.groundPlane, cloud[i].x, cloud[i].y);
        if (above <= params.groundThreshold) {
            continue;
        }
        pieceIndices[static_cast<std::size_t>(pieceId)].push_back(static_cast<int>(i));
    }

    std::vector<Workpiece, Eigen::aligned_allocator<Workpiece>> kept;
    kept.reserve(result.workpieces.size());
    std::size_t rejected = 0;
    for (Workpiece& piece : result.workpieces) {
        const std::vector<int>& indices = pieceIndices[static_cast<std::size_t>(piece.id)];
        if (indices.empty()) {
            continue;
        }
        Eigen::Vector2f minXY(std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
        Eigen::Vector2f maxXY = -minXY;
        piece.cloud->reserve(indices.size());
        for (int index : indices) {
            const pcl::PointXYZ& point = cloud[static_cast<std::size_t>(index)];
            piece.cloud->push_back(point);
            const Eigen::Vector2f xy(point.x, point.y);
            minXY = minXY.cwiseMin(xy);
            maxXY = maxXY.cwiseMax(xy);
        }
        piece.cloud->width = static_cast<std::uint32_t>(piece.cloud->size());
        piece.cloud->height = 1;
        piece.cloud->is_dense = true;
        // 外接框按留下的点重算。连通域格子框会把闭运算桥上的地面裙边算进去。
        piece.minXY = minXY;
        piece.maxXY = maxXY;
        piece.center = 0.5f * (piece.minXY + piece.maxXY);
        piece.id = static_cast<int>(kept.size());

        if (!ProcessWorkpiece(cloud, indices, result.groundPlane, params, piece)) {
            kept.push_back(std::move(piece));
            continue;
        }
        for (const InitialSeam& seam : piece.seams) {
            if (seam.confidence >= kMinConfidence) {
                result.seams.push_back(seam);
            }
            else {
                ++rejected;
            }
        }
        kept.push_back(std::move(piece));
    }
    result.workpieces = std::move(kept);

    result.success = true;
    result.message = "工件 " + std::to_string(result.workpieces.size()) + " 个，焊缝 " + std::to_string(result.seams.size())
        + " 条，低置信度 " + std::to_string(rejected) + " 条";
    return result;
}


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

float SceneSeamExtractor::PlaneZ(const Eigen::Vector4f& plane, float x, float y)
{
    if (std::fabs(plane[2]) < 1e-6f) {
        return 0.0f;
    }
    return -(plane[0] * x + plane[1] * y + plane[3]) / plane[2];
}

SceneSeamExtractor::SceneSeamExtractor() = default;

SceneSeamExtractor::SceneSeamExtractor(SceneSeamParams params)
    : params_(std::move(params))
{
}

void SceneSeamExtractor::setParams(const SceneSeamParams& params)
{
    params_ = params;
}

const SceneSeamParams& SceneSeamExtractor::params() const
{
    return params_;
}

SceneSeamResult SceneSeamExtractor::ExtractSceneSeams(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud) const
{
    return SceneSeamExtractor::ExtractSceneSeams(cloud, params_);
}

SceneSeamResult SceneSeamExtractor::ExtractSceneSeams(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud,
    const SceneSeamParams& params)
{
    try {
        SceneSeamResult result = ExtractImpl(cloud, params);
        if (params.zAxisDown) {
            SceneSeamExtractor::FlipResultZ(result);
        }
        return result;
    }
    catch (const std::exception& error) {
        SceneSeamResult failed;
        failed.message = std::string("异常: ") + error.what();
        return failed;
    }
}

// 高度类标量（baseHeight、ribHeight）不随坐标系翻转改变
void SceneSeamExtractor::FlipResultZ(SceneSeamResult& result)
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
