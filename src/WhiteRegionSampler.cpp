#include "WhiteRegionSampler.h"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <random>
#include <string>
#include <utility>

namespace {

constexpr float kPi = 3.14159265358979323846f;

cv::Mat toGray(const cv::Mat& src)
{
    cv::Mat gray;
    if (src.empty()) {
        return gray;
    }
    if (src.channels() == 4) {
        cv::cvtColor(src, gray, cv::COLOR_BGRA2GRAY);
    } else if (src.channels() == 3) {
        cv::cvtColor(src, gray, cv::COLOR_BGR2GRAY);
    } else if (src.channels() == 1) {
        gray = src;
    } else {
        cv::cvtColor(src, gray, cv::COLOR_BGR2GRAY);
    }
    return gray;
}

cv::Mat makeWhiteMask(const cv::Mat& image)
{
    cv::Mat gray = toGray(image);
    if (gray.empty()) {
        return {};
    }
    cv::Mat mask;
    cv::threshold(gray, mask, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
    // Keep the larger polarity as the "white" foreground.
    if (cv::countNonZero(mask) * 2 < mask.rows * mask.cols) {
        // already the smaller blob — typical white-on-black
    } else {
        cv::bitwise_not(mask, mask);
    }
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);
    return mask;
}

bool largestContour(const cv::Mat& mask, std::vector<cv::Point>& contour)
{
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    if (contours.empty()) {
        return false;
    }
    auto it = std::max_element(contours.begin(), contours.end(),
                               [](const std::vector<cv::Point>& a, const std::vector<cv::Point>& b) {
                                   return cv::contourArea(a) < cv::contourArea(b);
                               });
    contour = *it;
    return cv::contourArea(contour) > 16.0;
}

void orderQuad(const cv::Point2f src[4], cv::Point2f dst[4])
{
    // TL has min x+y, BR has max x+y, TR has min y-x among remaining, etc.
    int idx[4] = {0, 1, 2, 3};
    std::sort(idx, idx + 4, [&](int i, int j) {
        return src[i].x + src[i].y < src[j].x + src[j].y;
    });
    dst[0] = src[idx[0]]; // TL
    dst[2] = src[idx[3]]; // BR
    cv::Point2f a = src[idx[1]];
    cv::Point2f b = src[idx[2]];
    if (a.x > b.x) {
        dst[1] = a; // TR
        dst[3] = b; // BL
    } else {
        dst[1] = b;
        dst[3] = a;
    }
}

cv::Rect largestAxisAlignedRect(const cv::Mat& bin)
{
    const int rows = bin.rows;
    const int cols = bin.cols;
    std::vector<int> height(cols, 0);
    int bestArea = 0;
    cv::Rect best(0, 0, 0, 0);

    for (int r = 0; r < rows; ++r) {
        const uchar* row = bin.ptr<uchar>(r);
        for (int c = 0; c < cols; ++c) {
            height[c] = row[c] ? height[c] + 1 : 0;
        }

        std::vector<int> stack;
        stack.reserve(static_cast<size_t>(cols) + 1);
        for (int i = 0; i <= cols; ++i) {
            const int h = (i == cols) ? 0 : height[i];
            while (!stack.empty() && height[stack.back()] > h) {
                const int hh = height[stack.back()];
                stack.pop_back();
                const int left = stack.empty() ? 0 : stack.back() + 1;
                const int width = i - left;
                const int area = hh * width;
                if (area > bestArea) {
                    bestArea = area;
                    best = cv::Rect(left, r - hh + 1, width, hh);
                }
            }
            stack.push_back(i);
        }
    }
    return best;
}

cv::Point2f lerp2(const cv::Point2f& a, const cv::Point2f& b, float t)
{
    return a + (b - a) * t;
}

cv::Point2f bilinear(const cv::Point2f& tl, const cv::Point2f& tr,
                     const cv::Point2f& br, const cv::Point2f& bl,
                     float u, float v)
{
    const cv::Point2f top = lerp2(tl, tr, u);
    const cv::Point2f bot = lerp2(bl, br, u);
    return lerp2(top, bot, v);
}

bool onWhite(const cv::Mat& mask, const cv::Point2f& p)
{
    const int x = static_cast<int>(std::lround(p.x));
    const int y = static_cast<int>(std::lround(p.y));
    if (x < 0 || y < 0 || x >= mask.cols || y >= mask.rows) {
        return false;
    }
    return mask.at<uchar>(y, x) != 0;
}

cv::Point2f clampToDisk(const cv::Point2f& center, const cv::Point2f& p, float radius)
{
    const cv::Point2f d = p - center;
    const float n = std::sqrt(d.x * d.x + d.y * d.y);
    if (n <= radius || n < 1e-6f) {
        return p;
    }
    return center + d * (radius / n);
}

struct CellGeom {
    cv::Point2f center;
    float radius; // allowed sample radius
    int row;
    int col;
};

std::vector<CellGeom> buildCells(const cv::Point2f& tl, const cv::Point2f& tr,
                                 const cv::Point2f& br, const cv::Point2f& bl,
                                 int rows, int cols, float innerCircleScale)
{
    const float width = static_cast<float>(cv::norm(tr - tl));
    const float height = static_cast<float>(cv::norm(bl - tl));
    const float cellW = width / static_cast<float>(cols);
    const float cellH = height / static_cast<float>(rows);
    const float inscribedR = 0.5f * std::min(cellW, cellH);
    const float sampleR = innerCircleScale * inscribedR;

    std::vector<CellGeom> cells;
    cells.reserve(static_cast<size_t>(rows * cols));
    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c) {
            const float u = (static_cast<float>(c) + 0.5f) / static_cast<float>(cols);
            const float v = (static_cast<float>(r) + 0.5f) / static_cast<float>(rows);
            CellGeom cell;
            cell.center = bilinear(tl, tr, br, bl, u, v);
            cell.radius = sampleR;
            cell.row = r;
            cell.col = c;
            cells.push_back(cell);
        }
    }
    return cells;
}

float neighborScore(const std::vector<cv::Point2f>& pts,
                    int rows, int cols, float target)
{
    float score = 0.f;
    auto idx = [cols](int r, int c) { return r * cols + c; };
    auto add = [&](int i, int j) {
        const float d = static_cast<float>(cv::norm(pts[static_cast<size_t>(i)] - pts[static_cast<size_t>(j)]));
        const float e = d - target;
        score += e * e;
    };
    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c) {
            if (c + 1 < cols) {
                add(idx(r, c), idx(r, c + 1));
            }
            if (r + 1 < rows) {
                add(idx(r, c), idx(r + 1, c));
            }
        }
    }
    return score;
}

cv::Point2f randomInDisk(std::mt19937& rng, const CellGeom& cell)
{
    std::uniform_real_distribution<float> u01(0.f, 1.f);
    const float ang = u01(rng) * 2.f * kPi;
    const float rad = cell.radius * std::sqrt(u01(rng));
    return cell.center + cv::Point2f(std::cos(ang) * rad, std::sin(ang) * rad);
}

bool sampleInDiskOnMask(std::mt19937& rng, const CellGeom& cell, const cv::Mat& mask,
                        cv::Point2f& out, int attempts = 48)
{
    for (int i = 0; i < attempts; ++i) {
        cv::Point2f p = randomInDisk(rng, cell);
        p = clampToDisk(cell.center, p, cell.radius);
        if (onWhite(mask, p)) {
            out = p;
            return true;
        }
    }
    if (onWhite(mask, cell.center)) {
        out = cell.center;
        return true;
    }
    return false;
}

std::vector<cv::Point2f> optimizeSamples(const std::vector<CellGeom>& cells,
                                         const cv::Mat& mask,
                                         int rows, int cols,
                                         float target,
                                         uint64_t seed)
{
    std::mt19937 rng(static_cast<uint32_t>(seed ^ (seed >> 32)));
    const int n = static_cast<int>(cells.size());
    std::vector<cv::Point2f> best(static_cast<size_t>(n));
    float bestScore = std::numeric_limits<float>::infinity();

    const int restarts = 24;
    const int sweeps = 18;
    const int candidates = 28;

    for (int restart = 0; restart < restarts; ++restart) {
        std::vector<cv::Point2f> pts(static_cast<size_t>(n));
        bool ok = true;
        for (int i = 0; i < n; ++i) {
            if (!sampleInDiskOnMask(rng, cells[static_cast<size_t>(i)], mask, pts[static_cast<size_t>(i)])) {
                ok = false;
                break;
            }
        }
        if (!ok) {
            continue;
        }

        float score = neighborScore(pts, rows, cols, target);
        for (int sweep = 0; sweep < sweeps; ++sweep) {
            for (int i = 0; i < n; ++i) {
                cv::Point2f current = pts[static_cast<size_t>(i)];
                float localBest = score;
                cv::Point2f localPt = current;
                for (int k = 0; k < candidates; ++k) {
                    cv::Point2f cand;
                    if (!sampleInDiskOnMask(rng, cells[static_cast<size_t>(i)], mask, cand, 8)) {
                        continue;
                    }
                    pts[static_cast<size_t>(i)] = cand;
                    const float s = neighborScore(pts, rows, cols, target);
                    if (s < localBest) {
                        localBest = s;
                        localPt = cand;
                    }
                }
                pts[static_cast<size_t>(i)] = localPt;
                score = localBest;
            }
        }

        if (score < bestScore) {
            bestScore = score;
            best = std::move(pts);
        }
    }

    return best;
}

cv::RotatedRect rectFromCorners(const cv::Point2f& tl, const cv::Point2f& tr,
                                const cv::Point2f& br, const cv::Point2f& bl)
{
    const cv::Point2f center = (tl + tr + br + bl) * 0.25f;
    const float width = static_cast<float>(cv::norm(tr - tl));
    const float height = static_cast<float>(cv::norm(bl - tl));
    const cv::Point2f vx = tr - tl;
    float angle = std::atan2(vx.y, vx.x) * 180.f / kPi;
    return cv::RotatedRect(center, cv::Size2f(width, height), angle);
}

cv::Mat toBgr(const cv::Mat& image)
{
    cv::Mat bgr;
    if (image.channels() == 4) {
        cv::cvtColor(image, bgr, cv::COLOR_BGRA2BGR);
    } else if (image.channels() == 1) {
        cv::cvtColor(image, bgr, cv::COLOR_GRAY2BGR);
    } else {
        bgr = image.clone();
    }
    return bgr;
}

} // namespace

bool sampleWhiteRegionGrid(const cv::Mat& image,
                           WhiteRegionGridSampleResult& result,
                           int gridRows,
                           int gridCols,
                           float targetNeighborDistance,
                           float innerCircleScale,
                           uint64_t seed)
{
    result = {};
    result.gridRows = gridRows;
    result.gridCols = gridCols;
    result.targetNeighborDistance = targetNeighborDistance;
    result.innerCircleScale = innerCircleScale;

    if (image.empty() || gridRows < 1 || gridCols < 1 || innerCircleScale <= 0.f) {
        return false;
    }

    const cv::Mat mask = makeWhiteMask(image);
    if (mask.empty()) {
        return false;
    }

    std::vector<cv::Point> contour;
    if (!largestContour(mask, contour)) {
        return false;
    }

    const cv::RotatedRect outer = cv::minAreaRect(contour);
    cv::Point2f raw[4];
    outer.points(raw);
    cv::Point2f quad[4];
    orderQuad(raw, quad);

    float outerW = static_cast<float>(cv::norm(quad[1] - quad[0]));
    float outerH = static_cast<float>(cv::norm(quad[3] - quad[0]));
    // Long side is columns (8), short side is rows (2).
    if (outerW < outerH) {
        const cv::Point2f tl = quad[3];
        const cv::Point2f tr = quad[0];
        const cv::Point2f br = quad[1];
        const cv::Point2f bl = quad[2];
        quad[0] = tl;
        quad[1] = tr;
        quad[2] = br;
        quad[3] = bl;
        std::swap(outerW, outerH);
    }

    const int warpW = std::max(8, static_cast<int>(std::lround(outerW)));
    const int warpH = std::max(8, static_cast<int>(std::lround(outerH)));
    const cv::Point2f srcPts[4] = {quad[0], quad[1], quad[2], quad[3]};
    const cv::Point2f dstPts[4] = {
        {0.f, 0.f},
        {static_cast<float>(warpW - 1), 0.f},
        {static_cast<float>(warpW - 1), static_cast<float>(warpH - 1)},
        {0.f, static_cast<float>(warpH - 1)}};
    const cv::Mat H = cv::getPerspectiveTransform(srcPts, dstPts);
    cv::Mat warped;
    cv::warpPerspective(mask, warped, H, cv::Size(warpW, warpH), cv::INTER_NEAREST);
    cv::threshold(warped, warped, 127, 255, cv::THRESH_BINARY);

    const cv::Rect inner = largestAxisAlignedRect(warped);
    if (inner.area() < 16) {
        return false;
    }

    const cv::Point2f innerPts[4] = {
        {static_cast<float>(inner.x), static_cast<float>(inner.y)},
        {static_cast<float>(inner.x + inner.width - 1), static_cast<float>(inner.y)},
        {static_cast<float>(inner.x + inner.width - 1), static_cast<float>(inner.y + inner.height - 1)},
        {static_cast<float>(inner.x), static_cast<float>(inner.y + inner.height - 1)}};

    cv::Mat Hinv = H.inv();
    cv::Point2f tl, tr, br, bl;
    std::vector<cv::Point2f> in(innerPts, innerPts + 4);
    std::vector<cv::Point2f> out(4);
    cv::perspectiveTransform(in, out, Hinv);
    tl = out[0];
    tr = out[1];
    br = out[2];
    bl = out[3];

    result.inscribedCorners = {tl, tr, br, bl};
    result.inscribedRect = rectFromCorners(tl, tr, br, bl);

    const std::vector<CellGeom> cells =
        buildCells(tl, tr, br, bl, gridRows, gridCols, innerCircleScale);
    const std::vector<cv::Point2f> pts =
        optimizeSamples(cells, mask, gridRows, gridCols, targetNeighborDistance, seed);
    if (pts.size() != cells.size()) {
        return false;
    }

    result.samples.reserve(cells.size());
    for (size_t i = 0; i < cells.size(); ++i) {
        WhiteRegionSample s;
        s.point = pts[i];
        s.cellCenter = cells[i].center;
        s.sampleRadius = cells[i].radius;
        s.row = cells[i].row;
        s.col = cells[i].col;
        result.samples.push_back(s);
    }
    return true;
}

cv::Mat visualizeWhiteRegionGridSample(const cv::Mat& image,
                                       const WhiteRegionGridSampleResult& result)
{
    cv::Mat canvas = toBgr(image);
    if (result.inscribedCorners.size() != 4) {
        return canvas;
    }

    const cv::Point2f tl = result.inscribedCorners[0];
    const cv::Point2f tr = result.inscribedCorners[1];
    const cv::Point2f br = result.inscribedCorners[2];
    const cv::Point2f bl = result.inscribedCorners[3];
    const int rows = result.gridRows;
    const int cols = result.gridCols;

    const cv::Point pts[4] = {
        tl, tr, br, bl};
    for (int i = 0; i < 4; ++i) {
        cv::line(canvas, pts[i], pts[(i + 1) % 4], cv::Scalar(0, 220, 0), 2, cv::LINE_AA);
    }

    for (int c = 1; c < cols; ++c) {
        const float u = static_cast<float>(c) / static_cast<float>(cols);
        cv::line(canvas, lerp2(tl, tr, u), lerp2(bl, br, u), cv::Scalar(255, 200, 0), 1, cv::LINE_AA);
    }
    for (int r = 1; r < rows; ++r) {
        const float v = static_cast<float>(r) / static_cast<float>(rows);
        cv::line(canvas, lerp2(tl, bl, v), lerp2(tr, br, v), cv::Scalar(255, 200, 0), 1, cv::LINE_AA);
    }

    for (const auto& s : result.samples) {
        cv::circle(canvas, s.cellCenter, std::max(1, static_cast<int>(std::lround(s.sampleRadius))),
                   cv::Scalar(0, 255, 255), 1, cv::LINE_AA);
        cv::circle(canvas, s.cellCenter, 3, cv::Scalar(180, 180, 180), -1, cv::LINE_AA);
        cv::circle(canvas, s.point, 6, cv::Scalar(0, 0, 255), -1, cv::LINE_AA);
        cv::circle(canvas, s.point, 6, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
    }

    auto indexOf = [&](int r, int c) -> const WhiteRegionSample* {
        for (const auto& s : result.samples) {
            if (s.row == r && s.col == c) {
                return &s;
            }
        }
        return nullptr;
    };

    auto drawPair = [&](const WhiteRegionSample* a, const WhiteRegionSample* b) {
        if (!a || !b) {
            return;
        }
        cv::line(canvas, a->point, b->point, cv::Scalar(255, 0, 255), 1, cv::LINE_AA);
        const cv::Point2f mid = (a->point + b->point) * 0.5f;
        const int d = static_cast<int>(std::lround(cv::norm(a->point - b->point)));
        const std::string label = std::to_string(d);
        const cv::Point org(static_cast<int>(mid.x) - 16, static_cast<int>(mid.y) - 4);
        cv::putText(canvas, label, org, cv::FONT_HERSHEY_SIMPLEX, 0.42, cv::Scalar(0, 0, 0), 2, cv::LINE_AA);
        cv::putText(canvas, label, org, cv::FONT_HERSHEY_SIMPLEX, 0.42, cv::Scalar(40, 40, 200), 1, cv::LINE_AA);
    };

    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c) {
            drawPair(indexOf(r, c), indexOf(r, c + 1));
            drawPair(indexOf(r, c), indexOf(r + 1, c));
        }
    }
    return canvas;
}
