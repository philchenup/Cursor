#include "WhiteRegionSampler.h"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <string>

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
    if (cv::countNonZero(mask) * 2 >= mask.rows * mask.cols) {
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
    int idx[4] = {0, 1, 2, 3};
    std::sort(idx, idx + 4, [&](int i, int j) {
        return src[i].x + src[i].y < src[j].x + src[j].y;
    });
    dst[0] = src[idx[0]];
    dst[2] = src[idx[3]];
    cv::Point2f a = src[idx[1]];
    cv::Point2f b = src[idx[2]];
    if (a.x > b.x) {
        dst[1] = a;
        dst[3] = b;
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

int countAlongAxis(float length, float minSize)
{
    if (length <= 0.f || minSize <= 0.f) {
        return 1;
    }
    return std::max(1, static_cast<int>(std::floor(length / minSize)));
}

cv::RotatedRect rectFromCorners(const cv::Point2f& tl, const cv::Point2f& tr,
                                const cv::Point2f& br, const cv::Point2f& bl)
{
    const cv::Point2f center = (tl + tr + br + bl) * 0.25f;
    const float width = static_cast<float>(cv::norm(tr - tl));
    const float height = static_cast<float>(cv::norm(bl - tl));
    const cv::Point2f vx = tr - tl;
    const float angle = std::atan2(vx.y, vx.x) * 180.f / kPi;
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
                           float minCellWidth,
                           float minCellHeight)
{
    result = {};
    result.minCellWidth = minCellWidth;
    result.minCellHeight = minCellHeight;

    if (image.empty() || minCellWidth <= 0.f || minCellHeight <= 0.f) {
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

    std::vector<cv::Point2f> in(innerPts, innerPts + 4);
    std::vector<cv::Point2f> out(4);
    cv::perspectiveTransform(in, out, H.inv());
    const cv::Point2f tl = out[0];
    const cv::Point2f tr = out[1];
    const cv::Point2f br = out[2];
    const cv::Point2f bl = out[3];

    result.inscribedCorners = {tl, tr, br, bl};
    result.inscribedRect = rectFromCorners(tl, tr, br, bl);

    const float width = static_cast<float>(cv::norm(tr - tl));
    const float height = static_cast<float>(cv::norm(bl - tl));
    const int cols = countAlongAxis(width, minCellWidth);
    const int rows = countAlongAxis(height, minCellHeight);
    const float cellW = width / static_cast<float>(cols);
    const float cellH = height / static_cast<float>(rows);

    result.gridRows = rows;
    result.gridCols = cols;
    result.cellWidth = cellW;
    result.cellHeight = cellH;
    result.samples.reserve(static_cast<size_t>(rows * cols));

    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c) {
            const float u = (static_cast<float>(c) + 0.5f) / static_cast<float>(cols);
            const float v = (static_cast<float>(r) + 0.5f) / static_cast<float>(rows);
            WhiteRegionSample s;
            s.cellCenter = bilinear(tl, tr, br, bl, u, v);
            s.point = s.cellCenter;
            s.cellWidth = cellW;
            s.cellHeight = cellH;
            s.row = r;
            s.col = c;
            result.samples.push_back(s);
        }
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

    const cv::Point pts[4] = {tl, tr, br, bl};
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
        cv::circle(canvas, s.point, 6, cv::Scalar(0, 0, 255), -1, cv::LINE_AA);
        cv::circle(canvas, s.point, 6, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
        const std::string tag = std::to_string(s.row) + "," + std::to_string(s.col);
        cv::putText(canvas, tag,
                    cv::Point(static_cast<int>(s.point.x) + 8, static_cast<int>(s.point.y) - 8),
                    cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 0, 0), 2, cv::LINE_AA);
        cv::putText(canvas, tag,
                    cv::Point(static_cast<int>(s.point.x) + 8, static_cast<int>(s.point.y) - 8),
                    cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(40, 40, 200), 1, cv::LINE_AA);
    }

    const std::string info =
        std::to_string(rows) + "x" + std::to_string(cols) +
        "  cell " + std::to_string(static_cast<int>(std::lround(result.cellWidth))) +
        "x" + std::to_string(static_cast<int>(std::lround(result.cellHeight)));
    cv::putText(canvas, info, cv::Point(16, 28),
                cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 0), 3, cv::LINE_AA);
    cv::putText(canvas, info, cv::Point(16, 28),
                cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 255), 1, cv::LINE_AA);
    return canvas;
}
