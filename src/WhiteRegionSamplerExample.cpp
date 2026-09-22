#include "WhiteRegionSampler.h"

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <cmath>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

int main(int argc, char** argv)
{
    const std::string input = argc > 1 ? argv[1] : "data/white_region.png";
    const std::string output = argc > 2 ? argv[2] : "white_region_samples.png";

    const cv::Mat image = cv::imread(input, cv::IMREAD_UNCHANGED);
    if (image.empty()) {
        std::cerr << "Failed to read image: " << input << std::endl;
        return 1;
    }

    WhiteRegionGridSampleResult result;
    if (!sampleWhiteRegionGrid(image, result, 2, 8, 250.f, 0.7f, 1)) {
        std::cerr << "sampleWhiteRegionGrid failed" << std::endl;
        return 1;
    }

    const cv::Size2f sz = result.inscribedRect.size;
    std::cout << "inscribed rect: " << sz.width << " x " << sz.height
              << " angle=" << result.inscribedRect.angle << " deg\n";
    std::cout << "grid: " << result.gridRows << " x " << result.gridCols
              << "  target neighbor distance=" << result.targetNeighborDistance << " px\n";

    const float cellW = sz.width / static_cast<float>(result.gridCols);
    const float cellH = sz.height / static_cast<float>(result.gridRows);
    const float inscribedR = 0.5f * std::min(cellW, cellH);
    std::cout << "cell: " << cellW << " x " << cellH
              << "  inscribed-circle r=" << inscribedR
              << "  sample r=" << (0.7f * inscribedR) << "\n";

    bool allInDisk = true;
    bool allOnWhite = true;
    cv::Mat gray;
    if (image.channels() == 4) {
        cv::cvtColor(image, gray, cv::COLOR_BGRA2GRAY);
    } else if (image.channels() == 3) {
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    } else {
        gray = image;
    }
    cv::Mat mask;
    cv::threshold(gray, mask, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
    if (cv::countNonZero(mask) * 2 >= mask.rows * mask.cols) {
        cv::bitwise_not(mask, mask);
    }

    std::cout << std::fixed << std::setprecision(2);
    for (const auto& s : result.samples) {
        const float d = static_cast<float>(cv::norm(s.point - s.cellCenter));
        const bool inDisk = d <= s.sampleRadius + 0.75f;
        const int x = static_cast<int>(std::lround(s.point.x));
        const int y = static_cast<int>(std::lround(s.point.y));
        const bool white = x >= 0 && y >= 0 && x < mask.cols && y < mask.rows && mask.at<uchar>(y, x) != 0;
        allInDisk = allInDisk && inDisk;
        allOnWhite = allOnWhite && white;
        std::cout << "  cell(" << s.row << "," << s.col << ")  pt=(" << s.point.x << "," << s.point.y
                  << ")  center_dist=" << d << "/" << s.sampleRadius
                  << (inDisk ? "  OK" : "  OUT") << (white ? "  white" : "  BLACK") << "\n";
    }

    auto find = [&](int r, int c) -> const WhiteRegionSample* {
        for (const auto& s : result.samples) {
            if (s.row == r && s.col == c) return &s;
        }
        return nullptr;
    };

    float sumAbs = 0.f;
    int nPairs = 0;
    float minD = 1e9f;
    float maxD = 0.f;
    std::cout << "neighbor distances (target " << result.targetNeighborDistance << "):\n";
    for (int r = 0; r < result.gridRows; ++r) {
        for (int c = 0; c < result.gridCols; ++c) {
            const WhiteRegionSample* a = find(r, c);
            const WhiteRegionSample* right = find(r, c + 1);
            const WhiteRegionSample* down = find(r + 1, c);
            auto report = [&](const WhiteRegionSample* b, const char* tag) {
                if (!a || !b) return;
                const float d = static_cast<float>(cv::norm(a->point - b->point));
                sumAbs += std::abs(d - result.targetNeighborDistance);
                minD = std::min(minD, d);
                maxD = std::max(maxD, d);
                ++nPairs;
                std::cout << "  " << tag << " (" << a->row << "," << a->col << ")-("
                          << b->row << "," << b->col << ") = " << d << "\n";
            };
            report(right, "H");
            report(down, "V");
        }
    }
    std::cout << "pairs=" << nPairs
              << "  mean |d-250|=" << (nPairs ? sumAbs / nPairs : 0)
              << "  min=" << minD << "  max=" << maxD << "\n";
    std::cout << "all in 0.7-disk: " << (allInDisk ? "yes" : "NO") << "\n";
    std::cout << "all on white: " << (allOnWhite ? "yes" : "NO") << "\n";

    const cv::Mat vis = visualizeWhiteRegionGridSample(image, result);
    if (!cv::imwrite(output, vis)) {
        std::cerr << "Failed to write " << output << std::endl;
        return 1;
    }
    std::cout << "wrote " << output << "\n";
    return (allInDisk && allOnWhite && result.samples.size() == 16) ? 0 : 2;
}
