#include "WhiteRegionSampler.h"

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <cmath>
#include <iomanip>
#include <iostream>
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
    if (!sampleWhiteRegionGrid(image, result, 300.f, 200.f)) {
        std::cerr << "sampleWhiteRegionGrid failed" << std::endl;
        return 1;
    }

    const cv::Size2f sz = result.inscribedRect.size;
    std::cout << std::fixed << std::setprecision(2);
    std::cout << "inscribed rect: " << sz.width << " x " << sz.height
              << " angle=" << result.inscribedRect.angle << " deg\n";
    std::cout << "grid: " << result.gridRows << " x " << result.gridCols
              << "  cell=" << result.cellWidth << " x " << result.cellHeight << "\n";
    std::cout << "min cell: width>=" << result.minCellWidth
              << " height>=" << result.minCellHeight << "\n";

    const bool widthOk = result.cellWidth + 1e-3f >= result.minCellWidth
                         || sz.width + 1e-3f < result.minCellWidth;
    const bool heightOk = result.cellHeight + 1e-3f >= result.minCellHeight
                          || sz.height + 1e-3f < result.minCellHeight;
    bool centersMatch = true;

    for (const auto& s : result.samples) {
        const float d = static_cast<float>(cv::norm(s.point - s.cellCenter));
        centersMatch = centersMatch && d < 1e-3f;
        std::cout << "  cell(" << s.row << "," << s.col << ")  center=("
                  << s.point.x << "," << s.point.y << ")\n";
    }

    std::cout << "cell width  >= 300 (or rect narrower): " << (widthOk ? "yes" : "NO") << "\n";
    std::cout << "cell height >= 200 (or rect shorter):  " << (heightOk ? "yes" : "NO") << "\n";
    std::cout << "samples are cell centers: " << (centersMatch ? "yes" : "NO") << "\n";

    const cv::Mat vis = visualizeWhiteRegionGridSample(image, result);
    if (!cv::imwrite(output, vis)) {
        std::cerr << "Failed to write " << output << std::endl;
        return 1;
    }
    std::cout << "wrote " << output << "\n";
    return (widthOk && heightOk && centersMatch && !result.samples.empty()) ? 0 : 2;
}
