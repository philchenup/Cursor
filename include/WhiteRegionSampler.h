#ifndef WHITE_REGION_SAMPLER_H
#define WHITE_REGION_SAMPLER_H

#include <opencv2/core.hpp>
#include <vector>

/**
 * One cell of the inscribed-rectangle grid. The sample is the cell center.
 */
struct WhiteRegionSample {
    cv::Point2f point;       ///< Same as cellCenter
    cv::Point2f cellCenter;  ///< Center of this cell
    float cellWidth{0.f};
    float cellHeight{0.f};
    int row{0};
    int col{0};
};

/**
 * Largest inscribed rectangle plus an adaptive grid of cell centers.
 */
struct WhiteRegionGridSampleResult {
    cv::RotatedRect inscribedRect;
    std::vector<cv::Point2f> inscribedCorners; ///< TL, TR, BR, BL
    std::vector<WhiteRegionSample> samples;
    int gridRows{0};
    int gridCols{0};
    float cellWidth{0.f};
    float cellHeight{0.f};
    float minCellWidth{300.f};
    float minCellHeight{200.f};
};

/**
 * Find the largest rectangle inscribed in the white region and split it
 * adaptively:
 *  - columns = floor(width  / minCellWidth),  each cell width  >= minCellWidth
 *    when the rectangle is at least that wide (default 300 px)
 *  - rows    = floor(height / minCellHeight), each cell height >= minCellHeight
 *    when the rectangle is at least that tall (default 200 px)
 *  - if the rectangle is smaller than one minimum, that axis gets 1 cell
 *
 * Each sample is the geometric center of its cell. No random offset.
 *
 * @return true on success.
 */
bool sampleWhiteRegionGrid(const cv::Mat& image,
                           WhiteRegionGridSampleResult& result,
                           float minCellWidth = 300.f,
                           float minCellHeight = 200.f);

cv::Mat visualizeWhiteRegionGridSample(const cv::Mat& image,
                                       const WhiteRegionGridSampleResult& result);

#endif // WHITE_REGION_SAMPLER_H
