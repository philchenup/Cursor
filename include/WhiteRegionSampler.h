#ifndef WHITE_REGION_SAMPLER_H
#define WHITE_REGION_SAMPLER_H

#include <opencv2/core.hpp>
#include <cstdint>
#include <vector>

/**
 * One sample inside a 2x8 (or custom) cell of the white region's
 * largest inscribed rectangle.
 */
struct WhiteRegionSample {
    cv::Point2f point;       ///< Sampled pixel coordinate (image space)
    cv::Point2f cellCenter;  ///< Center of this cell
    float sampleRadius{0.f}; ///< Allowed radius: 0.7 * cell inscribed-circle radius
    int row{0};
    int col{0};
};

/**
 * Result of inscribed-rectangle extraction, grid split and sampling.
 */
struct WhiteRegionGridSampleResult {
    cv::RotatedRect inscribedRect;             ///< Largest rectangle inside the white blob
    std::vector<cv::Point2f> inscribedCorners; ///< TL, TR, BR, BL (along long / short axes)
    std::vector<WhiteRegionSample> samples;    ///< gridRows * gridCols points
    int gridRows{2};
    int gridCols{8};
    float minPairDistance{200.f};              ///< Required pairwise Euclidean distance
    float innerCircleScale{0.7f};
};

/**
 * Find the largest rectangle inscribed in the white region, split it into
 * a 2x8 grid (rows x cols along the rectangle's short x long axes), and
 * randomly sample one point per cell.
 *
 * Sampling constraints:
 *  - Each point is drawn uniformly in the *interior* of the disk centered
 *    at its cell, with radius `innerCircleScale * min(cellW, cellH) / 2`.
 *    Points are not forced onto the circle boundary.
 *  - Every pair of samples has Euclidean distance strictly greater than
 *    `minPairDistance` pixels (default 200).
 *  - Samples stay on white pixels of the mask.
 *
 * @param image  BGR / BGRA / gray image. White (high intensity) is the region.
 * @param result Output geometry and the 16 sample points.
 * @param gridRows Number of cells along the short axis (default 2).
 * @param gridCols Number of cells along the long axis (default 8).
 * @param minPairDistance Minimum allowed distance between any two samples.
 * @param innerCircleScale Fraction of the cell inscribed-circle radius.
 * @param seed RNG seed for reproducible sampling.
 * @return true on success.
 */
bool sampleWhiteRegionGrid(const cv::Mat& image,
                           WhiteRegionGridSampleResult& result,
                           int gridRows = 2,
                           int gridCols = 8,
                           float minPairDistance = 200.f,
                           float innerCircleScale = 0.7f,
                           uint64_t seed = 1);

/**
 * Draw the inscribed rectangle, 2x8 cells, sampling disks, points and
 * neighbor distances on a BGR copy of the input.
 */
cv::Mat visualizeWhiteRegionGridSample(const cv::Mat& image,
                                       const WhiteRegionGridSampleResult& result);

#endif // WHITE_REGION_SAMPLER_H
