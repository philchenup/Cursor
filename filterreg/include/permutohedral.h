#pragma once
// Permutohedral lattice high-dimensional Gaussian filter.
// From Philipp Krähenbühl (BSD), as used by neka-nat/probreg.
#pragma once

#include <Eigen/Core>
#include <vector>

class Permutohedral {
public:
    Permutohedral() = default;

    // features: (dim, N), each column is one feature.
    void init(const Eigen::MatrixXf& features, bool with_blur = true);
    int latticeSize() const { return M_; }

    // values: (value_size, N). Splats columns [start, N).
    Eigen::MatrixXf compute(const Eigen::MatrixXf& values, int start = 0) const;

private:
    struct Neighbors {
        int n1 = 0;
        int n2 = 0;
    };

    void splatAndSlice(float* out, const float* in, int value_size, int start) const;

    std::vector<int> offset_;
    std::vector<int> rank_;
    std::vector<float> barycentric_;
    std::vector<Neighbors> blur_neighbors_;
    int N_ = 0;
    int M_ = 0;
    int d_ = 0;
    bool with_blur_ = true;
};