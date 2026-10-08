#pragma once

#include "arm_avoidance/Types.h"

#include <cmath>
#include <utility>

namespace arm_avoidance {

// 列主元高斯消元，A 为行主序 n*n，调用后内容会被覆盖。
inline bool solveLinear(int n, double* A, const double* b, double* x) {
    double rhs[6];
    for (int i = 0; i < n; ++i) {
        rhs[i] = b[i];
    }

    for (int col = 0; col < n; ++col) {
        int pivot = col;
        double best = std::abs(A[col * n + col]);
        for (int row = col + 1; row < n; ++row) {
            const double value = std::abs(A[row * n + col]);
            if (value > best) {
                best = value;
                pivot = row;
            }
        }
        if (best < 1e-12) {
            return false;
        }
        if (pivot != col) {
            for (int k = col; k < n; ++k) {
                std::swap(A[col * n + k], A[pivot * n + k]);
            }
            std::swap(rhs[col], rhs[pivot]);
        }
        const double diag = A[col * n + col];
        for (int row = col + 1; row < n; ++row) {
            const double factor = A[row * n + col] / diag;
            A[row * n + col] = 0;
            for (int k = col + 1; k < n; ++k) {
                A[row * n + k] -= factor * A[col * n + k];
            }
            rhs[row] -= factor * rhs[col];
        }
    }

    for (int row = n - 1; row >= 0; --row) {
        double sum = rhs[row];
        for (int k = row + 1; k < n; ++k) {
            sum -= A[row * n + k] * x[k];
        }
        x[row] = sum / A[row * n + row];
    }
    return true;
}

// 阻尼最小二乘：q_dot = J^T (J J^T + λ^2 I)^{-1} v，J 为 3x6。
inline void dampedLeastSquares(const double J[3][6], const Vec3& velocity, double lambda,
                               double q_dot[6]) {
    double A[9];
    const double b[3] = {velocity.x, velocity.y, velocity.z};
    double y[3] = {0, 0, 0};
    const double damping = lambda * lambda;
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            double sum = (r == c) ? damping : 0.0;
            for (int k = 0; k < 6; ++k) {
                sum += J[r][k] * J[c][k];
            }
            A[r * 3 + c] = sum;
        }
    }
    if (!solveLinear(3, A, b, y)) {
        for (int k = 0; k < 6; ++k) {
            q_dot[k] = 0;
        }
        return;
    }
    for (int k = 0; k < 6; ++k) {
        double sum = 0;
        for (int r = 0; r < 3; ++r) {
            sum += J[r][k] * y[r];
        }
        q_dot[k] = sum;
    }
}

}  // namespace arm_avoidance
