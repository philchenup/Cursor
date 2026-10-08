#include "arm_avoidance/EsdfGrid.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace arm_avoidance {
namespace {

constexpr double kOccupancyInf = 1.0e6;

}

EsdfGrid::EsdfGrid(Vec3 origin, double resolution, int nx, int ny, int nz)
    : origin_(origin),
      resolution_(resolution),
      nx_(std::max(1, nx)),
      ny_(std::max(1, ny)),
      nz_(std::max(1, nz)) {
    const double diag = std::sqrt(static_cast<double>(nx_ * nx_ + ny_ * ny_ + nz_ * nz_));
    max_range_ = resolution_ * diag;
    const int count = nx_ * ny_ * nz_;
    occupied_.assign(static_cast<size_t>(count), 0);
    signed_distance_.assign(static_cast<size_t>(count), max_range_);
    const int longest = std::max(nx_, std::max(ny_, nz_));
    line_.assign(static_cast<size_t>(longest), 0);
    transformed_.assign(static_cast<size_t>(longest), 0);
    parabola_index_.assign(static_cast<size_t>(longest), 0);
    parabola_bound_.assign(static_cast<size_t>(longest) + 1, 0);
    dist_to_occupied_.assign(static_cast<size_t>(count), 0);
    dist_to_free_.assign(static_cast<size_t>(count), 0);
}

void EsdfGrid::clear() {
    std::fill(occupied_.begin(), occupied_.end(), 0);
    std::fill(signed_distance_.begin(), signed_distance_.end(), max_range_);
}

bool EsdfGrid::contains(const Vec3& p) const {
    return p.x >= origin_.x && p.y >= origin_.y && p.z >= origin_.z &&
           p.x < origin_.x + nx_ * resolution_ && p.y < origin_.y + ny_ * resolution_ &&
           p.z < origin_.z + nz_ * resolution_;
}

void EsdfGrid::insertPoint(const Vec3& p) {
    if (!contains(p)) {
        return;
    }
    const int ix = static_cast<int>(std::floor((p.x - origin_.x) / resolution_));
    const int iy = static_cast<int>(std::floor((p.y - origin_.y) / resolution_));
    const int iz = static_cast<int>(std::floor((p.z - origin_.z) / resolution_));
    if (ix < 0 || iy < 0 || iz < 0 || ix >= nx_ || iy >= ny_ || iz >= nz_) {
        return;
    }
    occupied_[static_cast<size_t>(index(ix, iy, iz))] = 1;
}

void EsdfGrid::insertSphere(const Vec3& center, double radius) {
    if (radius <= 0) {
        return;
    }
    const int x0 = static_cast<int>(std::floor((center.x - radius - origin_.x) / resolution_));
    const int y0 = static_cast<int>(std::floor((center.y - radius - origin_.y) / resolution_));
    const int z0 = static_cast<int>(std::floor((center.z - radius - origin_.z) / resolution_));
    const int x1 = static_cast<int>(std::floor((center.x + radius - origin_.x) / resolution_));
    const int y1 = static_cast<int>(std::floor((center.y + radius - origin_.y) / resolution_));
    const int z1 = static_cast<int>(std::floor((center.z + radius - origin_.z) / resolution_));
    const double radius_sq = radius * radius;
    for (int z = std::max(0, z0); z <= std::min(nz_ - 1, z1); ++z) {
        for (int y = std::max(0, y0); y <= std::min(ny_ - 1, y1); ++y) {
            for (int x = std::max(0, x0); x <= std::min(nx_ - 1, x1); ++x) {
                const Vec3 voxel{origin_.x + (static_cast<double>(x) + 0.5) * resolution_,
                                 origin_.y + (static_cast<double>(y) + 0.5) * resolution_,
                                 origin_.z + (static_cast<double>(z) + 0.5) * resolution_};
                const Vec3 delta = voxel - center;
                if (delta.dot(delta) <= radius_sq) {
                    occupied_[static_cast<size_t>(index(x, y, z))] = 1;
                }
            }
        }
    }
}

void EsdfGrid::distanceTransform1D(const double* f, int n, double* out) {
    int k = 0;
    parabola_index_[0] = 0;
    parabola_bound_[0] = -std::numeric_limits<double>::infinity();
    parabola_bound_[1] = std::numeric_limits<double>::infinity();
    for (int q = 1; q < n; ++q) {
        auto intersect = [&](int vk) {
            const double denom = static_cast<double>(2 * q - 2 * vk);
            return ((f[q] + static_cast<double>(q) * q) -
                    (f[vk] + static_cast<double>(vk) * vk)) /
                   denom;
        };
        double s = intersect(parabola_index_[static_cast<size_t>(k)]);
        while (k > 0 && s <= parabola_bound_[static_cast<size_t>(k)]) {
            --k;
            s = intersect(parabola_index_[static_cast<size_t>(k)]);
        }
        ++k;
        parabola_index_[static_cast<size_t>(k)] = q;
        parabola_bound_[static_cast<size_t>(k)] = s;
        parabola_bound_[static_cast<size_t>(k) + 1] = std::numeric_limits<double>::infinity();
    }
    k = 0;
    for (int q = 0; q < n; ++q) {
        while (parabola_bound_[static_cast<size_t>(k) + 1] < q) {
            ++k;
        }
        const int p = parabola_index_[static_cast<size_t>(k)];
        const double delta = static_cast<double>(q - p);
        out[q] = delta * delta + f[p];
    }
}

void EsdfGrid::squaredDistanceTransform(std::vector<double>& field) {
    for (int z = 0; z < nz_; ++z) {
        for (int y = 0; y < ny_; ++y) {
            for (int x = 0; x < nx_; ++x) {
                line_[static_cast<size_t>(x)] = field[static_cast<size_t>(index(x, y, z))];
            }
            distanceTransform1D(line_.data(), nx_, transformed_.data());
            for (int x = 0; x < nx_; ++x) {
                field[static_cast<size_t>(index(x, y, z))] = transformed_[static_cast<size_t>(x)];
            }
        }
    }
    for (int z = 0; z < nz_; ++z) {
        for (int x = 0; x < nx_; ++x) {
            for (int y = 0; y < ny_; ++y) {
                line_[static_cast<size_t>(y)] = field[static_cast<size_t>(index(x, y, z))];
            }
            distanceTransform1D(line_.data(), ny_, transformed_.data());
            for (int y = 0; y < ny_; ++y) {
                field[static_cast<size_t>(index(x, y, z))] = transformed_[static_cast<size_t>(y)];
            }
        }
    }
    for (int y = 0; y < ny_; ++y) {
        for (int x = 0; x < nx_; ++x) {
            for (int z = 0; z < nz_; ++z) {
                line_[static_cast<size_t>(z)] = field[static_cast<size_t>(index(x, y, z))];
            }
            distanceTransform1D(line_.data(), nz_, transformed_.data());
            for (int z = 0; z < nz_; ++z) {
                field[static_cast<size_t>(index(x, y, z))] = transformed_[static_cast<size_t>(z)];
            }
        }
    }
}

void EsdfGrid::updateDistance() {
    const int count = nx_ * ny_ * nz_;
    bool any_occupied = false;
    bool any_free = false;
    for (int i = 0; i < count; ++i) {
        if (occupied_[static_cast<size_t>(i)]) {
            dist_to_occupied_[static_cast<size_t>(i)] = 0;
            dist_to_free_[static_cast<size_t>(i)] = kOccupancyInf;
            any_occupied = true;
        } else {
            dist_to_occupied_[static_cast<size_t>(i)] = kOccupancyInf;
            dist_to_free_[static_cast<size_t>(i)] = 0;
            any_free = true;
        }
    }
    if (!any_occupied) {
        std::fill(signed_distance_.begin(), signed_distance_.end(), max_range_);
        return;
    }
    if (!any_free) {
        std::fill(signed_distance_.begin(), signed_distance_.end(), -max_range_);
        return;
    }
    squaredDistanceTransform(dist_to_occupied_);
    squaredDistanceTransform(dist_to_free_);
    for (int i = 0; i < count; ++i) {
        double meters = 0;
        if (occupied_[static_cast<size_t>(i)]) {
            meters = -std::sqrt(std::max(0.0, dist_to_free_[static_cast<size_t>(i)])) * resolution_;
        } else {
            meters = std::sqrt(std::max(0.0, dist_to_occupied_[static_cast<size_t>(i)])) * resolution_;
        }
        signed_distance_[static_cast<size_t>(i)] = clampDouble(meters, -max_range_, max_range_);
    }
}

double EsdfGrid::sample(const Vec3& p) const {
    const double u = (p.x - origin_.x) / resolution_ - 0.5;
    const double v = (p.y - origin_.y) / resolution_ - 0.5;
    const double w = (p.z - origin_.z) / resolution_ - 0.5;
    const int x0 = static_cast<int>(std::floor(u));
    const int y0 = static_cast<int>(std::floor(v));
    const int z0 = static_cast<int>(std::floor(w));
    const double tx = u - x0;
    const double ty = v - y0;
    const double tz = w - z0;
    auto at = [&](int x, int y, int z) {
        x = std::max(0, std::min(nx_ - 1, x));
        y = std::max(0, std::min(ny_ - 1, y));
        z = std::max(0, std::min(nz_ - 1, z));
        return signed_distance_[static_cast<size_t>(index(x, y, z))];
    };
    const double c000 = at(x0, y0, z0);
    const double c100 = at(x0 + 1, y0, z0);
    const double c010 = at(x0, y0 + 1, z0);
    const double c110 = at(x0 + 1, y0 + 1, z0);
    const double c001 = at(x0, y0, z0 + 1);
    const double c101 = at(x0 + 1, y0, z0 + 1);
    const double c011 = at(x0, y0 + 1, z0 + 1);
    const double c111 = at(x0 + 1, y0 + 1, z0 + 1);
    const double c00 = c000 * (1 - tx) + c100 * tx;
    const double c10 = c010 * (1 - tx) + c110 * tx;
    const double c01 = c001 * (1 - tx) + c101 * tx;
    const double c11 = c011 * (1 - tx) + c111 * tx;
    const double c0 = c00 * (1 - ty) + c10 * ty;
    const double c1 = c01 * (1 - ty) + c11 * ty;
    return c0 * (1 - tz) + c1 * tz;
}

EsdfQuery EsdfGrid::query(const Vec3& p) const {
    EsdfQuery result;
    if (!contains(p)) {
        result.distance = max_range_;
        result.valid = false;
        return result;
    }
    result.valid = true;
    result.distance = sample(p);
    const double h = resolution_;
    auto difference = [&](const Vec3& axis) {
        const Vec3 forward = p + axis * h;
        const Vec3 backward = p - axis * h;
        const bool has_forward = contains(forward);
        const bool has_backward = contains(backward);
        if (has_forward && has_backward) {
            return (sample(forward) - sample(backward)) / (2 * h);
        }
        if (has_forward) {
            return (sample(forward) - result.distance) / h;
        }
        if (has_backward) {
            return (result.distance - sample(backward)) / h;
        }
        return 0.0;
    };
    result.gradient = {difference({1, 0, 0}), difference({0, 1, 0}), difference({0, 0, 1})};
    const double gradient_norm = result.gradient.norm();
    if (gradient_norm < 1e-6) {
        result.gradient = {0, 0, 0};
    } else {
        result.gradient = result.gradient * (1.0 / gradient_norm);
    }
    return result;
}

}  // namespace arm_avoidance
