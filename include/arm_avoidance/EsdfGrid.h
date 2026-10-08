#pragma once

#include "arm_avoidance/Types.h"

#include <cstdint>
#include <vector>

namespace arm_avoidance {

struct EsdfQuery {
    double distance = 0;  // 米。外部为正，障碍内部为负。
    Vec3 gradient;        // 距离增加方向的单位向量，平坦处为零。
    bool valid = false;   // 查询点落在栅格外时为 false。
};

// 由深度点云维护的局部欧氏符号距离场。
// 每个视觉周期：清空 -> 写入占据 -> 一次三维距离变换。
// 距离变换是可分离的精确欧氏距离变换，复杂度与体素数成正比。
class EsdfGrid {
public:
    EsdfGrid(Vec3 origin, double resolution, int nx, int ny, int nz);

    const Vec3& origin() const { return origin_; }
    double resolution() const { return resolution_; }
    int nx() const { return nx_; }
    int ny() const { return ny_; }
    int nz() const { return nz_; }
    double maxRange() const { return max_range_; }

    void clear();
    void insertPoint(const Vec3& p);
    void insertSphere(const Vec3& center, double radius);
    void updateDistance();

    bool contains(const Vec3& p) const;
    EsdfQuery query(const Vec3& p) const;

private:
    int index(int x, int y, int z) const { return x + nx_ * (y + ny_ * z); }
    double sample(const Vec3& p) const;
    void distanceTransform1D(const double* f, int n, double* out);
    void squaredDistanceTransform(std::vector<double>& field);

    Vec3 origin_;
    double resolution_ = 0.02;
    int nx_ = 0;
    int ny_ = 0;
    int nz_ = 0;
    double max_range_ = 1;
    std::vector<uint8_t> occupied_;
    std::vector<double> signed_distance_;
    std::vector<double> dist_to_occupied_;
    std::vector<double> dist_to_free_;
    std::vector<double> line_;
    std::vector<double> transformed_;
    std::vector<int> parabola_index_;
    std::vector<double> parabola_bound_;
};

}  // namespace arm_avoidance
