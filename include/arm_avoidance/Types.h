#pragma once

#include <algorithm>
#include <cmath>

namespace arm_avoidance {

struct Vec3 {
    double x = 0;
    double y = 0;
    double z = 0;

    Vec3() = default;
    Vec3(double x_in, double y_in, double z_in) : x(x_in), y(y_in), z(z_in) {}

    Vec3 operator+(const Vec3& o) const { return {x + o.x, y + o.y, z + o.z}; }
    Vec3 operator-(const Vec3& o) const { return {x - o.x, y - o.y, z - o.z}; }
    Vec3 operator*(double s) const { return {x * s, y * s, z * s}; }
    Vec3 operator/(double s) const { return {x / s, y / s, z / s}; }
    Vec3 operator-() const { return {-x, -y, -z}; }

    double dot(const Vec3& o) const { return x * o.x + y * o.y + z * o.z; }

    Vec3 cross(const Vec3& o) const {
        return {y * o.z - z * o.y, z * o.x - x * o.z, x * o.y - y * o.x};
    }

    double norm() const { return std::sqrt(dot(*this)); }

    Vec3 normalized() const {
        const double n = norm();
        if (n < 1e-12) {
            return {0, 0, 0};
        }
        return (*this) * (1.0 / n);
    }
};

inline Vec3 operator*(double s, const Vec3& v) { return v * s; }

struct Mat3 {
    double m[3][3]{};

    static Mat3 identity() {
        Mat3 r;
        r.m[0][0] = r.m[1][1] = r.m[2][2] = 1.0;
        return r;
    }

    Vec3 col(int c) const { return {m[0][c], m[1][c], m[2][c]}; }

    Mat3 transpose() const {
        Mat3 r;
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                r.m[i][j] = m[j][i];
            }
        }
        return r;
    }

    Vec3 mul(const Vec3& v) const {
        return {m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z,
                m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z,
                m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z};
    }

    Mat3 mul(const Mat3& o) const {
        Mat3 r;
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                double s = 0;
                for (int k = 0; k < 3; ++k) {
                    s += m[i][k] * o.m[k][j];
                }
                r.m[i][j] = s;
            }
        }
        return r;
    }
};

struct Mat4 {
    double m[4][4]{};

    static Mat4 identity() {
        Mat4 t;
        t.m[0][0] = t.m[1][1] = t.m[2][2] = t.m[3][3] = 1.0;
        return t;
    }

    Mat4 mul(const Mat4& b) const {
        Mat4 c;
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                double s = 0;
                for (int k = 0; k < 4; ++k) {
                    s += m[i][k] * b.m[k][j];
                }
                c.m[i][j] = s;
            }
        }
        return c;
    }

    Vec3 translation() const { return {m[0][3], m[1][3], m[2][3]}; }

    Mat3 rotation() const {
        Mat3 r;
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                r.m[i][j] = m[i][j];
            }
        }
        return r;
    }
};

// 刚体变换：p_base = rotation * p_child + translation
struct RigidTransform {
    Mat3 rotation = Mat3::identity();
    Vec3 translation;

    Vec3 apply(const Vec3& p) const { return rotation.mul(p) + translation; }
};

inline double clampDouble(double v, double lo, double hi) {
    return std::max(lo, std::min(hi, v));
}

}  // namespace arm_avoidance
