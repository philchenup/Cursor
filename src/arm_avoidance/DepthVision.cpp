#include "arm_avoidance/DepthVision.h"

#include <algorithm>
#include <cmath>

namespace arm_avoidance {

std::vector<Vec3> backprojectDepth(const float* depth, int width, int height,
                                   const CameraModel& camera, float min_depth, float max_depth,
                                   int stride) {
    std::vector<Vec3> points;
    if (depth == nullptr || width <= 0 || height <= 0 || camera.fx == 0 || camera.fy == 0) {
        return points;
    }
    const int step = std::max(1, stride);
    points.reserve(static_cast<size_t>((width / step) * (height / step) / 4 + 1));
    for (int v = 0; v < height; v += step) {
        for (int u = 0; u < width; u += step) {
            const float z = depth[static_cast<size_t>(v * width + u)];
            if (!(z > min_depth && z < max_depth)) {
                continue;
            }
            const Vec3 point_camera{(static_cast<double>(u) - camera.cx) * z / camera.fx,
                                    (static_cast<double>(v) - camera.cy) * z / camera.fy,
                                    static_cast<double>(z)};
            points.push_back(camera.base_from_camera.apply(point_camera));
        }
    }
    return points;
}

void removeRobotBody(std::vector<Vec3>& points, const std::vector<Vec3>& centers,
                     const std::vector<double>& radii, double margin) {
    if (centers.empty() || centers.size() != radii.size()) {
        return;
    }
    size_t write = 0;
    for (size_t i = 0; i < points.size(); ++i) {
        bool on_body = false;
        for (size_t s = 0; s < centers.size(); ++s) {
            const double limit = radii[s] + margin;
            const Vec3 delta = points[i] - centers[s];
            if (delta.dot(delta) <= limit * limit) {
                on_body = true;
                break;
            }
        }
        if (!on_body) {
            points[write++] = points[i];
        }
    }
    points.resize(write);
}

void renderSphereDepth(std::vector<float>& depth, const CameraModel& camera,
                       const Vec3& center_base, double radius) {
    const int width = camera.width;
    const int height = camera.height;
    depth.assign(static_cast<size_t>(std::max(0, width) * std::max(0, height)), 0.f);
    if (width <= 0 || height <= 0 || camera.fx == 0 || camera.fy == 0 || radius <= 0) {
        return;
    }
    const Vec3 origin = camera.base_from_camera.translation;
    const Mat3& rotation = camera.base_from_camera.rotation;
    for (int v = 0; v < height; ++v) {
        for (int u = 0; u < width; ++u) {
            const Vec3 dir_camera{(static_cast<double>(u) - camera.cx) / camera.fx,
                                  (static_cast<double>(v) - camera.cy) / camera.fy, 1.0};
            const Vec3 dir_base = rotation.mul(dir_camera);
            const Vec3 oc = origin - center_base;
            const double a = dir_base.dot(dir_base);
            const double b = 2.0 * oc.dot(dir_base);
            const double c = oc.dot(oc) - radius * radius;
            const double discriminant = b * b - 4.0 * a * c;
            if (a <= 1e-12 || discriminant < 0) {
                continue;
            }
            const double root = std::sqrt(discriminant);
            const double t_near = (-b - root) / (2.0 * a);
            const double t_far = (-b + root) / (2.0 * a);
            const double t = t_near > 1e-4 ? t_near : t_far;
            if (t > 1e-4) {
                depth[static_cast<size_t>(v * width + u)] = static_cast<float>(t);
            }
        }
    }
}

}  // namespace arm_avoidance
