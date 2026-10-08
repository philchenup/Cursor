#pragma once

#include "arm_avoidance/Types.h"

#include <vector>

namespace arm_avoidance {

struct CameraModel {
    double fx = 0;
    double fy = 0;
    double cx = 0;
    double cy = 0;
    int width = 0;
    int height = 0;
    // 相机光学坐标系到机械臂基座：x 右、y 下、z 前。
    RigidTransform base_from_camera;
};

// 深度为相机坐标系的 z（米）。0 或非正值视为无效。
std::vector<Vec3> backprojectDepth(const float* depth, int width, int height,
                                   const CameraModel& camera, float min_depth,
                                   float max_depth, int stride);

// 去掉落在机械臂包络球内的点，避免把本体当成障碍。
void removeRobotBody(std::vector<Vec3>& points, const std::vector<Vec3>& centers,
                     const std::vector<double>& radii, double margin);

// 把基座系中的球渲染成深度图，供仿真相机使用。未击中的像素为 0。
void renderSphereDepth(std::vector<float>& depth, const CameraModel& camera,
                       const Vec3& center_base, double radius);

}  // namespace arm_avoidance
