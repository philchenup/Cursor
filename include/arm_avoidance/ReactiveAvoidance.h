#pragma once

#include "arm_avoidance/DepthVision.h"
#include "arm_avoidance/EsdfGrid.h"
#include "arm_avoidance/SixAxisArm.h"

#include <array>

namespace arm_avoidance {

struct AvoidanceParams {
    double dt = 0.02;                  // 控制周期，秒
    double attract_gain = 4.0;         // 趋近目标的速度增益，1/s
    double max_tcp_speed = 0.18;       // 末端最大线速度，m/s
    double max_joint_speed = 1.4;      // 关节速度限幅，rad/s
    double influence_distance = 0.06;  // 开始制动的间隙，米
    double stop_distance = 0.012;      // 允许的最小间隙，米
    double brake_horizon = 0.15;       // 把剩余间隙刹停的时间，秒
    double damping = 0.05;             // 阻尼最小二乘的 λ
    double joint_limit_margin = 0.05;  // 距限位多近时开始推回，rad
    double joint_limit_gain = 4.0;
    double goal_tolerance = 0.015;     // 认为到达的位置误差，米
    int damper_iterations = 6;
    int safety_bisections = 10;
};

struct AvoidanceState {
    std::array<double, 6> q{};
    Vec3 goal;
};

struct AvoidanceResult {
    std::array<double, 6> q_command{};
    std::array<double, 6> q_dot{};
    Vec3 tcp;
    double min_clearance = 0;  // 各控制球的 ESDF 间隙最小值
    double goal_distance = 0;
    bool goal_reached = false;
    int active_constraints = 0;
    double step_scale = 1;  // 前瞻把步长缩小的比例
};

// 每个控制周期：
// 1. 末端朝目标生成吸引速度，并按距离场梯度限制靠近障碍的分量；
// 2. 阻尼最小二乘映射到关节速度；
// 3. 对每个连杆控制球做速度阻尼，保证间隙变化率不低于制动极限；
// 4. 关节限位排斥；
// 5. 向前看一步，若积分后会突破最小间隙，则二分缩小步长。
class ReactiveAvoidance {
public:
    explicit ReactiveAvoidance(AvoidanceParams params = {});

    const AvoidanceParams& params() const { return params_; }

    AvoidanceResult step(const SixAxisArm& arm, const EsdfGrid& esdf, AvoidanceState& state) const;

    // 当前构型下，距离场给出的最小球体间隙。栅格外的点不参与。
    double minClearance(const SixAxisArm& arm, const EsdfGrid& esdf,
                        const std::array<double, 6>& q) const;

private:
    AvoidanceParams params_;
};

// 视觉更新与控制步进分开：距离场按相机帧率刷新，关节伺服可以更高频率查询同一场。
class VisionAvoidancePipeline {
public:
    VisionAvoidancePipeline(SixAxisArm arm, EsdfGrid grid, AvoidanceParams params = {});

    void setCamera(CameraModel camera);
    void setDepthRange(double min_depth, double max_depth);
    void setPixelStride(int stride);
    void setSelfFilterMargin(double margin);

    void updateFromDepth(const float* depth, int width, int height,
                         const std::array<double, 6>& q);
    void updateFromPoints(const std::vector<Vec3>& obstacle_points_base);

    AvoidanceResult step(AvoidanceState& state);

    const EsdfGrid& esdf() const { return esdf_; }
    const SixAxisArm& arm() const { return arm_; }
    EsdfGrid& esdf() { return esdf_; }

private:
    SixAxisArm arm_;
    EsdfGrid esdf_;
    ReactiveAvoidance avoidance_;
    CameraModel camera_{};
    bool camera_set_ = false;
    double min_depth_ = 0.05;
    double max_depth_ = 3.0;
    int stride_ = 1;
    double self_filter_margin_ = 0.015;
};

}  // namespace arm_avoidance
