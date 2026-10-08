#include "arm_avoidance/ReactiveAvoidance.h"

#include <iostream>
#include <vector>

namespace {

double geometricGap(const arm_avoidance::SixAxisArm& arm, const std::array<double, 6>& q,
                    const arm_avoidance::Vec3& obstacle, double obstacle_radius) {
    const arm_avoidance::ChainState chain = arm.forward(q);
    double gap = 1e9;
    for (const arm_avoidance::ControlPoint& point : arm.controlPoints(chain)) {
        const arm_avoidance::Vec3 center = arm.worldPoint(chain, point.link, point.local);
        gap = std::min(gap, (center - obstacle).norm() - obstacle_radius - point.radius);
    }
    return gap;
}

}  // namespace

// 仿真：深度相机看见球障碍，六轴臂边伺服边绕开它去目标。
int main() {
    using namespace arm_avoidance;
    SixAxisArm arm = SixAxisArm::DefaultIndustrial();
    EsdfGrid grid({-0.05, -0.55, -0.25}, 0.012, 80, 80, 60);
    AvoidanceParams params;
    VisionAvoidancePipeline pipeline(arm, grid, params);

    AvoidanceState state;
    state.q = {0.4, -0.8, 1.2, -0.9, 1.1, 0.3};
    const Vec3 tcp0 = arm.forward(state.q).frames[6].position;
    state.goal = tcp0 + Vec3{0.08, -0.02, 0.05};
    const Vec3 segment = state.goal - tcp0;
    const Vec3 direction = segment.normalized();
    const Vec3 lateral = direction.cross(Vec3{0, 0, 1}).normalized();
    const Vec3 obstacle = tcp0 + segment * 0.45 + lateral * 0.05;
    constexpr double kObstacleRadius = 0.03;

    CameraModel camera;
    camera.fx = 180;
    camera.fy = 180;
    camera.cx = 60;
    camera.cy = 45;
    camera.width = 120;
    camera.height = 90;
    // 光轴沿基座 +X，球心落在主点附近。
    camera.base_from_camera.rotation.m[0][0] = 0;
    camera.base_from_camera.rotation.m[0][1] = 0;
    camera.base_from_camera.rotation.m[0][2] = 1;
    camera.base_from_camera.rotation.m[1][0] = -1;
    camera.base_from_camera.rotation.m[1][1] = 0;
    camera.base_from_camera.rotation.m[1][2] = 0;
    camera.base_from_camera.rotation.m[2][0] = 0;
    camera.base_from_camera.rotation.m[2][1] = -1;
    camera.base_from_camera.rotation.m[2][2] = 0;
    camera.base_from_camera.translation = obstacle + Vec3{-0.40, 0, 0};
    pipeline.setCamera(camera);
    pipeline.setSelfFilterMargin(0.004);

    std::vector<float> depth;
    renderSphereDepth(depth, camera, obstacle, kObstacleRadius);
    pipeline.updateFromDepth(depth.data(), camera.width, camera.height, state.q);

    const double initial_goal = (state.goal - tcp0).norm();
    double min_gap = geometricGap(arm, state.q, obstacle, kObstacleRadius);
    double max_deviation = 0;
    std::cout << "step,x,y,z,goal_distance,geometric_gap\n";
    bool reached = false;
    for (int step = 0; step < 80 && !reached; ++step) {
        const AvoidanceResult result = pipeline.step(state);
        const double gap = geometricGap(arm, state.q, obstacle, kObstacleRadius);
        min_gap = std::min(min_gap, gap);
        const double t = clampDouble((result.tcp - tcp0).dot(segment) / segment.dot(segment), 0.0, 1.0);
        const double deviation = (result.tcp - (tcp0 + segment * t)).norm();
        max_deviation = std::max(max_deviation, deviation);
        if (step % 5 == 0 || result.goal_reached) {
            std::cout << step << "," << result.tcp.x << "," << result.tcp.y << "," << result.tcp.z
                      << "," << result.goal_distance << "," << gap << "\n";
        }
        reached = result.goal_reached;
    }

    std::cout << "initial_goal=" << initial_goal << " final_goal=" << (state.goal - arm.forward(state.q).frames[6].position).norm()
              << " min_geometric_gap=" << min_gap << " max_deviation=" << max_deviation
              << " reached=" << (reached ? 1 : 0) << "\n";
    if (!reached || min_gap < -0.005 || max_deviation < 0.004) {
        return 1;
    }
    return 0;
}
