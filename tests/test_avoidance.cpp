#include "arm_avoidance/ControlMath.h"
#include "arm_avoidance/ReactiveAvoidance.h"

#include <cmath>
#include <iostream>
#include <string>

namespace {

int g_failed = 0;

void expect(bool condition, const std::string& name) {
    if (!condition) {
        std::cerr << "FAIL " << name << "\n";
        ++g_failed;
        return;
    }
    std::cout << "ok   " << name << "\n";
}

arm_avoidance::EsdfGrid makeWorkspace() {
    return arm_avoidance::EsdfGrid({-0.05, -0.55, -0.25}, 0.012, 80, 80, 60);
}

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

void testForwardKinematics() {
    const arm_avoidance::SixAxisArm arm = arm_avoidance::SixAxisArm::DefaultIndustrial();
    const std::array<double, 6> q{0, 0, 0, 0, 0, 0};
    const arm_avoidance::Vec3 tcp = arm.forward(q).frames[6].position;
    expect(std::abs(tcp.x - 0.70) < 1e-9 && std::abs(tcp.y + 0.15) < 1e-9 &&
               std::abs(tcp.z - 0.20) < 1e-9,
           "zero configuration tool position");
}

void testJacobian() {
    const arm_avoidance::SixAxisArm arm = arm_avoidance::SixAxisArm::DefaultIndustrial();
    const std::array<double, 6> q{0.2, -0.4, 0.7, -0.3, 0.5, -0.2};
    const arm_avoidance::ChainState chain = arm.forward(q);
    const arm_avoidance::Vec3 local{0.02, -0.01, 0.03};
    double J[3][6];
    arm.positionalJacobian(chain, 4, local, J);
    const arm_avoidance::Vec3 p0 = arm.worldPoint(chain, 4, local);
    constexpr double kEps = 1e-6;
    bool matched = true;
    for (int joint = 0; joint < 6; ++joint) {
        std::array<double, 6> qp = q;
        std::array<double, 6> qm = q;
        qp[static_cast<size_t>(joint)] += kEps;
        qm[static_cast<size_t>(joint)] -= kEps;
        const arm_avoidance::Vec3 pp = arm.worldPoint(arm.forward(qp), 4, local);
        const arm_avoidance::Vec3 pm = arm.worldPoint(arm.forward(qm), 4, local);
        const arm_avoidance::Vec3 numerical = (pp - pm) * (1.0 / (2 * kEps));
        const arm_avoidance::Vec3 analytic{J[0][joint], J[1][joint], J[2][joint]};
        if ((numerical - analytic).norm() > 1e-5) {
            matched = false;
        }
        (void)p0;
    }
    expect(matched, "positional jacobian matches central difference");
}

void testDampedLeastSquares() {
    double J[3][6] = {};
    J[0][0] = 1;
    J[1][1] = 1;
    J[2][2] = 1;
    double q_dot[6];
    arm_avoidance::dampedLeastSquares(J, {1.0, 0.0, 0.0}, 1e-6, q_dot);
    expect(std::abs(q_dot[0] - 1.0) < 1e-5 && std::abs(q_dot[1]) < 1e-5 && std::abs(q_dot[2]) < 1e-5,
           "damped least squares tracks a unit translation");
}

void testEsdf() {
    arm_avoidance::EsdfGrid grid({0, 0, 0}, 1.0, 7, 7, 7);
    grid.insertPoint({3.5, 3.5, 3.5});
    grid.updateDistance();
    const arm_avoidance::EsdfQuery inside = grid.query({3.5, 3.5, 3.5});
    const arm_avoidance::EsdfQuery neighbor = grid.query({5.5, 3.5, 3.5});
    expect(inside.valid && inside.distance < 0, "occupied voxel has negative distance");
    expect(neighbor.valid && std::abs(neighbor.distance - 2.0) < 1e-6,
           "free voxel distance equals the index distance");
    expect(neighbor.gradient.x > 0.8, "distance gradient points away from the obstacle");

    arm_avoidance::EsdfGrid sphere_grid({-0.4, -0.4, -0.4}, 0.02, 40, 40, 40);
    sphere_grid.insertSphere({0, 0, 0}, 0.08);
    sphere_grid.updateDistance();
    const arm_avoidance::EsdfQuery outside = sphere_grid.query({0.20, 0, 0});
    expect(outside.valid && std::abs(outside.distance - 0.12) < 0.03 && outside.gradient.x > 0.9,
           "sphere distance field matches the geometric surface");
}

void testDepthAndBodyFilter() {
    arm_avoidance::CameraModel camera;
    camera.fx = 100;
    camera.fy = 100;
    camera.cx = 10;
    camera.cy = 8;
    camera.width = 21;
    camera.height = 17;
    std::vector<float> depth(static_cast<size_t>(camera.width * camera.height), 0.f);
    depth[static_cast<size_t>(8 * camera.width + 10)] = 1.5f;
    const std::vector<arm_avoidance::Vec3> points =
        arm_avoidance::backprojectDepth(depth.data(), camera.width, camera.height, camera, 0.1f, 3.f, 1);
    expect(points.size() == 1 && std::abs(points[0].x) < 1e-9 && std::abs(points[0].y) < 1e-9 &&
               std::abs(points[0].z - 1.5) < 1e-9,
           "principal-point depth backprojects onto the optical axis");

    std::vector<arm_avoidance::Vec3> cloud = {{0, 0, 0}, {1, 0, 0}};
    arm_avoidance::removeRobotBody(cloud, {{0, 0, 0}}, {0.1}, 0.02);
    expect(cloud.size() == 1 && std::abs(cloud[0].x - 1.0) < 1e-12,
           "points inside the robot envelope are removed");
}

void testFreeSpaceReachesGoal() {
    const arm_avoidance::SixAxisArm arm = arm_avoidance::SixAxisArm::DefaultIndustrial();
    arm_avoidance::EsdfGrid grid = makeWorkspace();
    grid.updateDistance();
    arm_avoidance::ReactiveAvoidance controller;
    arm_avoidance::AvoidanceState state;
    state.q = {0.4, -0.8, 1.2, -0.9, 1.1, 0.3};
    const arm_avoidance::Vec3 tcp0 = arm.forward(state.q).frames[6].position;
    state.goal = tcp0 + arm_avoidance::Vec3{0.08, -0.02, 0.05};
    bool reached = false;
    for (int i = 0; i < 80 && !reached; ++i) {
        reached = controller.step(arm, grid, state).goal_reached;
    }
    const double error = (arm.forward(state.q).frames[6].position - state.goal).norm();
    expect(reached && error <= 0.015, "free space motion reaches the cartesian goal");
}

void testObstacleAvoidance() {
    const arm_avoidance::SixAxisArm arm = arm_avoidance::SixAxisArm::DefaultIndustrial();
    arm_avoidance::AvoidanceState state;
    state.q = {0.4, -0.8, 1.2, -0.9, 1.1, 0.3};
    const arm_avoidance::Vec3 tcp0 = arm.forward(state.q).frames[6].position;
    state.goal = tcp0 + arm_avoidance::Vec3{0.08, -0.02, 0.05};
    const arm_avoidance::Vec3 segment = state.goal - tcp0;
    const arm_avoidance::Vec3 direction = segment.normalized();
    const arm_avoidance::Vec3 lateral = direction.cross(arm_avoidance::Vec3{0, 0, 1}).normalized();
    const arm_avoidance::Vec3 obstacle = tcp0 + segment * 0.45 + lateral * 0.05;
    constexpr double kRadius = 0.03;

    arm_avoidance::EsdfGrid unsafe_grid = makeWorkspace();
    unsafe_grid.insertSphere(obstacle, kRadius);
    unsafe_grid.updateDistance();
    arm_avoidance::AvoidanceParams blind;
    blind.influence_distance = 0;
    blind.stop_distance = -1;
    arm_avoidance::ReactiveAvoidance blind_controller(blind);
    arm_avoidance::AvoidanceState blind_state = state;
    double blind_gap = geometricGap(arm, blind_state.q, obstacle, kRadius);
    for (int i = 0; i < 80 && !blind_controller.step(arm, unsafe_grid, blind_state).goal_reached; ++i) {
        blind_gap = std::min(blind_gap, geometricGap(arm, blind_state.q, obstacle, kRadius));
    }
    blind_gap = std::min(blind_gap, geometricGap(arm, blind_state.q, obstacle, kRadius));

    arm_avoidance::EsdfGrid grid = makeWorkspace();
    // 体素中心落在球内才会被占据，向外扩半个栅格，避免真实球面露在距离场外面。
    grid.insertSphere(obstacle, kRadius + 0.5 * grid.resolution());
    grid.updateDistance();
    arm_avoidance::ReactiveAvoidance controller;
    double min_gap = geometricGap(arm, state.q, obstacle, kRadius);
    double min_esdf = controller.minClearance(arm, grid, state.q);
    double max_deviation = 0;
    bool reached = false;
    for (int i = 0; i < 80 && !reached; ++i) {
        const arm_avoidance::AvoidanceResult result = controller.step(arm, grid, state);
        min_gap = std::min(min_gap, geometricGap(arm, state.q, obstacle, kRadius));
        min_esdf = std::min(min_esdf, result.min_clearance);
        const double t =
            arm_avoidance::clampDouble((result.tcp - tcp0).dot(segment) / segment.dot(segment), 0.0, 1.0);
        max_deviation = std::max(max_deviation, (result.tcp - (tcp0 + segment * t)).norm());
        reached = result.goal_reached;
    }
    expect(blind_gap < 0.0, "straight tracking penetrates the obstacle");
    if (!(reached && min_gap >= 0.0 && min_esdf >= 0.01 && max_deviation > 0.005)) {
        std::cerr << "  reached=" << reached << " min_gap=" << min_gap << " min_esdf=" << min_esdf
                  << " deviation=" << max_deviation << "\n";
    }
    expect(reached && min_gap >= 0.0 && min_esdf >= 0.01 && max_deviation > 0.005,
           "avoidance reaches the goal without penetrating");
}

void testJointLimits() {
    const arm_avoidance::SixAxisArm arm = arm_avoidance::SixAxisArm::DefaultIndustrial();
    arm_avoidance::EsdfGrid grid = makeWorkspace();
    grid.updateDistance();
    arm_avoidance::ReactiveAvoidance controller;
    arm_avoidance::AvoidanceState state;
    for (int i = 0; i < 6; ++i) {
        state.q[static_cast<size_t>(i)] = arm.links()[static_cast<size_t>(i)].q_max - 0.02;
    }
    state.goal = arm.forward(state.q).frames[6].position + arm_avoidance::Vec3{0.2, 0.2, 0.2};
    bool inside = true;
    for (int step = 0; step < 40; ++step) {
        controller.step(arm, grid, state);
        for (int i = 0; i < 6; ++i) {
            const arm_avoidance::LinkDH& link = arm.links()[static_cast<size_t>(i)];
            if (state.q[static_cast<size_t>(i)] > link.q_max + 1e-9 ||
                state.q[static_cast<size_t>(i)] < link.q_min - 1e-9) {
                inside = false;
            }
        }
    }
    expect(inside, "joint commands stay inside the position limits");
}

void testDepthPipeline() {
    const arm_avoidance::SixAxisArm arm = arm_avoidance::SixAxisArm::DefaultIndustrial();
    arm_avoidance::VisionAvoidancePipeline pipeline(arm, makeWorkspace());
    arm_avoidance::AvoidanceState state;
    state.q = {0.4, -0.8, 1.2, -0.9, 1.1, 0.3};
    const arm_avoidance::Vec3 tcp0 = arm.forward(state.q).frames[6].position;
    state.goal = tcp0 + arm_avoidance::Vec3{0.08, -0.02, 0.05};
    const arm_avoidance::Vec3 segment = state.goal - tcp0;
    const arm_avoidance::Vec3 obstacle =
        tcp0 + segment * 0.45 + segment.normalized().cross(arm_avoidance::Vec3{0, 0, 1}).normalized() * 0.05;

    arm_avoidance::CameraModel camera;
    camera.fx = 180;
    camera.fy = 180;
    camera.cx = 60;
    camera.cy = 45;
    camera.width = 120;
    camera.height = 90;
    // 光轴沿基座 +X，图像下方对应基座 -Z，使球心落在主点附近。
    camera.base_from_camera.rotation.m[0][0] = 0;
    camera.base_from_camera.rotation.m[0][1] = 0;
    camera.base_from_camera.rotation.m[0][2] = 1;
    camera.base_from_camera.rotation.m[1][0] = -1;
    camera.base_from_camera.rotation.m[1][1] = 0;
    camera.base_from_camera.rotation.m[1][2] = 0;
    camera.base_from_camera.rotation.m[2][0] = 0;
    camera.base_from_camera.rotation.m[2][1] = -1;
    camera.base_from_camera.rotation.m[2][2] = 0;
    camera.base_from_camera.translation = obstacle + arm_avoidance::Vec3{-0.40, 0, 0};
    pipeline.setCamera(camera);
    pipeline.setSelfFilterMargin(0.004);

    std::vector<float> depth;
    arm_avoidance::renderSphereDepth(depth, camera, obstacle, 0.03);
    pipeline.updateFromDepth(depth.data(), camera.width, camera.height, state.q);

    const arm_avoidance::EsdfQuery seen =
        pipeline.esdf().query(obstacle + arm_avoidance::Vec3{-0.03, 0, 0});
    double min_clearance = 1e9;
    double min_gap = geometricGap(arm, state.q, obstacle, 0.03);
    bool reached = false;
    for (int i = 0; i < 80 && !reached; ++i) {
        const arm_avoidance::AvoidanceResult result = pipeline.step(state);
        min_clearance = std::min(min_clearance, result.min_clearance);
        min_gap = std::min(min_gap, geometricGap(arm, state.q, obstacle, 0.03));
        reached = result.goal_reached;
    }
    if (!(seen.valid && seen.distance < 0.025 && reached && min_clearance >= 0.008 && min_gap >= -0.005)) {
        std::cerr << "  seen=" << seen.distance << " valid=" << seen.valid << " reached=" << reached
                  << " min_clearance=" << min_clearance << " min_gap=" << min_gap << "\n";
    }
    expect(seen.valid && seen.distance < 0.025, "depth image of the sphere lands in the distance field");
    expect(reached && min_clearance >= 0.008 && min_gap >= -0.005,
           "vision pipeline avoids the observed obstacle and arrives");
}

}  // namespace

int main() {
    testForwardKinematics();
    testJacobian();
    testDampedLeastSquares();
    testEsdf();
    testDepthAndBodyFilter();
    testFreeSpaceReachesGoal();
    testObstacleAvoidance();
    testJointLimits();
    testDepthPipeline();
    if (g_failed != 0) {
        std::cerr << g_failed << " test(s) failed\n";
        return 1;
    }
    std::cout << "all tests passed\n";
    return 0;
}
