#include "arm_avoidance/ReactiveAvoidance.h"

#include "arm_avoidance/ControlMath.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace arm_avoidance {
namespace {

struct PlacedPoint {
    ControlPoint spec;
    Vec3 center;
    double J[3][6];
};

std::vector<PlacedPoint> placePoints(const SixAxisArm& arm, const ChainState& chain) {
    std::vector<PlacedPoint> placed;
    const std::vector<ControlPoint> specs = arm.controlPoints(chain);
    placed.reserve(specs.size());
    for (const ControlPoint& spec : specs) {
        PlacedPoint point;
        point.spec = spec;
        point.center = arm.worldPoint(chain, spec.link, spec.local);
        arm.positionalJacobian(chain, spec.link, spec.local, point.J);
        placed.push_back(point);
    }
    return placed;
}

double clearanceOf(const EsdfQuery& query, double radius) {
    if (!query.valid) {
        return std::numeric_limits<double>::infinity();
    }
    return query.distance - radius;
}

void limitApproach(Vec3& velocity, const Vec3& normal, double clearance, double influence,
                   double stop_distance, double horizon, double max_speed, const Vec3& to_goal) {
    if (!std::isfinite(clearance) || clearance >= influence || normal.norm() < 1e-6) {
        return;
    }
    const double vn_min = (stop_distance - clearance) / std::max(horizon, 1e-4);
    const double vn = velocity.dot(normal);
    if (vn < vn_min) {
        velocity = velocity + normal * (vn_min - vn);
    }
    const Vec3 tangent = velocity - normal * velocity.dot(normal);
    if (tangent.norm() < 0.04 * max_speed) {
        Vec3 aux = normal.cross(Vec3{0, 0, 1});
        if (aux.norm() < 1e-4) {
            aux = normal.cross(Vec3{1, 0, 0});
        }
        aux = aux.normalized();
        if (aux.dot(to_goal) < 0) {
            aux = -aux;
        }
        velocity = velocity + aux * (0.6 * max_speed);
        const double vn_after = velocity.dot(normal);
        if (vn_after < vn_min) {
            velocity = velocity + normal * (vn_min - vn_after);
        }
    }
    const double speed = velocity.norm();
    if (speed > max_speed && speed > 1e-12) {
        velocity = velocity * (max_speed / speed);
    }
}

void enforceDamper(double q_dot[6], const double J[3][6], const Vec3& normal, double min_rate,
                   int& active) {
    double row[6];
    double row_norm_sq = 0;
    double current = 0;
    for (int j = 0; j < 6; ++j) {
        row[j] = J[0][j] * normal.x + J[1][j] * normal.y + J[2][j] * normal.z;
        row_norm_sq += row[j] * row[j];
        current += row[j] * q_dot[j];
    }
    if (current < min_rate) {
        const double scale = (min_rate - current) / (row_norm_sq + 1e-9);
        for (int j = 0; j < 6; ++j) {
            q_dot[j] += row[j] * scale;
        }
        ++active;
    }
}

}  // namespace

ReactiveAvoidance::ReactiveAvoidance(AvoidanceParams params) : params_(params) {}

double ReactiveAvoidance::minClearance(const SixAxisArm& arm, const EsdfGrid& esdf,
                                       const std::array<double, 6>& q) const {
    const ChainState chain = arm.forward(q);
    const std::vector<PlacedPoint> points = placePoints(arm, chain);
    double minimum = std::numeric_limits<double>::infinity();
    for (const PlacedPoint& point : points) {
        minimum = std::min(minimum, clearanceOf(esdf.query(point.center), point.spec.radius));
    }
    return minimum;
}

AvoidanceResult ReactiveAvoidance::step(const SixAxisArm& arm, const EsdfGrid& esdf,
                                        AvoidanceState& state) const {
    AvoidanceResult result;
    const ChainState chain = arm.forward(state.q);
    const std::vector<PlacedPoint> points = placePoints(arm, chain);
    const Vec3 tcp = chain.frames[6].position;
    result.tcp = tcp;
    result.goal_distance = (state.goal - tcp).norm();
    result.min_clearance = minClearance(arm, esdf, state.q);
    result.goal_reached = result.goal_distance <= params_.goal_tolerance;

    const PlacedPoint* tcp_point = nullptr;
    for (const PlacedPoint& point : points) {
        if (point.spec.link == 6 && point.spec.local.norm() < 1e-9) {
            tcp_point = &point;
            break;
        }
    }

    Vec3 velocity{0, 0, 0};
    if (!result.goal_reached && result.goal_distance > 1e-9) {
        const double desired = std::min(params_.max_tcp_speed,
                                        params_.attract_gain * result.goal_distance);
        velocity = (state.goal - tcp) * (desired / result.goal_distance);
    }

    if (tcp_point != nullptr) {
        const EsdfQuery tcp_field = esdf.query(tcp_point->center);
        const double tcp_clearance = clearanceOf(tcp_field, tcp_point->spec.radius);
        limitApproach(velocity, tcp_field.gradient, tcp_clearance, params_.influence_distance,
                      params_.stop_distance, params_.brake_horizon, params_.max_tcp_speed,
                      state.goal - tcp);
    }

    double q_dot[6] = {0, 0, 0, 0, 0, 0};
    if (tcp_point != nullptr) {
        dampedLeastSquares(tcp_point->J, velocity, params_.damping, q_dot);
    }

    const std::array<LinkDH, 6>& links = arm.links();
    for (int j = 0; j < 6; ++j) {
        const LinkDH& link = links[static_cast<size_t>(j)];
        const double q = state.q[static_cast<size_t>(j)];
        const double lower = link.q_min + params_.joint_limit_margin;
        const double upper = link.q_max - params_.joint_limit_margin;
        if (q < lower) {
            q_dot[j] += params_.joint_limit_gain * (lower - q);
        } else if (q > upper) {
            q_dot[j] -= params_.joint_limit_gain * (q - upper);
        }
    }

    int active = 0;
    for (int iteration = 0; iteration < params_.damper_iterations; ++iteration) {
        for (const PlacedPoint& point : points) {
            const EsdfQuery field = esdf.query(point.center);
            const double clearance = clearanceOf(field, point.spec.radius);
            if (!field.valid || !std::isfinite(clearance) || clearance >= params_.influence_distance ||
                field.gradient.norm() < 1e-6) {
                continue;
            }
            const double min_rate =
                (params_.stop_distance - clearance) / std::max(params_.brake_horizon, 1e-4);
            enforceDamper(q_dot, point.J, field.gradient, min_rate, active);
        }
    }
    result.active_constraints = active;

    for (int j = 0; j < 6; ++j) {
        const LinkDH& link = links[static_cast<size_t>(j)];
        const double q = state.q[static_cast<size_t>(j)];
        q_dot[j] = clampDouble(q_dot[j], -params_.max_joint_speed, params_.max_joint_speed);
        if (q <= link.q_min + 1e-6) {
            q_dot[j] = std::max(0.0, q_dot[j]);
        }
        if (q >= link.q_max - 1e-6) {
            q_dot[j] = std::min(0.0, q_dot[j]);
        }
    }

    const double clearance_now = result.min_clearance;
    double scale = 1.0;
    std::array<double, 6> q_next = state.q;
    for (int attempt = 0; attempt <= params_.safety_bisections; ++attempt) {
        std::array<double, 6> candidate = state.q;
        for (int j = 0; j < 6; ++j) {
            candidate[static_cast<size_t>(j)] += q_dot[j] * params_.dt * scale;
        }
        candidate = arm.clampJoints(candidate);
        const double clearance_next = minClearance(arm, esdf, candidate);
        const bool keeps_stop = clearance_next + 1e-4 >= params_.stop_distance;
        const bool does_not_worsen = clearance_next + 1e-4 >= clearance_now;
        q_next = candidate;
        if (keeps_stop || does_not_worsen || !std::isfinite(clearance_now)) {
            break;
        }
        scale *= 0.5;
    }

    result.step_scale = scale;
    result.q_command = q_next;
    for (int j = 0; j < 6; ++j) {
        result.q_dot[static_cast<size_t>(j)] = q_dot[j] * scale;
    }
    state.q = q_next;
    result.tcp = arm.forward(state.q).frames[6].position;
    result.goal_distance = (state.goal - result.tcp).norm();
    result.min_clearance = minClearance(arm, esdf, state.q);
    result.goal_reached = result.goal_distance <= params_.goal_tolerance;
    return result;
}

VisionAvoidancePipeline::VisionAvoidancePipeline(SixAxisArm arm, EsdfGrid grid, AvoidanceParams params)
    : arm_(std::move(arm)), esdf_(std::move(grid)), avoidance_(params) {}

void VisionAvoidancePipeline::setCamera(CameraModel camera) {
    camera_ = std::move(camera);
    camera_set_ = true;
}

void VisionAvoidancePipeline::setDepthRange(double min_depth, double max_depth) {
    min_depth_ = min_depth;
    max_depth_ = max_depth;
}

void VisionAvoidancePipeline::setPixelStride(int stride) { stride_ = std::max(1, stride); }

void VisionAvoidancePipeline::setSelfFilterMargin(double margin) { self_filter_margin_ = margin; }

void VisionAvoidancePipeline::updateFromPoints(const std::vector<Vec3>& obstacle_points_base) {
    esdf_.clear();
    for (const Vec3& point : obstacle_points_base) {
        esdf_.insertPoint(point);
    }
    esdf_.updateDistance();
}

void VisionAvoidancePipeline::updateFromDepth(const float* depth, int width, int height,
                                              const std::array<double, 6>& q) {
    std::vector<Vec3> points;
    if (camera_set_) {
        points = backprojectDepth(depth, width, height, camera_, static_cast<float>(min_depth_),
                                  static_cast<float>(max_depth_), stride_);
        const ChainState chain = arm_.forward(q);
        const std::vector<ControlPoint> specs = arm_.controlPoints(chain);
        std::vector<Vec3> centers;
        std::vector<double> radii;
        centers.reserve(specs.size());
        radii.reserve(specs.size());
        for (const ControlPoint& spec : specs) {
            centers.push_back(arm_.worldPoint(chain, spec.link, spec.local));
            radii.push_back(spec.radius);
        }
        removeRobotBody(points, centers, radii, self_filter_margin_);
    }
    updateFromPoints(points);
}

AvoidanceResult VisionAvoidancePipeline::step(AvoidanceState& state) {
    return avoidance_.step(arm_, esdf_, state);
}

}  // namespace arm_avoidance
