#include "filterreg.h"

#include "permutohedral.h"

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>
#include <vector>

namespace filterreg {
namespace {

constexpr float kPi = 3.14159265358979323846f;

float squaredKernelSum(const Eigen::MatrixX3f& x, const Eigen::MatrixX3f& y) {
    // sum_ij ||x_i - y_j||^2 / (m * dim * n)
    const float m = static_cast<float>(x.rows());
    const float n = static_cast<float>(y.rows());
    const float dim = 3.f;
    const float sum_x2 = x.squaredNorm();
    const float sum_y2 = y.squaredNorm();
    const Eigen::Vector3f sx = x.colwise().sum();
    const Eigen::Vector3f sy = y.colwise().sum();
    const float sum = n * sum_x2 + m * sum_y2 - 2.f * sx.dot(sy);
    return sum / (m * dim * n);
}

Eigen::Matrix3f skew(const Eigen::Vector3f& x) {
    Eigen::Matrix3f s;
    s << 0.f, -x.z(), x.y(),
         x.z(), 0.f, -x.x(),
         -x.y(), x.x(), 0.f;
    return s;
}

void twistToRt(const Eigen::Matrix<float, 6, 1>& tw, Eigen::Matrix3f& r, Eigen::Vector3f& t) {
    const float ang = tw.head<3>().norm();
    t = tw.tail<3>();
    if (ang == 0.f) {
        r = Eigen::Matrix3f::Identity();
        return;
    }
    const Eigen::Vector3f n = tw.head<3>() / ang;
    const float c = std::cos(ang);
    const float s = std::sin(ang);
    r = c * Eigen::Matrix3f::Identity() + (1.f - c) * n * n.transpose() + s * skew(n);
}

void twistMul(const Eigen::Matrix<float, 6, 1>& tw,
              const Eigen::Matrix3f& rot,
              const Eigen::Vector3f& t,
              Eigen::Matrix3f& rot_out,
              Eigen::Vector3f& t_out) {
    Eigen::Matrix3f tr;
    Eigen::Vector3f tt;
    twistToRt(tw, tr, tt);
    rot_out = tr * rot;
    t_out = tr * t + tt;
}

std::pair<Eigen::Matrix3f, Eigen::Vector3f> kabsch(const Eigen::MatrixX3f& model,
                                                   const Eigen::MatrixX3f& target,
                                                   const Eigen::VectorXf& weight) {
    Eigen::Vector3f model_c = Eigen::Vector3f::Zero();
    Eigen::Vector3f target_c = Eigen::Vector3f::Zero();
    float total_w = 0.f;
    for (int i = 0; i < model.rows(); ++i) {
        const float w = weight[i];
        total_w += w;
        model_c += w * model.row(i).transpose();
        target_c += w * target.row(i).transpose();
    }
    if (total_w == 0.f) return {Eigen::Matrix3f::Identity(), Eigen::Vector3f::Zero()};
    model_c /= total_w;
    target_c /= total_w;

    float h_weight = 0.f;
    Eigen::Matrix3f hh = Eigen::Matrix3f::Zero();
    for (int k = 0; k < model.rows(); ++k) {
        const Eigen::Vector3f cm = model.row(k).transpose() - model_c;
        const Eigen::Vector3f ct = target.row(k).transpose() - target_c;
        const float w2 = weight[k] * weight[k];
        h_weight += w2;
        hh += w2 * cm * ct.transpose();
    }
    if (h_weight == 0.f) return {Eigen::Matrix3f::Identity(), target_c - model_c};
    hh /= h_weight;

    Eigen::JacobiSVD<Eigen::Matrix3f> svd(hh, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Vector3f ss = Eigen::Vector3f::Ones();
    ss[2] = (svd.matrixU() * svd.matrixV()).determinant();
    const Eigen::Matrix3f r = svd.matrixV() * ss.asDiagonal() * svd.matrixU().transpose();
    return {r, target_c - r * model_c};
}

std::pair<Eigen::Matrix<float, 6, 1>, float> twistPt2Pl(const Eigen::MatrixX3f& model,
                                                        const Eigen::MatrixX3f& target,
                                                        const Eigen::MatrixX3f& normals,
                                                        const Eigen::VectorXf& weight) {
    Eigen::Matrix<float, 6, 6> ata = Eigen::Matrix<float, 6, 6>::Zero();
    Eigen::Matrix<float, 6, 1> atb = Eigen::Matrix<float, 6, 1>::Zero();
    float r_sum = 0.f;
    for (int k = 0; k < model.rows(); ++k) {
        const Eigen::Vector3f v = model.row(k).transpose();
        const Eigen::Vector3f y = target.row(k).transpose();
        const Eigen::Vector3f n = normals.row(k).transpose();
        const float w = weight[k];
        const float residual = n.dot(y - v);
        Eigen::Matrix<float, 6, 1> jac;
        jac.head<3>() = v.cross(n);
        jac.tail<3>() = n;
        ata += w * jac * jac.transpose();
        atb += w * residual * jac;
        r_sum += w * w * residual * residual;
    }
    return {ata.selfadjointView<Eigen::Lower>().ldlt().solve(atb), r_sum};
}

struct Estep {
    Eigen::VectorXf m0;
    Eigen::MatrixX3f m1;
    Eigen::VectorXf m2;
    Eigen::MatrixX3f nx;
    bool has_m2 = false;
    bool has_nx = false;
};

Estep expectation(const Eigen::MatrixX3f& t_source,
                  const Eigen::MatrixX3f& target,
                  const Eigen::MatrixX3f& y,
                  const Eigen::MatrixX3f& target_normals,
                  float sigma2,
                  bool update_sigma2,
                  Objective objective,
                  float alpha) {
    const int m = static_cast<int>(t_source.rows());
    const int n = static_cast<int>(target.rows());
    const float sigma = std::sqrt(sigma2);

    Eigen::MatrixXf fin(3, m + n);
    fin.leftCols(m) = t_source.transpose() / sigma;
    fin.rightCols(n) = target.transpose() / sigma;

    Permutohedral ph;
    ph.init(fin, true);
    if (ph.latticeSize() > static_cast<int>(static_cast<float>(n) * alpha))
        ph.init(fin, false);

    Eigen::MatrixXf vin0 = Eigen::MatrixXf::Zero(1, m + n);
    vin0.rightCols(n).setOnes();
    const Eigen::MatrixXf m0f = ph.compute(vin0, m);

    Eigen::MatrixXf vin1 = Eigen::MatrixXf::Zero(3, m + n);
    vin1.rightCols(n) = y.transpose();
    const Eigen::MatrixXf m1f = ph.compute(vin1, m);

    Estep out;
    out.m0 = m0f.leftCols(m).transpose();
    out.m1 = m1f.leftCols(m).transpose();

    if (update_sigma2) {
        Eigen::MatrixXf vin2 = Eigen::MatrixXf::Zero(1, m + n);
        vin2.rightCols(n) = y.rowwise().squaredNorm().transpose();
        out.m2 = ph.compute(vin2, m).leftCols(m).transpose();
        out.has_m2 = true;
    }
    if (objective == Objective::PointToPlane) {
        if (target_normals.rows() != y.rows())
            throw std::invalid_argument("pt2pl requires target normals");
        Eigen::MatrixXf vin = Eigen::MatrixXf::Zero(3, m + n);
        vin.rightCols(n) = target_normals.transpose();
        out.nx = ph.compute(vin, m).leftCols(m).transpose();
        out.has_nx = true;
    }
    return out;
}

struct Mstep {
    RigidTransform tf;
    float sigma2 = 0.f;
    float q = 0.f;
    bool valid = true;
};

Mstep maximization(const Eigen::MatrixX3f& t_source,
                   const Eigen::MatrixX3f& target,
                   const Estep& estep,
                   const RigidTransform& trans_p,
                   float sigma2,
                   float w,
                   Objective objective,
                   bool update_sigma2) {
    const int m = static_cast<int>(t_source.rows());
    const int n = static_cast<int>(target.rows());
    const float c = (w <= 0.f)
                        ? 0.f
                        : (w / (1.f - w)) * (static_cast<float>(n) / static_cast<float>(m)) *
                              std::pow(2.f * sigma2 * kPi, 1.5f);

    std::vector<int> keep;
    keep.reserve(m);
    for (int i = 0; i < m; ++i)
        if (estep.m0[i] != 0.f) keep.push_back(i);
    if (keep.empty()) return {trans_p, sigma2, 0.f, false};

    const int mk = static_cast<int>(keep.size());
    Eigen::VectorXf m0(mk);
    Eigen::MatrixX3f m1(mk, 3);
    Eigen::MatrixX3f src(mk, 3);
    Eigen::VectorXf m2;
    Eigen::MatrixX3f nx;
    if (update_sigma2) m2.resize(mk);
    if (objective == Objective::PointToPlane) nx.resize(mk, 3);
    for (int i = 0; i < mk; ++i) {
        const int idx = keep[i];
        m0[i] = estep.m0[idx];
        m1.row(i) = estep.m1.row(idx);
        src.row(i) = t_source.row(idx);
        if (update_sigma2) m2[i] = estep.m2[idx];
        if (objective == Objective::PointToPlane) nx.row(i) = estep.nx.row(idx);
    }

    Eigen::MatrixX3f m1m0(mk, 3);
    for (int i = 0; i < mk; ++i) m1m0.row(i) = m1.row(i) / m0[i];
    const Eigen::VectorXf m0m0 = m0.array() / (m0.array() + c);
    const Eigen::VectorXf drxdx = (m0m0.array() / sigma2).sqrt();

    Mstep out;
    out.valid = true;
    if (objective == Objective::PointToPoint) {
        const auto rt = kabsch(src, m1m0, drxdx);
        out.tf.rot = rt.first * trans_p.rot;
        out.tf.t = rt.first * trans_p.t + rt.second;
        float q = 0.f;
        for (int i = 0; i < mk; ++i)
            q += drxdx[i] * (src.row(i) - m1m0.row(i)).norm();
        out.q = q;
    } else {
        Eigen::MatrixX3f nxm0(mk, 3);
        for (int i = 0; i < mk; ++i) nxm0.row(i) = nx.row(i) / m0[i];
        const auto twq = twistPt2Pl(src, m1m0, nxm0, drxdx);
        twistMul(twq.first, trans_p.rot, trans_p.t, out.tf.rot, out.tf.t);
        out.q = twq.second;
    }

    if (update_sigma2) {
        float num = 0.f;
        for (int i = 0; i < mk; ++i) {
            const float term = m0[i] * src.row(i).squaredNorm() - 2.f * src.row(i).dot(m1.row(i)) + m2[i];
            num += term / (m0[i] + c);
        }
        out.sigma2 = num / (3.f * m0m0.sum());
    } else {
        out.sigma2 = sigma2;
    }
    return out;
}

}  // namespace

Eigen::MatrixX3f RigidTransform::apply(const Eigen::MatrixX3f& points) const {
    Eigen::MatrixX3f out(points.rows(), 3);
    for (int i = 0; i < points.rows(); ++i)
        out.row(i) = (rot * points.row(i).transpose() + t).transpose();
    return out;
}

Result registration(const Eigen::MatrixX3f& source,
                    const Eigen::MatrixX3f& target,
                    const Eigen::MatrixX3f& target_normals,
                    const Options& opt) {
    if (source.cols() != 3 || target.cols() != 3)
        throw std::invalid_argument("source and target must be N x 3");
    if (opt.objective == Objective::PointToPlane && target_normals.rows() != target.rows())
        throw std::invalid_argument("pt2pl requires one normal per target point");

    Result res;
    res.transformation = opt.init;
    res.sigma2 = opt.sigma2;
    if (res.sigma2 < 0.f)
        res.sigma2 = std::max(squaredKernelSum(source, target), opt.min_sigma2);

    float prev_q = 0.f;
    bool have_q = false;
    for (int i = 0; i < opt.max_iter; ++i) {
        const Eigen::MatrixX3f t_source = res.transformation.apply(source);
        const Estep e = expectation(t_source, target, target, target_normals, res.sigma2,
                                    opt.update_sigma2, opt.objective, opt.alpha);
        const Mstep m = maximization(t_source, target, e, res.transformation, res.sigma2, opt.w,
                                     opt.objective, opt.update_sigma2);
        res.iterations = i + 1;
        if (!m.valid) {
            res.converged = have_q;
            break;
        }
        res.transformation = m.tf;
        res.sigma2 = std::max(m.sigma2, opt.min_sigma2);
        res.q = m.q;
        if (have_q && std::abs(m.q - prev_q) < opt.tol) {
            res.converged = true;
            break;
        }
        prev_q = m.q;
        have_q = true;
    }
    return res;
}

}  // namespace filterreg
