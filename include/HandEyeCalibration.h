#ifndef HAND_EYE_CALIBRATION_H
#define HAND_EYE_CALIBRATION_H

#include <Eigen/Geometry>
#include <string>
#include <vector>

/**
 * 眼在手上（Eye-in-Hand）点法手眼标定。
 *
 * ---------------------------------------------------------------------------
 * 1. 问题定义
 * ---------------------------------------------------------------------------
 * 相机固连在机械臂末端法兰上。记
 *   {B} 机器人基座,  {F} 法兰,  {C} 相机。
 * 待求常值手眼矩阵
 *
 *     X = ^{F}T_{C} ∈ SE(3)
 *
 * 即把相机系下的点变到法兰系：
 *
 *     ^{F}P = R · ^{C}P + t ,     X = [ R  t ;  0  1 ]
 *
 * ---------------------------------------------------------------------------
 * 2. 输入（不少于 3 组，且法兰系下的对应点不能共线）
 * ---------------------------------------------------------------------------
 * 第 i 组：
 *   ^{C}P_i     标定点在相机坐标系中的 XYZ
 *   ^{B}T_{F,i} 拍照时刻法兰相对基座的位姿
 *   ^{B}P_i     同一标定点在基座下的 XYZ（用 TCP 触碰得到，故称 TCP 的 XYZ）
 *
 * 同一物理点既可用一个固定标定点（各组 TCP 相同）也可用多个不同点。
 *
 * ---------------------------------------------------------------------------
 * 3. 约束方程
 * ---------------------------------------------------------------------------
 * 坐标链：
 *
 *     ^{B}P_i = ^{B}T_{F,i} · ^{F}T_{C} · ^{C}P_i
 *
 * 左乘法兰位姿的逆，把 TCP 变到法兰系：
 *
 *     ^{F}P_i := ^{B}T_{F,i}^{-1} · ^{B}P_i = X · ^{C}P_i
 *
 * 于是得到 3D–3D 刚体对应：
 *
 *     q_i = R p_i + t ,   p_i = ^{C}P_i ,  q_i = ^{F}P_i
 *
 * ---------------------------------------------------------------------------
 * 4. Kabsch–Umeyama / PCL SVD 闭式解
 * ---------------------------------------------------------------------------
 * (1) 质心
 *       p̄ = (1/N) Σ p_i ,   q̄ = (1/N) Σ q_i
 * (2) 去质心
 *       p'_i = p_i − p̄ ,   q'_i = q_i − q̄
 * (3) 协方差
 *       H = Σ_i p'_i q'_iᵀ
 * (4) SVD
 *       H = U Σ Vᵀ
 * (5) 旋转（强制 det R = +1，排除反射）
 *       R = V · diag(1, 1, det(V Uᵀ)) · Uᵀ
 * (6) 平移
 *       t = q̄ − R p̄
 *
 * N = 3 且不共线时 SE(3) 唯一；N > 3 为各向同性噪声下的最小二乘最优。
 *
 * 残差（基座系）：
 *       e_i = || ^{B}T_{F,i} · X · ^{C}P_i − ^{B}P_i ||
 *       RMSE = sqrt( (1/N) Σ e_i² )
 *
 * ---------------------------------------------------------------------------
 * 5. 与棋盘 AX=XB 的关系
 * ---------------------------------------------------------------------------
 * 本函数只用点的 XYZ，不需要标定板完整位姿，因此不是 Tsai 的 AX=XB，
 * 而是把 TCP 变到法兰系后做一次刚体配准。相机相对法兰刚体固连时两种
 * 模型在理想数据上等价。
 */
struct EyeInHandCalibResult {
    bool success = false;
    std::string message;

    /// ^{F}T_{C}：相机系 → 法兰系
    Eigen::Isometry3d T_flange_camera = Eigen::Isometry3d::Identity();

    /// 基座系下标定点的 RMS 误差，单位与输入坐标一致
    double rmse = 0.0;

    std::vector<double> per_sample_error;
};

using Isometry3dVector =
    std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>>;

/**
 * @brief 由标定点相机坐标、法兰位姿、TCP 基座坐标计算眼在手上的手眼矩阵。
 *
 * @param points_in_camera  标定点在相机系下的 XYZ，长度 N ≥ 3
 * @param flanges_in_base   拍照时 ^{B}T_{F}，长度 N
 * @param tcps_in_base      同一标定点在基座下的 XYZ（TCP 触碰坐标），长度 N
 * @return EyeInHandCalibResult 成功时 T_flange_camera 为 ^{F}T_{C}
 *
 * 所有平移量必须使用同一长度单位（mm 或 m）。
 */
EyeInHandCalibResult CalibrateEyeInHand(
    const std::vector<Eigen::Vector3d>& points_in_camera,
    const Isometry3dVector& flanges_in_base,
    const std::vector<Eigen::Vector3d>& tcps_in_base);

/// 由平移 + 单位四元数 (w, x, y, z) 构造 ^{B}T_{F}。
Eigen::Isometry3d FlangePoseFromXyzQuat(
    double x, double y, double z,
    double qw, double qx, double qy, double qz);

/**
 * 由平移 + ZYX 欧拉角（先 Rx 后 Ry 再 Rz，即 R = Rz(yaw)*Ry(pitch)*Rx(roll)）
 * 构造 ^{B}T_{F}。角度单位为弧度。
 */
Eigen::Isometry3d FlangePoseFromXyzRpyZYX(
    double x, double y, double z,
    double roll, double pitch, double yaw);

#endif // HAND_EYE_CALIBRATION_H
