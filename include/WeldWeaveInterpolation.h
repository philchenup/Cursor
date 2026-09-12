#ifndef WELD_WEAVE_INTERPOLATION_H
#define WELD_WEAVE_INTERPOLATION_H

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>

/**
 * @brief 焊缝摆动插值方式。
 */
enum class WeavePattern {
    Triangle,  ///< 三角摆动：沿焊缝分段线性左右摆动
    Sine       ///< 正弦摆动：沿焊缝按正弦曲线摆动
};

/**
 * @brief 按三角摆动或正弦摆动对焊缝进行轨迹插值。
 *
 * @p start 与 @p end 为位姿（Eigen::Affine3f）：平移为焊缝起终点，
 * 旋转表示方向且二者一致。焊缝前进方向由终点平移减起点平移确定。
 * 摆动方向取姿态 Z 轴与焊缝方向的叉积，因而垂直于 Z 轴且垂直于焊缝。
 * 若 Z 与焊缝平行，则退化为姿态 Y 轴（本身垂直于 Z）。
 *
 * 弦长 @p chordLength 表示一个完整摆动周期沿焊缝的长度。
 * 若剩余到终点的焊缝长度不足半个摆动周期，则不再摆动，沿直线到达终点。
 * 输出各位姿保持与起点相同的方向，仅平移沿摆动轨迹变化。
 *
 * @param start        焊缝起点位姿（平移 + 方向）
 * @param end          焊缝终点位姿（平移 + 方向，方向与起点一致）
 * @param pattern      三角摆动或正弦摆动
 * @param amplitude    摆动幅度（侧向峰值偏移，与坐标同单位）
 * @param chordLength  弦长，一个完整摆动周期沿焊缝的长度，必须 > 0
 * @param sampleStep   沿焊缝方向的采样步长；<= 0 时自动取 chordLength / 32
 * @return 插值轨迹位姿，至少包含起点；可到达时包含终点
 *
 * @note 半周期结束时侧向偏移为 0，随后的直线段沿焊缝中心线到达终点。
 */
std::vector<Eigen::Affine3f> interpolateWeldWeave(
    const Eigen::Affine3f& start,
    const Eigen::Affine3f& end,
    WeavePattern pattern,
    float amplitude,
    float chordLength,
    float sampleStep = 0.0f);

#endif // WELD_WEAVE_INTERPOLATION_H
