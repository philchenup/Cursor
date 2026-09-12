#ifndef WELD_WEAVE_INTERPOLATION_H
#define WELD_WEAVE_INTERPOLATION_H

#include <Eigen/Dense>
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
 * 焊缝前进方向由 @p end - @p start 确定；摆动发生在与焊缝方向垂直、
 * 并由 @p weaveDir 给出的侧向。弦长 @p chordLength 表示一个完整摆动周期
 * 沿焊缝的长度。若剩余到终点的焊缝长度不足半个摆动周期，则不再摆动，
 * 沿直线到达终点。
 *
 * @param start        焊缝起点
 * @param end          焊缝终点
 * @param weaveDir     摆动方向（不必单位化；会投影到垂直于焊缝的平面）
 * @param pattern      三角摆动或正弦摆动
 * @param amplitude    摆动幅度（侧向峰值偏移，与坐标同单位）
 * @param chordLength  弦长，一个完整摆动周期沿焊缝的长度，必须 > 0
 * @param sampleStep   沿焊缝方向的采样步长；<= 0 时自动取 chordLength / 32
 * @return 插值轨迹点，至少包含起点；可到达时包含终点
 *
 * @note 半周期结束时侧向偏移为 0，随后的直线段沿焊缝中心线到达终点。
 */
std::vector<Eigen::Vector3d> interpolateWeldWeave(
    const Eigen::Vector3d& start,
    const Eigen::Vector3d& end,
    const Eigen::Vector3d& weaveDir,
    WeavePattern pattern,
    double amplitude,
    double chordLength,
    double sampleStep = 0.0);

#endif // WELD_WEAVE_INTERPOLATION_H
