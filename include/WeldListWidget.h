#ifndef WELD_LIST_WIDGET_H
#define WELD_LIST_WIDGET_H

#include <Eigen/Dense>

#include <QDockWidget>
#include <QTableWidget>
#include <utility>
#include <vector>

/**
 * @brief 在名为 weldListWidget 的 QDockWidget 中插入焊缝工艺表。
 *
 * 首次调用会创建 QTableWidget 并放入该 Dock；之后再调用且传入 seams
 * 时，按起点/终点追加行。列宽按内容适配，总宽超出时出现横向滚动条；
 * 空间有余时再拉满列表宽度。字体为微软雅黑 9 号。
 *
 * 列：序号、起点、终点、内缩、焊接速度、摆动方式、[幅度、弦长]、
 * 多层多道、[板厚、坡口角度、装配间隙、熔深]。
 * 幅度/弦长仅在存在“摆动焊”行时显示；多层四列仅在存在“多层多道”行时显示。
 *
 * 内缩：沿焊缝方向从原起点、原终点各收回给定长度（mm），并刷新起终点显示。
 *
 * @param weldListWidget 已有的 Dock（objectName 建议为 weldListWidget）
 * @param seams          可选，每项为 (起点, 终点) Eigen::Vector3d，单位与界面一致
 * @return Dock 内的工艺表；weldListWidget 为空时返回 nullptr
 */
QTableWidget* setupWeldListWidget(
    QDockWidget* weldListWidget,
    const std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>& seams = {});

#endif // WELD_LIST_WIDGET_H
