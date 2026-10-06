#ifndef WELD_LIST_WIDGET_H
#define WELD_LIST_WIDGET_H

#include <Eigen/Dense>

#include <QDockWidget>
#include <QTableWidget>
#include <functional>
#include <utility>
#include <vector>

/**
 * @brief 选中焊缝行时的回调。
 *
 * row >= 0：该行当前起点、终点已转换为 Eigen::Vector3d（与表格“起点”“终点”
 * 列一致，已计入内缩，精度高于单元格里保留 3 位小数的文本）。
 * row < 0：没有选中行（点击表格空白处取消选中，或列表被清空）。此时 start、end 为零向量。
 */
using WeldSelectionCallback = std::function<void(
    int row,
    const Eigen::Vector3d& start,
    const Eigen::Vector3d& end)>;

/**
 * @brief 在名为 weldListWidget 的 QDockWidget 中插入焊缝工艺表。
 *
 * 首次调用会创建 QTableWidget 并放入该 Dock；之后再调用且传入 seams
 * 时，按起点/终点追加行。列宽随 Dock 表格区域拉伸；字体为微软雅黑 9 号。
 * 表格上方有“清空”按钮，用于删除全部焊缝行。
 *
 * 列：序号、起点、终点、内缩、焊接速度、摆动方式、[幅度、弦长]、
 * 多层多道、[板厚、坡口角度、装配间隙、熔深]。
 * 幅度/弦长仅在存在“摆动焊”行时显示；多层四列仅在存在“多层多道”行时显示。
 *
 * 内缩：沿焊缝方向从原起点、原终点各收回给定长度（mm），并刷新起终点显示。
 * 单击一行（含单元格里的编辑控件）会选中该行，可通过
 * selectedWeldEndpoints() 或 setWeldSelectionCallback() 读到起点和终点。
 * 单击表格空白处取消选中。
 *
 * @param weldListWidget 已有的 Dock（objectName 建议为 weldListWidget）
 * @param seams          可选，每项为 (起点, 终点) Eigen::Vector3d，单位与界面一致
 * @return Dock 内的工艺表；weldListWidget 为空时返回 nullptr
 */
QTableWidget* setupWeldListWidget(
    QDockWidget* weldListWidget,
    const std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>& seams = {});

/**
 * @brief 清空焊缝表的全部数据行，并取消选中。
 * 表头和列结构保留。table 为空时不做任何事。
 */
void clearWeldList(QTableWidget* table);

/**
 * @brief 读取某一行当前起点、终点并转换为 Eigen::Vector3d。
 *
 * 返回值与“起点”“终点”列显示的坐标一致（已按内缩收过），但保留完整双精度。
 * 行号无效或单元格缺失时返回 false，start、end 置为零向量。
 */
bool weldRowEndpoints(const QTableWidget* table,
    int row,
    Eigen::Vector3d& start,
    Eigen::Vector3d& end);

/**
 * @brief 读取当前选中行的起点、终点。没有选中行时返回 false。
 */
bool selectedWeldEndpoints(const QTableWidget* table,
    Eigen::Vector3d& start,
    Eigen::Vector3d& end);

/**
 * @brief 注册选中变化回调。传入空 callback 表示取消注册。
 * 注册后会立刻用当前选中状态回调一次。table 为空时不做任何事。
 */
void setWeldSelectionCallback(QTableWidget* table, WeldSelectionCallback callback);

#endif // WELD_LIST_WIDGET_H
