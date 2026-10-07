#ifndef WELD_LIST_WIDGET_H
#define WELD_LIST_WIDGET_H

#include <Eigen/Dense>

#include <QDockWidget>
#include <QObject>
#include <QTableWidget>
#include <functional>
#include <utility>
#include <vector>

Q_DECLARE_METATYPE(Eigen::Vector3d)

/**
 * @brief 高亮焊缝行时的回调。
 *
 * 这里的选中是表格行高亮，与第二列“选中”勾选无关。
 * row >= 0：该行起点、终点已转换为 Eigen::Vector3d（与表格“起点”“终点”
 * 列一致，精度高于单元格里保留 3 位小数的文本）。
 * row < 0：没有高亮行（点击表格空白处取消高亮，或列表被清空）。此时 start、end 为零向量。
 */
using WeldSelectionCallback = std::function<void(
    int row,
    const Eigen::Vector3d& start,
    const Eigen::Vector3d& end)>;

/**
 * @brief 焊缝表的高亮信号。
 *
 * weldSelected：高亮某行时发出，参数为该行起点、终点。
 * blankClicked：单击表格空白处取消高亮时发出。
 */
class WeldListSignals : public QObject
{
    Q_OBJECT
public:
    explicit WeldListSignals(QTableWidget* table);

    void setCallback(WeldSelectionCallback callback);
    void notify();
    void notifyBlankClick();

signals:
    void weldSelected(const Eigen::Vector3d& start, const Eigen::Vector3d& end);
    void blankClicked();

private:
    QTableWidget* table_ = nullptr;
    WeldSelectionCallback callback_;
    bool notifying_ = false;
    bool hasLast_ = false;
    int lastRow_ = -1;
    Eigen::Vector3d lastStart_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d lastEnd_ = Eigen::Vector3d::Zero();
};

/**
 * @brief 在名为 weldListWidget 的 QDockWidget 中插入焊缝工艺表。
 *
 * 首次调用会创建 QTableWidget 并放入该 Dock；之后再调用且传入 seams
 * 时，按起点/终点追加行。列宽随 Dock 表格区域拉伸；字体为微软雅黑 9 号。
 *
 * 列：序号、选中、起点、终点、焊接速度、摆动方式、[摆动类型、幅度、弦长]、
 * 多层多道、[板厚、坡口角度、装配间隙、熔深]。
 * 摆动类型/幅度/弦长仅在存在“摆动焊”行时显示；多层四列仅在存在“多层多道”行时显示。
 * 这些扩展列出现后，没选对应工艺的行单元格为空：直线焊不显示摆动参数，
 * 单层单道不显示多层参数。
 *
 * 第二列是选中勾选框，默认勾选。全流程焊接只应处理勾选行，
 * 用 weldRowIncluded / includedWeldRows 过滤不需要的焊缝。勾选与行高亮相互独立。
 *
 * 单击一行（含单元格里的编辑控件）会高亮该行，并发出
 * WeldListSignals::weldSelected(start, end)。
 * 单击表格空白处取消高亮，并发出 WeldListSignals::blankClicked()。
 *
 * @param weldListWidget 已有的 Dock（objectName 建议为 weldListWidget）
 * @param seams          可选，每项为 (起点, 终点) Eigen::Vector3d，单位与界面一致
 * @return Dock 内的工艺表；weldListWidget 为空时返回 nullptr
 */
QTableWidget* setupWeldListWidget(
    QDockWidget* weldListWidget,
    const std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>& seams = {});

/**
 * @brief 清空焊缝表的全部数据行，并取消高亮。
 * 表头和列结构保留。table 为空时不做任何事。
 */
void clearWeldList(QTableWidget* table);

/**
 * @brief 读取某一行当前起点、终点并转换为 Eigen::Vector3d。
 *
 * 返回值与“起点”“终点”列显示的坐标一致，但保留完整双精度。
 * 行号无效或单元格缺失时返回 false，start、end 置为零向量。
 */
bool weldRowEndpoints(const QTableWidget* table,
    int row,
    Eigen::Vector3d& start,
    Eigen::Vector3d& end);

/**
 * @brief 读取当前高亮行的起点、终点。没有高亮行时返回 false。
 */
bool selectedWeldEndpoints(const QTableWidget* table,
    Eigen::Vector3d& start,
    Eigen::Vector3d& end);

/**
 * @brief 该行第二列是否勾选，即是否纳入全流程焊接。
 * 未勾选的焊缝应在后续流程中过滤掉。行号无效或 table 为空时返回 false。
 */
bool weldRowIncluded(const QTableWidget* table, int row);

/**
 * @brief 设置第二列勾选状态。行号无效或 table 为空时不做任何事。
 */
void setWeldRowIncluded(QTableWidget* table, int row, bool included);

/**
 * @brief 所有勾选纳入全流程焊接的行号，按行号升序。
 * table 为空时返回空列表。
 */
std::vector<int> includedWeldRows(const QTableWidget* table);

/**
 * @brief 注册高亮变化回调。传入空 callback 表示取消注册。
 * 注册后会立刻用当前高亮状态回调一次。table 为空时不做任何事。
 */
void setWeldSelectionCallback(QTableWidget* table, WeldSelectionCallback callback);

/**
 * @brief 取表格上的信号对象，用于 connect 起点/终点和空白点击。
 * table 不是焊缝表时返回 nullptr。
 */
WeldListSignals* weldListSignals(QTableWidget* table);

#endif // WELD_LIST_WIDGET_H
