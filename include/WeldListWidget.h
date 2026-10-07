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
 * @brief 追加到焊缝表的一条焊缝。
 *
 * vertical 为 true 时，新行的焊接位置设为立焊，并显示行走角。
 * 为 false 时保持平焊。
 */
struct WeldListSeam {
    Eigen::Vector3d start = Eigen::Vector3d::Zero();
    Eigen::Vector3d end = Eigen::Vector3d::Zero();
    bool vertical = false;

    WeldListSeam() = default;
    WeldListSeam(const Eigen::Vector3d& startIn,
        const Eigen::Vector3d& endIn,
        bool verticalIn = false)
        : start(startIn)
        , end(endIn)
        , vertical(verticalIn)
    {
    }
};

enum class WeldWeaveMode {
    Straight = 0,
    Weave = 1
};

enum class WeldWeaveType {
    Sine = 0,
    Triangle = 1
};

enum class WeldPosition {
    Flat = 0,
    Vertical = 1,
    Horizontal = 2
};

enum class WeldContinuity {
    Continuous = 0,
    Intermittent = 1
};

enum class WeldLayerMode {
    Single = 0,
    Multi = 1
};

/**
 * @brief 高亮行的全部工艺类型和数值。
 *
 * valid 为 false、row 为 -1 时表示没有高亮行。
 * 扩展参数在对应工艺未启用时仍保留单元格里的数值，由类型字段决定是否使用：
 * 直线焊不使用摆动参数，平焊和横焊不使用行走角，
 * 连续焊不使用焊段长度和净距，单层单道不使用多层参数。
 */
struct WeldRowData {
    bool valid = false;
    int row = -1;
    bool included = false;
    Eigen::Vector3d start = Eigen::Vector3d::Zero();
    Eigen::Vector3d end = Eigen::Vector3d::Zero();
    double speed = 0.0;
    WeldWeaveMode weaveMode = WeldWeaveMode::Straight;
    WeldWeaveType weaveType = WeldWeaveType::Sine;
    double amplitude = 0.0;
    double chord = 0.0;
    WeldPosition position = WeldPosition::Flat;
    double travelAngle = 0.0;
    WeldContinuity continuity = WeldContinuity::Continuous;
    double segmentLength = 0.0;
    double clearDistance = 0.0;
    WeldLayerMode layerMode = WeldLayerMode::Single;
    double thickness = 0.0;
    double grooveAngle = 0.0;
    double fitUpGap = 0.0;
    double penetration = 0.0;
};

Q_DECLARE_METATYPE(WeldRowData)

using WeldRowDataList = std::vector<WeldRowData>;
Q_DECLARE_METATYPE(WeldRowDataList)

/**
 * @brief 高亮焊缝行时的回调。
 *
 * 这里的选中是表格行高亮，与第二列“选中”勾选无关。
 * weld.valid 为 true：该行全部类型和数值已填入结构体。
 * weld.valid 为 false：没有高亮行（点击表格空白处取消高亮，或列表被清空）。
 */
using WeldSelectionCallback = std::function<void(const WeldRowData& weld)>;

/**
 * @brief 焊缝表的高亮信号。
 *
 * weldSelected：高亮某行时发出，参数为该行全部类型和数值。
 * blankClicked：单击表格空白处取消高亮时发出。
 *
 * 主窗口接收示例：
 * @code
 * connect(weldListSignals(table), &WeldListSignals::weldSelected,
 *         this, &MainWindow::onWeldSelected);
 * connect(weldListSignals(table), &WeldListSignals::runCheck,
 *         this, &MainWindow::onRunCheck);
 * connect(weldListSignals(table), &WeldListSignals::runAll,
 *         this, &MainWindow::onRunAll);
 * @endcode
 */
class WeldListSignals : public QObject
{
    Q_OBJECT
public:
    explicit WeldListSignals(QTableWidget* table);

    void setCallback(WeldSelectionCallback callback);
    void notify();
    void notifyBlankClick();
    void emitRunCheck();
    void emitRunAll();

signals:
    void weldSelected(const WeldRowData& weld);
    void blankClicked();
    void runCheck();
    void runAll(const WeldRowDataList& welds);

private:
    QTableWidget* table_ = nullptr;
    WeldSelectionCallback callback_;
    bool notifying_ = false;
    bool hasLast_ = false;
    WeldRowData last_;
};

/**
 * @brief 在名为 weldListWidget 的 QDockWidget 中插入焊缝工艺表。
 *
 * 首次调用会创建 QTableWidget 并放入该 Dock；之后再调用且传入 seams
 * 时，按起点/终点追加行。列宽随 Dock 表格区域拉伸；字体为微软雅黑 9 号。
 *
 * 列：序号、选中、起点、终点、焊接速度、摆动方式、[摆动类型、幅度、弦长]、
 * 焊接位置、[行走角]、连续/间断、[焊段长度、净距]、
 * 多层多道、[板厚、坡口角度、装配间隙、熔深]。
 * 摆动类型/幅度/弦长仅在存在“摆动焊”行时显示；行走角仅在存在“立焊”行时显示；
 * 焊段长度/净距仅在存在“间断焊”行时显示；多层四列仅在存在“多层多道”行时显示。
 * 这些扩展列出现后，没选对应工艺的行单元格为空：直线焊不显示摆动参数，
 * 平焊和横焊不显示行走角，连续焊不显示焊段长度和净距，单层单道不显示多层参数。
 *
 * 第二列是选中勾选框，默认勾选。全流程焊接只应处理勾选行，
 * 用 weldRowIncluded / includedWeldRows 过滤不需要的焊缝。勾选与行高亮相互独立。
 * 表格左上方有一个全选按钮，在全选和取消全选之间切换，图标为资源库中的
 * check.svg / uncheck.svg。旁边是“运行选中”和“全部运行”：
 * 运行选中发出 runCheck()；全部运行把所有勾选行打包为 WeldRowDataList 后发出 runAll。
 *
 * 单击一行（含单元格里的编辑控件）会高亮该行，并发出
 * WeldListSignals::weldSelected(weld)，weld 为该行全部类型和数值。
 * 单击表格空白处取消高亮，并发出 WeldListSignals::blankClicked()。
 *
 * 追加行时用 vertical 标记是否立焊：
 * @code
 * setupWeldListWidget(ui->weldListWidget, {
 *     { seam.first.cast<double>(), seam.second.cast<double>(), vertical }
 * });
 * @endcode
 *
 * @param weldListWidget 已有的 Dock（objectName 建议为 weldListWidget）
 * @param seams          可选，每项含起点、终点，以及是否立焊
 * @return Dock 内的工艺表；weldListWidget 为空时返回 nullptr
 */
QTableWidget* setupWeldListWidget(
    QDockWidget* weldListWidget,
    const std::vector<WeldListSeam>& seams = {});

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
 * @brief 读取某一行的全部类型和数值。
 * 行号无效或 table 为空时返回 false，out.valid 为 false。
 */
bool weldRowData(const QTableWidget* table, int row, WeldRowData& out);

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
