#include "WeldListWidget.h"

#include <QDockWidget>
#include <QObject>

/**
 * @brief 主窗口槽函数。高亮某行时接收该行全部类型和数值。
 *
 * 在主窗口中声明：
 *   void onWeldSelected(const WeldRowData& weld);
 * 建表后连接一次：
 *   connect(weldListSignals(table), &WeldListSignals::weldSelected,
 *           this, &MainWindow::onWeldSelected);
 *   connect(weldListSignals(table), &WeldListSignals::runCheck,
 *           this, &MainWindow::onRunCheck);
 *   connect(weldListSignals(table), &WeldListSignals::runAll,
 *           this, &MainWindow::onRunAll);
 *
 * void MainWindow::onRunCheck();
 * void MainWindow::onRunAll(const WeldRowDataList& welds);
 */
void onWeldSelected(const WeldRowData& weld)
{
    if (!weld.valid) {
        return;
    }

    const bool vertical = weld.position == WeldPosition::Vertical;
    const double travelAngle = vertical ? weld.travelAngle : 0.0;
    const bool intermittent = weld.continuity == WeldContinuity::Intermittent;
    (void)travelAngle;
    (void)intermittent;
    (void)weld.included;
    (void)weld.speed;
}

/**
 * @brief 按起点、终点追加一行，并用 vertical 指定是否立焊。
 *
 * 调用形式：
 *   setupWeldListWidget(ui->weldListWidget, {
 *       { seam.first.cast<double>(), seam.second.cast<double>(), vertical }
 *   });
 */
QTableWidget* appendWeldSeam(QDockWidget* weldListWidget,
    const Eigen::Vector3d& start,
    const Eigen::Vector3d& end,
    bool vertical)
{
    return setupWeldListWidget(weldListWidget, { { start, end, vertical } });
}
