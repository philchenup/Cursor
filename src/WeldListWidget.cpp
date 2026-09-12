#include "WeldListWidget.h"

#include <QAbstractItemView>
#include <QAbstractSpinBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QEvent>
#include <QFont>
#include <QHeaderView>
#include <QList>
#include <QObject>
#include <QString>
#include <QTableWidgetItem>
#include <QVariant>
#include <QWidget>
#include <algorithm>
#include <cmath>

namespace {

constexpr int kRoleOriginalVec = Qt::UserRole;
constexpr double kEps = 1e-12;

enum WeldCol {
    ColIndex = 0,
    ColStart,
    ColEnd,
    ColInset,
    ColSpeed,
    ColWeaveMode,
    ColWeaveType,
    ColAmplitude,
    ColChord,
    ColMultiMode,
    ColThickness,
    ColGrooveAngle,
    ColFitUpGap,
    ColPenetration,
    ColCount
};

const char* kTableObjectName = "weldListTable";

QString vecText(const Eigen::Vector3d& v)
{
    return QString::fromUtf8("(%1, %2, %3)")
        .arg(v.x(), 0, 'f', 3)
        .arg(v.y(), 0, 'f', 3)
        .arg(v.z(), 0, 'f', 3);
}

QVariant vecToVar(const Eigen::Vector3d& v)
{
    return QVariant::fromValue(QList<QVariant>() << v.x() << v.y() << v.z());
}

Eigen::Vector3d varToVec(const QVariant& var)
{
    const QList<QVariant> list = var.toList();
    if (list.size() != 3) {
        return Eigen::Vector3d::Zero();
    }
    return Eigen::Vector3d(list[0].toDouble(), list[1].toDouble(), list[2].toDouble());
}

Eigen::Vector3d insetPoint(const Eigen::Vector3d& start,
                           const Eigen::Vector3d& end,
                           double inset,
                           bool fromStart)
{
    const Eigen::Vector3d seam = end - start;
    const double length = seam.norm();
    if (length <= kEps) {
        return fromStart ? start : end;
    }
    const double clamped = std::max(0.0, std::min(inset, 0.5 * length));
    const Eigen::Vector3d dir = seam / length;
    if (fromStart) {
        return start + clamped * dir;
    }
    return end - clamped * dir;
}

QDoubleSpinBox* makeSpin(QWidget* parent,
                         double minVal,
                         double maxVal,
                         int decimals,
                         const QString& suffix,
                         double value)
{
    auto* spin = new QDoubleSpinBox(parent);
    spin->setRange(minVal, maxVal);
    spin->setDecimals(decimals);
    spin->setSingleStep(decimals >= 2 ? 0.1 : 1.0);
    spin->setSuffix(suffix);
    spin->setValue(value);
    spin->setButtonSymbols(QAbstractSpinBox::NoButtons);
    spin->setAlignment(Qt::AlignCenter);
    spin->setFrame(false);
    spin->setMinimumHeight(22);
    return spin;
}

QComboBox* makeCombo(QWidget* parent, const QStringList& items)
{
    auto* combo = new QComboBox(parent);
    combo->addItems(items);
    combo->setFrame(false);
    combo->setMinimumHeight(22);
    return combo;
}

QTableWidgetItem* makeReadOnlyItem(const QString& text)
{
    auto* item = new QTableWidgetItem(text);
    item->setTextAlignment(Qt::AlignCenter);
    item->setFlags(item->flags() & ~Qt::ItemIsEditable);
    return item;
}

QDoubleSpinBox* spinAt(QTableWidget* table, int row, int col)
{
    return qobject_cast<QDoubleSpinBox*>(table->cellWidget(row, col));
}

QComboBox* comboAt(QTableWidget* table, int row, int col)
{
    return qobject_cast<QComboBox*>(table->cellWidget(row, col));
}

void fitColumnWidths(QTableWidget* table);

void applyInsetDisplay(QTableWidget* table, int row)
{
    QTableWidgetItem* startItem = table->item(row, ColStart);
    QTableWidgetItem* endItem = table->item(row, ColEnd);
    QDoubleSpinBox* insetSpin = spinAt(table, row, ColInset);
    if (!startItem || !endItem || !insetSpin) {
        return;
    }
    const Eigen::Vector3d start = varToVec(startItem->data(kRoleOriginalVec));
    const Eigen::Vector3d end = varToVec(endItem->data(kRoleOriginalVec));
    const double inset = insetSpin->value();
    startItem->setText(vecText(insetPoint(start, end, inset, true)));
    endItem->setText(vecText(insetPoint(start, end, inset, false)));
    startItem->setToolTip(QString::fromUtf8("原始起点 %1").arg(vecText(start)));
    endItem->setToolTip(QString::fromUtf8("原始终点 %1").arg(vecText(end)));
}

void updateRowEditors(QTableWidget* table, int row)
{
    QComboBox* weaveCombo = comboAt(table, row, ColWeaveMode);
    QComboBox* multiCombo = comboAt(table, row, ColMultiMode);
    const bool weaving = weaveCombo && weaveCombo->currentIndex() == 1;
    const bool multilayer = multiCombo && multiCombo->currentIndex() == 1;

    const int weaveCols[] = {ColWeaveType, ColAmplitude, ColChord};
    for (int col : weaveCols) {
        if (QWidget* w = table->cellWidget(row, col)) {
            w->setEnabled(weaving);
        }
    }
    const int multiCols[] = {ColThickness, ColGrooveAngle, ColFitUpGap, ColPenetration};
    for (int col : multiCols) {
        if (QWidget* w = table->cellWidget(row, col)) {
            w->setEnabled(multilayer);
        }
    }
}

void updateDynamicColumns(QTableWidget* table)
{
    bool anyWeave = false;
    bool anyMulti = false;
    for (int row = 0; row < table->rowCount(); ++row) {
        if (QComboBox* weave = comboAt(table, row, ColWeaveMode)) {
            anyWeave = anyWeave || (weave->currentIndex() == 1);
        }
        if (QComboBox* multi = comboAt(table, row, ColMultiMode)) {
            anyMulti = anyMulti || (multi->currentIndex() == 1);
        }
        updateRowEditors(table, row);
    }
    table->setColumnHidden(ColWeaveType, !anyWeave);
    table->setColumnHidden(ColAmplitude, !anyWeave);
    table->setColumnHidden(ColChord, !anyWeave);
    table->setColumnHidden(ColThickness, !anyMulti);
    table->setColumnHidden(ColGrooveAngle, !anyMulti);
    table->setColumnHidden(ColFitUpGap, !anyMulti);
    table->setColumnHidden(ColPenetration, !anyMulti);
    fitColumnWidths(table);
}

void fitColumnWidths(QTableWidget* table)
{
    if (!table || table->property("_fittingColumns").toBool()) {
        return;
    }
    table->setProperty("_fittingColumns", true);

    QHeaderView* header = table->horizontalHeader();
    header->setStretchLastSection(false);
    for (int col = 0; col < table->columnCount(); ++col) {
        if (!table->isColumnHidden(col)) {
            header->setSectionResizeMode(col, QHeaderView::ResizeToContents);
        }
    }
    table->resizeColumnsToContents();

    int total = 0;
    for (int col = 0; col < table->columnCount(); ++col) {
        if (table->isColumnHidden(col)) {
            continue;
        }
        header->setSectionResizeMode(col, QHeaderView::Interactive);
        total += header->sectionSize(col);
    }

    const int viewW = table->viewport()->width();
    if (viewW > 0 && total > 0 && total < viewW) {
        int used = 0;
        int lastVisible = -1;
        for (int col = 0; col < table->columnCount(); ++col) {
            if (table->isColumnHidden(col)) {
                continue;
            }
            lastVisible = col;
            const int w = qMax(header->minimumSectionSize(),
                               qRound(header->sectionSize(col) * (static_cast<double>(viewW) / total)));
            header->resizeSection(col, w);
            used += w;
        }
        if (lastVisible >= 0 && used != viewW) {
            header->resizeSection(lastVisible,
                                  qMax(header->minimumSectionSize(),
                                       header->sectionSize(lastVisible) + (viewW - used)));
        }
    }

    table->setProperty("_fittingColumns", false);
}

class WeldTableResizeFilter : public QObject
{
public:
    using QObject::QObject;

    bool eventFilter(QObject* watched, QEvent* event) override
    {
        if (event->type() == QEvent::Resize) {
            if (auto* table = qobject_cast<QTableWidget*>(watched)) {
                fitColumnWidths(table);
            }
        }
        return QObject::eventFilter(watched, event);
    }
};

void applyTableStyle(QTableWidget* table)
{
    QFont font;
    font.setFamily(QString::fromUtf8("微软雅黑"));
    font.setPointSize(9);
    font.setStyleHint(QFont::SansSerif);
    table->setFont(font);
    table->horizontalHeader()->setFont(font);
    table->verticalHeader()->setFont(font);

    table->verticalHeader()->setVisible(false);
    table->setSelectionBehavior(QAbstractItemView::SelectRows);
    table->setSelectionMode(QAbstractItemView::SingleSelection);
    table->setAlternatingRowColors(true);
    table->setShowGrid(true);
    table->setEditTriggers(QAbstractItemView::NoEditTriggers);
    table->setFocusPolicy(Qt::StrongFocus);
    table->setWordWrap(false);
    table->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOn);
    table->setHorizontalScrollMode(QAbstractItemView::ScrollPerPixel);

    QHeaderView* header = table->horizontalHeader();
    header->setHighlightSections(false);
    header->setDefaultAlignment(Qt::AlignCenter);
    header->setMinimumSectionSize(56);
    header->setStretchLastSection(false);
    header->setSectionResizeMode(QHeaderView::Interactive);
    table->verticalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
    fitColumnWidths(table);
}

QTableWidget* createWeldTable(QDockWidget* dock)
{
    auto* table = new QTableWidget(dock);
    table->setObjectName(QLatin1String(kTableObjectName));
    table->setColumnCount(ColCount);
    table->setHorizontalHeaderLabels({
        QString::fromUtf8("序号"),
        QString::fromUtf8("起点"),
        QString::fromUtf8("终点"),
        QString::fromUtf8("内缩"),
        QString::fromUtf8("焊接速度"),
        QString::fromUtf8("摆动方式"),
        QString::fromUtf8("摆动类型"),
        QString::fromUtf8("幅度"),
        QString::fromUtf8("弦长"),
        QString::fromUtf8("多层多道"),
        QString::fromUtf8("板厚"),
        QString::fromUtf8("坡口角度"),
        QString::fromUtf8("装配间隙"),
        QString::fromUtf8("熔深"),
    });
    applyTableStyle(table);
    table->setColumnHidden(ColWeaveType, true);
    table->setColumnHidden(ColAmplitude, true);
    table->setColumnHidden(ColChord, true);
    table->setColumnHidden(ColThickness, true);
    table->setColumnHidden(ColGrooveAngle, true);
    table->setColumnHidden(ColFitUpGap, true);
    table->setColumnHidden(ColPenetration, true);
    table->installEventFilter(new WeldTableResizeFilter(table));
    dock->setWidget(table);
    return table;
}

void addWeldRow(QTableWidget* table, const Eigen::Vector3d& start, const Eigen::Vector3d& end)
{
    const int row = table->rowCount();
    table->insertRow(row);

    table->setItem(row, ColIndex, makeReadOnlyItem(QString::number(row)));

    auto* startItem = makeReadOnlyItem(vecText(start));
    startItem->setData(kRoleOriginalVec, vecToVar(start));
    table->setItem(row, ColStart, startItem);

    auto* endItem = makeReadOnlyItem(vecText(end));
    endItem->setData(kRoleOriginalVec, vecToVar(end));
    table->setItem(row, ColEnd, endItem);

    QDoubleSpinBox* insetSpin = makeSpin(table, 0.0, 1.0e6, 2, QString::fromUtf8(" mm"), 0.0);
    QDoubleSpinBox* speedSpin = makeSpin(table, 0.0, 1.0e5, 1, QString::fromUtf8(" mm/s"), 10.0);
    QDoubleSpinBox* ampSpin = makeSpin(table, 0.0, 1.0e4, 2, QString::fromUtf8(" mm"), 5.0);
    QDoubleSpinBox* chordSpin = makeSpin(table, 0.0, 1.0e5, 2, QString::fromUtf8(" mm"), 20.0);
    QDoubleSpinBox* thickSpin = makeSpin(table, 0.0, 1.0e4, 2, QString::fromUtf8(" mm"), 0.0);
    QDoubleSpinBox* grooveSpin = makeSpin(table, 0.0, 90.0, 1, QString::fromUtf8(" °"), 0.0);
    QDoubleSpinBox* gapSpin = makeSpin(table, 0.0, 1.0e3, 2, QString::fromUtf8(" mm"), 0.0);
    QDoubleSpinBox* penSpin = makeSpin(table, 0.0, 1.0e4, 2, QString::fromUtf8(" mm"), 0.0);

    QComboBox* weaveCombo = makeCombo(
        table, {QString::fromUtf8("直线焊"), QString::fromUtf8("摆动焊")});
    QComboBox* weaveTypeCombo = makeCombo(
        table, {QString::fromUtf8("正弦"), QString::fromUtf8("三角")});
    QComboBox* multiCombo = makeCombo(
        table, {QString::fromUtf8("单层单道"), QString::fromUtf8("多层多道")});

    const QWidgetList editors = {insetSpin, speedSpin, weaveCombo, weaveTypeCombo,
                                 ampSpin, chordSpin, multiCombo, thickSpin, grooveSpin,
                                 gapSpin, penSpin};
    for (QWidget* editor : editors) {
        editor->setFont(table->font());
    }

    table->setCellWidget(row, ColInset, insetSpin);
    table->setCellWidget(row, ColSpeed, speedSpin);
    table->setCellWidget(row, ColWeaveMode, weaveCombo);
    table->setCellWidget(row, ColWeaveType, weaveTypeCombo);
    table->setCellWidget(row, ColAmplitude, ampSpin);
    table->setCellWidget(row, ColChord, chordSpin);
    table->setCellWidget(row, ColMultiMode, multiCombo);
    table->setCellWidget(row, ColThickness, thickSpin);
    table->setCellWidget(row, ColGrooveAngle, grooveSpin);
    table->setCellWidget(row, ColFitUpGap, gapSpin);
    table->setCellWidget(row, ColPenetration, penSpin);

    QObject::connect(insetSpin,
                     static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),
                     table,
                     [table, insetSpin](double) {
                         for (int r = 0; r < table->rowCount(); ++r) {
                             if (table->cellWidget(r, ColInset) == insetSpin) {
                                 applyInsetDisplay(table, r);
                                 break;
                             }
                         }
                     });
    QObject::connect(weaveCombo,
                     static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged),
                     table,
                     [table](int) { updateDynamicColumns(table); });
    QObject::connect(multiCombo,
                     static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged),
                     table,
                     [table](int) { updateDynamicColumns(table); });

    applyInsetDisplay(table, row);
    updateDynamicColumns(table);
}

}  // namespace

QTableWidget* setupWeldListWidget(
    QDockWidget* weldListWidget,
    const std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>& seams)
{
    if (!weldListWidget) {
        return nullptr;
    }

    QTableWidget* table = weldListWidget->findChild<QTableWidget*>(QLatin1String(kTableObjectName));
    if (!table) {
        table = createWeldTable(weldListWidget);
    }

    for (const auto& seam : seams) {
        addWeldRow(table, seam.first, seam.second);
    }
    return table;
}
