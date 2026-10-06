#include "WeldListWidget.h"

#include <QAbstractItemView>
#include <QAbstractSpinBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QEvent>
#include <QFont>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QItemSelectionModel>
#include <QLineEdit>
#include <QList>
#include <QMouseEvent>
#include <QObject>
#include <QPushButton>
#include <QSignalBlocker>
#include <QString>
#include <QStringList>
#include <QTableWidgetItem>
#include <QVBoxLayout>
#include <QVariant>
#include <QWidget>
#include <algorithm>
#include <cmath>

namespace {

    constexpr int kRoleOriginalVec = Qt::UserRole;
    constexpr int kRoleCurrentVec = Qt::UserRole + 1;
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
    const char* kNotifierObjectName = "weldListNotifier";
    const char* kFilterObjectName = "weldTableEventFilter";
    const char* kClearButtonObjectName = "weldListClearButton";
    const char* kClearingProperty = "_clearing";

    QString vecText(const Eigen::Vector3d& v)
    {
        return QStringLiteral("(%1, %2, %3)")
            .arg(v.x(), 0, 'f', 3)
            .arg(v.y(), 0, 'f', 3)
            .arg(v.z(), 0, 'f', 3);
    }

    QVariant vecToVar(const Eigen::Vector3d& v)
    {
        return QVariant::fromValue(QList<QVariant>() << v.x() << v.y() << v.z());
    }

    bool variantToVec(const QVariant& var, Eigen::Vector3d& out)
    {
        const QList<QVariant> list = var.toList();
        if (list.size() != 3) {
            return false;
        }
        out = Eigen::Vector3d(list[0].toDouble(), list[1].toDouble(), list[2].toDouble());
        return true;
    }

    Eigen::Vector3d varToVec(const QVariant& var)
    {
        Eigen::Vector3d out = Eigen::Vector3d::Zero();
        variantToVec(var, out);
        return out;
    }

    bool parseVecText(const QString& text, Eigen::Vector3d& out)
    {
        QString body = text.trimmed();
        if (body.startsWith(QLatin1Char('(')) && body.endsWith(QLatin1Char(')'))) {
            body = body.mid(1, body.size() - 2);
        }
        const QStringList parts = body.split(QLatin1Char(','));
        if (parts.size() != 3) {
            return false;
        }
        bool okX = false;
        bool okY = false;
        bool okZ = false;
        const double x = parts[0].trimmed().toDouble(&okX);
        const double y = parts[1].trimmed().toDouble(&okY);
        const double z = parts[2].trimmed().toDouble(&okZ);
        if (!okX || !okY || !okZ) {
            return false;
        }
        out = Eigen::Vector3d(x, y, z);
        return true;
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

    QPoint mousePosition(const QMouseEvent* mouse)
    {
#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
        return mouse->position().toPoint();
#else
        return mouse->pos();
#endif
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

    QDoubleSpinBox* spinAt(const QTableWidget* table, int row, int col)
    {
        return qobject_cast<QDoubleSpinBox*>(table->cellWidget(row, col));
    }

    QComboBox* comboAt(const QTableWidget* table, int row, int col)
    {
        return qobject_cast<QComboBox*>(table->cellWidget(row, col));
    }

    void fitColumnWidths(QTableWidget* table);

    class WeldListNotifier : public QObject
    {
    public:
        explicit WeldListNotifier(QTableWidget* table)
            : QObject(table)
            , table_(table)
        {
            setObjectName(QLatin1String(kNotifierObjectName));
        }

        WeldSelectionCallback callback;

        void setCallback(WeldSelectionCallback next)
        {
            callback = std::move(next);
            hasLast_ = false;
            notify();
        }

        void notify()
        {
            if (!callback || !table_ || notifying_) {
                return;
            }

            int row = -1;
            Eigen::Vector3d start = Eigen::Vector3d::Zero();
            Eigen::Vector3d end = Eigen::Vector3d::Zero();
            if (const QItemSelectionModel* selection = table_->selectionModel()) {
                if (selection->hasSelection()) {
                    const QModelIndexList rows = selection->selectedRows();
                    if (!rows.isEmpty()) {
                        const int selected = rows.first().row();
                        if (weldRowEndpoints(table_, selected, start, end)) {
                            row = selected;
                        } else {
                            start.setZero();
                            end.setZero();
                        }
                    }
                }
            }

            if (hasLast_ && row == lastRow_
                && start.isApprox(lastStart_) && end.isApprox(lastEnd_)) {
                return;
            }
            hasLast_ = true;
            lastRow_ = row;
            lastStart_ = start;
            lastEnd_ = end;

            struct NotifyGuard {
                bool& flag;
                explicit NotifyGuard(bool& value)
                    : flag(value)
                {
                    flag = true;
                }
                ~NotifyGuard()
                {
                    flag = false;
                }
            } guard(notifying_);
            callback(row, start, end);
        }

    private:
        QTableWidget* table_ = nullptr;
        bool notifying_ = false;
        bool hasLast_ = false;
        int lastRow_ = -1;
        Eigen::Vector3d lastStart_ = Eigen::Vector3d::Zero();
        Eigen::Vector3d lastEnd_ = Eigen::Vector3d::Zero();
    };

    WeldListNotifier* notifierOf(QTableWidget* table)
    {
        if (!table) {
            return nullptr;
        }
        return static_cast<WeldListNotifier*>(
            table->findChild<QObject*>(QLatin1String(kNotifierObjectName), Qt::FindDirectChildrenOnly));
    }

    void notifySelection(QTableWidget* table)
    {
        if (WeldListNotifier* notifier = notifierOf(table)) {
            notifier->notify();
        }
    }

    void syncClearButton(QTableWidget* table)
    {
        if (!table) {
            return;
        }
        QWidget* panel = table->parentWidget();
        if (!panel) {
            return;
        }
        if (auto* button = panel->findChild<QPushButton*>(QLatin1String(kClearButtonObjectName))) {
            button->setEnabled(table->rowCount() > 0);
        }
    }

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
        const Eigen::Vector3d shownStart = insetPoint(start, end, inset, true);
        const Eigen::Vector3d shownEnd = insetPoint(start, end, inset, false);
        startItem->setText(vecText(shownStart));
        endItem->setText(vecText(shownEnd));
        startItem->setData(kRoleCurrentVec, vecToVar(shownStart));
        endItem->setData(kRoleCurrentVec, vecToVar(shownEnd));
        startItem->setToolTip(QStringLiteral("原始起点 %1").arg(vecText(start)));
        endItem->setToolTip(QStringLiteral("原始终点 %1").arg(vecText(end)));
    }

    void updateRowEditors(QTableWidget* table, int row)
    {
        QComboBox* weaveCombo = comboAt(table, row, ColWeaveMode);
        QComboBox* multiCombo = comboAt(table, row, ColMultiMode);
        const bool weaving = weaveCombo && weaveCombo->currentIndex() == 1;
        const bool multilayer = multiCombo && multiCombo->currentIndex() == 1;

        const int weaveCols[] = { ColWeaveType, ColAmplitude, ColChord };
        for (int col : weaveCols) {
            if (QWidget* w = table->cellWidget(row, col)) {
                w->setEnabled(weaving);
            }
        }
        const int multiCols[] = { ColThickness, ColGrooveAngle, ColFitUpGap, ColPenetration };
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

    class WeldTableEventFilter : public QObject
    {
    public:
        using QObject::QObject;

        bool eventFilter(QObject* watched, QEvent* event) override
        {
            QTableWidget* table = nullptr;
            for (QObject* obj = watched; obj; obj = obj->parent()) {
                table = qobject_cast<QTableWidget*>(obj);
                if (table) {
                    break;
                }
            }
            if (!table) {
                return false;
            }

            if (event->type() == QEvent::Resize && watched == table) {
                fitColumnWidths(table);
                return false;
            }

            if (event->type() != QEvent::MouseButtonPress) {
                return false;
            }
            auto* mouse = static_cast<QMouseEvent*>(event);
            if (mouse->button() != Qt::LeftButton) {
                return false;
            }

            const QPoint localPos = mousePosition(mouse);
            QPoint viewportPos = localPos;
            if (watched != table->viewport()) {
                auto* widget = qobject_cast<QWidget*>(watched);
                if (!widget) {
                    return false;
                }
                viewportPos = table->viewport()->mapFromGlobal(widget->mapToGlobal(localPos));
            }

            const QModelIndex index = table->indexAt(viewportPos);
            if (!index.isValid()) {
                if (watched == table->viewport()) {
                    table->setFocus(Qt::MouseFocusReason);
                    if (QItemSelectionModel* selection = table->selectionModel()) {
                        selection->clear();
                    }
                    return true;
                }
                return false;
            }

            // 单元格里的数值框、下拉框会吃掉鼠标事件，这里补上整行选中。
            if (watched != table->viewport()
                && table->selectionModel()
                && !table->selectionModel()->isRowSelected(index.row(), QModelIndex())) {
                table->selectRow(index.row());
            }
            return false;
        }
    };

    void watchEditor(QTableWidget* table, QWidget* editor)
    {
        QObject* filterObject = table->findChild<QObject*>(
            QLatin1String(kFilterObjectName), Qt::FindDirectChildrenOnly);
        if (!filterObject || !editor) {
            return;
        }
        editor->installEventFilter(filterObject);
        if (auto* spin = qobject_cast<QAbstractSpinBox*>(editor)) {
            if (auto* edit = spin->findChild<QLineEdit*>()) {
                edit->installEventFilter(filterObject);
            }
        }
    }

    void applyTableStyle(QTableWidget* table)
    {
        QFont font;
        font.setFamily(QStringLiteral("微软雅黑"));
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
        auto* panel = new QWidget(dock);
        panel->setObjectName(QStringLiteral("weldListPanel"));
        auto* layout = new QVBoxLayout(panel);
        layout->setContentsMargins(0, 0, 0, 0);
        layout->setSpacing(0);

        auto* table = new QTableWidget(panel);
        table->setObjectName(QLatin1String(kTableObjectName));
        table->setColumnCount(ColCount);
        table->setHorizontalHeaderLabels({
            QStringLiteral("序号"),
            QStringLiteral("起点"),
            QStringLiteral("终点"),
            QStringLiteral("内缩"),
            QStringLiteral("焊接速度"),
            QStringLiteral("摆动方式"),
            QStringLiteral("摆动类型"),
            QStringLiteral("幅度"),
            QStringLiteral("弦长"),
            QStringLiteral("多层多道"),
            QStringLiteral("板厚"),
            QStringLiteral("坡口角度"),
            QStringLiteral("装配间隙"),
            QStringLiteral("熔深"),
            });
        applyTableStyle(table);
        table->setColumnHidden(ColWeaveType, true);
        table->setColumnHidden(ColAmplitude, true);
        table->setColumnHidden(ColChord, true);
        table->setColumnHidden(ColThickness, true);
        table->setColumnHidden(ColGrooveAngle, true);
        table->setColumnHidden(ColFitUpGap, true);
        table->setColumnHidden(ColPenetration, true);

        auto* filter = new WeldTableEventFilter(table);
        filter->setObjectName(QLatin1String(kFilterObjectName));
        table->installEventFilter(filter);
        table->viewport()->installEventFilter(filter);

        auto* notifier = new WeldListNotifier(table);
        QObject::connect(table, &QTableWidget::itemSelectionChanged, notifier, [notifier]() {
            notifier->notify();
        });

        auto* bar = new QWidget(panel);
        auto* barLayout = new QHBoxLayout(bar);
        barLayout->setContentsMargins(4, 2, 4, 2);
        barLayout->addStretch(1);
        auto* clearButton = new QPushButton(QStringLiteral("清空"), bar);
        clearButton->setObjectName(QLatin1String(kClearButtonObjectName));
        clearButton->setFont(table->font());
        clearButton->setToolTip(QStringLiteral("清空焊缝列表"));
        clearButton->setEnabled(false);
        barLayout->addWidget(clearButton);

        layout->addWidget(bar);
        layout->addWidget(table, 1);
        dock->setWidget(panel);

        QObject::connect(clearButton,
            static_cast<void (QPushButton::*)(bool)>(&QPushButton::clicked),
            table,
            [table](bool) { clearWeldList(table); });
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

        QDoubleSpinBox* insetSpin = makeSpin(table, 0.0, 1.0e6, 2, QStringLiteral(" mm"), 0.0);
        QDoubleSpinBox* speedSpin = makeSpin(table, 0.0, 1.0e5, 1, QStringLiteral(" mm/s"), 10.0);
        QDoubleSpinBox* ampSpin = makeSpin(table, 0.0, 1.0e4, 2, QStringLiteral(" mm"), 5.0);
        QDoubleSpinBox* chordSpin = makeSpin(table, 0.0, 1.0e5, 2, QStringLiteral(" mm"), 20.0);
        QDoubleSpinBox* thickSpin = makeSpin(table, 0.0, 1.0e4, 2, QStringLiteral(" mm"), 0.0);
        QDoubleSpinBox* grooveSpin = makeSpin(table, 0.0, 90.0, 1, QStringLiteral(" °"), 0.0);
        QDoubleSpinBox* gapSpin = makeSpin(table, 0.0, 1.0e3, 2, QStringLiteral(" mm"), 0.0);
        QDoubleSpinBox* penSpin = makeSpin(table, 0.0, 1.0e4, 2, QStringLiteral(" mm"), 0.0);

        QComboBox* weaveCombo = makeCombo(
            table, { QStringLiteral("直线焊"), QStringLiteral("摆动焊") });
        QComboBox* weaveTypeCombo = makeCombo(
            table, { QStringLiteral("正弦"), QStringLiteral("三角") });
        QComboBox* multiCombo = makeCombo(
            table, { QStringLiteral("单层单道"), QStringLiteral("多层多道") });

        const QWidgetList editors = { insetSpin, speedSpin, weaveCombo, weaveTypeCombo,
                                     ampSpin, chordSpin, multiCombo, thickSpin, grooveSpin,
                                     gapSpin, penSpin };
        for (QWidget* editor : editors) {
            editor->setFont(table->font());
            watchEditor(table, editor);
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
                if (table->property(kClearingProperty).toBool()) {
                    return;
                }
                for (int r = 0; r < table->rowCount(); ++r) {
                    if (table->cellWidget(r, ColInset) == insetSpin) {
                        applyInsetDisplay(table, r);
                        if (table->selectionModel()
                            && table->selectionModel()->isRowSelected(r, QModelIndex())) {
                            notifySelection(table);
                        }
                        break;
                    }
                }
            });
        QObject::connect(weaveCombo,
            static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged),
            table,
            [table](int) {
                if (!table->property(kClearingProperty).toBool()) {
                    updateDynamicColumns(table);
                }
            });
        QObject::connect(multiCombo,
            static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged),
            table,
            [table](int) {
                if (!table->property(kClearingProperty).toBool()) {
                    updateDynamicColumns(table);
                }
            });

        applyInsetDisplay(table, row);
        updateDynamicColumns(table);
        syncClearButton(table);
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

void clearWeldList(QTableWidget* table)
{
    if (!table || table->property(kClearingProperty).toBool()) {
        return;
    }
    table->setProperty(kClearingProperty, true);

    if (QItemSelectionModel* selection = table->selectionModel()) {
        selection->clear();
    }

    {
        QSignalBlocker blocker(table);
        const int rows = table->rowCount();
        const int cols = table->columnCount();
        for (int row = 0; row < rows; ++row) {
            for (int col = 0; col < cols; ++col) {
                if (QWidget* widget = table->cellWidget(row, col)) {
                    widget->blockSignals(true);
                    table->removeCellWidget(row, col);
                }
            }
        }
        table->setRowCount(0);
    }

    updateDynamicColumns(table);
    syncClearButton(table);
    table->setProperty(kClearingProperty, false);
    notifySelection(table);
}

bool weldRowEndpoints(const QTableWidget* table,
    int row,
    Eigen::Vector3d& start,
    Eigen::Vector3d& end)
{
    start.setZero();
    end.setZero();
    if (!table || row < 0 || row >= table->rowCount()) {
        return false;
    }

    const QTableWidgetItem* startItem = table->item(row, ColStart);
    const QTableWidgetItem* endItem = table->item(row, ColEnd);
    if (!startItem || !endItem) {
        return false;
    }

    Eigen::Vector3d currentStart;
    Eigen::Vector3d currentEnd;
    if (variantToVec(startItem->data(kRoleCurrentVec), currentStart)
        && variantToVec(endItem->data(kRoleCurrentVec), currentEnd)) {
        start = currentStart;
        end = currentEnd;
        return true;
    }

    if (parseVecText(startItem->text(), currentStart) && parseVecText(endItem->text(), currentEnd)) {
        start = currentStart;
        end = currentEnd;
        return true;
    }
    return false;
}

bool selectedWeldEndpoints(const QTableWidget* table,
    Eigen::Vector3d& start,
    Eigen::Vector3d& end)
{
    start.setZero();
    end.setZero();
    if (!table || !table->selectionModel() || !table->selectionModel()->hasSelection()) {
        return false;
    }
    const QModelIndexList rows = table->selectionModel()->selectedRows();
    if (rows.isEmpty()) {
        return false;
    }
    return weldRowEndpoints(table, rows.first().row(), start, end);
}

void setWeldSelectionCallback(QTableWidget* table, WeldSelectionCallback callback)
{
    if (WeldListNotifier* notifier = notifierOf(table)) {
        notifier->setCallback(std::move(callback));
    }
}
