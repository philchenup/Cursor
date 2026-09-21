#ifndef REDUCE_STACKED_WIDGET_TOP_GAP_H
#define REDUCE_STACKED_WIDGET_TOP_GAP_H

#include <QBoxLayout>
#include <QGridLayout>
#include <QLayout>
#include <QMainWindow>
#include <QMargins>
#include <QStackedWidget>
#include <QTabWidget>
#include <QWidget>

/**
 * 缩小 QStackedWidget 与其上方控件之间的空隙。
 *
 * QSS 的 margin / padding 清不掉这块空带。空隙通常来自：
 *   1. 父 QVBoxLayout / QGridLayout 的 spacing（样式默认约 6px）
 *      和 contentsMargins（样式默认约 9px）
 *   2. QStackedWidget 自身及每一页 layout 的 contentsMargins
 *   3. 若在 QTabWidget 里：非 documentMode 的页签框与 pane 间距
 *
 * 在创建完布局、把页面 addWidget 之后调用：
 *   reduceGapAboveStackedWidget(stackedWidget);
 *
 * siblingSpacingPx 是「上方兄弟控件」与堆叠页之间的间距，默认 0（贴齐）。
 * 若仍要保留 3px 网格，传入 3。
 */
inline void reduceGapAboveStackedWidget(QStackedWidget *stack, int siblingSpacingPx = 0)
{
    if (stack == nullptr) {
        return;
    }

    stack->setContentsMargins(0, 0, 0, 0);
    if (QLayout *own = stack->layout()) {
        own->setContentsMargins(0, 0, 0, 0);
        own->setSpacing(0);
    }

    for (int i = 0; i < stack->count(); ++i) {
        QWidget *page = stack->widget(i);
        if (page == nullptr) {
            continue;
        }
        page->setContentsMargins(0, 0, 0, 0);
        if (QLayout *pageLayout = page->layout()) {
            const QMargins m = pageLayout->contentsMargins();
            pageLayout->setContentsMargins(m.left(), 0, m.right(), m.bottom());
        }
    }

    if (QMainWindow *mainWindow = qobject_cast<QMainWindow *>(stack->window())) {
        if (QWidget *central = mainWindow->centralWidget()) {
            if (central == stack || central->isAncestorOf(stack)) {
                central->setContentsMargins(0, 0, 0, 0);
                if (QLayout *centralLayout = central->layout()) {
                    const QMargins m = centralLayout->contentsMargins();
                    centralLayout->setContentsMargins(m.left(), 0, m.right(), m.bottom());
                }
            }
        }
    }

    for (QWidget *child = stack; child != nullptr; child = child->parentWidget()) {
        QWidget *parent = child->parentWidget();
        if (parent == nullptr) {
            break;
        }

        if (QTabWidget *tabs = qobject_cast<QTabWidget *>(parent)) {
            tabs->setDocumentMode(true);
            tabs->setContentsMargins(0, 0, 0, 0);
            if (QLayout *tabLayout = tabs->layout()) {
                tabLayout->setContentsMargins(0, 0, 0, 0);
                tabLayout->setSpacing(0);
            }
            break;
        }

        QLayout *layout = parent->layout();
        if (layout == nullptr) {
            continue;
        }

        const int index = layout->indexOf(child);
        if (index < 0) {
            continue;
        }

        if (index > 0) {
            if (QGridLayout *grid = qobject_cast<QGridLayout *>(layout)) {
                grid->setVerticalSpacing(siblingSpacingPx);
            } else {
                layout->setSpacing(siblingSpacingPx);
            }
            break;
        }

        const QMargins m = layout->contentsMargins();
        if (m.top() != 0) {
            layout->setContentsMargins(m.left(), 0, m.right(), m.bottom());
            break;
        }
    }
}

#endif
