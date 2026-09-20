#ifndef WINDOWS_MAXIMIZED_EDGE_CLICK_H
#define WINDOWS_MAXIMIZED_EDGE_CLICK_H

#include <QAbstractNativeEventFilter>
#include <QApplication>
#include <QWidget>
#include <QtGlobal>

#if defined(Q_OS_WIN)
#  include <windows.h>
#endif

/**
 * QWidget::nativeEvent 往往收不到 WM_NCHITTEST：Windows 插件在窗口过程里
 * 先处理非客户区。必须用 QAbstractNativeEventFilter 拦截。
 *
 * 最大化 / 全屏时把整窗当作客户区，避免约 8px 系统边框把贴边按钮点击吃掉。
 *
 * 在创建 QApplication 之后调用：
 *   installWindowsMaximizedEdgeClickFix(qApp);
 */
class WindowsMaximizedEdgeClickFilter : public QAbstractNativeEventFilter
{
public:
#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
    bool nativeEventFilter(const QByteArray &eventType, void *message, qintptr *result) override
#else
    bool nativeEventFilter(const QByteArray &eventType, void *message, long *result) override
#endif
    {
#if defined(Q_OS_WIN)
        if (eventType != "windows_generic_MSG" && eventType != "windows_dispatcher_MSG")
            return false;

        MSG *msg = static_cast<MSG *>(message);
        QWidget *found = QWidget::find(reinterpret_cast<WId>(msg->hwnd));
        if (!found)
            return false;

        QWidget *win = found->window();
        const Qt::WindowType type = win->windowType();
        if (type != Qt::Window && type != Qt::Dialog)
            return false;
        if (!win->isMaximized() && !win->isFullScreen())
            return false;

        if (msg->message == WM_NCCALCSIZE && msg->wParam) {
            *result = 0;
            return true;
        }
        if (msg->message == WM_NCHITTEST) {
            *result = HTCLIENT;
            return true;
        }
#else
        Q_UNUSED(eventType);
        Q_UNUSED(message);
        Q_UNUSED(result);
#endif
        return false;
    }
};

inline void installWindowsMaximizedEdgeClickFix(QApplication *app)
{
#if defined(Q_OS_WIN)
    static WindowsMaximizedEdgeClickFilter filter;
    app->installNativeEventFilter(&filter);
#else
    Q_UNUSED(app);
#endif
}

#endif
