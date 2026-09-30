#ifndef FAST_MAXIMIZE_H
#define FAST_MAXIMIZE_H

#include <QCoreApplication>
#include <QEvent>
#include <QGuiApplication>
#include <QResizeEvent>
#include <QScreen>
#include <QTimer>
#include <QWidget>

#ifdef Q_OS_WIN
#  include <dwmapi.h>
#  include <windows.h>
#  ifndef DWMWA_TRANSITIONS_FORCEDISABLED
#    define DWMWA_TRANSITIONS_FORCEDISABLED 3
#  endif
#  ifdef _MSC_VER
#    pragma comment(lib, "dwmapi.lib")
#  endif
#endif

/**
 * 从小窗口拉到最大化会卡：系统过渡动画触发几十次 Resize，
 * 每次都做完整布局 + VTK/OCC/OpenGL 整帧渲染。
 *
 * 处理：关掉窗口过渡；缩放过程冻结重绘并吞掉 3D 视口的中间 Resize，
 * 结束后只渲染一次。
 *
 *   showMaximizedFast(&w);            // 无 3D 视口
 *   showMaximizedFast(&w, occView);   // 推荐：传入 VTK/OCC 窗口
 */
class FastMaximizeFilter : public QObject
{
public:
    FastMaximizeFilter(QWidget *window, QWidget *heavyView)
        : QObject(window)
        , m_win(window)
        , m_heavy(heavyView)
    {
#ifdef Q_OS_WIN
        BOOL off = TRUE;
        DwmSetWindowAttribute(reinterpret_cast<HWND>(window->winId()),
                              DWMWA_TRANSITIONS_FORCEDISABLED, &off, sizeof(off));
#endif
        m_idle.setSingleShot(true);
        m_idle.setInterval(40);
        QObject::connect(&m_idle, &QTimer::timeout, this, [this] { flush(); });
        window->installEventFilter(this);
        if (heavyView != nullptr) {
            heavyView->installEventFilter(this);
        }
    }

    bool eventFilter(QObject *obj, QEvent *e) override
    {
        if (e->type() != QEvent::Resize || m_flushing) {
            return false;
        }
        if (m_win->updatesEnabled()) {
            m_win->setUpdatesEnabled(false);
        }
        m_idle.start();
        return obj == m_heavy;
    }

private:
    void flush()
    {
        m_flushing = true;
        m_win->setUpdatesEnabled(true);
        if (m_heavy != nullptr) {
            QResizeEvent ev(m_heavy->size(), m_heavy->size());
            QCoreApplication::sendEvent(m_heavy, &ev);
        }
        m_win->update();
        m_flushing = false;
    }

    QWidget *m_win;
    QWidget *m_heavy;
    QTimer m_idle;
    bool m_flushing = false;
};

inline void showMaximizedFast(QWidget *w, QWidget *heavyView = nullptr)
{
    new FastMaximizeFilter(w, heavyView);
    if (QScreen *s = QGuiApplication::primaryScreen()) {
        w->setGeometry(s->availableGeometry());
    }
    w->showMaximized();
}

#endif
