#ifndef FAST_MAXIMIZE_H
#define FAST_MAXIMIZE_H

#include <QByteArray>
#include <QCoreApplication>
#include <QEvent>
#include <QGuiApplication>
#include <QList>
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

inline bool isInventorOrVtkView(QWidget *w)
{
    const QByteArray n = QByteArray(w->metaObject()->className()).toLower();
    const QByteArray o = w->objectName().toLower().toLatin1();
    auto hit = [](const QByteArray &s) {
        return s.contains("vtk") || s.contains("qvtk") || s.contains("cloudview")
            || s.contains("soqt") || s.contains("inventor") || s.contains("quarter")
            || s.contains("examiner") || s == "viewer";
    };
    return hit(n) || hit(o);
}

inline bool isGlSurface(QWidget *w)
{
    const QByteArray n = QByteArray(w->metaObject()->className()).toLower();
    return n.contains("opengl") || n.contains("glwidget") || n.contains("soqt")
        || n.contains("vtk") || n.contains("quarter") || n.contains("inventor");
}

inline QList<QWidget *> findInventorVtkViews(QWidget *root)
{
    QList<QWidget *> out;
    const auto kids = root->findChildren<QWidget *>();
    for (QWidget *c : kids) {
        if (isInventorOrVtkView(c)) {
            out << c;
        }
    }
    return out;
}

/**
 * 最大化卡顿：Win 过渡动画连续 Resize，OpenInventor(SoQt Viewer)
 * 与 VTK(cloudview) 每次都整帧重绘。关掉过渡，吞掉两个 3D 视口
 * 及其 GL 子窗口的中间 Resize，结束只刷一次。
 *
 *   showMaximizedFast(&w, w.viewer, w.findChild<QWidget *>("cloudview"));
 */
class FastMaximizeFilter : public QObject
{
public:
    FastMaximizeFilter(QWidget *window, QList<QWidget *> heavies)
        : QObject(window)
        , m_win(window)
        , m_heavies(heavies)
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
        attachHeavies();
    }

    bool eventFilter(QObject *obj, QEvent *e) override
    {
        if (e->type() != QEvent::Resize || m_flushing) {
            return false;
        }
        attachHeavies();
        if (m_win->updatesEnabled()) {
            m_win->setUpdatesEnabled(false);
        }
        m_idle.start();
        return m_heavies.contains(static_cast<QWidget *>(obj));
    }

private:
    void attachHeavies()
    {
        if (m_attached) {
            return;
        }
        if (m_heavies.isEmpty()) {
            m_heavies = findInventorVtkViews(m_win);
        }
        if (m_heavies.isEmpty()) {
            return;
        }

        QList<QWidget *> extra = m_heavies;
        for (QWidget *h : extra) {
            const auto kids = h->findChildren<QWidget *>();
            for (QWidget *c : kids) {
                if (isGlSurface(c) && !m_heavies.contains(c)) {
                    m_heavies << c;
                }
            }
        }
        for (QWidget *h : m_heavies) {
            h->installEventFilter(this);
        }
        m_attached = true;
    }

    void flush()
    {
        m_flushing = true;
        m_win->setUpdatesEnabled(true);
        for (QWidget *h : m_heavies) {
            QResizeEvent ev(h->size(), h->size());
            QCoreApplication::sendEvent(h, &ev);
        }
        m_win->update();
        m_flushing = false;
    }

    QWidget *m_win;
    QList<QWidget *> m_heavies;
    QTimer m_idle;
    bool m_flushing = false;
    bool m_attached = false;
};

inline void showMaximizedFast(QWidget *w, QWidget *inventorView = nullptr, QWidget *vtkView = nullptr)
{
    QList<QWidget *> heavies;
    if (inventorView != nullptr) {
        heavies << inventorView;
    }
    if (vtkView != nullptr) {
        heavies << vtkView;
    }
    new FastMaximizeFilter(w, heavies);
    if (QScreen *s = QGuiApplication::primaryScreen()) {
        w->setGeometry(s->availableGeometry());
    }
    w->showMaximized();
}

#endif
