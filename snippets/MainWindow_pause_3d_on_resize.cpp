#include <QCoreApplication>
#include <QEvent>
#include <QGuiApplication>
#include <QResizeEvent>
#include <QScreen>
#include <vtkRenderWindow.h>

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

// 构造函数里 viewer / cloudview 创建之后、this->resize(1600, 1200) 改成：
//
// #ifdef Q_OS_WIN
//     BOOL off = TRUE;
//     DwmSetWindowAttribute(reinterpret_cast<HWND>(winId()),
//                           DWMWA_TRANSITIONS_FORCEDISABLED, &off, sizeof(off));
// #endif
//     if (QScreen* s = QGuiApplication::primaryScreen())
//         setGeometry(s->availableGeometry());
//     m_resizeIdle.setSingleShot(true);
//     m_resizeIdle.setInterval(120);
//     connect(&m_resizeIdle, &QTimer::timeout, this, &MainWindow::resume3DRefresh);
//     install3DResizeFilter();

void MainWindow::install3DResizeFilter()
{
    auto install = [this](QWidget* w) {
        if (w == nullptr) {
            return;
        }
        w->installEventFilter(this);
        const auto kids = w->findChildren<QWidget*>();
        for (QWidget* c : kids) {
            c->installEventFilter(this);
        }
    };
    install(this->viewer);
    install(ui->cloudview);
}

bool MainWindow::eventFilter(QObject* obj, QEvent* event)
{
    // 放大过程中吞掉 Inventor / VTK 的中间 Resize，避免反复分配更大的 FBO
    if (event->type() == QEvent::Resize && m_3dPaused && !m_flushing3D) {
        return true;
    }
    return QMainWindow::eventFilter(obj, event);
}

void MainWindow::pause3DRefresh()
{
    if (m_3dPaused) {
        return;
    }
    m_3dPaused = true;

    if (this->viewer != nullptr && this->viewer->viewer != nullptr) {
        this->viewer->viewer->setAutoRedraw(FALSE);
    }
    if (ui->cloudview != nullptr) {
        ui->cloudview->setUpdatesEnabled(false);
        if (ui->cloudview->viewer()) {
            ui->cloudview->viewer()->getRenderWindow()->SetAbortRender(1);
        }
    }
}

void MainWindow::flush3DResize(QWidget* root)
{
    if (root == nullptr) {
        return;
    }
    QResizeEvent ev(root->size(), root->size());
    QCoreApplication::sendEvent(root, &ev);
    const auto kids = root->findChildren<QWidget*>();
    for (QWidget* c : kids) {
        QResizeEvent cev(c->size(), c->size());
        QCoreApplication::sendEvent(c, &cev);
    }
}

void MainWindow::resume3DRefresh()
{
    if (!m_3dPaused) {
        return;
    }

    m_flushing3D = true;
    flush3DResize(this->viewer);
    flush3DResize(ui->cloudview);
    m_flushing3D = false;
    m_3dPaused = false;

    if (this->viewer != nullptr && this->viewer->viewer != nullptr) {
        this->viewer->viewer->setAutoRedraw(TRUE);
        this->viewer->viewer->render();
    }
    if (ui->cloudview != nullptr) {
        if (ui->cloudview->viewer()) {
            ui->cloudview->viewer()->getRenderWindow()->SetAbortRender(0);
            ui->cloudview->viewer()->getRenderWindow()->Render();
        }
        ui->cloudview->setUpdatesEnabled(true);
        ui->cloudview->update();
    }
}

void MainWindow::resizeEvent(QResizeEvent* event)
{
    pause3DRefresh();
    QMainWindow::resizeEvent(event);
    m_resizeIdle.start();
}

void MainWindow::changeEvent(QEvent* event)
{
    QMainWindow::changeEvent(event);
    if (event->type() == QEvent::WindowStateChange) {
        pause3DRefresh();
        m_resizeIdle.start();
    }
}
