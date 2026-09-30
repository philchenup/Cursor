#ifdef Q_OS_WIN
#  include <dwmapi.h>
#  include <windows.h>
#  pragma comment(lib, "dwmapi.lib")
#endif

// 构造函数里 viewer/cloudview 创建后：
//   m_resizeIdle.setSingleShot(true);
//   connect(&m_resizeIdle, &QTimer::timeout, this, [this] { freeze3D(false); });
// #ifdef Q_OS_WIN
//   BOOL off = TRUE;
//   DwmSetWindowAttribute((HWND)winId(), 3, &off, sizeof(off)); // 关最大化动画
// #endif

void MainWindow::freeze3D(bool freeze)
{
    auto hold = [](QWidget* w, bool on) {
        if (!w) return;
        if (on) w->setFixedSize(w->size());
        else { w->setMinimumSize(0, 0); w->setMaximumSize(QWIDGETSIZE_MAX, QWIDGETSIZE_MAX); }
    };
    hold(viewer, freeze);
    hold(ui->cloudview, freeze);

    if (viewer && viewer->viewer) {
        viewer->viewer->setAutoRedraw(!freeze);
        if (!freeze) viewer->viewer->render();
    }
    if (ui->cloudview && !freeze)
        ui->cloudview->update();
}

void MainWindow::resizeEvent(QResizeEvent* e)
{
    freeze3D(true);
    QMainWindow::resizeEvent(e);
    m_resizeIdle.start(150);
}
