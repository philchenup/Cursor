#include <QEvent>
#include <QResizeEvent>
#include <vtkRenderWindow.h>

// 构造函数里 this->resize(1600, 1200); 之后加上：
//     m_resizeIdle.setSingleShot(true);
//     m_resizeIdle.setInterval(40);
//     connect(&m_resizeIdle, &QTimer::timeout, this, &MainWindow::resume3DRefresh);

void MainWindow::pause3DRefresh()
{
    if (m_3dPaused) {
        return;
    }
    m_3dPaused = true;

    // OpenInventor / SoQt：关掉自动刷新
    if (this->viewer != nullptr && this->viewer->viewer != nullptr) {
        this->viewer->viewer->setAutoRedraw(FALSE);
    }

    // VTK 点云：禁止 Qt 绘制，并中止 vtk 渲染
    if (ui->cloudview != nullptr) {
        ui->cloudview->setUpdatesEnabled(false);
        if (ui->cloudview->viewer()) {
            ui->cloudview->viewer()->getRenderWindow()->SetAbortRender(1);
        }
    }
}

void MainWindow::resume3DRefresh()
{
    if (!m_3dPaused) {
        return;
    }
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
