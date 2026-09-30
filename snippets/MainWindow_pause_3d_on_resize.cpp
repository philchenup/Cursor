// 构造：connect(&m_resizeIdle, &QTimer::timeout, this, [this] { set3DRefresh(true); });
//       m_resizeIdle.setSingleShot(true);

void MainWindow::set3DRefresh(bool on)
{
    if (viewer && viewer->viewer) {
        viewer->viewer->setAutoRedraw(on);
        if (on) viewer->viewer->render();
    }
    if (ui->cloudview) {
        ui->cloudview->setUpdatesEnabled(on);
        if (on) ui->cloudview->update();
    }
}

void MainWindow::resizeEvent(QResizeEvent* e)
{
    set3DRefresh(false);
    QMainWindow::resizeEvent(e);
    m_resizeIdle.start(100);
}
