// 加到 mainwindow.h：#include <QTimer>
// 加到 MainWindow 的 protected / private：

protected:
    void resizeEvent(QResizeEvent* event) override;
    void changeEvent(QEvent* event) override;

private:
    void pause3DRefresh();
    void resume3DRefresh();
    QTimer m_resizeIdle;
    bool m_3dPaused = false;
