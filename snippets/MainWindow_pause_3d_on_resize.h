// 加到 mainwindow.h：
//   #include <QTimer>
//   若 CloudView::viewer() 不存在，去掉相关两行即可

protected:
    void resizeEvent(QResizeEvent* event) override;
    void changeEvent(QEvent* event) override;
    bool eventFilter(QObject* obj, QEvent* event) override;

private:
    void install3DResizeFilter();
    void pause3DRefresh();
    void resume3DRefresh();
    void flush3DResize(QWidget* root);
    QTimer m_resizeIdle;
    bool m_3dPaused = false;
    bool m_flushing3D = false;
