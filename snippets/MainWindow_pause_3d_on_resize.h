protected:
    void resizeEvent(QResizeEvent* e) override;
private:
    void set3DRefresh(bool on);
    QTimer m_resizeIdle;
