protected:
    void resizeEvent(QResizeEvent* e) override;
private:
    void freeze3D(bool freeze);
    QTimer m_resizeIdle;
