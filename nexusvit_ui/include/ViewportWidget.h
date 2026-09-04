#ifndef NEXUSVIT_VIEWPORT_WIDGET_H
#define NEXUSVIT_VIEWPORT_WIDGET_H

#include <QWidget>

class ViewportWidget : public QWidget
{
    Q_OBJECT
public:
    explicit ViewportWidget(QWidget* parent = nullptr);

    void setMode(const QString& mode); // "scene" | "camera"

protected:
    void paintEvent(QPaintEvent* event) override;
    void mousePressEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void mouseReleaseEvent(QMouseEvent* event) override;
    void wheelEvent(QWheelEvent* event) override;

private:
    void paintScene(QPainter& p);
    void paintCamera(QPainter& p);
    void paintViewCube(QPainter& p);
    void paintAxes(QPainter& p);

    QString m_mode = QStringLiteral("scene");
    float m_yaw = 0.55f;
    float m_pitch = 0.38f;
    float m_zoom = 1.0f;
    bool m_dragging = false;
    QPoint m_lastPos;
};

#endif
