#include "ViewportWidget.h"

#include <QLinearGradient>
#include <QMouseEvent>
#include <QPainter>
#include <QPainterPath>
#include <QRadialGradient>
#include <QWheelEvent>
#include <QtMath>

namespace {

struct Vec3 {
    float x = 0;
    float y = 0;
    float z = 0;
};

} // namespace

ViewportWidget::ViewportWidget(QWidget* parent)
    : QWidget(parent)
{
    setObjectName(QStringLiteral("viewportWidget"));
    setMinimumSize(420, 280);
    setMouseTracking(true);
}

void ViewportWidget::setMode(const QString& mode)
{
    m_mode = mode;
    update();
}

void ViewportWidget::mousePressEvent(QMouseEvent* event)
{
    if (event->button() == Qt::LeftButton || event->button() == Qt::MiddleButton) {
        m_dragging = true;
        m_lastPos = event->pos();
    }
}

void ViewportWidget::mouseMoveEvent(QMouseEvent* event)
{
    if (!m_dragging) {
        return;
    }
    const QPoint d = event->pos() - m_lastPos;
    m_lastPos = event->pos();
    m_yaw += d.x() * 0.008f;
    m_pitch = qBound(0.12f, m_pitch + d.y() * 0.006f, 1.15f);
    update();
}

void ViewportWidget::mouseReleaseEvent(QMouseEvent* event)
{
    Q_UNUSED(event);
    m_dragging = false;
}

void ViewportWidget::wheelEvent(QWheelEvent* event)
{
    const float step = event->angleDelta().y() > 0 ? 1.08f : 0.92f;
    m_zoom = qBound(0.55f, m_zoom * step, 2.2f);
    update();
}

void ViewportWidget::paintEvent(QPaintEvent*)
{
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing, true);
    if (m_mode == QLatin1String("camera")) {
        paintCamera(p);
    } else {
        paintScene(p);
        paintAxes(p);
        paintViewCube(p);
    }
}

void ViewportWidget::paintScene(QPainter& p)
{
    QLinearGradient bg(0, 0, 0, height());
    bg.setColorAt(0.0, QColor(28, 30, 34));
    bg.setColorAt(0.55, QColor(46, 47, 48));
    bg.setColorAt(1.0, QColor(22, 24, 26));
    p.fillRect(rect(), bg);

    const float cx = width() * 0.52f;
    const float cy = height() * 0.62f;
    const float scale = qMin(width(), height()) * 0.018f * m_zoom;

    auto project = [&](Vec3 v) -> QPointF {
        const float cyaw = qCos(m_yaw);
        const float syaw = qSin(m_yaw);
        const float cp = qCos(m_pitch);
        const float sp = qSin(m_pitch);
        const float x = v.x * cyaw - v.z * syaw;
        const float z = v.x * syaw + v.z * cyaw;
        const float y = v.y;
        const float xr = x;
        const float yr = y * cp - z * sp;
        return QPointF(cx + xr * scale * 0.92, cy - yr * scale);
    };

    auto drawQuad = [&](Vec3 a, Vec3 b, Vec3 c, Vec3 d, const QColor& fill, const QColor& line) {
        QPolygonF poly;
        poly << project(a) << project(b) << project(c) << project(d);
        p.setPen(QPen(line, 1.0));
        p.setBrush(fill);
        p.drawPolygon(poly);
    };

    auto drawLink = [&](Vec3 a, Vec3 b, qreal width, const QColor& color) {
        p.setPen(QPen(color, width, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
        p.drawLine(project(a), project(b));
    };

    // Floor grid
    p.setPen(QPen(QColor(70, 72, 74, 90), 1));
    for (int i = -14; i <= 14; ++i) {
        drawLink({float(i), 0, -14}, {float(i), 0, 14}, 1.0, QColor(80, 82, 84, 70));
        drawLink({-14, 0, float(i)}, {14, 0, float(i)}, 1.0, QColor(80, 82, 84, 70));
    }

    // Table
    drawQuad({-8, 0, -5}, {10, 0, -5}, {10, 0, 7}, {-8, 0, 7}, QColor(92, 96, 100), QColor(60, 62, 64));
    drawQuad({-8, -2.2f, -5}, {-8, 0, -5}, {10, 0, -5}, {10, -2.2f, -5}, QColor(70, 73, 76), QColor(40, 42, 44));
    drawQuad({10, -2.2f, -5}, {10, 0, -5}, {10, 0, 7}, {10, -2.2f, 7}, QColor(58, 60, 62), QColor(36, 38, 40));

    // Robot base
    drawQuad({-1.6f, 0, -1.6f}, {1.6f, 0, -1.6f}, {1.6f, 0, 1.6f}, {-1.6f, 0, 1.6f},
             QColor(210, 212, 216), QColor(140, 142, 146));
    drawLink({0, 0, 0}, {0, 2.4f, 0}, 14, QColor(228, 230, 234));

    const Vec3 j1{0, 2.4f, 0};
    const Vec3 j2{0, 5.6f, 1.4f};
    const Vec3 j3{0, 5.2f, 5.4f};
    const Vec3 j4{0, 3.6f, 7.2f};
    const Vec3 j5{0, 3.2f, 8.6f};

    drawLink(j1, j2, 11, QColor(236, 238, 242));
    drawLink(j2, j3, 9, QColor(228, 230, 234));
    drawLink(j3, j4, 7, QColor(220, 80, 70));
    drawLink(j4, j5, 5, QColor(200, 202, 206));

    auto joint = [&](Vec3 v, qreal r, const QColor& c) {
        p.setPen(Qt::NoPen);
        p.setBrush(c);
        p.drawEllipse(project(v), r, r * 0.78);
    };
    joint(j1, 8, QColor(190, 50, 48));
    joint(j2, 7, QColor(190, 50, 48));
    joint(j3, 6, QColor(210, 212, 216));
    joint(j5, 4, QColor(80, 160, 255));

    // Parts bin
    drawQuad({-6.5f, 0.05f, 1.5f}, {-2.4f, 0.05f, 1.5f}, {-2.4f, 0.05f, 5.2f}, {-6.5f, 0.05f, 5.2f},
             QColor(18, 18, 20), QColor(8, 8, 8));
    drawQuad({-6.5f, 0.05f, 1.5f}, {-6.5f, 1.4f, 1.5f}, {-2.4f, 1.4f, 1.5f}, {-2.4f, 0.05f, 1.5f},
             QColor(28, 28, 30), QColor(10, 10, 12));
    drawQuad({-2.4f, 0.05f, 1.5f}, {-2.4f, 1.4f, 1.5f}, {-2.4f, 1.4f, 5.2f}, {-2.4f, 0.05f, 5.2f},
             QColor(36, 36, 38), QColor(12, 12, 14));

    const QColor parts[] = {
        QColor(196, 150, 70), QColor(170, 175, 180), QColor(120, 80, 50),
        QColor(210, 170, 90), QColor(90, 95, 100), QColor(180, 90, 60),
        QColor(150, 155, 160), QColor(200, 140, 70)
    };
    int idx = 0;
    for (float z = 2.0f; z <= 4.6f; z += 0.85f) {
        for (float x = -5.9f; x <= -2.9f; x += 0.95f) {
            const QColor c = parts[idx++ % 8];
            drawQuad({x, 0.2f, z}, {x + 0.55f, 0.2f, z}, {x + 0.55f, 0.2f, z + 0.45f}, {x, 0.2f, z + 0.45f},
                     c, c.darker(140));
        }
    }

    // Soft vignette
    QRadialGradient vig(rect().center(), qMax(width(), height()) * 0.72);
    vig.setColorAt(0.55, QColor(0, 0, 0, 0));
    vig.setColorAt(1.0, QColor(0, 0, 0, 90));
    p.fillRect(rect(), vig);
}

void ViewportWidget::paintCamera(QPainter& p)
{
    p.fillRect(rect(), QColor(8, 10, 12));
    QLinearGradient g(0, 0, 0, height());
    g.setColorAt(0, QColor(18, 22, 28));
    g.setColorAt(1, QColor(6, 8, 10));
    p.fillRect(rect().adjusted(18, 18, -18, -18), g);

    p.setPen(QPen(QColor(40, 160, 150), 1.2));
    const int cx = width() / 2;
    const int cy = height() / 2;
    p.drawLine(cx - 48, cy, cx + 48, cy);
    p.drawLine(cx, cy - 48, cx, cy + 48);
    p.setBrush(Qt::NoBrush);
    p.drawEllipse(QPoint(cx, cy), 70, 70);
    p.drawRect(rect().adjusted(18, 18, -18, -18));

    p.setPen(QColor(90, 200, 190));
    p.setFont(QFont(QStringLiteral("Noto Sans CJK SC"), 11));
    p.drawText(rect().adjusted(28, 28, -28, -28), Qt::AlignTop | Qt::AlignLeft,
               QStringLiteral("CAM-01  ·  1920x1080  ·  30 FPS"));
    p.setPen(QColor(140, 150, 160));
    p.drawText(rect(), Qt::AlignCenter, QStringLiteral("No camera signal"));
}

void ViewportWidget::paintViewCube(QPainter& p)
{
    const int s = 72;
    const QRect cube(width() - s - 16, 14, s, s);
    p.setPen(QPen(QColor(80, 90, 98), 1));
    p.setBrush(QColor(24, 28, 32, 220));
    p.drawRoundedRect(cube, 6, 6);

    auto face = [&](const QRect& r, const QString& t, const QColor& c) {
        p.setBrush(c);
        p.setPen(QPen(QColor(20, 24, 28), 1));
        p.drawRoundedRect(r, 2, 2);
        p.setPen(QColor(230, 235, 240));
        p.setFont(QFont(QStringLiteral("Noto Sans"), 8, QFont::DemiBold));
        p.drawText(r, Qt::AlignCenter, t);
    };
    face(QRect(cube.left() + 20, cube.top() + 8, 32, 18), QStringLiteral("TOP"), QColor(70, 90, 110));
    face(QRect(cube.left() + 8, cube.top() + 28, 32, 24), QStringLiteral("FRONT"), QColor(60, 80, 70));
    face(QRect(cube.left() + 40, cube.top() + 28, 24, 24), QStringLiteral("R"), QColor(90, 70, 70));
}

void ViewportWidget::paintAxes(QPainter& p)
{
    const QPoint o(36, height() - 36);
    auto axis = [&](QPoint d, const QColor& c, const QString& t) {
        p.setPen(QPen(c, 2.4, Qt::SolidLine, Qt::RoundCap));
        p.drawLine(o, o + d);
        p.setFont(QFont(QStringLiteral("Noto Sans"), 9, QFont::Bold));
        p.drawText(o + d + QPoint(4, 4), t);
    };
    axis(QPoint(34, 4), QColor(220, 70, 70), QStringLiteral("X"));
    axis(QPoint(4, -34), QColor(70, 200, 90), QStringLiteral("Y"));
    axis(QPoint(-18, 16), QColor(70, 150, 240), QStringLiteral("Z"));
}
