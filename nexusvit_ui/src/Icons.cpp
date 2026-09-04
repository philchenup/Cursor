#include "Icons.h"

#include <QPainter>
#include <QPainterPath>
#include <QPen>
#include <QtMath>

namespace {

QPixmap canvas(int size)
{
    QPixmap pm(size, size);
    pm.fill(Qt::transparent);
    return pm;
}

void setupPainter(QPainter& p)
{
    p.setRenderHint(QPainter::Antialiasing, true);
    p.setRenderHint(QPainter::SmoothPixmapTransform, true);
}

QPen stroke(const QColor& color, qreal width)
{
    QPen pen(color, width, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
    return pen;
}

} // namespace

namespace NexusIcons {

QPixmap logoMark(int size)
{
    QPixmap pm = canvas(size);
    QPainter p(&pm);
    setupPainter(p);
    const qreal s = size;
    QPainterPath hex;
    const QPointF c(s / 2.0, s / 2.0);
    const qreal r = s * 0.46;
    for (int i = 0; i < 6; ++i) {
        const qreal a = (i * 60.0 - 90.0) * 3.14159265 / 180.0;
        const QPointF pt(c.x() + r * qCos(a), c.y() + r * qSin(a));
        if (i == 0) {
            hex.moveTo(pt);
        } else {
            hex.lineTo(pt);
        }
    }
    hex.closeSubpath();
    QLinearGradient g(0, 0, s, s);
    g.setColorAt(0.0, QColor(20, 180, 170));
    g.setColorAt(1.0, QColor(10, 90, 110));
    p.setPen(QPen(QColor(70, 230, 210), s * 0.06));
    p.setBrush(g);
    p.drawPath(hex);
    p.setPen(QPen(QColor(230, 245, 250), s * 0.08, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
    p.setBrush(Qt::NoBrush);
    QPainterPath n;
    n.moveTo(s * 0.32, s * 0.70);
    n.lineTo(s * 0.32, s * 0.30);
    n.lineTo(s * 0.68, s * 0.70);
    n.lineTo(s * 0.68, s * 0.30);
    p.drawPath(n);
    return pm;
}

QPixmap eyeOpen(int size)
{
    QPixmap pm = canvas(size);
    QPainter p(&pm);
    setupPainter(p);
    p.setPen(stroke(QColor(150, 165, 175), 1.4));
    p.setBrush(Qt::NoBrush);
    QRectF eye(size * 0.12, size * 0.30, size * 0.76, size * 0.40);
    p.drawEllipse(eye);
    p.setBrush(QColor(90, 200, 190));
    p.setPen(Qt::NoPen);
    p.drawEllipse(QRectF(size * 0.38, size * 0.36, size * 0.24, size * 0.28));
    return pm;
}

QPixmap eyeClosed(int size)
{
    QPixmap pm = canvas(size);
    QPainter p(&pm);
    setupPainter(p);
    p.setPen(stroke(QColor(90, 100, 110), 1.4));
    p.drawLine(QPointF(size * 0.15, size * 0.50), QPointF(size * 0.85, size * 0.50));
    p.drawLine(QPointF(size * 0.28, size * 0.38), QPointF(size * 0.38, size * 0.48));
    p.drawLine(QPointF(size * 0.72, size * 0.38), QPointF(size * 0.62, size * 0.48));
    return pm;
}

QPixmap search(int size)
{
    QPixmap pm = canvas(size);
    QPainter p(&pm);
    setupPainter(p);
    p.setPen(stroke(QColor(140, 155, 165), 1.6));
    p.setBrush(Qt::NoBrush);
    p.drawEllipse(QRectF(size * 0.12, size * 0.12, size * 0.52, size * 0.52));
    p.drawLine(QPointF(size * 0.58, size * 0.58), QPointF(size * 0.84, size * 0.84));
    return pm;
}

QPixmap statusDot(const QColor& color, int size)
{
    QPixmap pm = canvas(size);
    QPainter p(&pm);
    setupPainter(p);
    p.setPen(Qt::NoPen);
    p.setBrush(color.darker(180));
    p.drawEllipse(QRectF(1, 1, size - 2, size - 2));
    p.setBrush(color);
    p.drawEllipse(QRectF(2.2, 2.2, size - 4.4, size - 4.4));
    p.setBrush(QColor(255, 255, 255, 90));
    p.drawEllipse(QRectF(size * 0.28, size * 0.22, size * 0.28, size * 0.22));
    return pm;
}

QIcon toolbar(const QString& name, const QColor& color)
{
    const int s = 64;
    QPixmap pm = canvas(s);
    QPainter p(&pm);
    setupPainter(p);
    p.setPen(stroke(color, 3.4));
    p.setBrush(Qt::NoBrush);
    const qreal m = 10;

    auto box = [&](qreal x, qreal y, qreal w, qreal h) {
        p.drawRoundedRect(QRectF(x, y, w, h), 4, 4);
    };

    if (name == QLatin1String("open")) {
        QPainterPath folder;
        folder.moveTo(m, 24);
        folder.lineTo(m, 18);
        folder.lineTo(22, 18);
        folder.lineTo(26, 14);
        folder.lineTo(s - m, 14);
        folder.lineTo(s - m, 50);
        folder.lineTo(m, 50);
        folder.closeSubpath();
        p.drawPath(folder);
    } else if (name == QLatin1String("save")) {
        QPainterPath disk;
        disk.moveTo(m + 2, m);
        disk.lineTo(s - m - 8, m);
        disk.lineTo(s - m, m + 8);
        disk.lineTo(s - m, s - m);
        disk.lineTo(m + 2, s - m);
        disk.closeSubpath();
        p.drawPath(disk);
        p.drawRect(QRectF(22, m, 20, 14));
        p.drawRoundedRect(QRectF(18, 34, 28, 16), 3, 3);
    } else if (name == QLatin1String("delete")) {
        p.drawLine(QPointF(18, 16), QPointF(46, 16));
        p.drawLine(QPointF(26, 16), QPointF(28, 12));
        p.drawLine(QPointF(38, 16), QPointF(36, 12));
        p.drawRoundedRect(QRectF(18, 16, 28, 36), 4, 4);
        p.drawLine(QPointF(26, 26), QPointF(26, 42));
        p.drawLine(QPointF(32, 26), QPointF(32, 42));
        p.drawLine(QPointF(38, 26), QPointF(38, 42));
    } else if (name == QLatin1String("delete_all")) {
        p.drawRoundedRect(QRectF(16, 20, 28, 30), 3, 3);
        p.drawRoundedRect(QRectF(22, 14, 28, 30), 3, 3);
        p.drawLine(QPointF(28, 28), QPointF(44, 44));
        p.drawLine(QPointF(44, 28), QPointF(28, 44));
    } else if (name == QLatin1String("color")) {
        p.drawEllipse(QRectF(14, 14, 36, 36));
        p.setBrush(QColor(70, 200, 190, 80));
        p.drawPie(QRectF(18, 18, 28, 28), 30 * 16, 120 * 16);
        p.setBrush(QColor(60, 140, 230, 80));
        p.drawPie(QRectF(18, 18, 28, 28), 150 * 16, 120 * 16);
    } else if (name == QLatin1String("coordinate")) {
        p.setPen(stroke(QColor(220, 70, 70), 3.4));
        p.drawLine(QPointF(12, 48), QPointF(52, 48));
        p.setPen(stroke(QColor(70, 200, 90), 3.4));
        p.drawLine(QPointF(12, 48), QPointF(12, 12));
        p.setPen(stroke(QColor(70, 140, 230), 3.4));
        p.drawLine(QPointF(12, 48), QPointF(40, 22));
    } else if (name == QLatin1String("transform")) {
        p.drawLine(QPointF(32, 12), QPointF(32, 52));
        p.drawLine(QPointF(12, 32), QPointF(52, 32));
        p.drawLine(QPointF(32, 12), QPointF(26, 20));
        p.drawLine(QPointF(32, 12), QPointF(38, 20));
        p.drawLine(QPointF(52, 32), QPointF(44, 26));
        p.drawLine(QPointF(52, 32), QPointF(44, 38));
    } else if (name == QLatin1String("sample")) {
        for (int y = 0; y < 3; ++y) {
            for (int x = 0; x < 3; ++x) {
                p.drawEllipse(QRectF(16 + x * 14, 16 + y * 14, 6, 6));
            }
        }
    } else if (name == QLatin1String("merge")) {
        p.drawEllipse(QRectF(10, 18, 28, 28));
        p.drawEllipse(QRectF(26, 18, 28, 28));
    } else if (name == QLatin1String("clip")) {
        p.drawEllipse(QRectF(14, 12, 16, 16));
        p.drawEllipse(QRectF(34, 12, 16, 16));
        p.drawLine(QPointF(22, 26), QPointF(16, 52));
        p.drawLine(QPointF(42, 26), QPointF(48, 52));
        p.drawLine(QPointF(22, 26), QPointF(42, 26));
    } else if (name == QLatin1String("tcp")) {
        p.drawRect(QRectF(22, 12, 20, 14));
        p.drawLine(QPointF(32, 26), QPointF(32, 40));
        p.drawEllipse(QRectF(24, 38, 16, 16));
    } else if (name == QLatin1String("handeye")) {
        box(12, 28, 22, 16);
        p.drawLine(QPointF(23, 28), QPointF(23, 18));
        p.drawEllipse(QRectF(34, 14, 18, 14));
        p.drawRect(QRectF(38, 28, 12, 10));
    } else if (name == QLatin1String("library")) {
        box(14, 16, 12, 34);
        box(28, 16, 12, 34);
        box(42, 16, 10, 34);
    } else if (name == QLatin1String("vision")) {
        p.drawRoundedRect(QRectF(14, 18, 36, 28), 4, 4);
        p.drawEllipse(QRectF(24, 24, 16, 16));
        p.drawLine(QPointF(20, 18), QPointF(24, 12));
        p.drawLine(QPointF(44, 18), QPointF(40, 12));
    } else if (name == QLatin1String("place")) {
        p.drawRoundedRect(QRectF(14, 28, 36, 22), 3, 3);
        p.drawLine(QPointF(14, 34), QPointF(32, 18));
        p.drawLine(QPointF(32, 18), QPointF(50, 34));
    } else if (name == QLatin1String("screenshot")) {
        p.drawRoundedRect(QRectF(12, 20, 40, 28), 5, 5);
        p.drawEllipse(QRectF(24, 26, 16, 16));
        p.drawRect(QRectF(18, 16, 12, 6));
    } else if (name == QLatin1String("step")) {
        QPainterPath tri;
        tri.moveTo(18, 16);
        tri.lineTo(18, 48);
        tri.lineTo(34, 32);
        tri.closeSubpath();
        p.setBrush(color);
        p.drawPath(tri);
        p.setBrush(Qt::NoBrush);
        p.drawLine(QPointF(42, 16), QPointF(42, 48));
        p.drawLine(QPointF(48, 16), QPointF(48, 48));
    } else if (name == QLatin1String("continuous")) {
        QPainterPath tri;
        tri.moveTo(22, 14);
        tri.lineTo(22, 50);
        tri.lineTo(50, 32);
        tri.closeSubpath();
        p.setBrush(color);
        p.drawPath(tri);
    } else if (name == QLatin1String("pause")) {
        p.setBrush(color);
        p.setPen(Qt::NoPen);
        p.drawRoundedRect(QRectF(18, 14, 10, 36), 2, 2);
        p.drawRoundedRect(QRectF(36, 14, 10, 36), 2, 2);
    } else if (name == QLatin1String("reset")) {
        QPainterPath arc;
        arc.arcMoveTo(16, 16, 32, 32, 50);
        arc.arcTo(16, 16, 32, 32, 50, 260);
        p.drawPath(arc);
        p.drawLine(QPointF(40, 16), QPointF(50, 16));
        p.drawLine(QPointF(40, 16), QPointF(40, 26));
    } else {
        box(16, 16, 32, 32);
    }

    return QIcon(pm);
}

} // namespace NexusIcons
