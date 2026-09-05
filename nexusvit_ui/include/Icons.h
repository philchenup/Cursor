#ifndef NEXUSVIT_ICONS_H
#define NEXUSVIT_ICONS_H

#include <QColor>
#include <QIcon>
#include <QPixmap>
#include <QString>

namespace NexusIcons {

QIcon toolbar(const QString& name, const QColor& color = QColor(210, 220, 228));
QPixmap logoMark(int size = 28);
QPixmap eyeOpen(int size = 16);
QPixmap eyeClosed(int size = 16);
QPixmap search(int size = 14);
QPixmap statusDot(const QColor& color, int size = 10);

} // namespace NexusIcons

#endif
