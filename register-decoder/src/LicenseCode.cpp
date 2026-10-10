#include "LicenseCode.h"

#include <QCryptographicHash>

QString makeAuthorizationCode(const QString& serial)
{
    const QByteArray hex = QCryptographicHash::hash(serial.toUtf8(), QCryptographicHash::Md5).toHex();
    const QString md5 = QString::fromLatin1(hex);
    return QStringLiteral("5") + md5.mid(0, 10) + QStringLiteral("1") + md5.mid(10) + QStringLiteral("2");
}
