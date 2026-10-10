#include "DecoderWindow.h"
#include "LicenseCode.h"

#include <QApplication>
#include <QFont>
#include <QTextStream>
#include <QtGlobal>

namespace {

bool selfTest()
{
    struct Sample {
        const char* serial;
        const char* code;
    };
    const Sample samples[] = {
        {"", "5d41d8cd98f100b204e9800998ecf8427e2"},
        {"TEST-SERIAL", "50da0fadaa519be9c19a5f0f5e362ee6cb2"},
        {"BFEBs1234567897BFEB6", "50e82b80b7010e77edfbb25b4a538521982"},
    };
    for (const Sample& sample : samples)
    {
        const QString actual = makeAuthorizationCode(QString::fromLatin1(sample.serial));
        if (actual != QLatin1String(sample.code) || actual.size() != 35)
            return false;
    }
    return true;
}

} // namespace

int main(int argc, char* argv[])
{
    QApplication app(argc, argv);
    QApplication::setApplicationName(QStringLiteral("RegisterDecoder"));
    QFont font;
#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
    font.setFamilies({QStringLiteral("Noto Sans CJK SC"), QStringLiteral("Microsoft YaHei"), QStringLiteral("sans-serif")});
#else
    font.setFamily(QStringLiteral("Microsoft YaHei"));
#endif
    font.setPointSize(11);
    app.setFont(font);

    const QStringList args = QCoreApplication::arguments();
    if (args.size() == 2 && args.at(1) == QStringLiteral("--self-test"))
        return selfTest() ? 0 : 1;
    if (args.size() == 3 && args.at(1) == QStringLiteral("--code"))
    {
        QTextStream(stdout) << makeAuthorizationCode(args.at(2)) << "\n";
        return 0;
    }

    DecoderWindow window;
    window.resize(640, 460);
    window.show();
    return app.exec();
}
