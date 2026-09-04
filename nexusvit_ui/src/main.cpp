#include "MainWindow.h"

#include <QApplication>
#include <QFont>

int main(int argc, char* argv[])
{
    QApplication::setHighDpiScaleFactorRoundingPolicy(
        Qt::HighDpiScaleFactorRoundingPolicy::PassThrough);

    QApplication app(argc, argv);
    app.setApplicationName(QStringLiteral("NexusVIT"));
    app.setOrganizationName(QStringLiteral("NexusVIT"));

    QFont font(QStringLiteral("Noto Sans CJK SC"));
    font.setPixelSize(12);
    app.setFont(font);

    MainWindow window;
    window.show();
    return app.exec();
}
