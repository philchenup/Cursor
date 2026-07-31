#include "SplashScreen.h"

#include <QApplication>
#include <QMainWindow>
#include <QLabel>
#include <QVBoxLayout>
#include <QWidget>

int main(int argc, char* argv[])
{
    QApplication app(argc, argv);
    app.setApplicationName(QStringLiteral("HWI"));
    app.setApplicationVersion(QStringLiteral("1.0.0"));

    // Logo：1490x468 透明 PNG，路径可按部署调整
    SplashScreen splash(QStringLiteral("./icon/preview.png"),
                        QStringLiteral("1.0.0"));
    splash.run();

    // 主窗口示例：启动画面结束后再显示
    QMainWindow window;
    window.setWindowTitle(QStringLiteral("HWI"));
    window.resize(1280, 720);

    auto* central = new QWidget(&window);
    auto* layout = new QVBoxLayout(central);
    auto* hello = new QLabel(QString::fromUtf8(u8"主界面已启动"), central);
    hello->setAlignment(Qt::AlignCenter);
    hello->setStyleSheet(QStringLiteral("font-size: 24px; color: #1e3a55;"));
    layout->addWidget(hello);
    window.setCentralWidget(central);
    window.show();

    splash.finish(&window);
    return app.exec();
}
