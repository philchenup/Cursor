#include "voice_input_dialog.h"

#include <QApplication>
#include <QCommandLineParser>
#include <QMessageBox>

int main(int argc, char* argv[])
{
    QApplication app(argc, argv);
    QApplication::setApplicationName(QStringLiteral("XfyunQtAsr"));
    QApplication::setApplicationVersion(QStringLiteral("1.0.0"));

    QCommandLineParser parser;
    parser.setApplicationDescription(
        QStringLiteral("科大讯飞语音听写 + Qt 对话框（支持语音与打字输入）"));
    parser.addHelpOption();
    parser.addVersionOption();

    QCommandLineOption appIdOption(
        QStringList{QStringLiteral("a"), QStringLiteral("appid")},
        QStringLiteral("讯飞开放平台 APPID"),
        QStringLiteral("appid"));
    parser.addOption(appIdOption);
    parser.process(app);

    // 也可通过环境变量 XFYUN_APPID 注入，避免把密钥写进代码。
    QString appId = parser.value(appIdOption);
    if (appId.isEmpty()) {
        appId = qEnvironmentVariable("XFYUN_APPID");
    }

    VoiceInputDialog dialog(appId);
    if (dialog.exec() == QDialog::Accepted) {
        const QString text = dialog.text().trimmed();
        if (!text.isEmpty()) {
            QMessageBox::information(nullptr,
                                     QStringLiteral("输入结果"),
                                     text);
        }
    }

    return 0;
}
