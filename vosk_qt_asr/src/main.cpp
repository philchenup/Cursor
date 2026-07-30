#include "main_window.h"

#include <QApplication>
#include <QCommandLineParser>
#include <QCoreApplication>
#include <QDir>
#include <QFileInfo>
#include <QMessageBox>

static QString resolveModelPath(const QString& cliPath)
{
    if (!cliPath.isEmpty()) {
        return QFileInfo(cliPath).absoluteFilePath();
    }

    const QString envPath = qEnvironmentVariable("VOSK_MODEL_PATH");
    if (!envPath.isEmpty()) {
        return QFileInfo(envPath).absoluteFilePath();
    }

    // 优先：可执行文件旁的 model/
    const QDir appDir(QCoreApplication::applicationDirPath());
    const QString besideExe = appDir.filePath(QStringLiteral("model"));
    if (QFileInfo::exists(besideExe)) {
        return QFileInfo(besideExe).absoluteFilePath();
    }

    // 其次：当前工作目录 ./model（Visual Studio / Qt Creator 调试常用）
    const QString cwdModel = QFileInfo(QStringLiteral("model")).absoluteFilePath();
    if (QFileInfo::exists(cwdModel)) {
        return cwdModel;
    }

    // 再次：源码树相对路径（从 build/ 运行时）
    const QString fromBuild = appDir.filePath(QStringLiteral("../model"));
    if (QFileInfo::exists(fromBuild)) {
        return QFileInfo(fromBuild).absoluteFilePath();
    }

    return QStringLiteral("model");
}

int main(int argc, char* argv[])
{
    QApplication app(argc, argv);
    QApplication::setApplicationName(QStringLiteral("VoskQtAsr"));
    QApplication::setApplicationVersion(QStringLiteral("1.0.0"));
    QApplication::setOrganizationName(QStringLiteral("vosk-qt-asr"));

    QCommandLineParser parser;
    parser.setApplicationDescription(
        QStringLiteral("Vosk 中文离线语音识别（Qt 音响按钮：点按开始/结束录音）"));
    parser.addHelpOption();
    parser.addVersionOption();

    QCommandLineOption modelOption(
        QStringList{QStringLiteral("m"), QStringLiteral("model")},
        QStringLiteral("中文模型目录路径（默认: model）"),
        QStringLiteral("path"));
    parser.addOption(modelOption);
    parser.process(app);

    const QString modelPath = resolveModelPath(parser.value(modelOption));

    if (!QFileInfo::exists(modelPath)) {
        QMessageBox::critical(
            nullptr,
            QStringLiteral("缺少模型"),
            QStringLiteral("未找到中文模型目录:\n%1\n\n"
                           "请将 vosk-model-small-cn-0.22（或 vosk-model-cn-0.22）"
                           "解压内容放到 model/ 目录。\n"
                           "下载: https://alphacephei.com/vosk/models")
                .arg(modelPath));
        return 1;
    }

    MainWindow window(modelPath);
    window.show();
    return app.exec();
}
