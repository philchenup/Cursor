/**
 * 无 GUI 冒烟测试：验证 Mock MSC + XfyunAsrEngine 登录/会话/结果链路。
 * 用法：在 build 目录执行 ./smoke_asr_engine
 */
#include "xfyun_asr_engine.h"

#include <QCoreApplication>
#include <QTimer>
#include <cstdio>

int main(int argc, char* argv[])
{
    QCoreApplication app(argc, argv);

    XfyunAsrEngine engine;
    QString finalText;
    QString errorText;
    bool gotFinal = false;

    QObject::connect(&engine, &XfyunAsrEngine::finalResult, [&](const QString& text) {
        finalText = text;
        gotFinal = true;
        app.quit();
    });
    QObject::connect(&engine, &XfyunAsrEngine::errorOccurred, [&](const QString& msg) {
        errorText = msg;
        app.quit();
    });

    if (!engine.login(QStringLiteral("demo-appid"))) {
        std::fprintf(stderr, "login failed: %s\n", qPrintable(errorText));
        return 1;
    }
    if (!engine.startSession()) {
        std::fprintf(stderr, "startSession failed: %s\n", qPrintable(errorText));
        return 2;
    }

    // 16-bit silence frame
    QByteArray pcm(3200, '\0');
    engine.writeAudio(pcm);
    engine.finishSession();

    QTimer::singleShot(3000, &app, &QCoreApplication::quit);
    app.exec();

    if (!gotFinal) {
        std::fprintf(stderr, "no final result, error=%s\n", qPrintable(errorText));
        return 3;
    }

    std::printf("OK final=%s\n", qPrintable(finalText));
    return 0;
}
