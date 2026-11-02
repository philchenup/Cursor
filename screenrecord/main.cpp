/**
 * @file main.cpp
 * @brief 屏幕录制对话框的独立入口。嵌入已有 Qt 程序时不需要这个文件。
 */
#include "screenrecord.h"

#include <QApplication>

int main(int argc, char* argv[])
{
    QApplication app(argc, argv);
    Record recorder;
    recorder.init();
    recorder.setAttribute(Qt::WA_QuitOnClose, true);
    recorder.show();
    return app.exec();
}
