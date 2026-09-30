#include "fast_maximize.h"

#include <QApplication>
#include <QMainWindow>
// 已有主窗口时改成：#include "MainWindow.h"

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    QMainWindow w;   // 已有主窗口时改成：MainWindow w;
    showMaximizedFast(&w);            // 有 3D 视口时：showMaximizedFast(&w, occView);
    return a.exec();
}
