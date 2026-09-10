#include "SDCameraSafe.h"

// ---------------------------------------------------------------------------
// 用法 1：MainWindow 里，调用 SDCamera.dll 之前先挡 0 尺寸
//
// 0xC0000094 出在 DLL 内部，主程序改不了那条 idiv，但可以不把
// 0 宽高交进去：窗口未显示 / 最小化 / 相机尚未 Open 拿到分辨率。
// ---------------------------------------------------------------------------

#if 0  // 贴进 MainWindow 时改成 1，并接上真实成员

#include <QMainWindow>
#include <QResizeEvent>
#include <QShowEvent>

void MainWindow::startCameraPreview()
{
    int imageW = 0;
    int imageH = 0;
    // 先读分辨率，失败或仍是 0 就不要 StartGrab / Display
    if (!m_camera.GetResolution(&imageW, &imageH)
        || !sdc::CanDisplay(ui->cameraWidget->width(),
                             ui->cameraWidget->height(),
                             imageW, imageH)) {
        return;
    }
    m_imageW = imageW;
    m_imageH = imageH;
    m_camera.StartGrab(reinterpret_cast<HWND>(ui->cameraWidget->winId()));
}

void MainWindow::resizeEvent(QResizeEvent* event)
{
    QMainWindow::resizeEvent(event);
    const int w = ui->cameraWidget->width();
    const int h = ui->cameraWidget->height();
    if (!sdc::ShouldHandleResize(w, h)) {
        return; // 最小化或尚未布局完成：SDK 里会做 wndH * srcW / srcH
    }
    if (!sdc::CanDisplay(w, h, m_imageW, m_imageH)) {
        return;
    }
    m_camera.NotifyDisplaySize(w, h);
}

void MainWindow::showEvent(QShowEvent* event)
{
    QMainWindow::showEvent(event);
    // 首次 show 后控件才有非零客户区，再绑定 HWND
    if (sdc::ShouldHandleResize(ui->cameraWidget->width(),
                               ui->cameraWidget->height())) {
        startCameraPreview();
    }
}

#endif

// ---------------------------------------------------------------------------
// 用法 2：若 SDCamera.dll 是自己编的，把显示缩放改成这条路径
// （原代码 dstW = wndH * srcW / srcH 在 srcH==0 时就是这次崩溃）
// ---------------------------------------------------------------------------

bool SDCameraDisplayFit(int srcW, int srcH, int wndW, int wndH,
                         int& dstX, int& dstY, int& dstW, int& dstH)
{
    return sdc::FitKeepAspect(srcW, srcH, wndW, wndH, dstX, dstY, dstW, dstH);
}

int SDCameraSafeStride(std::size_t bufferBytes, int height)
{
    int stride = 0;
    if (!sdc::RowStride(bufferBytes, height, stride)) {
        return 0;
    }
    return stride;
}
