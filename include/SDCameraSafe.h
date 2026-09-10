#ifndef SDCAMERA_SAFE_H
#define SDCAMERA_SAFE_H

#include <cstdint>
#include <cstddef>

/**
 * 拦截 SDCamera.dll 里 0xC0000094（Integer division by zero）。
 *
 * Visual Studio 典型报错：
 *   0x........ (SDCamera.dll) (mainwindow.exe 中)处有未经处理的异常:
 *   0xC0000094: Integer division by zero。
 *
 * DLL 内部常见整数除法（任一项为 0 都会直接崩）：
 *   dstW = wndH * srcW / srcH;   // 图像高度还没读到，或自定义 ROI 失败
 *   dstH = wndW * srcH / srcW;   // 图像宽度为 0
 *   stride = bufferBytes / height;
 *   fps    = 1000000 / periodUs; // 曝光/帧周期未初始化
 *
 * 调用侧（MainWindow）在窗口尚未显示、最小化、停靠栏收起时，
 * 预览控件宽高为 0，一旦把 HWND 交给 SDK 就会走进上述除法。
 * 必须在调用 SDCamera 之前用本头文件的接口挡掉。
 */
namespace sdc {

struct Size2i {
    int width = 0;
    int height = 0;

    bool isValid() const { return width > 0 && height > 0; }
};

/// 整数除法：分母为 0 时返回 fallback，绝不触发 0xC0000094。
inline int SafeDiv(int numerator, int denominator, int fallback = 0)
{
    if (denominator == 0) {
        return fallback;
    }
    return numerator / denominator;
}

/// 整数取模同样会 idiv，分母为 0 时返回 fallback。
inline int SafeMod(int numerator, int denominator, int fallback = 0)
{
    if (denominator == 0) {
        return fallback;
    }
    return numerator % denominator;
}

/// 预览窗口和相机画面都必须有正的像素尺寸，才能进 SDK 的显示/缩放路径。
inline bool CanDisplay(Size2i window, Size2i image)
{
    return window.isValid() && image.isValid();
}

inline bool CanDisplay(int windowW, int windowH, int imageW, int imageH)
{
    return CanDisplay(Size2i{windowW, windowH}, Size2i{imageW, imageH});
}

/// 最小化、尚未 show、QSplitter 收起：不要把 0x0 的 HWND 传给 SDCamera。
inline bool ShouldHandleResize(int windowW, int windowH)
{
    return windowW > 0 && windowH > 0;
}

/**
 * 按比例把 src 装进 window（letterbox），中间乘法用 int64，避免溢出。
 * 任一尺寸 <= 0 时返回 false，不写输出、不除零。
 */
inline bool FitKeepAspect(int srcW, int srcH, int wndW, int wndH,
                          int& dstX, int& dstY, int& dstW, int& dstH)
{
    if (srcW <= 0 || srcH <= 0 || wndW <= 0 || wndH <= 0) {
        return false;
    }

    const std::int64_t srcW64 = srcW;
    const std::int64_t srcH64 = srcH;
    const std::int64_t wndW64 = wndW;
    const std::int64_t wndH64 = wndH;

    // srcW/srcH >= wndW/wndH  →  以窗口宽度为约束
    if (srcW64 * wndH64 >= wndW64 * srcH64) {
        dstW = wndW;
        dstH = static_cast<int>(wndW64 * srcH64 / srcW64);
    } else {
        dstH = wndH;
        dstW = static_cast<int>(wndH64 * srcW64 / srcH64);
    }

    if (dstW <= 0) {
        dstW = 1;
    }
    if (dstH <= 0) {
        dstH = 1;
    }

    dstX = (wndW - dstW) / 2;
    dstY = (wndH - dstH) / 2;
    return true;
}

/// 由整帧缓冲反推行跨距。height==0 时失败，避免 bufferBytes / 0。
inline bool RowStride(std::size_t bufferBytes, int height, int& stride)
{
    if (height <= 0) {
        return false;
    }
    stride = static_cast<int>(bufferBytes / static_cast<std::size_t>(height));
    return stride > 0;
}

/// 帧周期（微秒）→ FPS。periodUs==0 时返回 fallback。
inline int FpsFromPeriodUs(int periodUs, int fallback = 0)
{
    return SafeDiv(1000000, periodUs, fallback);
}

} // namespace sdc

#endif // SDCAMERA_SAFE_H
