# Cursor

## SDCamera 整数除零（0xC0000094）

`SDCamera.dll` 在 `mainwindow.exe` 里抛出未经处理异常 `0xC0000094`，是整数除以 0。
SDK 显示缩放、行跨距、帧率换算都会做整数除法，下面任一值为 0 就会崩：

- 预览控件尚未 `show`、最小化、停靠栏收起 → 窗口宽/高为 0
- 相机未 Open 或读分辨率失败 → 图像宽/高为 0
- 帧周期 / 曝光未初始化 → `1000000 / periodUs`

在调用 DLL 之前用 `sdc::CanDisplay` / `sdc::FitKeepAspect` 挡掉；若 DLL 是自己编的，显示路径改走 `FitKeepAspect`，不要写 `wndH * srcW / srcH`。

```cpp
#include "SDCameraSafe.h"

if (!sdc::CanDisplay(widget->width(), widget->height(), imageW, imageH)) {
    return; // 不要 StartGrab / Display / NotifyDisplaySize
}

int x, y, w, h;
if (sdc::FitKeepAspect(imageW, imageH, widget->width(), widget->height(),
                      x, y, w, h)) {
    // 用 dst 矩形画到 HWND，srcH/srcW 已保证 > 0
}
```
