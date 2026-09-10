#include "SDCameraSafe.h"

#include <cstdio>
#include <cstdlib>

namespace {

int g_failed = 0;

void Expect(bool cond, const char* expr, const char* file, int line)
{
    if (cond) {
        return;
    }
    std::fprintf(stderr, "FAIL %s:%d: %s\n", file, line, expr);
    ++g_failed;
}

#define EXPECT(cond) Expect(static_cast<bool>(cond), #cond, __FILE__, __LINE__)

void TestSafeDiv()
{
    EXPECT(sdc::SafeDiv(10, 2) == 5);
    EXPECT(sdc::SafeDiv(10, 0) == 0);
    EXPECT(sdc::SafeDiv(10, 0, -1) == -1);
    EXPECT(sdc::SafeDiv(-9, 3) == -3);
    EXPECT(sdc::SafeMod(10, 3) == 1);
    EXPECT(sdc::SafeMod(10, 0) == 0);
    EXPECT(sdc::SafeMod(10, 0, 7) == 7);
}

void TestCanDisplay()
{
    EXPECT(!sdc::CanDisplay(0, 480, 1920, 1080));
    EXPECT(!sdc::CanDisplay(640, 0, 1920, 1080));
    EXPECT(!sdc::CanDisplay(640, 480, 0, 1080));
    EXPECT(!sdc::CanDisplay(640, 480, 1920, 0));
    EXPECT(!sdc::CanDisplay(-1, 480, 1920, 1080));
    EXPECT(sdc::CanDisplay(640, 480, 1920, 1080));
    EXPECT(!sdc::ShouldHandleResize(0, 0));
    EXPECT(!sdc::ShouldHandleResize(800, 0));
    EXPECT(sdc::ShouldHandleResize(800, 600));
}

void TestFitKeepAspectRejectsZero()
{
    int x = -1, y = -1, w = -1, h = -1;
    EXPECT(!sdc::FitKeepAspect(0, 1080, 640, 480, x, y, w, h));
    EXPECT(!sdc::FitKeepAspect(1920, 0, 640, 480, x, y, w, h));
    EXPECT(!sdc::FitKeepAspect(1920, 1080, 0, 480, x, y, w, h));
    EXPECT(!sdc::FitKeepAspect(1920, 1080, 640, 0, x, y, w, h));
    EXPECT(x == -1 && y == -1 && w == -1 && h == -1);
}

void TestFitKeepAspectWideSource()
{
    int x = 0, y = 0, w = 0, h = 0;
    EXPECT(sdc::FitKeepAspect(1920, 1080, 640, 480, x, y, w, h));
    EXPECT(w == 640);
    EXPECT(h == 360);
    EXPECT(x == 0);
    EXPECT(y == 60);
}

void TestFitKeepAspectTallSource()
{
    int x = 0, y = 0, w = 0, h = 0;
    EXPECT(sdc::FitKeepAspect(1080, 1920, 640, 480, x, y, w, h));
    EXPECT(h == 480);
    EXPECT(w == 270);
    EXPECT(y == 0);
    EXPECT(x == 185);
}

void TestFitKeepAspectSameRatio()
{
    int x = 0, y = 0, w = 0, h = 0;
    EXPECT(sdc::FitKeepAspect(800, 600, 800, 600, x, y, w, h));
    EXPECT(x == 0 && y == 0 && w == 800 && h == 600);
}

void TestRowStrideAndFps()
{
    int stride = -1;
    EXPECT(!sdc::RowStride(1920 * 1080, 0, stride));
    EXPECT(stride == -1);
    EXPECT(sdc::RowStride(1920 * 1080, 1080, stride));
    EXPECT(stride == 1920);
    EXPECT(sdc::FpsFromPeriodUs(0) == 0);
    EXPECT(sdc::FpsFromPeriodUs(0, 30) == 30);
    EXPECT(sdc::FpsFromPeriodUs(40000) == 25);
}

} // namespace

int main()
{
    TestSafeDiv();
    TestCanDisplay();
    TestFitKeepAspectRejectsZero();
    TestFitKeepAspectWideSource();
    TestFitKeepAspectTallSource();
    TestFitKeepAspectSameRatio();
    TestRowStrideAndFps();

    if (g_failed != 0) {
        std::fprintf(stderr, "%d check(s) failed\n", g_failed);
        return EXIT_FAILURE;
    }
    std::puts("test_sdcamera_safe: all checks passed");
    return EXIT_SUCCESS;
}
