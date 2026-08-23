#include "YsGripperProtocol.h"

#include <cstdio>
#include <cstdlib>

namespace {

void expect_eq(int line, unsigned actual, unsigned expected, const char* what)
{
    if (actual != expected) {
        std::fprintf(stderr, "test_protocol.cpp:%d: %s expected 0x%02X, got 0x%02X\n",
            line, what, expected, actual);
        std::exit(1);
    }
}

} // namespace

int main()
{
    const auto open = ys_gripper::buildOpenFrame(800);
    expect_eq(__LINE__, static_cast<unsigned>(open.size()), 8u, "open size");
    expect_eq(__LINE__, open[0], 0xEB, "open header0");
    expect_eq(__LINE__, open[1], 0x90, "open header1");
    expect_eq(__LINE__, open[2], 0x01, "open id");
    expect_eq(__LINE__, open[3], 0x03, "open len");
    expect_eq(__LINE__, open[4], 0x11, "open cmd");
    expect_eq(__LINE__, open[5], 0x20, "open speed_lo"); // 800 = 0x0320
    expect_eq(__LINE__, open[6], 0x03, "open speed_hi");
    expect_eq(__LINE__, open[7], static_cast<unsigned>((0x01 + 0x03 + 0x11 + 0x20 + 0x03) & 0xFF),
        "open checksum");

    const auto close = ys_gripper::buildCloseFrame(800, 100);
    expect_eq(__LINE__, static_cast<unsigned>(close.size()), 10u, "close size");
    expect_eq(__LINE__, close[0], 0xEB, "close header0");
    expect_eq(__LINE__, close[4], 0x10, "close cmd");
    expect_eq(__LINE__, close[7], 100u, "close power_lo");
    expect_eq(__LINE__, close[8], 0x00, "close power_hi");
    expect_eq(__LINE__, close[9],
        static_cast<unsigned>((0x01 + 0x05 + 0x10 + 0x20 + 0x03 + 100 + 0x00) & 0xFF),
        "close checksum");

    return 0;
}
