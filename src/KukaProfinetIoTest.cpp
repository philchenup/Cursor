#include "KukaProfinetIo.h"

#include <cmath>
#include <cstring>
#include <iostream>

namespace {

int g_failed = 0;

void expect(bool cond, const char* msg)
{
    if (!cond) {
        std::cerr << "FAIL: " << msg << '\n';
        ++g_failed;
    }
}

bool near(float a, float b, float eps = 1e-5f)
{
    return std::fabs(a - b) <= eps;
}

bool poseEq(const KukaAxisPose& a, const KukaAxisPose& b)
{
    return near(a.e1, b.e1) && near(a.e2, b.e2) && near(a.e3, b.e3) && near(a.a1, b.a1) &&
           near(a.a2, b.a2) && near(a.a3, b.a3) && near(a.a4, b.a4) && near(a.a5, b.a5) &&
           near(a.a6, b.a6) && near(a.x, b.x) && near(a.y, b.y) && near(a.z, b.z) && near(a.a, b.a) &&
           near(a.b, b.b) && near(a.c, b.c);
}

}  // namespace

int main()
{
    expect(kukaPnMap().size() == 15, "15 fields");
    expect(kKukaPnPayloadBytes == 60, "payload 60");
    expect(kKukaPnFrameBytes == 64, "frame 64");
    expect(kukaPnFieldOffset(KukaPnField::E1) == 0, "E1 at 0");
    expect(kukaPnFieldOffset(KukaPnField::A1) == 12, "A1 at 12");
    expect(kukaPnFieldOffset(KukaPnField::X) == 36, "X at 36");
    expect(kukaPnFieldOffset(KukaPnField::C) == 56, "C at 56");
    expect(std::strcmp(kukaPnFieldName(KukaPnField::E2), "E2") == 0, "name E2");

    KukaAxisPose src;
    src.e1 = 100.5f;
    src.e2 = -20.f;
    src.e3 = 3.25f;
    src.a1 = 10.f;
    src.a2 = -45.f;
    src.a3 = 90.f;
    src.a4 = 1.f;
    src.a5 = 2.f;
    src.a6 = 180.f;
    src.x = 1234.5f;
    src.y = -10.25f;
    src.z = 800.f;
    src.a = 0.f;
    src.b = 90.f;
    src.c = -180.f;

    uint8_t frame[kKukaPnFrameBytes];
    std::memset(frame, 0xFF, sizeof(frame));
    expect(encodeKukaPnAxisPose(src, frame, sizeof(frame)), "encode 64");
    expect(frame[60] == 0 && frame[63] == 0, "padding cleared");

    KukaAxisPose dst;
    expect(decodeKukaPnAxisPose(frame, sizeof(frame), &dst), "decode");
    expect(poseEq(src, dst), "roundtrip LE");

    uint8_t be[kKukaPnPayloadBytes];
    expect(encodeKukaPnAxisPose(src, be, sizeof(be), KukaPnEndian::Big), "encode BE");
    expect(be[0] != frame[0] || src.e1 == 0.f, "endian differs");
    KukaAxisPose dst_be;
    expect(decodeKukaPnAxisPose(be, sizeof(be), &dst_be, KukaPnEndian::Big), "decode BE");
    expect(poseEq(src, dst_be), "roundtrip BE");

    expect(!decodeKukaPnAxisPose(frame, 59, &dst), "short read");
    expect(!encodeKukaPnAxisPose(src, frame, 59), "short write");
    expect(!decodeKukaPnAxisPose(nullptr, 64, &dst), "null buf");
    expect(!decodeKukaPnAxisPose(frame, 64, nullptr), "null out");

    const std::string md = kukaPnMapMarkdown();
    expect(md.find("| 0 | E1 |") != std::string::npos, "md E1");
    expect(md.find("| 56 | C |") != std::string::npos, "md C");
    expect(kukaPnMapCsv().find("字节偏移,信号") == 0, "csv header");

    if (g_failed != 0) {
        std::cerr << g_failed << " assertion(s) failed\n";
        return 1;
    }
    std::cout << "KukaProfinetIo tests passed (60/64 bytes, 15 fields)\n";
    return 0;
}
