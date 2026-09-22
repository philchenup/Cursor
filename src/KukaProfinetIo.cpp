#include "KukaProfinetIo.h"

#include <cstring>
#include <sstream>

namespace {

uint32_t bswap32(uint32_t v)
{
    return ((v & 0x000000FFu) << 24) | ((v & 0x0000FF00u) << 8) |
           ((v & 0x00FF0000u) >> 8) | ((v & 0xFF000000u) >> 24);
}

float loadF32(const uint8_t* p, KukaPnEndian endian)
{
    uint32_t bits = 0;
    std::memcpy(&bits, p, 4);
    if (endian == KukaPnEndian::Big) {
        bits = bswap32(bits);
    }
    float v = 0.f;
    std::memcpy(&v, &bits, 4);
    return v;
}

void storeF32(uint8_t* p, float v, KukaPnEndian endian)
{
    uint32_t bits = 0;
    std::memcpy(&bits, &v, 4);
    if (endian == KukaPnEndian::Big) {
        bits = bswap32(bits);
    }
    std::memcpy(p, &bits, 4);
}

float* fieldPtr(KukaAxisPose* pose, int index)
{
    return reinterpret_cast<float*>(pose) + index;
}

const float* fieldPtr(const KukaAxisPose& pose, int index)
{
    return reinterpret_cast<const float*>(&pose) + index;
}

}  // namespace

const std::vector<KukaPnMap>& kukaPnMap()
{
    static const std::vector<KukaPnMap> kMap = {
        {KukaPnField::E1, 0, 1, 1, "E1", "$AXIS_ACT.E1", "mm/deg"},
        {KukaPnField::E2, 4, 33, 33, "E2", "$AXIS_ACT.E2", "mm/deg"},
        {KukaPnField::E3, 8, 65, 65, "E3", "$AXIS_ACT.E3", "mm/deg"},
        {KukaPnField::A1, 12, 97, 97, "A1", "$AXIS_ACT.A1", "deg"},
        {KukaPnField::A2, 16, 129, 129, "A2", "$AXIS_ACT.A2", "deg"},
        {KukaPnField::A3, 20, 161, 161, "A3", "$AXIS_ACT.A3", "deg"},
        {KukaPnField::A4, 24, 193, 193, "A4", "$AXIS_ACT.A4", "deg"},
        {KukaPnField::A5, 28, 225, 225, "A5", "$AXIS_ACT.A5", "deg"},
        {KukaPnField::A6, 32, 257, 257, "A6", "$AXIS_ACT.A6", "deg"},
        {KukaPnField::X, 36, 289, 289, "X", "$POS_ACT.X", "mm"},
        {KukaPnField::Y, 40, 321, 321, "Y", "$POS_ACT.Y", "mm"},
        {KukaPnField::Z, 44, 353, 353, "Z", "$POS_ACT.Z", "mm"},
        {KukaPnField::A, 48, 385, 385, "A", "$POS_ACT.A", "deg"},
        {KukaPnField::B, 52, 417, 417, "B", "$POS_ACT.B", "deg"},
        {KukaPnField::C, 56, 449, 449, "C", "$POS_ACT.C", "deg"},
    };
    return kMap;
}

int kukaPnFieldOffset(KukaPnField field)
{
    const int i = static_cast<int>(field);
    if (i < 0 || i >= kKukaPnFieldCount) {
        return -1;
    }
    return i * kKukaPnWordBytes;
}

const char* kukaPnFieldName(KukaPnField field)
{
    const int i = static_cast<int>(field);
    if (i < 0 || i >= kKukaPnFieldCount) {
        return "";
    }
    return kukaPnMap()[static_cast<std::size_t>(i)].name;
}

bool decodeKukaPnAxisPose(const uint8_t* buf, std::size_t len, KukaAxisPose* out,
                          KukaPnEndian endian)
{
    if (buf == nullptr || out == nullptr || len < kKukaPnPayloadBytes) {
        return false;
    }
    for (int i = 0; i < kKukaPnFieldCount; ++i) {
        *fieldPtr(out, i) = loadF32(buf + i * kKukaPnWordBytes, endian);
    }
    return true;
}

bool encodeKukaPnAxisPose(const KukaAxisPose& in, uint8_t* buf, std::size_t len,
                          KukaPnEndian endian)
{
    if (buf == nullptr || len < kKukaPnPayloadBytes) {
        return false;
    }
    for (int i = 0; i < kKukaPnFieldCount; ++i) {
        storeF32(buf + i * kKukaPnWordBytes, *fieldPtr(in, i), endian);
    }
    if (len > kKukaPnPayloadBytes) {
        std::memset(buf + kKukaPnPayloadBytes, 0, len - kKukaPnPayloadBytes);
    }
    return true;
}

std::string kukaPnMapMarkdown()
{
    std::ostringstream oss;
    oss << "CIFX `DATALENGTH=64`。有效载荷 60 字节（15×FLOAT32），偏移 60–63 保留。\n\n"
        << "读：`xChannelIORead` ← KUKA `$OUT[1..480]`（`CAST_TO($AXIS_ACT/$POS_ACT)`）。\n"
        << "写：`xChannelIOWrite` → KUKA `$IN[1..480]`（`CAST_FROM` 后赋给目标轴/位姿）。\n\n"
        << "| 字节偏移 | 信号 | KRL | 单位 | $OUT 位 | $IN 位 |\n"
        << "| --- | --- | --- | --- | --- | --- |\n";
    for (const KukaPnMap& m : kukaPnMap()) {
        oss << "| " << m.byte_offset << " | " << m.name << " | `" << m.krl << "` | " << m.unit
            << " | " << m.out_bit_first << ".." << (m.out_bit_first + 31) << " | " << m.in_bit_first
            << ".." << (m.in_bit_first + 31) << " |\n";
    }
    return oss.str();
}

std::string kukaPnMapCsv()
{
    std::ostringstream oss;
    oss << "字节偏移,信号,KRL,单位,$OUT起始位,$IN起始位\n";
    for (const KukaPnMap& m : kukaPnMap()) {
        oss << m.byte_offset << ',' << m.name << ',' << m.krl << ',' << m.unit << ','
            << m.out_bit_first << ',' << m.in_bit_first << '\n';
    }
    return oss.str();
}
