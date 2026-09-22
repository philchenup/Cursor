#ifndef KUKA_PROFINET_IO_H
#define KUKA_PROFINET_IO_H

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

/**
 * 上位机 CIFX 卡通过 PROFINET 读写 KUKA 过程数据。
 *
 * 一帧 15 个 FLOAT32（IEEE754）：
 *   E1 E2 E3 | A1 A2 A3 A4 A5 A6 | X Y Z A B C
 * 共 60 字节，放入 64 字节 IO 镜像（与 CIFX DATALENGTH 对齐，末尾 4 字节保留）。
 *
 * 方向（KUKA 作 PN Device、CIFX 作 Controller）：
 *   xChannelIORead  ← KUKA $OUT  （读外部轴/关节/TCP）
 *   xChannelIOWrite → KUKA $IN   （写下发目标）
 *
 * KUKA 侧用 32 位 SIGNAL + CAST_TO/CAST_FROM 搬运 REAL，不能直接把 REAL 接到 $IN/$OUT。
 */

enum class KukaPnEndian : int32_t {
    Little = 0,  ///< 默认：KRC SIGNAL 低位在前，Windows/CIFX 主机直接 memcpy
    Big = 1      ///< WorkVisual 若按大端映射 DWORD，解码时置此项
};

enum class KukaPnField : int32_t {
    E1 = 0,
    E2,
    E3,
    A1,
    A2,
    A3,
    A4,
    A5,
    A6,
    X,
    Y,
    Z,
    A,
    B,
    C
};

/** 3 外部轴 + 6 关节 + 6 TCP 位姿。 */
struct KukaAxisPose {
    float e1 = 0.f;
    float e2 = 0.f;
    float e3 = 0.f;
    float a1 = 0.f;
    float a2 = 0.f;
    float a3 = 0.f;
    float a4 = 0.f;
    float a5 = 0.f;
    float a6 = 0.f;
    float x = 0.f;
    float y = 0.f;
    float z = 0.f;
    float a = 0.f;
    float b = 0.f;
    float c = 0.f;
};

struct KukaPnMap {
    KukaPnField field;
    int byte_offset;     ///< CIFX 缓冲区字节偏移
    int out_bit_first;   ///< KUKA $OUT 起始位（读：机器人发出）
    int in_bit_first;    ///< KUKA $IN 起始位（写：上位机下发）
    const char* name;
    const char* krl;     ///< 对应 KRL 系统变量
    const char* unit;
};

constexpr int kKukaPnFieldCount = 15;
constexpr int kKukaPnWordBytes = 4;
constexpr int kKukaPnPayloadBytes = kKukaPnFieldCount * kKukaPnWordBytes;  // 60
constexpr int kKukaPnFrameBytes = 64;  ///< 与 MainWindow DATALENGTH 一致
constexpr int kKukaPnSignalBits = kKukaPnPayloadBytes * 8;                 // 480

static_assert(sizeof(KukaAxisPose) == kKukaPnPayloadBytes,
              "KukaAxisPose must stay 15 tightly packed floats");

const std::vector<KukaPnMap>& kukaPnMap();

int kukaPnFieldOffset(KukaPnField field);
const char* kukaPnFieldName(KukaPnField field);

/** 从 CIFX xChannelIORead 缓冲解码 KUKA 回传。len 至少 60。 */
bool decodeKukaPnAxisPose(const uint8_t* buf, std::size_t len, KukaAxisPose* out,
                          KukaPnEndian endian = KukaPnEndian::Little);

/** 编码后交给 xChannelIOWrite。len 至少 60；64 字节时末尾清零。 */
bool encodeKukaPnAxisPose(const KukaAxisPose& in, uint8_t* buf, std::size_t len,
                          KukaPnEndian endian = KukaPnEndian::Little);

std::string kukaPnMapMarkdown();
std::string kukaPnMapCsv();

#endif  // KUKA_PROFINET_IO_H
