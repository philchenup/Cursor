#ifndef KUKA_WELD_COMM_H
#define KUKA_WELD_COMM_H

#include <cstdint>
#include <string>
#include <vector>

/**
 * 上位机编排焊接总流程的最小通讯表。
 *
 * 下发：参考起终点、焊枪姿态、焊速、摆动、寻缝使能。
 * 回传：激光是否找到起终点、找到的点、阶段。
 * 焊中纠偏走 RSI，不在本表闭环。
 */

enum class HostCmd : int32_t {
    Idle = 0,
    Reset = 1,
    Start = 6,              ///< 自动：寻起点 → TCP 到位 → 寻终点 → 焊到终点
    Pause = 7,
    Resume = 8,
    Stop = 9,
    GoHome = 11,
    AckFault = 12,
    FindStart = 13,         ///< 激光在焊枪前方寻起点
    MoveToFoundStart = 14,  ///< TCP 移到找到的起点
    FindEnd = 15,           ///< 激光寻终点
    WeldToFoundEnd = 16     ///< 插补到找到的终点（Weld_ArcEnable=1 则焊接）
};

enum class KukaPhase : int32_t {
    Disconnected = 0,
    Idle = 2,
    Ready = 4,
    SearchApproach = 19,
    FindingStart = 20,
    FoundStart = 21,
    MoveToFoundStart = 22,
    AtStart = 7,
    FindingEnd = 23,
    FoundEnd = 24,
    Welding = 10,
    WeldToFoundEnd = 25,
    ReturnHome = 15,
    JobDone = 16,
    Fault = 17,
    EStop = 18
};

enum class WeaveMode : int32_t {
    Straight = 0,
    Weave = 1
};

enum class WeaveType : int32_t {
    Sine = 0,
    Triangle = 1
};

enum class LaserMode : int32_t {
    Off = 0,
    Find = 1,
    FindAndTrack = 2
};

enum class CommDirection : int32_t {
    HostToKuka = 0,
    KukaToHost = 1
};

struct Xyz {
    float x = 0.f;
    float y = 0.f;
    float z = 0.f;
};

struct HostCyclic {
    int32_t heartbeat = 0;
    HostCmd cmd = HostCmd::Idle;
    int32_t cmd_seq = 0;
    int32_t seam_id = 0;
    Xyz ref_start;
    Xyz ref_end;
    float torch_a = 0.f;
    float torch_b = 90.f;
    float torch_c = 180.f;
    float weld_speed_mm_s = 10.f;
    WeaveMode weave_mode = WeaveMode::Straight;
    WeaveType weave_type = WeaveType::Sine;
    float amplitude_mm = 5.f;
    float chord_mm = 20.f;
    LaserMode laser_mode = LaserMode::Find;
    float laser_look_ahead_mm = 30.f;
    float laser_search_radius_mm = 20.f;
    int32_t laser_timeout_ms = 5000;
    int32_t arc_enable = 0;
};

struct KukaCyclic {
    int32_t heartbeat = 0;
    int32_t cmd_ack_seq = 0;
    KukaPhase phase = KukaPhase::Disconnected;
    int32_t estop = 0;
    int32_t msg_id = 0;
    int32_t laser_ready = 0;
    int32_t laser_start_valid = 0;
    int32_t laser_end_valid = 0;
    int32_t laser_lost = 0;
    Xyz found_start;
    Xyz found_end;
    int32_t job_done = 0;
};

struct CommSignal {
    int index;
    const char* type;
    const char* name;
    CommDirection direction;
    const char* meaning;
};

/** 每个信号按 EKI INT/REAL 对齐；BOOL 也按 INT 占 4 字节。 */
constexpr int kCommPackedWordBytes = 4;

/** 上位机→KUKA 下发打包字节数：23 信号 × 4 = 92。 */
constexpr int kHostToKukaPackedBytes = 92;

/** KUKA→上位机 读取打包字节数：16 信号 × 4 = 64。 */
constexpr int kKukaToHostPackedBytes = 64;

static_assert(sizeof(HostCyclic) == kHostToKukaPackedBytes,
              "HostCyclic packed size must stay 92 bytes");
static_assert(sizeof(KukaCyclic) == kKukaToHostPackedBytes,
              "KukaCyclic packed size must stay 64 bytes");

const std::vector<CommSignal>& kukaWeldCommSignals();

/** INT32 / FLOAT32 / BOOL 均为 4 字节。 */
int commSignalPackedBytes(const char* type);

/** 该方向整帧打包字节数。读取数据（KukaToHost）为 64。 */
int kukaWeldPackedBytes(CommDirection dir);

/** 信号在该方向帧内的字节偏移，找不到返回 -1。 */
int kukaWeldPackedOffset(const CommSignal& signal);

std::string commDirectionName(CommDirection dir);
std::string hostCmdName(HostCmd cmd);
std::string kukaPhaseName(KukaPhase phase);

std::string commTableMarkdown();
std::string commTableCsv();
std::string ekiConfigXml(const std::string& host_ip = "192.168.1.100", int port = 54600);

std::string encodeHostCyclicXml(const HostCyclic& msg);
std::string encodeKukaCyclicXml(const KukaCyclic& msg);

bool decodeHostCyclicXml(const std::string& xml, HostCyclic* msg);
bool decodeKukaCyclicXml(const std::string& xml, KukaCyclic* msg);

#endif // KUKA_WELD_COMM_H
