#ifndef KUKA_WELD_COMM_H
#define KUKA_WELD_COMM_H

#include <cstdint>
#include <string>
#include <vector>

/**
 * 上位机 ↔ KUKA 焊接通讯数据模型。
 *
 * 字段与焊缝工艺表（起点/终点/内缩/焊接速度/摆动/多层多道）以及
 * V 坡口多层多道焊道规划对齐，供 Ethernet KRL XML 与 CSV 通讯表使用。
 */

// ---------------------------------------------------------------------------
// 枚举：命令、工艺阶段、摆动、多层、焊道类型、运行模式
// ---------------------------------------------------------------------------

enum class HostCmd : int32_t {
    Idle = 0,
    Reset = 1,
    DownloadJob = 2,
    DownloadSeam = 3,
    DownloadPass = 4,
    DownloadTraj = 5,
    Start = 6,
    Pause = 7,
    Resume = 8,
    Stop = 9,
    ArcOff = 10,
    GoHome = 11,
    AckFault = 12
};

enum class KukaPhase : int32_t {
    Disconnected = 0,
    Connected = 1,
    Idle = 2,
    Downloading = 3,
    Ready = 4,
    RailMove = 5,
    Approach = 6,
    AtStart = 7,
    GasPreflow = 8,
    ArcStarting = 9,
    Welding = 10,
    Crater = 11,
    GasPostflow = 12,
    Retract = 13,
    BetweenPass = 14,
    ReturnHome = 15,
    JobDone = 16,
    Fault = 17,
    EStop = 18
};

enum class WeaveMode : int32_t {
    Straight = 0,  ///< 直线焊
    Weave = 1      ///< 摆动焊
};

enum class WeaveType : int32_t {
    Sine = 0,      ///< 正弦
    Triangle = 1   ///< 三角
};

enum class MultiMode : int32_t {
    Single = 0,    ///< 单层单道
    Multi = 1      ///< 多层多道
};

enum class PassKind : int32_t {
    Root = 0,      ///< 打底
    Fill = 1,      ///< 填充
    Cap = 2        ///< 盖面
};

enum class KukaOpMode : int32_t {
    T1 = 0,
    T2 = 1,
    Aut = 2,
    Ext = 3
};

enum class CommDirection : int32_t {
    HostToKuka = 0,
    KukaToHost = 1
};

// ---------------------------------------------------------------------------
// 位姿与工艺结构（KUKA E6POS：X Y Z A B C + 地轨 E1）
// ---------------------------------------------------------------------------

struct KukaPose {
    float x = 0.f;   ///< mm，基坐标系
    float y = 0.f;
    float z = 0.f;
    float a = 0.f;   ///< deg，KUKA ABC（绕 Z、Y、X）
    float b = 0.f;
    float c = 0.f;
    float e1 = 0.f;  ///< mm，外部轴/地轨
};

/// 与 PLC/示教器通讯表一致的运动字：外部轴速度加速度、机器人速度加速度、E1–E3、XYZABC、J1–J6。
struct MotionBlock {
    float extern_speed = 0.f;
    float extern_acc = 0.f;
    float robot_speed = 0.f;
    float robot_acc = 0.f;
    float e1 = 0.f;
    float e2 = 0.f;
    float e3 = 0.f;
    float x = 0.f;
    float y = 0.f;
    float z = 0.f;
    float a = 0.f;
    float b = 0.f;
    float c = 0.f;
    float j1 = 0.f;
    float j2 = 0.f;
    float j3 = 0.f;
    float j4 = 0.f;
    float j5 = 0.f;
    float j6 = 0.f;
};

struct HostJobHeader {
    int32_t job_id = 0;
    int32_t seam_count = 0;
    int32_t pass_count = 0;
    int32_t tool_no = 1;     ///< $TOOL[]
    int32_t base_no = 0;     ///< $BASE[]
    float override_pct = 100.f;
    float approach_mm = 50.f;
    float retract_mm = 50.f;
};

/// 与焊缝工艺表一行对应：序号、起点、终点、内缩、速度、摆动、多层多道。
struct SeamRecipe {
    int32_t seam_id = 0;
    KukaPose start;
    KukaPose end;
    float inset_mm = 0.f;
    float speed_mm_s = 10.f;
    WeaveMode weave_mode = WeaveMode::Straight;
    WeaveType weave_type = WeaveType::Sine;
    float amplitude_mm = 5.f;
    float chord_mm = 20.f;
    MultiMode multi_mode = MultiMode::Single;
    float thickness_mm = 0.f;
    float groove_deg = 0.f;
    float fitup_gap_mm = 0.f;
    float penetration_mm = 0.f;
};

/// V 坡口规划出的一条焊道（层/道序/打底填充盖面）。
struct PassRecipe {
    int32_t seam_id = 0;
    int32_t layer = 1;
    int32_t local_index = 1;
    int32_t sequence = 1;
    PassKind kind = PassKind::Root;
    KukaPose start;
    KukaPose end;
    float speed_mm_s = 10.f;
};

struct TrajPoint {
    int32_t index = 0;
    int32_t count = 0;
    KukaPose pose;
    float speed_mm_s = 10.f;
    int32_t flag = 0;  ///< bit0 起弧点, bit1 收弧点, bit2 本焊道末点
};

struct WelderRecipe {
    int32_t arc_enable = 0;
    int32_t gas_enable = 0;
    float current_a = 0.f;
    float voltage_v = 0.f;
    float wire_m_min = 0.f;
    int32_t gas_preflow_ms = 200;
    int32_t gas_postflow_ms = 400;
    int32_t crater_ms = 200;
};

struct HostCyclic {
    int32_t heartbeat = 0;
    HostCmd cmd = HostCmd::Idle;
    int32_t cmd_seq = 0;
    HostJobHeader job;
    MotionBlock motion;
    SeamRecipe seam;
    PassRecipe pass;
    TrajPoint traj;
    WelderRecipe welder;
};

struct KukaCyclic {
    int32_t heartbeat = 0;
    int32_t cmd_ack_seq = 0;
    KukaPhase phase = KukaPhase::Disconnected;
    KukaOpMode op_mode = KukaOpMode::T1;
    int32_t pro_active = 0;
    int32_t drives_on = 0;
    int32_t estop = 0;
    int32_t msg_id = 0;
    int32_t seam_id = 0;
    int32_t layer = 0;
    int32_t pass_seq = 0;
    int32_t traj_index = 0;
    MotionBlock motion;
    int32_t arc_on = 0;
    int32_t collision = 0;
    int32_t download_ok = 0;
    int32_t job_done = 0;
    float progress_pct = 0.f;
};

// ---------------------------------------------------------------------------
// 通讯表行（通讯数据结构表）
// ---------------------------------------------------------------------------

struct CommSignal {
    int index;                 ///< 序号
    const char* type;          ///< FLOAT32 / INT32 / BOOL
    const char* name;          ///< 如 Extern_Speed、Robot_X
    CommDirection direction;
    const char* meaning;       ///< 参数含义
};

const std::vector<CommSignal>& kukaWeldCommSignals();

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

/// 轨迹点组内逗号、组间分号、结尾句点，与既有 CommConfig 默认分隔符一致。
std::string formatTrajCsv(const std::vector<TrajPoint>& points);

/// 内缩后的焊缝起终点（沿焊缝方向各收回 inset_mm）。
KukaPose insetPose(const KukaPose& start, const KukaPose& end, float inset_mm, bool from_start);

#endif // KUKA_WELD_COMM_H
