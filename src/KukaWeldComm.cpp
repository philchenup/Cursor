#include "KukaWeldComm.h"

#include <cstdint>
#include <cstdlib>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

namespace {

const char* kDirHost = "上位机→KUKA";
const char* kDirKuka = "KUKA→上位机";

std::string xmlLeaf(const std::string& tag, const std::string& value)
{
    return "<" + tag + ">" + value + "</" + tag + ">";
}

std::string xmlInt(const char* tag, int32_t v)
{
    return xmlLeaf(tag, std::to_string(v));
}

std::string xmlReal(const char* tag, float v)
{
    std::ostringstream oss;
    oss << std::setprecision(9) << v;
    return xmlLeaf(tag, oss.str());
}

bool extract(const std::string& xml, const std::string& tag, std::string* out)
{
    const std::string open = "<" + tag + ">";
    const std::string close = "</" + tag + ">";
    const auto b = xml.find(open);
    if (b == std::string::npos) {
        return false;
    }
    const auto start = b + open.size();
    const auto e = xml.find(close, start);
    if (e == std::string::npos) {
        return false;
    }
    *out = xml.substr(start, e - start);
    return true;
}

bool extractInt(const std::string& xml, const char* tag, int32_t* v)
{
    std::string s;
    if (!extract(xml, tag, &s)) {
        return false;
    }
    *v = static_cast<int32_t>(std::strtol(s.c_str(), nullptr, 10));
    return true;
}

bool extractReal(const std::string& xml, const char* tag, float* v)
{
    std::string s;
    if (!extract(xml, tag, &s)) {
        return false;
    }
    *v = std::strtof(s.c_str(), nullptr);
    return true;
}

bool readXyz(const std::string& xml, const char* px, const char* py, const char* pz, Xyz* p)
{
    return extractReal(xml, px, &p->x) && extractReal(xml, py, &p->y) && extractReal(xml, pz, &p->z);
}

const char* ekiType(const char* plc_type)
{
    return (std::string(plc_type) == "FLOAT32") ? "REAL" : "INT";
}

std::string csvEscape(const std::string& s)
{
    if (s.find_first_of(",\"\n") == std::string::npos) {
        return s;
    }
    std::string out = "\"";
    for (char c : s) {
        if (c == '"') {
            out += "\"\"";
        } else {
            out += c;
        }
    }
    out += '"';
    return out;
}

void appendTable(std::ostringstream& oss, CommDirection dir)
{
    oss << "| 字节偏移 | 字节数 | 数据类型 | 信号名 | 含义 |\n| --- | --- | --- | --- | --- |\n";
    int offset = 0;
    for (const CommSignal& s : kukaWeldCommSignals()) {
        if (s.direction != dir) {
            continue;
        }
        const int n = commSignalPackedBytes(s.type);
        oss << "| " << offset << " | " << n << " | " << s.type << " | " << s.name
            << " | " << s.meaning << " |\n";
        offset += n;
    }
}

}  // namespace

int commSignalPackedBytes(const char* type)
{
    if (type == nullptr || type[0] == '\0') {
        return 0;
    }
    return kCommPackedWordBytes;
}

int kukaWeldPackedBytes(CommDirection dir)
{
    int n = 0;
    for (const CommSignal& s : kukaWeldCommSignals()) {
        if (s.direction == dir) {
            n += commSignalPackedBytes(s.type);
        }
    }
    return n;
}

int kukaWeldPackedOffset(const CommSignal& signal)
{
    int offset = 0;
    for (const CommSignal& s : kukaWeldCommSignals()) {
        if (s.direction != signal.direction) {
            continue;
        }
        if (s.index == signal.index) {
            return offset;
        }
        offset += commSignalPackedBytes(s.type);
    }
    return -1;
}

const std::vector<CommSignal>& kukaWeldCommSignals()
{
    static const std::vector<CommSignal> kSignals = {
        {1, "INT32", "Host_Heartbeat", CommDirection::HostToKuka, "上位机心跳"},
        {2, "INT32", "Host_Cmd", CommDirection::HostToKuka, "0空闲 1复位 6自动全流程 7暂停 8继续 9停止 11回Home 12故障确认 13寻起点 14TCP到找到的起点 15寻终点 16焊到找到的终点"},
        {3, "INT32", "Host_CmdSeq", CommDirection::HostToKuka, "命令序号，变化时执行一次"},
        {4, "INT32", "Seam_Id", CommDirection::HostToKuka, "焊缝号"},
        {5, "FLOAT32", "Seam_StartX", CommDirection::HostToKuka, "参考起点 X mm（寻缝搜索中心）"},
        {6, "FLOAT32", "Seam_StartY", CommDirection::HostToKuka, "参考起点 Y mm"},
        {7, "FLOAT32", "Seam_StartZ", CommDirection::HostToKuka, "参考起点 Z mm"},
        {8, "FLOAT32", "Seam_EndX", CommDirection::HostToKuka, "参考终点 X mm（寻缝搜索中心）"},
        {9, "FLOAT32", "Seam_EndY", CommDirection::HostToKuka, "参考终点 Y mm"},
        {10, "FLOAT32", "Seam_EndZ", CommDirection::HostToKuka, "参考终点 Z mm"},
        {11, "FLOAT32", "Torch_A", CommDirection::HostToKuka, "焊枪姿态 A deg"},
        {12, "FLOAT32", "Torch_B", CommDirection::HostToKuka, "焊枪姿态 B deg"},
        {13, "FLOAT32", "Torch_C", CommDirection::HostToKuka, "焊枪姿态 C deg"},
        {14, "FLOAT32", "Seam_WeldSpeed", CommDirection::HostToKuka, "焊接速度 mm/s"},
        {15, "INT32", "Seam_WeaveMode", CommDirection::HostToKuka, "0 直线焊  1 摆动焊"},
        {16, "INT32", "Seam_WeaveType", CommDirection::HostToKuka, "0 正弦  1 三角"},
        {17, "FLOAT32", "Seam_Amplitude", CommDirection::HostToKuka, "摆动幅度 mm"},
        {18, "FLOAT32", "Seam_Chord", CommDirection::HostToKuka, "摆动弦长 mm"},
        {19, "INT32", "Laser_Mode", CommDirection::HostToKuka, "0 不用激光  1 寻缝  2 寻缝+焊中RSI跟踪"},
        {20, "FLOAT32", "Laser_LookAhead", CommDirection::HostToKuka, "激光超前距 mm（焊枪前方）"},
        {21, "FLOAT32", "Laser_SearchRadius", CommDirection::HostToKuka, "相对参考点的寻缝范围 mm"},
        {22, "INT32", "Laser_Timeout", CommDirection::HostToKuka, "寻缝超时 ms"},
        {23, "BOOL", "Weld_ArcEnable", CommDirection::HostToKuka, "1 焊接到终点  0 只移动到终点"},

        {24, "INT32", "Kuka_Heartbeat", CommDirection::KukaToHost, "机器人心跳"},
        {25, "INT32", "Kuka_CmdAck", CommDirection::KukaToHost, "已接受的命令序号"},
        {26, "INT32", "Kuka_Phase", CommDirection::KukaToHost, "寻缝/找到起点/TCP到位/寻终点/焊接/故障"},
        {27, "BOOL", "Kuka_EStop", CommDirection::KukaToHost, "急停"},
        {28, "INT32", "Kuka_MsgId", CommDirection::KukaToHost, "故障码，0 为正常"},
        {29, "BOOL", "Laser_Ready", CommDirection::KukaToHost, "激光寻缝器就绪"},
        {30, "BOOL", "Laser_StartValid", CommDirection::KukaToHost, "已找到起点，Found_Start* 有效"},
        {31, "BOOL", "Laser_EndValid", CommDirection::KukaToHost, "已找到终点，Found_End* 有效"},
        {32, "BOOL", "Laser_Lost", CommDirection::KukaToHost, "寻缝或跟踪丢失"},
        {33, "FLOAT32", "Found_StartX", CommDirection::KukaToHost, "激光找到的起点 X mm"},
        {34, "FLOAT32", "Found_StartY", CommDirection::KukaToHost, "激光找到的起点 Y mm"},
        {35, "FLOAT32", "Found_StartZ", CommDirection::KukaToHost, "激光找到的起点 Z mm"},
        {36, "FLOAT32", "Found_EndX", CommDirection::KukaToHost, "激光找到的终点 X mm"},
        {37, "FLOAT32", "Found_EndY", CommDirection::KukaToHost, "激光找到的终点 Y mm"},
        {38, "FLOAT32", "Found_EndZ", CommDirection::KukaToHost, "激光找到的终点 Z mm"},
        {39, "BOOL", "Kuka_JobDone", CommDirection::KukaToHost, "本焊缝流程完成"},
    };
    return kSignals;
}

std::string commDirectionName(CommDirection dir)
{
    return dir == CommDirection::HostToKuka ? kDirHost : kDirKuka;
}

std::string hostCmdName(HostCmd cmd)
{
    switch (cmd) {
    case HostCmd::Idle: return "Idle";
    case HostCmd::Reset: return "Reset";
    case HostCmd::Start: return "Start";
    case HostCmd::Pause: return "Pause";
    case HostCmd::Resume: return "Resume";
    case HostCmd::Stop: return "Stop";
    case HostCmd::GoHome: return "GoHome";
    case HostCmd::AckFault: return "AckFault";
    case HostCmd::FindStart: return "FindStart";
    case HostCmd::MoveToFoundStart: return "MoveToFoundStart";
    case HostCmd::FindEnd: return "FindEnd";
    case HostCmd::WeldToFoundEnd: return "WeldToFoundEnd";
    }
    return "Unknown";
}

std::string kukaPhaseName(KukaPhase phase)
{
    switch (phase) {
    case KukaPhase::Disconnected: return "Disconnected";
    case KukaPhase::Idle: return "Idle";
    case KukaPhase::Ready: return "Ready";
    case KukaPhase::SearchApproach: return "SearchApproach";
    case KukaPhase::FindingStart: return "FindingStart";
    case KukaPhase::FoundStart: return "FoundStart";
    case KukaPhase::MoveToFoundStart: return "MoveToFoundStart";
    case KukaPhase::AtStart: return "AtStart";
    case KukaPhase::FindingEnd: return "FindingEnd";
    case KukaPhase::FoundEnd: return "FoundEnd";
    case KukaPhase::Welding: return "Welding";
    case KukaPhase::WeldToFoundEnd: return "WeldToFoundEnd";
    case KukaPhase::ReturnHome: return "ReturnHome";
    case KukaPhase::JobDone: return "JobDone";
    case KukaPhase::Fault: return "Fault";
    case KukaPhase::EStop: return "EStop";
    }
    return "Unknown";
}

std::string commTableMarkdown()
{
    std::ostringstream oss;
    oss << "INT32 / FLOAT32 / BOOL 均按 4 字节对齐（EKI INT/REAL；BOOL 以 INT 传输）。\n\n"
        << "| 方向 | 信号数 | 打包字节数 |\n| --- | --- | --- |\n"
        << "| 上位机→KUKA（下发） | 23 | " << kHostToKukaPackedBytes << " |\n"
        << "| KUKA→上位机（读取） | 16 | **" << kKukaToHostPackedBytes << "** |\n\n"
        << "配置通讯“读取数据字节大小”时填 **" << kKukaToHostPackedBytes
        << "**。EKI XML 线上是变长文本，socket 接收缓冲建议 ≥ 4096。\n\n";
    oss << "## 上位机 → KUKA（92 字节）\n\n";
    appendTable(oss, CommDirection::HostToKuka);
    oss << "\n## KUKA → 上位机（读取，64 字节）\n\n";
    appendTable(oss, CommDirection::KukaToHost);
    return oss.str();
}

std::string commTableCsv()
{
    std::ostringstream oss;
    oss << "字节偏移,字节数,数据类型,信号名,含义,方向\n";
    int host_off = 0;
    int kuka_off = 0;
    for (const CommSignal& s : kukaWeldCommSignals()) {
        const int n = commSignalPackedBytes(s.type);
        int& off = (s.direction == CommDirection::HostToKuka) ? host_off : kuka_off;
        oss << off << ',' << n << ',' << s.type << ',' << s.name << ','
            << csvEscape(s.meaning) << ',' << csvEscape(commDirectionName(s.direction))
            << '\n';
        off += n;
    }
    return oss.str();
}

std::string ekiConfigXml(const std::string& host_ip, int port)
{
    std::ostringstream oss;
    oss << "<?xml version=\"1.0\" encoding=\"utf-8\"?>\n"
        << "<ETHERNETKRL>\n"
        << "  <CONFIGURATION>\n"
        << "    <EXTERNAL>\n"
        << "      <IP>" << host_ip << "</IP>\n"
        << "      <PORT>" << port << "</PORT>\n"
        << "      <TYPE>Client</TYPE>\n"
        << "    </EXTERNAL>\n"
        << "    <INTERNAL>\n"
        << "      <ENVIRONMENT>Program</ENVIRONMENT>\n"
        << "      <BUFFERING Mode=\"FIFO\" Limit=\"10\"/>\n"
        << "      <ALIVE Set_Flag=\"2688\" Ping=\"2000\" Timeout=\"6000\"/>\n"
        << "    </INTERNAL>\n"
        << "  </CONFIGURATION>\n"
        << "  <RECEIVE>\n"
        << "    <XML>\n";
    for (const CommSignal& s : kukaWeldCommSignals()) {
        if (s.direction == CommDirection::HostToKuka) {
            oss << "      <ELEMENT Tag=\"" << s.name << "\" Type=\"" << ekiType(s.type) << "\"/>\n";
        }
    }
    oss << "    </XML>\n"
        << "  </RECEIVE>\n"
        << "  <SEND>\n"
        << "    <XML>\n";
    for (const CommSignal& s : kukaWeldCommSignals()) {
        if (s.direction == CommDirection::KukaToHost) {
            oss << "      <ELEMENT Tag=\"" << s.name << "\" Type=\"" << ekiType(s.type) << "\"/>\n";
        }
    }
    oss << "    </XML>\n"
        << "  </SEND>\n"
        << "</ETHERNETKRL>\n";
    return oss.str();
}

std::string encodeHostCyclicXml(const HostCyclic& msg)
{
    std::ostringstream oss;
    oss << "<Host>";
    oss << xmlInt("Host_Heartbeat", msg.heartbeat);
    oss << xmlInt("Host_Cmd", static_cast<int32_t>(msg.cmd));
    oss << xmlInt("Host_CmdSeq", msg.cmd_seq);
    oss << xmlInt("Seam_Id", msg.seam_id);
    oss << xmlReal("Seam_StartX", msg.ref_start.x);
    oss << xmlReal("Seam_StartY", msg.ref_start.y);
    oss << xmlReal("Seam_StartZ", msg.ref_start.z);
    oss << xmlReal("Seam_EndX", msg.ref_end.x);
    oss << xmlReal("Seam_EndY", msg.ref_end.y);
    oss << xmlReal("Seam_EndZ", msg.ref_end.z);
    oss << xmlReal("Torch_A", msg.torch_a);
    oss << xmlReal("Torch_B", msg.torch_b);
    oss << xmlReal("Torch_C", msg.torch_c);
    oss << xmlReal("Seam_WeldSpeed", msg.weld_speed_mm_s);
    oss << xmlInt("Seam_WeaveMode", static_cast<int32_t>(msg.weave_mode));
    oss << xmlInt("Seam_WeaveType", static_cast<int32_t>(msg.weave_type));
    oss << xmlReal("Seam_Amplitude", msg.amplitude_mm);
    oss << xmlReal("Seam_Chord", msg.chord_mm);
    oss << xmlInt("Laser_Mode", static_cast<int32_t>(msg.laser_mode));
    oss << xmlReal("Laser_LookAhead", msg.laser_look_ahead_mm);
    oss << xmlReal("Laser_SearchRadius", msg.laser_search_radius_mm);
    oss << xmlInt("Laser_Timeout", msg.laser_timeout_ms);
    oss << xmlInt("Weld_ArcEnable", msg.arc_enable);
    oss << "</Host>";
    return oss.str();
}

std::string encodeKukaCyclicXml(const KukaCyclic& msg)
{
    std::ostringstream oss;
    oss << "<Kuka>";
    oss << xmlInt("Kuka_Heartbeat", msg.heartbeat);
    oss << xmlInt("Kuka_CmdAck", msg.cmd_ack_seq);
    oss << xmlInt("Kuka_Phase", static_cast<int32_t>(msg.phase));
    oss << xmlInt("Kuka_EStop", msg.estop);
    oss << xmlInt("Kuka_MsgId", msg.msg_id);
    oss << xmlInt("Laser_Ready", msg.laser_ready);
    oss << xmlInt("Laser_StartValid", msg.laser_start_valid);
    oss << xmlInt("Laser_EndValid", msg.laser_end_valid);
    oss << xmlInt("Laser_Lost", msg.laser_lost);
    oss << xmlReal("Found_StartX", msg.found_start.x);
    oss << xmlReal("Found_StartY", msg.found_start.y);
    oss << xmlReal("Found_StartZ", msg.found_start.z);
    oss << xmlReal("Found_EndX", msg.found_end.x);
    oss << xmlReal("Found_EndY", msg.found_end.y);
    oss << xmlReal("Found_EndZ", msg.found_end.z);
    oss << xmlInt("Kuka_JobDone", msg.job_done);
    oss << "</Kuka>";
    return oss.str();
}

bool decodeHostCyclicXml(const std::string& xml, HostCyclic* msg)
{
    if (!msg) {
        return false;
    }
    HostCyclic out;
    int32_t cmd = 0;
    int32_t weave_mode = 0;
    int32_t weave_type = 0;
    int32_t laser_mode = 0;
    if (!(extractInt(xml, "Host_Heartbeat", &out.heartbeat)
          && extractInt(xml, "Host_Cmd", &cmd)
          && extractInt(xml, "Host_CmdSeq", &out.cmd_seq)
          && extractInt(xml, "Seam_Id", &out.seam_id)
          && readXyz(xml, "Seam_StartX", "Seam_StartY", "Seam_StartZ", &out.ref_start)
          && readXyz(xml, "Seam_EndX", "Seam_EndY", "Seam_EndZ", &out.ref_end)
          && extractReal(xml, "Torch_A", &out.torch_a)
          && extractReal(xml, "Torch_B", &out.torch_b)
          && extractReal(xml, "Torch_C", &out.torch_c)
          && extractReal(xml, "Seam_WeldSpeed", &out.weld_speed_mm_s)
          && extractInt(xml, "Seam_WeaveMode", &weave_mode)
          && extractInt(xml, "Seam_WeaveType", &weave_type)
          && extractReal(xml, "Seam_Amplitude", &out.amplitude_mm)
          && extractReal(xml, "Seam_Chord", &out.chord_mm)
          && extractInt(xml, "Laser_Mode", &laser_mode)
          && extractReal(xml, "Laser_LookAhead", &out.laser_look_ahead_mm)
          && extractReal(xml, "Laser_SearchRadius", &out.laser_search_radius_mm)
          && extractInt(xml, "Laser_Timeout", &out.laser_timeout_ms)
          && extractInt(xml, "Weld_ArcEnable", &out.arc_enable))) {
        return false;
    }
    out.cmd = static_cast<HostCmd>(cmd);
    out.weave_mode = static_cast<WeaveMode>(weave_mode);
    out.weave_type = static_cast<WeaveType>(weave_type);
    out.laser_mode = static_cast<LaserMode>(laser_mode);
    *msg = out;
    return true;
}

bool decodeKukaCyclicXml(const std::string& xml, KukaCyclic* msg)
{
    if (!msg) {
        return false;
    }
    KukaCyclic out;
    int32_t phase = 0;
    if (!(extractInt(xml, "Kuka_Heartbeat", &out.heartbeat)
          && extractInt(xml, "Kuka_CmdAck", &out.cmd_ack_seq)
          && extractInt(xml, "Kuka_Phase", &phase)
          && extractInt(xml, "Kuka_EStop", &out.estop)
          && extractInt(xml, "Kuka_MsgId", &out.msg_id)
          && extractInt(xml, "Laser_Ready", &out.laser_ready)
          && extractInt(xml, "Laser_StartValid", &out.laser_start_valid)
          && extractInt(xml, "Laser_EndValid", &out.laser_end_valid)
          && extractInt(xml, "Laser_Lost", &out.laser_lost)
          && readXyz(xml, "Found_StartX", "Found_StartY", "Found_StartZ", &out.found_start)
          && readXyz(xml, "Found_EndX", "Found_EndY", "Found_EndZ", &out.found_end)
          && extractInt(xml, "Kuka_JobDone", &out.job_done))) {
        return false;
    }
    out.phase = static_cast<KukaPhase>(phase);
    *msg = out;
    return true;
}
