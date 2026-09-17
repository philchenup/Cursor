#include "KukaWeldComm.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

namespace {

constexpr float kEps = 1e-12f;

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

void appendPoseFields(std::ostringstream& oss, const KukaPose& p)
{
    oss << xmlReal("X", p.x) << xmlReal("Y", p.y) << xmlReal("Z", p.z);
    oss << xmlReal("A", p.a) << xmlReal("B", p.b) << xmlReal("C", p.c);
    oss << xmlReal("E1", p.e1);
}

void appendPose(std::ostringstream& oss, const char* tag, const KukaPose& p)
{
    oss << '<' << tag << '>';
    appendPoseFields(oss, p);
    oss << "</" << tag << '>';
}

std::vector<std::string> splitPath(const std::string& path)
{
    std::vector<std::string> parts;
    std::string cur;
    for (char c : path) {
        if (c == '/') {
            if (!cur.empty()) {
                parts.push_back(cur);
                cur.clear();
            }
        } else {
            cur.push_back(c);
        }
    }
    if (!cur.empty()) {
        parts.push_back(cur);
    }
    return parts;
}

bool extractPath(const std::string& xml, const std::string& path, std::string* out)
{
    std::string current = xml;
    const std::vector<std::string> parts = splitPath(path);
    if (parts.empty()) {
        return false;
    }
    for (size_t i = 0; i < parts.size(); ++i) {
        std::string inner;
        if (!extract(current, parts[i], &inner)) {
            return false;
        }
        if (i + 1 == parts.size()) {
            *out = inner;
            return true;
        }
        current = inner;
    }
    return false;
}

bool extractPathInt(const std::string& xml, const char* path, int32_t* v)
{
    std::string s;
    if (!extractPath(xml, path, &s)) {
        return false;
    }
    *v = static_cast<int32_t>(std::strtol(s.c_str(), nullptr, 10));
    return true;
}

bool extractPathReal(const std::string& xml, const char* path, float* v)
{
    std::string s;
    if (!extractPath(xml, path, &s)) {
        return false;
    }
    *v = std::strtof(s.c_str(), nullptr);
    return true;
}

bool readPosePath(const std::string& xml, const std::string& prefix, KukaPose* p)
{
    return extractPathReal(xml, (prefix + "/X").c_str(), &p->x)
        && extractPathReal(xml, (prefix + "/Y").c_str(), &p->y)
        && extractPathReal(xml, (prefix + "/Z").c_str(), &p->z)
        && extractPathReal(xml, (prefix + "/A").c_str(), &p->a)
        && extractPathReal(xml, (prefix + "/B").c_str(), &p->b)
        && extractPathReal(xml, (prefix + "/C").c_str(), &p->c)
        && extractPathReal(xml, (prefix + "/E1").c_str(), &p->e1);
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

}  // namespace

const std::vector<CommSignal>& kukaWeldCommSignals()
{
    static const std::vector<CommSignal> kSignals = {
        // ----- 握手与作业控制（上位机→KUKA）-----
        {1, "握手控制", "Heartbeat", "Host/Heartbeat", CommDirection::HostToKuka, "INT", 4, "-", "0..2^31-1", "全程", "上位机心跳，周期递增，KUKA 用于判断通讯中断"},
        {2, "握手控制", "Cmd", "Host/Cmd", CommDirection::HostToKuka, "INT", 4, "-", "0..12", "作业控制", "命令字 Idle Reset Download Start Pause Resume Stop ArcOff GoHome AckFault"},
        {3, "握手控制", "CmdSeq", "Host/CmdSeq", CommDirection::HostToKuka, "INT", 4, "-", "1..2^31-1", "作业控制", "命令序号，KUKA 仅在序号变化时执行一次"},
        {4, "握手控制", "JobId", "Host/JobId", CommDirection::HostToKuka, "INT", 4, "-", "1..9999", "下载作业", "焊接作业号"},
        {5, "握手控制", "SeamCount", "Host/SeamCount", CommDirection::HostToKuka, "INT", 4, "-", "1..256", "下载作业", "本作业焊缝条数"},
        {6, "握手控制", "PassCount", "Host/PassCount", CommDirection::HostToKuka, "INT", 4, "-", "1..1024", "下载作业", "本作业焊道总数（单层单道时等于焊缝数）"},
        {7, "握手控制", "ToolNo", "Host/ToolNo", CommDirection::HostToKuka, "INT", 4, "-", "1..16", "下载作业", "焊枪 TCP 对应 $TOOL 编号"},
        {8, "握手控制", "BaseNo", "Host/BaseNo", CommDirection::HostToKuka, "INT", 4, "-", "0..32", "下载作业", "工件坐标系 $BASE 编号"},
        {9, "握手控制", "OverridePct", "Host/OverridePct", CommDirection::HostToKuka, "REAL", 4, "%", "1..100", "运行", "建议速度倍率；实际仍受示教器倍率限制"},
        {10, "握手控制", "ApproachMm", "Host/ApproachMm", CommDirection::HostToKuka, "REAL", 4, "mm", "0..500", "接近", "焊枪沿 TCP -Z 的接近高度"},
        {11, "握手控制", "RetractMm", "Host/RetractMm", CommDirection::HostToKuka, "REAL", 4, "mm", "0..500", "回撤", "收弧后沿 TCP -Z 的回撤高度"},

        // ----- 焊缝工艺（与上位机焊缝表列对应）-----
        {12, "焊缝工艺", "SeamId", "Host/Seam/Id", CommDirection::HostToKuka, "INT", 4, "-", "1..256", "下载焊缝", "焊缝序号，对应工艺表「序号」"},
        {13, "焊缝工艺", "StartX", "Host/Seam/Start/X", CommDirection::HostToKuka, "REAL", 4, "mm", "±10000", "下载焊缝", "原始起点 X（内缩前），对应「起点」"},
        {14, "焊缝工艺", "StartY", "Host/Seam/Start/Y", CommDirection::HostToKuka, "REAL", 4, "mm", "±10000", "下载焊缝", "原始起点 Y"},
        {15, "焊缝工艺", "StartZ", "Host/Seam/Start/Z", CommDirection::HostToKuka, "REAL", 4, "mm", "±10000", "下载焊缝", "原始起点 Z"},
        {16, "焊缝工艺", "StartA", "Host/Seam/Start/A", CommDirection::HostToKuka, "REAL", 4, "deg", "±180", "下载焊缝", "起点姿态 A（绕 Z）"},
        {17, "焊缝工艺", "StartB", "Host/Seam/Start/B", CommDirection::HostToKuka, "REAL", 4, "deg", "±180", "下载焊缝", "起点姿态 B（绕 Y）"},
        {18, "焊缝工艺", "StartC", "Host/Seam/Start/C", CommDirection::HostToKuka, "REAL", 4, "deg", "±180", "下载焊缝", "起点姿态 C（绕 X）"},
        {19, "焊缝工艺", "StartE1", "Host/Seam/Start/E1", CommDirection::HostToKuka, "REAL", 4, "mm", "地轨行程", "下载焊缝", "地轨：先对齐焊点 Y，再手臂到位"},
        {20, "焊缝工艺", "EndX", "Host/Seam/End/X", CommDirection::HostToKuka, "REAL", 4, "mm", "±10000", "下载焊缝", "原始终点 X，对应「终点」"},
        {21, "焊缝工艺", "EndY", "Host/Seam/End/Y", CommDirection::HostToKuka, "REAL", 4, "mm", "±10000", "下载焊缝", "原始终点 Y"},
        {22, "焊缝工艺", "EndZ", "Host/Seam/End/Z", CommDirection::HostToKuka, "REAL", 4, "mm", "±10000", "下载焊缝", "原始终点 Z"},
        {23, "焊缝工艺", "EndA", "Host/Seam/End/A", CommDirection::HostToKuka, "REAL", 4, "deg", "±180", "下载焊缝", "终点姿态 A"},
        {24, "焊缝工艺", "EndB", "Host/Seam/End/B", CommDirection::HostToKuka, "REAL", 4, "deg", "±180", "下载焊缝", "终点姿态 B"},
        {25, "焊缝工艺", "EndC", "Host/Seam/End/C", CommDirection::HostToKuka, "REAL", 4, "deg", "±180", "下载焊缝", "终点姿态 C"},
        {26, "焊缝工艺", "EndE1", "Host/Seam/End/E1", CommDirection::HostToKuka, "REAL", 4, "mm", "地轨行程", "下载焊缝", "终点地轨位置，通常与起点 E1 相同"},
        {27, "焊缝工艺", "InsetMm", "Host/Seam/InsetMm", CommDirection::HostToKuka, "REAL", 4, "mm", "0..半缝长", "下载焊缝", "沿焊缝从起终点各收回，对应「内缩」"},
        {28, "焊缝工艺", "SpeedMmS", "Host/Seam/SpeedMmS", CommDirection::HostToKuka, "REAL", 4, "mm/s", "0..1e5", "焊接", "焊接速度，对应「焊接速度」，默认 10 mm/s"},
        {29, "焊缝工艺", "WeaveMode", "Host/Seam/WeaveMode", CommDirection::HostToKuka, "INT", 4, "-", "0直线 1摆动", "焊接", "对应「摆动方式」"},
        {30, "焊缝工艺", "WeaveType", "Host/Seam/WeaveType", CommDirection::HostToKuka, "INT", 4, "-", "0正弦 1三角", "摆动焊", "对应「摆动类型」；直线焊时忽略"},
        {31, "焊缝工艺", "AmplitudeMm", "Host/Seam/AmplitudeMm", CommDirection::HostToKuka, "REAL", 4, "mm", "0..1e4", "摆动焊", "侧向峰值偏移，对应「幅度」，默认 5 mm"},
        {32, "焊缝工艺", "ChordMm", "Host/Seam/ChordMm", CommDirection::HostToKuka, "REAL", 4, "mm", ">0", "摆动焊", "一个摆动周期沿焊缝的长度，对应「弦长」，默认 20 mm"},
        {33, "焊缝工艺", "MultiMode", "Host/Seam/MultiMode", CommDirection::HostToKuka, "INT", 4, "-", "0单层单道 1多层多道", "焊接", "对应「多层多道」"},
        {34, "焊缝工艺", "ThicknessMm", "Host/Seam/ThicknessMm", CommDirection::HostToKuka, "REAL", 4, "mm", "0..1e4", "多层多道", "板厚，对应「板厚」"},
        {35, "焊缝工艺", "GrooveDeg", "Host/Seam/GrooveDeg", CommDirection::HostToKuka, "REAL", 4, "deg", "0..90", "多层多道", "坡口角度，对应「坡口角度」"},
        {36, "焊缝工艺", "FitUpGapMm", "Host/Seam/FitUpGapMm", CommDirection::HostToKuka, "REAL", 4, "mm", "0..1e3", "多层多道", "装配间隙，对应「装配间隙」"},
        {37, "焊缝工艺", "PenetrationMm", "Host/Seam/PenetrationMm", CommDirection::HostToKuka, "REAL", 4, "mm", "0..1e4", "多层多道", "熔深，对应「熔深」"},

        // ----- 焊道（多层多道规划结果）-----
        {38, "焊道", "PassSeamId", "Host/Pass/SeamId", CommDirection::HostToKuka, "INT", 4, "-", "1..256", "下载焊道", "本焊道所属焊缝"},
        {39, "焊道", "PassLayer", "Host/Pass/Layer", CommDirection::HostToKuka, "INT", 4, "-", "1..64", "下载焊道", "层号，打底层为 1"},
        {40, "焊道", "PassLocal", "Host/Pass/LocalIndex", CommDirection::HostToKuka, "INT", 4, "-", "1..32", "下载焊道", "层内道号"},
        {41, "焊道", "PassSeq", "Host/Pass/Sequence", CommDirection::HostToKuka, "INT", 4, "-", "1..1024", "下载焊道", "全局焊接顺序"},
        {42, "焊道", "PassKind", "Host/Pass/Kind", CommDirection::HostToKuka, "INT", 4, "-", "0打底 1填充 2盖面", "下载焊道", "焊道类型，决定焊枪倾角与规范"},
        {43, "焊道", "PassStartX", "Host/Pass/Start/X", CommDirection::HostToKuka, "REAL", 4, "mm", "±10000", "下载焊道", "本焊道起点 X（已含层/道偏移）"},
        {44, "焊道", "PassStartY", "Host/Pass/Start/Y", CommDirection::HostToKuka, "REAL", 4, "mm", "±10000", "下载焊道", "本焊道起点 Y"},
        {45, "焊道", "PassStartZ", "Host/Pass/Start/Z", CommDirection::HostToKuka, "REAL", 4, "mm", "±10000", "下载焊道", "本焊道起点 Z"},
        {46, "焊道", "PassStartA", "Host/Pass/Start/A", CommDirection::HostToKuka, "REAL", 4, "deg", "±180", "下载焊道", "本焊道焊枪姿态 A"},
        {47, "焊道", "PassStartB", "Host/Pass/Start/B", CommDirection::HostToKuka, "REAL", 4, "deg", "±180", "下载焊道", "本焊道焊枪姿态 B"},
        {48, "焊道", "PassStartC", "Host/Pass/Start/C", CommDirection::HostToKuka, "REAL", 4, "deg", "±180", "下载焊道", "本焊道焊枪姿态 C"},
        {49, "焊道", "PassStartE1", "Host/Pass/Start/E1", CommDirection::HostToKuka, "REAL", 4, "mm", "地轨行程", "下载焊道", "本焊道地轨"},
        {50, "焊道", "PassEndX", "Host/Pass/End/X", CommDirection::HostToKuka, "REAL", 4, "mm", "±10000", "下载焊道", "本焊道终点 X"},
        {51, "焊道", "PassEndY", "Host/Pass/End/Y", CommDirection::HostToKuka, "REAL", 4, "mm", "±10000", "下载焊道", "本焊道终点 Y"},
        {52, "焊道", "PassEndZ", "Host/Pass/End/Z", CommDirection::HostToKuka, "REAL", 4, "mm", "±10000", "下载焊道", "本焊道终点 Z"},
        {53, "焊道", "PassEndA", "Host/Pass/End/A", CommDirection::HostToKuka, "REAL", 4, "deg", "±180", "下载焊道", "本焊道终点姿态 A"},
        {54, "焊道", "PassEndB", "Host/Pass/End/B", CommDirection::HostToKuka, "REAL", 4, "deg", "±180", "下载焊道", "本焊道终点姿态 B"},
        {55, "焊道", "PassEndC", "Host/Pass/End/C", CommDirection::HostToKuka, "REAL", 4, "deg", "±180", "下载焊道", "本焊道终点姿态 C"},
        {56, "焊道", "PassEndE1", "Host/Pass/End/E1", CommDirection::HostToKuka, "REAL", 4, "mm", "地轨行程", "下载焊道", "本焊道终点地轨"},
        {57, "焊道", "PassSpeedMmS", "Host/Pass/SpeedMmS", CommDirection::HostToKuka, "REAL", 4, "mm/s", "0..1e5", "焊接", "本焊道焊接速度，可覆盖焊缝默认速度"},

        // ----- 轨迹点（摆动插值后）-----
        {58, "轨迹", "TrajIndex", "Host/Traj/Index", CommDirection::HostToKuka, "INT", 4, "-", "0..N-1", "下载轨迹", "当前轨迹点序号"},
        {59, "轨迹", "TrajCount", "Host/Traj/Count", CommDirection::HostToKuka, "INT", 4, "-", "2..4096", "下载轨迹", "本焊道轨迹点总数"},
        {60, "轨迹", "TrajX", "Host/Traj/X", CommDirection::HostToKuka, "REAL", 4, "mm", "±10000", "下载轨迹", "插值点 X（摆动后 TCP）"},
        {61, "轨迹", "TrajY", "Host/Traj/Y", CommDirection::HostToKuka, "REAL", 4, "mm", "±10000", "下载轨迹", "插值点 Y"},
        {62, "轨迹", "TrajZ", "Host/Traj/Z", CommDirection::HostToKuka, "REAL", 4, "mm", "±10000", "下载轨迹", "插值点 Z"},
        {63, "轨迹", "TrajA", "Host/Traj/A", CommDirection::HostToKuka, "REAL", 4, "deg", "±180", "下载轨迹", "插值点姿态 A"},
        {64, "轨迹", "TrajB", "Host/Traj/B", CommDirection::HostToKuka, "REAL", 4, "deg", "±180", "下载轨迹", "插值点姿态 B"},
        {65, "轨迹", "TrajC", "Host/Traj/C", CommDirection::HostToKuka, "REAL", 4, "deg", "±180", "下载轨迹", "插值点姿态 C"},
        {66, "轨迹", "TrajE1", "Host/Traj/E1", CommDirection::HostToKuka, "REAL", 4, "mm", "地轨行程", "下载轨迹", "插值点地轨"},
        {67, "轨迹", "TrajSpeedMmS", "Host/Traj/SpeedMmS", CommDirection::HostToKuka, "REAL", 4, "mm/s", "0..1e5", "焊接", "该点进给速度"},
        {68, "轨迹", "TrajFlag", "Host/Traj/Flag", CommDirection::HostToKuka, "INT", 4, "-", "bit 掩码", "起弧/收弧", "bit0=起弧点 bit1=收弧点 bit2=本焊道末点"},

        // ----- 焊机规范（经 KUKA IO/模拟量到焊机）-----
        {69, "焊机", "ArcEnable", "Host/Welder/ArcEnable", CommDirection::HostToKuka, "BOOL", 4, "-", "0/1", "起弧", "允许起弧；KUKA 在 AtStart 后置位焊机起弧输出"},
        {70, "焊机", "GasEnable", "Host/Welder/GasEnable", CommDirection::HostToKuka, "BOOL", 4, "-", "0/1", "气体", "允许送气"},
        {71, "焊机", "CurrentA", "Host/Welder/CurrentA", CommDirection::HostToKuka, "REAL", 4, "A", "0..500", "焊接", "焊接电流设定"},
        {72, "焊机", "VoltageV", "Host/Welder/VoltageV", CommDirection::HostToKuka, "REAL", 4, "V", "0..50", "焊接", "电弧电压设定"},
        {73, "焊机", "WireMMin", "Host/Welder/WireMMin", CommDirection::HostToKuka, "REAL", 4, "m/min", "0..25", "焊接", "送丝速度"},
        {74, "焊机", "GasPreflowMs", "Host/Welder/GasPreflowMs", CommDirection::HostToKuka, "INT", 4, "ms", "0..5000", "气体预吹", "起弧前保护气时间"},
        {75, "焊机", "GasPostflowMs", "Host/Welder/GasPostflowMs", CommDirection::HostToKuka, "INT", 4, "ms", "0..5000", "气体滞后", "收弧后保护气时间"},
        {76, "焊机", "CraterMs", "Host/Welder/CraterMs", CommDirection::HostToKuka, "INT", 4, "ms", "0..2000", "收弧", "填弧坑时间"},

        // ----- KUKA 状态回传 -----
        {77, "状态回传", "KukaHeartbeat", "Kuka/Heartbeat", CommDirection::KukaToHost, "INT", 4, "-", "0..2^31-1", "全程", "机器人心跳"},
        {78, "状态回传", "CmdAckSeq", "Kuka/CmdAckSeq", CommDirection::KukaToHost, "INT", 4, "-", "0..2^31-1", "作业控制", "已接受/已执行的命令序号"},
        {79, "状态回传", "Phase", "Kuka/Phase", CommDirection::KukaToHost, "INT", 4, "-", "0..18", "全程", "工艺阶段：地轨/接近/到位/预吹/起弧/焊接/收弧/回撤/回Home/故障"},
        {80, "状态回传", "OpMode", "Kuka/OpMode", CommDirection::KukaToHost, "INT", 4, "-", "0T1 1T2 2AUT 3EXT", "联机", "运行模式，正式焊接应为 EXT"},
        {81, "状态回传", "ProActive", "Kuka/ProActive", CommDirection::KukaToHost, "BOOL", 4, "-", "0/1", "联机", "解释器程序正在运行"},
        {82, "状态回传", "DrivesOn", "Kuka/DrivesOn", CommDirection::KukaToHost, "BOOL", 4, "-", "0/1", "联机", "驱动使能"},
        {83, "状态回传", "EStop", "Kuka/EStop", CommDirection::KukaToHost, "BOOL", 4, "-", "0/1", "安全", "急停"},
        {84, "状态回传", "MsgId", "Kuka/MsgId", CommDirection::KukaToHost, "INT", 4, "-", "0=无", "故障", "KUKA 报警号或自定义故障码"},
        {85, "状态回传", "FbSeamId", "Kuka/SeamId", CommDirection::KukaToHost, "INT", 4, "-", "0..256", "焊接", "当前焊缝"},
        {86, "状态回传", "FbLayer", "Kuka/Layer", CommDirection::KukaToHost, "INT", 4, "-", "0..64", "多层多道", "当前层"},
        {87, "状态回传", "FbPassSeq", "Kuka/PassSeq", CommDirection::KukaToHost, "INT", 4, "-", "0..1024", "多层多道", "当前焊道全局序号"},
        {88, "状态回传", "FbTrajIndex", "Kuka/TrajIndex", CommDirection::KukaToHost, "INT", 4, "-", "0..N", "焊接", "当前轨迹点"},
        {89, "状态回传", "TcpX", "Kuka/Tcp/X", CommDirection::KukaToHost, "REAL", 4, "mm", "±10000", "全程", "实际 TCP X"},
        {90, "状态回传", "TcpY", "Kuka/Tcp/Y", CommDirection::KukaToHost, "REAL", 4, "mm", "±10000", "全程", "实际 TCP Y"},
        {91, "状态回传", "TcpZ", "Kuka/Tcp/Z", CommDirection::KukaToHost, "REAL", 4, "mm", "±10000", "全程", "实际 TCP Z"},
        {92, "状态回传", "TcpA", "Kuka/Tcp/A", CommDirection::KukaToHost, "REAL", 4, "deg", "±180", "全程", "实际姿态 A"},
        {93, "状态回传", "TcpB", "Kuka/Tcp/B", CommDirection::KukaToHost, "REAL", 4, "deg", "±180", "全程", "实际姿态 B"},
        {94, "状态回传", "TcpC", "Kuka/Tcp/C", CommDirection::KukaToHost, "REAL", 4, "deg", "±180", "全程", "实际姿态 C"},
        {95, "状态回传", "TcpE1", "Kuka/Tcp/E1", CommDirection::KukaToHost, "REAL", 4, "mm", "地轨行程", "全程", "实际地轨"},
        {96, "状态回传", "ActSpeedMmS", "Kuka/ActSpeedMmS", CommDirection::KukaToHost, "REAL", 4, "mm/s", "≥0", "焊接", "实际焊接速度"},
        {97, "状态回传", "ArcOn", "Kuka/ArcOn", CommDirection::KukaToHost, "BOOL", 4, "-", "0/1", "起弧", "焊机起弧成功反馈"},
        {98, "状态回传", "Collision", "Kuka/Collision", CommDirection::KukaToHost, "BOOL", 4, "-", "0/1", "安全", "碰撞/力矩超限"},
        {99, "状态回传", "DownloadOk", "Kuka/DownloadOk", CommDirection::KukaToHost, "BOOL", 4, "-", "0/1", "下载作业", "作业/焊缝/焊道/轨迹下载完成"},
        {100, "状态回传", "JobDone", "Kuka/JobDone", CommDirection::KukaToHost, "BOOL", 4, "-", "0/1", "作业完成", "全部焊缝焊完并已回 Home"},
        {101, "状态回传", "ProgressPct", "Kuka/ProgressPct", CommDirection::KukaToHost, "REAL", 4, "%", "0..100", "全程", "当前作业进度"},
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
    case HostCmd::DownloadJob: return "DownloadJob";
    case HostCmd::DownloadSeam: return "DownloadSeam";
    case HostCmd::DownloadPass: return "DownloadPass";
    case HostCmd::DownloadTraj: return "DownloadTraj";
    case HostCmd::Start: return "Start";
    case HostCmd::Pause: return "Pause";
    case HostCmd::Resume: return "Resume";
    case HostCmd::Stop: return "Stop";
    case HostCmd::ArcOff: return "ArcOff";
    case HostCmd::GoHome: return "GoHome";
    case HostCmd::AckFault: return "AckFault";
    }
    return "Unknown";
}

std::string kukaPhaseName(KukaPhase phase)
{
    switch (phase) {
    case KukaPhase::Disconnected: return "Disconnected";
    case KukaPhase::Connected: return "Connected";
    case KukaPhase::Idle: return "Idle";
    case KukaPhase::Downloading: return "Downloading";
    case KukaPhase::Ready: return "Ready";
    case KukaPhase::RailMove: return "RailMove";
    case KukaPhase::Approach: return "Approach";
    case KukaPhase::AtStart: return "AtStart";
    case KukaPhase::GasPreflow: return "GasPreflow";
    case KukaPhase::ArcStarting: return "ArcStarting";
    case KukaPhase::Welding: return "Welding";
    case KukaPhase::Crater: return "Crater";
    case KukaPhase::GasPostflow: return "GasPostflow";
    case KukaPhase::Retract: return "Retract";
    case KukaPhase::BetweenPass: return "BetweenPass";
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
    oss << "| 序号 | 分组 | 信号名 | XPath | 方向 | KRL类型 | 字节 | 单位 | 取值 | 工艺步骤 | 说明 |\n";
    oss << "| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |\n";
    for (const CommSignal& s : kukaWeldCommSignals()) {
        oss << "| " << s.index
            << " | " << s.group
            << " | " << s.name
            << " | `" << s.xpath << "`"
            << " | " << commDirectionName(s.direction)
            << " | " << s.krl_type
            << " | " << s.size_bytes
            << " | " << s.unit
            << " | " << s.range
            << " | " << s.process_step
            << " | " << s.description
            << " |\n";
    }
    return oss.str();
}

std::string commTableCsv()
{
    std::ostringstream oss;
    oss << "序号,分组,信号名,XPath,方向,KRL类型,字节,单位,取值,工艺步骤,说明\n";
    for (const CommSignal& s : kukaWeldCommSignals()) {
        oss << s.index << ','
            << csvEscape(s.group) << ','
            << csvEscape(s.name) << ','
            << csvEscape(s.xpath) << ','
            << csvEscape(commDirectionName(s.direction)) << ','
            << csvEscape(s.krl_type) << ','
            << s.size_bytes << ','
            << csvEscape(s.unit) << ','
            << csvEscape(s.range) << ','
            << csvEscape(s.process_step) << ','
            << csvEscape(s.description) << '\n';
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
        if (s.direction != CommDirection::HostToKuka) {
            continue;
        }
        const char* t = (std::string(s.krl_type) == "BOOL") ? "INT" : s.krl_type;
        oss << "      <ELEMENT Tag=\"" << s.xpath << "\" Type=\"" << t << "\"/>\n";
    }
    oss << "    </XML>\n"
        << "  </RECEIVE>\n"
        << "  <SEND>\n"
        << "    <XML>\n";
    for (const CommSignal& s : kukaWeldCommSignals()) {
        if (s.direction != CommDirection::KukaToHost) {
            continue;
        }
        const char* t = (std::string(s.krl_type) == "BOOL") ? "INT" : s.krl_type;
        oss << "      <ELEMENT Tag=\"" << s.xpath << "\" Type=\"" << t << "\"/>\n";
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
    oss << xmlInt("Heartbeat", msg.heartbeat);
    oss << xmlInt("Cmd", static_cast<int32_t>(msg.cmd));
    oss << xmlInt("CmdSeq", msg.cmd_seq);
    oss << xmlInt("JobId", msg.job.job_id);
    oss << xmlInt("SeamCount", msg.job.seam_count);
    oss << xmlInt("PassCount", msg.job.pass_count);
    oss << xmlInt("ToolNo", msg.job.tool_no);
    oss << xmlInt("BaseNo", msg.job.base_no);
    oss << xmlReal("OverridePct", msg.job.override_pct);
    oss << xmlReal("ApproachMm", msg.job.approach_mm);
    oss << xmlReal("RetractMm", msg.job.retract_mm);

    oss << "<Seam>";
    oss << xmlInt("Id", msg.seam.seam_id);
    appendPose(oss, "Start", msg.seam.start);
    appendPose(oss, "End", msg.seam.end);
    oss << xmlReal("InsetMm", msg.seam.inset_mm);
    oss << xmlReal("SpeedMmS", msg.seam.speed_mm_s);
    oss << xmlInt("WeaveMode", static_cast<int32_t>(msg.seam.weave_mode));
    oss << xmlInt("WeaveType", static_cast<int32_t>(msg.seam.weave_type));
    oss << xmlReal("AmplitudeMm", msg.seam.amplitude_mm);
    oss << xmlReal("ChordMm", msg.seam.chord_mm);
    oss << xmlInt("MultiMode", static_cast<int32_t>(msg.seam.multi_mode));
    oss << xmlReal("ThicknessMm", msg.seam.thickness_mm);
    oss << xmlReal("GrooveDeg", msg.seam.groove_deg);
    oss << xmlReal("FitUpGapMm", msg.seam.fitup_gap_mm);
    oss << xmlReal("PenetrationMm", msg.seam.penetration_mm);
    oss << "</Seam>";

    oss << "<Pass>";
    oss << xmlInt("SeamId", msg.pass.seam_id);
    oss << xmlInt("Layer", msg.pass.layer);
    oss << xmlInt("LocalIndex", msg.pass.local_index);
    oss << xmlInt("Sequence", msg.pass.sequence);
    oss << xmlInt("Kind", static_cast<int32_t>(msg.pass.kind));
    appendPose(oss, "Start", msg.pass.start);
    appendPose(oss, "End", msg.pass.end);
    oss << xmlReal("SpeedMmS", msg.pass.speed_mm_s);
    oss << "</Pass>";

    oss << "<Traj>";
    oss << xmlInt("Index", msg.traj.index);
    oss << xmlInt("Count", msg.traj.count);
    appendPoseFields(oss, msg.traj.pose);
    oss << xmlReal("SpeedMmS", msg.traj.speed_mm_s);
    oss << xmlInt("Flag", msg.traj.flag);
    oss << "</Traj>";

    oss << "<Welder>";
    oss << xmlInt("ArcEnable", msg.welder.arc_enable);
    oss << xmlInt("GasEnable", msg.welder.gas_enable);
    oss << xmlReal("CurrentA", msg.welder.current_a);
    oss << xmlReal("VoltageV", msg.welder.voltage_v);
    oss << xmlReal("WireMMin", msg.welder.wire_m_min);
    oss << xmlInt("GasPreflowMs", msg.welder.gas_preflow_ms);
    oss << xmlInt("GasPostflowMs", msg.welder.gas_postflow_ms);
    oss << xmlInt("CraterMs", msg.welder.crater_ms);
    oss << "</Welder>";
    oss << "</Host>";
    return oss.str();
}

std::string encodeKukaCyclicXml(const KukaCyclic& msg)
{
    std::ostringstream oss;
    oss << "<Kuka>";
    oss << xmlInt("Heartbeat", msg.heartbeat);
    oss << xmlInt("CmdAckSeq", msg.cmd_ack_seq);
    oss << xmlInt("Phase", static_cast<int32_t>(msg.phase));
    oss << xmlInt("OpMode", static_cast<int32_t>(msg.op_mode));
    oss << xmlInt("ProActive", msg.pro_active);
    oss << xmlInt("DrivesOn", msg.drives_on);
    oss << xmlInt("EStop", msg.estop);
    oss << xmlInt("MsgId", msg.msg_id);
    oss << xmlInt("SeamId", msg.seam_id);
    oss << xmlInt("Layer", msg.layer);
    oss << xmlInt("PassSeq", msg.pass_seq);
    oss << xmlInt("TrajIndex", msg.traj_index);
    appendPose(oss, "Tcp", msg.tcp);
    oss << xmlReal("ActSpeedMmS", msg.act_speed_mm_s);
    oss << xmlInt("ArcOn", msg.arc_on);
    oss << xmlInt("Collision", msg.collision);
    oss << xmlInt("DownloadOk", msg.download_ok);
    oss << xmlInt("JobDone", msg.job_done);
    oss << xmlReal("ProgressPct", msg.progress_pct);
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
    int32_t multi_mode = 0;
    int32_t kind = 0;
    if (!(extractPathInt(xml, "Host/Heartbeat", &out.heartbeat)
          && extractPathInt(xml, "Host/Cmd", &cmd)
          && extractPathInt(xml, "Host/CmdSeq", &out.cmd_seq)
          && extractPathInt(xml, "Host/JobId", &out.job.job_id)
          && extractPathInt(xml, "Host/SeamCount", &out.job.seam_count)
          && extractPathInt(xml, "Host/PassCount", &out.job.pass_count)
          && extractPathInt(xml, "Host/ToolNo", &out.job.tool_no)
          && extractPathInt(xml, "Host/BaseNo", &out.job.base_no)
          && extractPathReal(xml, "Host/OverridePct", &out.job.override_pct)
          && extractPathReal(xml, "Host/ApproachMm", &out.job.approach_mm)
          && extractPathReal(xml, "Host/RetractMm", &out.job.retract_mm)
          && extractPathInt(xml, "Host/Seam/Id", &out.seam.seam_id)
          && readPosePath(xml, "Host/Seam/Start", &out.seam.start)
          && readPosePath(xml, "Host/Seam/End", &out.seam.end)
          && extractPathReal(xml, "Host/Seam/InsetMm", &out.seam.inset_mm)
          && extractPathReal(xml, "Host/Seam/SpeedMmS", &out.seam.speed_mm_s)
          && extractPathInt(xml, "Host/Seam/WeaveMode", &weave_mode)
          && extractPathInt(xml, "Host/Seam/WeaveType", &weave_type)
          && extractPathReal(xml, "Host/Seam/AmplitudeMm", &out.seam.amplitude_mm)
          && extractPathReal(xml, "Host/Seam/ChordMm", &out.seam.chord_mm)
          && extractPathInt(xml, "Host/Seam/MultiMode", &multi_mode)
          && extractPathReal(xml, "Host/Seam/ThicknessMm", &out.seam.thickness_mm)
          && extractPathReal(xml, "Host/Seam/GrooveDeg", &out.seam.groove_deg)
          && extractPathReal(xml, "Host/Seam/FitUpGapMm", &out.seam.fitup_gap_mm)
          && extractPathReal(xml, "Host/Seam/PenetrationMm", &out.seam.penetration_mm)
          && extractPathInt(xml, "Host/Pass/SeamId", &out.pass.seam_id)
          && extractPathInt(xml, "Host/Pass/Layer", &out.pass.layer)
          && extractPathInt(xml, "Host/Pass/LocalIndex", &out.pass.local_index)
          && extractPathInt(xml, "Host/Pass/Sequence", &out.pass.sequence)
          && extractPathInt(xml, "Host/Pass/Kind", &kind)
          && readPosePath(xml, "Host/Pass/Start", &out.pass.start)
          && readPosePath(xml, "Host/Pass/End", &out.pass.end)
          && extractPathReal(xml, "Host/Pass/SpeedMmS", &out.pass.speed_mm_s)
          && extractPathInt(xml, "Host/Traj/Index", &out.traj.index)
          && extractPathInt(xml, "Host/Traj/Count", &out.traj.count)
          && readPosePath(xml, "Host/Traj", &out.traj.pose)
          && extractPathReal(xml, "Host/Traj/SpeedMmS", &out.traj.speed_mm_s)
          && extractPathInt(xml, "Host/Traj/Flag", &out.traj.flag)
          && extractPathInt(xml, "Host/Welder/ArcEnable", &out.welder.arc_enable)
          && extractPathInt(xml, "Host/Welder/GasEnable", &out.welder.gas_enable)
          && extractPathReal(xml, "Host/Welder/CurrentA", &out.welder.current_a)
          && extractPathReal(xml, "Host/Welder/VoltageV", &out.welder.voltage_v)
          && extractPathReal(xml, "Host/Welder/WireMMin", &out.welder.wire_m_min)
          && extractPathInt(xml, "Host/Welder/GasPreflowMs", &out.welder.gas_preflow_ms)
          && extractPathInt(xml, "Host/Welder/GasPostflowMs", &out.welder.gas_postflow_ms)
          && extractPathInt(xml, "Host/Welder/CraterMs", &out.welder.crater_ms))) {
        return false;
    }
    out.cmd = static_cast<HostCmd>(cmd);
    out.seam.weave_mode = static_cast<WeaveMode>(weave_mode);
    out.seam.weave_type = static_cast<WeaveType>(weave_type);
    out.seam.multi_mode = static_cast<MultiMode>(multi_mode);
    out.pass.kind = static_cast<PassKind>(kind);
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
    int32_t mode = 0;
    if (!(extractPathInt(xml, "Kuka/Heartbeat", &out.heartbeat)
          && extractPathInt(xml, "Kuka/CmdAckSeq", &out.cmd_ack_seq)
          && extractPathInt(xml, "Kuka/Phase", &phase)
          && extractPathInt(xml, "Kuka/OpMode", &mode)
          && extractPathInt(xml, "Kuka/ProActive", &out.pro_active)
          && extractPathInt(xml, "Kuka/DrivesOn", &out.drives_on)
          && extractPathInt(xml, "Kuka/EStop", &out.estop)
          && extractPathInt(xml, "Kuka/MsgId", &out.msg_id)
          && extractPathInt(xml, "Kuka/SeamId", &out.seam_id)
          && extractPathInt(xml, "Kuka/Layer", &out.layer)
          && extractPathInt(xml, "Kuka/PassSeq", &out.pass_seq)
          && extractPathInt(xml, "Kuka/TrajIndex", &out.traj_index)
          && readPosePath(xml, "Kuka/Tcp", &out.tcp)
          && extractPathReal(xml, "Kuka/ActSpeedMmS", &out.act_speed_mm_s)
          && extractPathInt(xml, "Kuka/ArcOn", &out.arc_on)
          && extractPathInt(xml, "Kuka/Collision", &out.collision)
          && extractPathInt(xml, "Kuka/DownloadOk", &out.download_ok)
          && extractPathInt(xml, "Kuka/JobDone", &out.job_done)
          && extractPathReal(xml, "Kuka/ProgressPct", &out.progress_pct))) {
        return false;
    }
    out.phase = static_cast<KukaPhase>(phase);
    out.op_mode = static_cast<KukaOpMode>(mode);
    *msg = out;
    return true;
}

std::string formatTrajCsv(const std::vector<TrajPoint>& points)
{
    std::ostringstream oss;
    oss << std::setprecision(9);
    for (size_t i = 0; i < points.size(); ++i) {
        if (i > 0) {
            oss << ';';
        }
        const TrajPoint& p = points[i];
        oss << p.pose.x << ',' << p.pose.y << ',' << p.pose.z << ','
            << p.pose.a << ',' << p.pose.b << ',' << p.pose.c << ','
            << p.pose.e1 << ',' << p.speed_mm_s << ',' << p.flag;
    }
    oss << '.';
    return oss.str();
}

KukaPose insetPose(const KukaPose& start, const KukaPose& end, float inset_mm, bool from_start)
{
    const float dx = end.x - start.x;
    const float dy = end.y - start.y;
    const float dz = end.z - start.z;
    const float len = std::sqrt(dx * dx + dy * dy + dz * dz);
    KukaPose out = from_start ? start : end;
    if (len <= kEps) {
        return out;
    }
    const float clamped = std::max(0.f, std::min(inset_mm, 0.5f * len));
    const float ux = dx / len;
    const float uy = dy / len;
    const float uz = dz / len;
    if (from_start) {
        out.x = start.x + clamped * ux;
        out.y = start.y + clamped * uy;
        out.z = start.z + clamped * uz;
    } else {
        out.x = end.x - clamped * ux;
        out.y = end.y - clamped * uy;
        out.z = end.z - clamped * uz;
    }
    return out;
}
