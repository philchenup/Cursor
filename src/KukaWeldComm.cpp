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

void appendMotion(std::ostringstream& oss, const MotionBlock& m)
{
    oss << xmlReal("Extern_Speed", m.extern_speed);
    oss << xmlReal("Extern_Acc", m.extern_acc);
    oss << xmlReal("Robot_Speed", m.robot_speed);
    oss << xmlReal("Robot_Acc", m.robot_acc);
    oss << xmlReal("Extern_E1", m.e1);
    oss << xmlReal("Extern_E2", m.e2);
    oss << xmlReal("Extern_E3", m.e3);
    oss << xmlReal("Robot_X", m.x);
    oss << xmlReal("Robot_Y", m.y);
    oss << xmlReal("Robot_Z", m.z);
    oss << xmlReal("Robot_A", m.a);
    oss << xmlReal("Robot_B", m.b);
    oss << xmlReal("Robot_C", m.c);
    oss << xmlReal("Robot_J1", m.j1);
    oss << xmlReal("Robot_J2", m.j2);
    oss << xmlReal("Robot_J3", m.j3);
    oss << xmlReal("Robot_J4", m.j4);
    oss << xmlReal("Robot_J5", m.j5);
    oss << xmlReal("Robot_J6", m.j6);
}

bool readMotion(const std::string& xml, MotionBlock* m)
{
    return extractReal(xml, "Extern_Speed", &m->extern_speed)
        && extractReal(xml, "Extern_Acc", &m->extern_acc)
        && extractReal(xml, "Robot_Speed", &m->robot_speed)
        && extractReal(xml, "Robot_Acc", &m->robot_acc)
        && extractReal(xml, "Extern_E1", &m->e1)
        && extractReal(xml, "Extern_E2", &m->e2)
        && extractReal(xml, "Extern_E3", &m->e3)
        && extractReal(xml, "Robot_X", &m->x)
        && extractReal(xml, "Robot_Y", &m->y)
        && extractReal(xml, "Robot_Z", &m->z)
        && extractReal(xml, "Robot_A", &m->a)
        && extractReal(xml, "Robot_B", &m->b)
        && extractReal(xml, "Robot_C", &m->c)
        && extractReal(xml, "Robot_J1", &m->j1)
        && extractReal(xml, "Robot_J2", &m->j2)
        && extractReal(xml, "Robot_J3", &m->j3)
        && extractReal(xml, "Robot_J4", &m->j4)
        && extractReal(xml, "Robot_J5", &m->j5)
        && extractReal(xml, "Robot_J6", &m->j6);
}

void appendNamedPose(std::ostringstream& oss, const char* prefix, const KukaPose& p)
{
    const std::string pre = prefix;
    oss << xmlReal((pre + "X").c_str(), p.x);
    oss << xmlReal((pre + "Y").c_str(), p.y);
    oss << xmlReal((pre + "Z").c_str(), p.z);
    oss << xmlReal((pre + "A").c_str(), p.a);
    oss << xmlReal((pre + "B").c_str(), p.b);
    oss << xmlReal((pre + "C").c_str(), p.c);
}

bool readNamedPose(const std::string& xml, const char* prefix, KukaPose* p)
{
    const std::string pre = prefix;
    return extractReal(xml, (pre + "X").c_str(), &p->x)
        && extractReal(xml, (pre + "Y").c_str(), &p->y)
        && extractReal(xml, (pre + "Z").c_str(), &p->z)
        && extractReal(xml, (pre + "A").c_str(), &p->a)
        && extractReal(xml, (pre + "B").c_str(), &p->b)
        && extractReal(xml, (pre + "C").c_str(), &p->c);
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
    oss << "| 数据类型 | 信号名 | 含义 |\n| --- | --- | --- |\n";
    for (const CommSignal& s : kukaWeldCommSignals()) {
        if (s.direction == dir) {
            oss << "| " << s.type << " | " << s.name << " | " << s.meaning << " |\n";
        }
    }
}

}  // namespace

const std::vector<CommSignal>& kukaWeldCommSignals()
{
    static const std::vector<CommSignal> kSignals = {
        {1, "FLOAT32", "Extern_Speed", CommDirection::HostToKuka, "外部轴速度指令 mm/s"},
        {2, "FLOAT32", "Extern_Acc", CommDirection::HostToKuka, "外部轴加速度指令"},
        {3, "FLOAT32", "Robot_Speed", CommDirection::HostToKuka, "机器人速度指令；焊接时为焊速 mm/s"},
        {4, "FLOAT32", "Robot_Acc", CommDirection::HostToKuka, "机器人加速度指令"},
        {5, "FLOAT32", "Extern_E1", CommDirection::HostToKuka, "外部轴 E1 位置指令（地轨）mm"},
        {6, "FLOAT32", "Extern_E2", CommDirection::HostToKuka, "外部轴 E2 位置指令 mm"},
        {7, "FLOAT32", "Extern_E3", CommDirection::HostToKuka, "外部轴 E3 位置指令 mm"},
        {8, "FLOAT32", "Robot_X", CommDirection::HostToKuka, "TCP X 位置指令 mm"},
        {9, "FLOAT32", "Robot_Y", CommDirection::HostToKuka, "TCP Y 位置指令 mm"},
        {10, "FLOAT32", "Robot_Z", CommDirection::HostToKuka, "TCP Z 位置指令 mm"},
        {11, "FLOAT32", "Robot_A", CommDirection::HostToKuka, "TCP 姿态 A 指令 deg（绕 Z）"},
        {12, "FLOAT32", "Robot_B", CommDirection::HostToKuka, "TCP 姿态 B 指令 deg（绕 Y）"},
        {13, "FLOAT32", "Robot_C", CommDirection::HostToKuka, "TCP 姿态 C 指令 deg（绕 X）"},
        {14, "FLOAT32", "Robot_J1", CommDirection::HostToKuka, "关节 1 角度指令 deg"},
        {15, "FLOAT32", "Robot_J2", CommDirection::HostToKuka, "关节 2 角度指令 deg"},
        {16, "FLOAT32", "Robot_J3", CommDirection::HostToKuka, "关节 3 角度指令 deg"},
        {17, "FLOAT32", "Robot_J4", CommDirection::HostToKuka, "关节 4 角度指令 deg"},
        {18, "FLOAT32", "Robot_J5", CommDirection::HostToKuka, "关节 5 角度指令 deg"},
        {19, "FLOAT32", "Robot_J6", CommDirection::HostToKuka, "关节 6 角度指令 deg"},

        {20, "INT32", "Host_Heartbeat", CommDirection::HostToKuka, "上位机心跳，周期递增"},
        {21, "INT32", "Host_Cmd", CommDirection::HostToKuka, "命令字：0空闲 1复位 2-5下载 6自动全流程 7暂停 8继续 9停止 10收弧 11回Home 12故障确认 13寻起点 14TCP到找到的起点 15寻终点 16焊/移动到找到的终点"},
        {22, "INT32", "Host_CmdSeq", CommDirection::HostToKuka, "命令序号，KUKA 仅在变化时执行一次"},
        {23, "INT32", "Job_Id", CommDirection::HostToKuka, "焊接作业号"},
        {24, "INT32", "Job_SeamCount", CommDirection::HostToKuka, "本作业焊缝条数"},
        {25, "INT32", "Job_PassCount", CommDirection::HostToKuka, "本作业焊道总数"},
        {26, "INT32", "Job_ToolNo", CommDirection::HostToKuka, "焊枪工具坐标系 $TOOL 编号"},
        {27, "INT32", "Job_BaseNo", CommDirection::HostToKuka, "工件坐标系 $BASE 编号"},
        {28, "FLOAT32", "Job_Override", CommDirection::HostToKuka, "建议速度倍率 %"},
        {29, "FLOAT32", "Job_Approach", CommDirection::HostToKuka, "接近高度 mm（沿 TCP -Z）"},
        {30, "FLOAT32", "Job_Retract", CommDirection::HostToKuka, "收弧后回撤高度 mm"},

        {31, "INT32", "Seam_Id", CommDirection::HostToKuka, "焊缝序号，对应工艺表序号"},
        {32, "FLOAT32", "Seam_StartX", CommDirection::HostToKuka, "焊缝参考起点 X mm（内缩前，寻缝搜索中心）"},
        {33, "FLOAT32", "Seam_StartY", CommDirection::HostToKuka, "焊缝参考起点 Y mm"},
        {34, "FLOAT32", "Seam_StartZ", CommDirection::HostToKuka, "焊缝参考起点 Z mm"},
        {35, "FLOAT32", "Seam_StartA", CommDirection::HostToKuka, "焊缝参考起点姿态 A deg"},
        {36, "FLOAT32", "Seam_StartB", CommDirection::HostToKuka, "焊缝参考起点姿态 B deg"},
        {37, "FLOAT32", "Seam_StartC", CommDirection::HostToKuka, "焊缝参考起点姿态 C deg"},
        {38, "FLOAT32", "Seam_EndX", CommDirection::HostToKuka, "焊缝参考终点 X mm（内缩前，寻缝搜索中心）"},
        {39, "FLOAT32", "Seam_EndY", CommDirection::HostToKuka, "焊缝参考终点 Y mm"},
        {40, "FLOAT32", "Seam_EndZ", CommDirection::HostToKuka, "焊缝参考终点 Z mm"},
        {41, "FLOAT32", "Seam_EndA", CommDirection::HostToKuka, "焊缝参考终点姿态 A deg"},
        {42, "FLOAT32", "Seam_EndB", CommDirection::HostToKuka, "焊缝参考终点姿态 B deg"},
        {43, "FLOAT32", "Seam_EndC", CommDirection::HostToKuka, "焊缝参考终点姿态 C deg"},
        {44, "FLOAT32", "Seam_Inset", CommDirection::HostToKuka, "内缩 mm，沿焊缝从起终点各收回"},
        {45, "FLOAT32", "Seam_WeldSpeed", CommDirection::HostToKuka, "焊接速度 mm/s，对应工艺表焊接速度"},
        {46, "INT32", "Seam_WeaveMode", CommDirection::HostToKuka, "摆动方式：0 直线焊  1 摆动焊"},
        {47, "INT32", "Seam_WeaveType", CommDirection::HostToKuka, "摆动类型：0 正弦  1 三角"},
        {48, "FLOAT32", "Seam_Amplitude", CommDirection::HostToKuka, "摆动幅度 mm（侧向峰值）"},
        {49, "FLOAT32", "Seam_Chord", CommDirection::HostToKuka, "摆动弦长 mm（一个周期沿焊缝长度）"},
        {50, "INT32", "Seam_MultiMode", CommDirection::HostToKuka, "0 单层单道  1 多层多道"},
        {51, "FLOAT32", "Seam_Thickness", CommDirection::HostToKuka, "板厚 mm"},
        {52, "FLOAT32", "Seam_Groove", CommDirection::HostToKuka, "坡口角度 deg"},
        {53, "FLOAT32", "Seam_FitUpGap", CommDirection::HostToKuka, "装配间隙 mm"},
        {54, "FLOAT32", "Seam_Penetration", CommDirection::HostToKuka, "熔深 mm"},

        {55, "INT32", "Pass_SeamId", CommDirection::HostToKuka, "本焊道所属焊缝号"},
        {56, "INT32", "Pass_Layer", CommDirection::HostToKuka, "层号，打底层为 1"},
        {57, "INT32", "Pass_Local", CommDirection::HostToKuka, "层内道号"},
        {58, "INT32", "Pass_Seq", CommDirection::HostToKuka, "全局焊接顺序"},
        {59, "INT32", "Pass_Kind", CommDirection::HostToKuka, "焊道类型：0 打底  1 填充  2 盖面"},
        {60, "FLOAT32", "Pass_StartX", CommDirection::HostToKuka, "本焊道起点 X mm（含层/道偏移）"},
        {61, "FLOAT32", "Pass_StartY", CommDirection::HostToKuka, "本焊道起点 Y mm"},
        {62, "FLOAT32", "Pass_StartZ", CommDirection::HostToKuka, "本焊道起点 Z mm"},
        {63, "FLOAT32", "Pass_StartA", CommDirection::HostToKuka, "本焊道起点姿态 A deg"},
        {64, "FLOAT32", "Pass_StartB", CommDirection::HostToKuka, "本焊道起点姿态 B deg"},
        {65, "FLOAT32", "Pass_StartC", CommDirection::HostToKuka, "本焊道起点姿态 C deg"},
        {66, "FLOAT32", "Pass_EndX", CommDirection::HostToKuka, "本焊道终点 X mm"},
        {67, "FLOAT32", "Pass_EndY", CommDirection::HostToKuka, "本焊道终点 Y mm"},
        {68, "FLOAT32", "Pass_EndZ", CommDirection::HostToKuka, "本焊道终点 Z mm"},
        {69, "FLOAT32", "Pass_EndA", CommDirection::HostToKuka, "本焊道终点姿态 A deg"},
        {70, "FLOAT32", "Pass_EndB", CommDirection::HostToKuka, "本焊道终点姿态 B deg"},
        {71, "FLOAT32", "Pass_EndC", CommDirection::HostToKuka, "本焊道终点姿态 C deg"},
        {72, "FLOAT32", "Pass_Speed", CommDirection::HostToKuka, "本焊道焊接速度 mm/s"},

        {73, "INT32", "Traj_Index", CommDirection::HostToKuka, "当前轨迹点序号"},
        {74, "INT32", "Traj_Count", CommDirection::HostToKuka, "本焊道轨迹点总数"},
        {75, "FLOAT32", "Traj_Speed", CommDirection::HostToKuka, "该轨迹点进给速度 mm/s"},
        {76, "INT32", "Traj_Flag", CommDirection::HostToKuka, "bit0 起弧点  bit1 收弧点  bit2 本焊道末点"},

        {77, "BOOL", "Weld_ArcEnable", CommDirection::HostToKuka, "允许起弧"},
        {78, "BOOL", "Weld_GasEnable", CommDirection::HostToKuka, "允许送保护气"},
        {79, "FLOAT32", "Weld_Current", CommDirection::HostToKuka, "焊接电流设定 A"},
        {80, "FLOAT32", "Weld_Voltage", CommDirection::HostToKuka, "电弧电压设定 V"},
        {81, "FLOAT32", "Weld_WireSpeed", CommDirection::HostToKuka, "送丝速度 m/min"},
        {82, "INT32", "Weld_GasPreflow", CommDirection::HostToKuka, "起弧前气体预吹时间 ms"},
        {83, "INT32", "Weld_GasPostflow", CommDirection::HostToKuka, "收弧后气体滞后时间 ms"},
        {84, "INT32", "Weld_Crater", CommDirection::HostToKuka, "填弧坑时间 ms"},
        {85, "FLOAT32", "Torch_A", CommDirection::HostToKuka, "焊枪姿态 A deg；寻到焊缝点后按此姿态 TCP 到位"},
        {86, "FLOAT32", "Torch_B", CommDirection::HostToKuka, "焊枪姿态 B deg"},
        {87, "FLOAT32", "Torch_C", CommDirection::HostToKuka, "焊枪姿态 C deg"},
        {88, "INT32", "Laser_Mode", CommDirection::HostToKuka, "0 不用激光按参考点焊  1 寻缝后到位  2 寻缝+焊中RSI跟踪"},
        {89, "BOOL", "Laser_FindEnable", CommDirection::HostToKuka, "允许激光寻缝（起点/终点）"},
        {90, "BOOL", "Laser_TrackEnable", CommDirection::HostToKuka, "允许焊中 RSI 纠偏（纠偏闭环不走本表）"},
        {91, "FLOAT32", "Laser_LookAhead", CommDirection::HostToKuka, "激光超前距 mm（焊枪前方寻缝）"},
        {92, "FLOAT32", "Laser_SearchRadius", CommDirection::HostToKuka, "相对参考点的寻缝范围 mm"},
        {93, "FLOAT32", "Laser_SearchSpeed", CommDirection::HostToKuka, "寻缝移动速度 mm/s"},
        {94, "INT32", "Laser_Timeout", CommDirection::HostToKuka, "单次寻缝超时 ms"},

        {95, "INT32", "Kuka_Heartbeat", CommDirection::KukaToHost, "机器人心跳"},
        {96, "INT32", "Kuka_CmdAck", CommDirection::KukaToHost, "已接受的命令序号"},
        {97, "INT32", "Kuka_Phase", CommDirection::KukaToHost, "工艺阶段：地轨/接近/寻缝/找到起点/TCP到位/寻终点/焊接到终点/收弧/回Home/故障"},
        {98, "INT32", "Kuka_OpMode", CommDirection::KukaToHost, "运行模式：0 T1  1 T2  2 AUT  3 EXT"},
        {99, "BOOL", "Kuka_ProActive", CommDirection::KukaToHost, "解释器程序正在运行"},
        {100, "BOOL", "Kuka_DrivesOn", CommDirection::KukaToHost, "驱动已使能"},
        {101, "BOOL", "Kuka_EStop", CommDirection::KukaToHost, "急停"},
        {102, "INT32", "Kuka_MsgId", CommDirection::KukaToHost, "报警号，0 表示无报警"},
        {103, "INT32", "Kuka_SeamId", CommDirection::KukaToHost, "当前焊缝号"},
        {104, "INT32", "Kuka_Layer", CommDirection::KukaToHost, "当前层号"},
        {105, "INT32", "Kuka_PassSeq", CommDirection::KukaToHost, "当前焊道全局序号"},
        {106, "INT32", "Kuka_TrajIndex", CommDirection::KukaToHost, "当前轨迹点序号"},
        {107, "FLOAT32", "Act_Extern_Speed", CommDirection::KukaToHost, "外部轴实际速度 mm/s"},
        {108, "FLOAT32", "Act_Extern_Acc", CommDirection::KukaToHost, "外部轴实际加速度"},
        {109, "FLOAT32", "Act_Robot_Speed", CommDirection::KukaToHost, "机器人实际速度 mm/s"},
        {110, "FLOAT32", "Act_Robot_Acc", CommDirection::KukaToHost, "机器人实际加速度"},
        {111, "FLOAT32", "Act_Extern_E1", CommDirection::KukaToHost, "外部轴 E1 实际位置 mm"},
        {112, "FLOAT32", "Act_Extern_E2", CommDirection::KukaToHost, "外部轴 E2 实际位置 mm"},
        {113, "FLOAT32", "Act_Extern_E3", CommDirection::KukaToHost, "外部轴 E3 实际位置 mm"},
        {114, "FLOAT32", "Act_Robot_X", CommDirection::KukaToHost, "TCP 实际 X mm"},
        {115, "FLOAT32", "Act_Robot_Y", CommDirection::KukaToHost, "TCP 实际 Y mm"},
        {116, "FLOAT32", "Act_Robot_Z", CommDirection::KukaToHost, "TCP 实际 Z mm"},
        {117, "FLOAT32", "Act_Robot_A", CommDirection::KukaToHost, "TCP 实际姿态 A deg"},
        {118, "FLOAT32", "Act_Robot_B", CommDirection::KukaToHost, "TCP 实际姿态 B deg"},
        {119, "FLOAT32", "Act_Robot_C", CommDirection::KukaToHost, "TCP 实际姿态 C deg"},
        {120, "FLOAT32", "Act_Robot_J1", CommDirection::KukaToHost, "关节 1 实际角度 deg"},
        {121, "FLOAT32", "Act_Robot_J2", CommDirection::KukaToHost, "关节 2 实际角度 deg"},
        {122, "FLOAT32", "Act_Robot_J3", CommDirection::KukaToHost, "关节 3 实际角度 deg"},
        {123, "FLOAT32", "Act_Robot_J4", CommDirection::KukaToHost, "关节 4 实际角度 deg"},
        {124, "FLOAT32", "Act_Robot_J5", CommDirection::KukaToHost, "关节 5 实际角度 deg"},
        {125, "FLOAT32", "Act_Robot_J6", CommDirection::KukaToHost, "关节 6 实际角度 deg"},
        {126, "BOOL", "Kuka_ArcOn", CommDirection::KukaToHost, "焊机起弧成功反馈"},
        {127, "BOOL", "Kuka_Collision", CommDirection::KukaToHost, "碰撞或力矩超限"},
        {128, "BOOL", "Kuka_DownloadOk", CommDirection::KukaToHost, "作业/焊缝/焊道/轨迹下载完成"},
        {129, "BOOL", "Kuka_JobDone", CommDirection::KukaToHost, "全部焊缝焊完并已回 Home"},
        {130, "FLOAT32", "Kuka_Progress", CommDirection::KukaToHost, "当前作业进度 %"},
        {131, "BOOL", "Laser_Ready", CommDirection::KukaToHost, "激光寻缝器就绪"},
        {132, "BOOL", "Laser_Finding", CommDirection::KukaToHost, "正在寻缝"},
        {133, "BOOL", "Laser_StartValid", CommDirection::KukaToHost, "已找到焊缝起点，Found_Start* 有效"},
        {134, "BOOL", "Laser_EndValid", CommDirection::KukaToHost, "已找到焊缝终点，Found_End* 有效"},
        {135, "BOOL", "Laser_Lost", CommDirection::KukaToHost, "寻缝丢失或跟踪丢失"},
        {136, "INT32", "Laser_ErrId", CommDirection::KukaToHost, "激光故障码，0 表示正常"},
        {137, "FLOAT32", "Found_StartX", CommDirection::KukaToHost, "激光找到的焊缝起点 X mm"},
        {138, "FLOAT32", "Found_StartY", CommDirection::KukaToHost, "激光找到的焊缝起点 Y mm"},
        {139, "FLOAT32", "Found_StartZ", CommDirection::KukaToHost, "激光找到的焊缝起点 Z mm"},
        {140, "FLOAT32", "Found_StartA", CommDirection::KukaToHost, "到位用焊枪姿态 A（通常回传 Torch_A）"},
        {141, "FLOAT32", "Found_StartB", CommDirection::KukaToHost, "到位用焊枪姿态 B"},
        {142, "FLOAT32", "Found_StartC", CommDirection::KukaToHost, "到位用焊枪姿态 C"},
        {143, "FLOAT32", "Found_EndX", CommDirection::KukaToHost, "激光找到的焊缝终点 X mm"},
        {144, "FLOAT32", "Found_EndY", CommDirection::KukaToHost, "激光找到的焊缝终点 Y mm"},
        {145, "FLOAT32", "Found_EndZ", CommDirection::KukaToHost, "激光找到的焊缝终点 Z mm"},
        {146, "FLOAT32", "Found_EndA", CommDirection::KukaToHost, "终点焊枪姿态 A"},
        {147, "FLOAT32", "Found_EndB", CommDirection::KukaToHost, "终点焊枪姿态 B"},
        {148, "FLOAT32", "Found_EndC", CommDirection::KukaToHost, "终点焊枪姿态 C"},
        {149, "FLOAT32", "Laser_dY", CommDirection::KukaToHost, "RSI 横向纠偏监视值 mm（闭环不走本表）"},
        {150, "FLOAT32", "Laser_dZ", CommDirection::KukaToHost, "RSI 高度纠偏监视值 mm（闭环不走本表）"},
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
    case KukaPhase::SearchApproach: return "SearchApproach";
    case KukaPhase::FindingStart: return "FindingStart";
    case KukaPhase::FoundStart: return "FoundStart";
    case KukaPhase::MoveToFoundStart: return "MoveToFoundStart";
    case KukaPhase::FindingEnd: return "FindingEnd";
    case KukaPhase::FoundEnd: return "FoundEnd";
    case KukaPhase::WeldToFoundEnd: return "WeldToFoundEnd";
    }
    return "Unknown";
}

std::string commTableMarkdown()
{
    std::ostringstream oss;
    oss << "## 上位机 → KUKA\n\n";
    appendTable(oss, CommDirection::HostToKuka);
    oss << "\n## KUKA → 上位机\n\n";
    appendTable(oss, CommDirection::KukaToHost);
    return oss.str();
}

std::string commTableCsv()
{
    std::ostringstream oss;
    oss << "数据类型,信号名,含义,方向\n";
    for (const CommSignal& s : kukaWeldCommSignals()) {
        oss << s.type << ',' << s.name << ',' << csvEscape(s.meaning) << ','
            << csvEscape(commDirectionName(s.direction)) << '\n';
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
    appendMotion(oss, msg.motion);
    oss << xmlInt("Host_Heartbeat", msg.heartbeat);
    oss << xmlInt("Host_Cmd", static_cast<int32_t>(msg.cmd));
    oss << xmlInt("Host_CmdSeq", msg.cmd_seq);
    oss << xmlInt("Job_Id", msg.job.job_id);
    oss << xmlInt("Job_SeamCount", msg.job.seam_count);
    oss << xmlInt("Job_PassCount", msg.job.pass_count);
    oss << xmlInt("Job_ToolNo", msg.job.tool_no);
    oss << xmlInt("Job_BaseNo", msg.job.base_no);
    oss << xmlReal("Job_Override", msg.job.override_pct);
    oss << xmlReal("Job_Approach", msg.job.approach_mm);
    oss << xmlReal("Job_Retract", msg.job.retract_mm);
    oss << xmlInt("Seam_Id", msg.seam.seam_id);
    appendNamedPose(oss, "Seam_Start", msg.seam.start);
    appendNamedPose(oss, "Seam_End", msg.seam.end);
    oss << xmlReal("Torch_A", msg.seam.torch_a);
    oss << xmlReal("Torch_B", msg.seam.torch_b);
    oss << xmlReal("Torch_C", msg.seam.torch_c);
    oss << xmlReal("Seam_Inset", msg.seam.inset_mm);
    oss << xmlReal("Seam_WeldSpeed", msg.seam.speed_mm_s);
    oss << xmlInt("Seam_WeaveMode", static_cast<int32_t>(msg.seam.weave_mode));
    oss << xmlInt("Seam_WeaveType", static_cast<int32_t>(msg.seam.weave_type));
    oss << xmlReal("Seam_Amplitude", msg.seam.amplitude_mm);
    oss << xmlReal("Seam_Chord", msg.seam.chord_mm);
    oss << xmlInt("Seam_MultiMode", static_cast<int32_t>(msg.seam.multi_mode));
    oss << xmlReal("Seam_Thickness", msg.seam.thickness_mm);
    oss << xmlReal("Seam_Groove", msg.seam.groove_deg);
    oss << xmlReal("Seam_FitUpGap", msg.seam.fitup_gap_mm);
    oss << xmlReal("Seam_Penetration", msg.seam.penetration_mm);
    oss << xmlInt("Pass_SeamId", msg.pass.seam_id);
    oss << xmlInt("Pass_Layer", msg.pass.layer);
    oss << xmlInt("Pass_Local", msg.pass.local_index);
    oss << xmlInt("Pass_Seq", msg.pass.sequence);
    oss << xmlInt("Pass_Kind", static_cast<int32_t>(msg.pass.kind));
    appendNamedPose(oss, "Pass_Start", msg.pass.start);
    appendNamedPose(oss, "Pass_End", msg.pass.end);
    oss << xmlReal("Pass_Speed", msg.pass.speed_mm_s);
    oss << xmlInt("Traj_Index", msg.traj.index);
    oss << xmlInt("Traj_Count", msg.traj.count);
    oss << xmlReal("Traj_Speed", msg.traj.speed_mm_s);
    oss << xmlInt("Traj_Flag", msg.traj.flag);
    oss << xmlInt("Weld_ArcEnable", msg.welder.arc_enable);
    oss << xmlInt("Weld_GasEnable", msg.welder.gas_enable);
    oss << xmlReal("Weld_Current", msg.welder.current_a);
    oss << xmlReal("Weld_Voltage", msg.welder.voltage_v);
    oss << xmlReal("Weld_WireSpeed", msg.welder.wire_m_min);
    oss << xmlInt("Weld_GasPreflow", msg.welder.gas_preflow_ms);
    oss << xmlInt("Weld_GasPostflow", msg.welder.gas_postflow_ms);
    oss << xmlInt("Weld_Crater", msg.welder.crater_ms);
    oss << xmlInt("Laser_Mode", static_cast<int32_t>(msg.laser.mode));
    oss << xmlInt("Laser_FindEnable", msg.laser.find_enable);
    oss << xmlInt("Laser_TrackEnable", msg.laser.track_enable);
    oss << xmlReal("Laser_LookAhead", msg.laser.look_ahead_mm);
    oss << xmlReal("Laser_SearchRadius", msg.laser.search_radius_mm);
    oss << xmlReal("Laser_SearchSpeed", msg.laser.search_speed_mm_s);
    oss << xmlInt("Laser_Timeout", msg.laser.timeout_ms);
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
    oss << xmlInt("Kuka_OpMode", static_cast<int32_t>(msg.op_mode));
    oss << xmlInt("Kuka_ProActive", msg.pro_active);
    oss << xmlInt("Kuka_DrivesOn", msg.drives_on);
    oss << xmlInt("Kuka_EStop", msg.estop);
    oss << xmlInt("Kuka_MsgId", msg.msg_id);
    oss << xmlInt("Kuka_SeamId", msg.seam_id);
    oss << xmlInt("Kuka_Layer", msg.layer);
    oss << xmlInt("Kuka_PassSeq", msg.pass_seq);
    oss << xmlInt("Kuka_TrajIndex", msg.traj_index);
    oss << xmlReal("Act_Extern_Speed", msg.motion.extern_speed);
    oss << xmlReal("Act_Extern_Acc", msg.motion.extern_acc);
    oss << xmlReal("Act_Robot_Speed", msg.motion.robot_speed);
    oss << xmlReal("Act_Robot_Acc", msg.motion.robot_acc);
    oss << xmlReal("Act_Extern_E1", msg.motion.e1);
    oss << xmlReal("Act_Extern_E2", msg.motion.e2);
    oss << xmlReal("Act_Extern_E3", msg.motion.e3);
    oss << xmlReal("Act_Robot_X", msg.motion.x);
    oss << xmlReal("Act_Robot_Y", msg.motion.y);
    oss << xmlReal("Act_Robot_Z", msg.motion.z);
    oss << xmlReal("Act_Robot_A", msg.motion.a);
    oss << xmlReal("Act_Robot_B", msg.motion.b);
    oss << xmlReal("Act_Robot_C", msg.motion.c);
    oss << xmlReal("Act_Robot_J1", msg.motion.j1);
    oss << xmlReal("Act_Robot_J2", msg.motion.j2);
    oss << xmlReal("Act_Robot_J3", msg.motion.j3);
    oss << xmlReal("Act_Robot_J4", msg.motion.j4);
    oss << xmlReal("Act_Robot_J5", msg.motion.j5);
    oss << xmlReal("Act_Robot_J6", msg.motion.j6);
    oss << xmlInt("Kuka_ArcOn", msg.arc_on);
    oss << xmlInt("Kuka_Collision", msg.collision);
    oss << xmlInt("Kuka_DownloadOk", msg.download_ok);
    oss << xmlInt("Kuka_JobDone", msg.job_done);
    oss << xmlReal("Kuka_Progress", msg.progress_pct);
    oss << xmlInt("Laser_Ready", msg.laser.ready);
    oss << xmlInt("Laser_Finding", msg.laser.finding);
    oss << xmlInt("Laser_StartValid", msg.laser.start_valid);
    oss << xmlInt("Laser_EndValid", msg.laser.end_valid);
    oss << xmlInt("Laser_Lost", msg.laser.lost);
    oss << xmlInt("Laser_ErrId", msg.laser.err_id);
    appendNamedPose(oss, "Found_Start", msg.laser.found_start);
    appendNamedPose(oss, "Found_End", msg.laser.found_end);
    oss << xmlReal("Laser_dY", msg.laser.dy);
    oss << xmlReal("Laser_dZ", msg.laser.dz);
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
    int32_t laser_mode = 0;
    if (!(readMotion(xml, &out.motion)
          && extractInt(xml, "Host_Heartbeat", &out.heartbeat)
          && extractInt(xml, "Host_Cmd", &cmd)
          && extractInt(xml, "Host_CmdSeq", &out.cmd_seq)
          && extractInt(xml, "Job_Id", &out.job.job_id)
          && extractInt(xml, "Job_SeamCount", &out.job.seam_count)
          && extractInt(xml, "Job_PassCount", &out.job.pass_count)
          && extractInt(xml, "Job_ToolNo", &out.job.tool_no)
          && extractInt(xml, "Job_BaseNo", &out.job.base_no)
          && extractReal(xml, "Job_Override", &out.job.override_pct)
          && extractReal(xml, "Job_Approach", &out.job.approach_mm)
          && extractReal(xml, "Job_Retract", &out.job.retract_mm)
          && extractInt(xml, "Seam_Id", &out.seam.seam_id)
          && readNamedPose(xml, "Seam_Start", &out.seam.start)
          && readNamedPose(xml, "Seam_End", &out.seam.end)
          && extractReal(xml, "Torch_A", &out.seam.torch_a)
          && extractReal(xml, "Torch_B", &out.seam.torch_b)
          && extractReal(xml, "Torch_C", &out.seam.torch_c)
          && extractReal(xml, "Seam_Inset", &out.seam.inset_mm)
          && extractReal(xml, "Seam_WeldSpeed", &out.seam.speed_mm_s)
          && extractInt(xml, "Seam_WeaveMode", &weave_mode)
          && extractInt(xml, "Seam_WeaveType", &weave_type)
          && extractReal(xml, "Seam_Amplitude", &out.seam.amplitude_mm)
          && extractReal(xml, "Seam_Chord", &out.seam.chord_mm)
          && extractInt(xml, "Seam_MultiMode", &multi_mode)
          && extractReal(xml, "Seam_Thickness", &out.seam.thickness_mm)
          && extractReal(xml, "Seam_Groove", &out.seam.groove_deg)
          && extractReal(xml, "Seam_FitUpGap", &out.seam.fitup_gap_mm)
          && extractReal(xml, "Seam_Penetration", &out.seam.penetration_mm)
          && extractInt(xml, "Pass_SeamId", &out.pass.seam_id)
          && extractInt(xml, "Pass_Layer", &out.pass.layer)
          && extractInt(xml, "Pass_Local", &out.pass.local_index)
          && extractInt(xml, "Pass_Seq", &out.pass.sequence)
          && extractInt(xml, "Pass_Kind", &kind)
          && readNamedPose(xml, "Pass_Start", &out.pass.start)
          && readNamedPose(xml, "Pass_End", &out.pass.end)
          && extractReal(xml, "Pass_Speed", &out.pass.speed_mm_s)
          && extractInt(xml, "Traj_Index", &out.traj.index)
          && extractInt(xml, "Traj_Count", &out.traj.count)
          && extractReal(xml, "Traj_Speed", &out.traj.speed_mm_s)
          && extractInt(xml, "Traj_Flag", &out.traj.flag)
          && extractInt(xml, "Weld_ArcEnable", &out.welder.arc_enable)
          && extractInt(xml, "Weld_GasEnable", &out.welder.gas_enable)
          && extractReal(xml, "Weld_Current", &out.welder.current_a)
          && extractReal(xml, "Weld_Voltage", &out.welder.voltage_v)
          && extractReal(xml, "Weld_WireSpeed", &out.welder.wire_m_min)
          && extractInt(xml, "Weld_GasPreflow", &out.welder.gas_preflow_ms)
          && extractInt(xml, "Weld_GasPostflow", &out.welder.gas_postflow_ms)
          && extractInt(xml, "Weld_Crater", &out.welder.crater_ms)
          && extractInt(xml, "Laser_Mode", &laser_mode)
          && extractInt(xml, "Laser_FindEnable", &out.laser.find_enable)
          && extractInt(xml, "Laser_TrackEnable", &out.laser.track_enable)
          && extractReal(xml, "Laser_LookAhead", &out.laser.look_ahead_mm)
          && extractReal(xml, "Laser_SearchRadius", &out.laser.search_radius_mm)
          && extractReal(xml, "Laser_SearchSpeed", &out.laser.search_speed_mm_s)
          && extractInt(xml, "Laser_Timeout", &out.laser.timeout_ms))) {
        return false;
    }
    out.cmd = static_cast<HostCmd>(cmd);
    out.seam.weave_mode = static_cast<WeaveMode>(weave_mode);
    out.seam.weave_type = static_cast<WeaveType>(weave_type);
    out.seam.multi_mode = static_cast<MultiMode>(multi_mode);
    out.pass.kind = static_cast<PassKind>(kind);
    out.laser.mode = static_cast<LaserMode>(laser_mode);
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
    if (!(extractInt(xml, "Kuka_Heartbeat", &out.heartbeat)
          && extractInt(xml, "Kuka_CmdAck", &out.cmd_ack_seq)
          && extractInt(xml, "Kuka_Phase", &phase)
          && extractInt(xml, "Kuka_OpMode", &mode)
          && extractInt(xml, "Kuka_ProActive", &out.pro_active)
          && extractInt(xml, "Kuka_DrivesOn", &out.drives_on)
          && extractInt(xml, "Kuka_EStop", &out.estop)
          && extractInt(xml, "Kuka_MsgId", &out.msg_id)
          && extractInt(xml, "Kuka_SeamId", &out.seam_id)
          && extractInt(xml, "Kuka_Layer", &out.layer)
          && extractInt(xml, "Kuka_PassSeq", &out.pass_seq)
          && extractInt(xml, "Kuka_TrajIndex", &out.traj_index)
          && extractReal(xml, "Act_Extern_Speed", &out.motion.extern_speed)
          && extractReal(xml, "Act_Extern_Acc", &out.motion.extern_acc)
          && extractReal(xml, "Act_Robot_Speed", &out.motion.robot_speed)
          && extractReal(xml, "Act_Robot_Acc", &out.motion.robot_acc)
          && extractReal(xml, "Act_Extern_E1", &out.motion.e1)
          && extractReal(xml, "Act_Extern_E2", &out.motion.e2)
          && extractReal(xml, "Act_Extern_E3", &out.motion.e3)
          && extractReal(xml, "Act_Robot_X", &out.motion.x)
          && extractReal(xml, "Act_Robot_Y", &out.motion.y)
          && extractReal(xml, "Act_Robot_Z", &out.motion.z)
          && extractReal(xml, "Act_Robot_A", &out.motion.a)
          && extractReal(xml, "Act_Robot_B", &out.motion.b)
          && extractReal(xml, "Act_Robot_C", &out.motion.c)
          && extractReal(xml, "Act_Robot_J1", &out.motion.j1)
          && extractReal(xml, "Act_Robot_J2", &out.motion.j2)
          && extractReal(xml, "Act_Robot_J3", &out.motion.j3)
          && extractReal(xml, "Act_Robot_J4", &out.motion.j4)
          && extractReal(xml, "Act_Robot_J5", &out.motion.j5)
          && extractReal(xml, "Act_Robot_J6", &out.motion.j6)
          && extractInt(xml, "Kuka_ArcOn", &out.arc_on)
          && extractInt(xml, "Kuka_Collision", &out.collision)
          && extractInt(xml, "Kuka_DownloadOk", &out.download_ok)
          && extractInt(xml, "Kuka_JobDone", &out.job_done)
          && extractReal(xml, "Kuka_Progress", &out.progress_pct)
          && extractInt(xml, "Laser_Ready", &out.laser.ready)
          && extractInt(xml, "Laser_Finding", &out.laser.finding)
          && extractInt(xml, "Laser_StartValid", &out.laser.start_valid)
          && extractInt(xml, "Laser_EndValid", &out.laser.end_valid)
          && extractInt(xml, "Laser_Lost", &out.laser.lost)
          && extractInt(xml, "Laser_ErrId", &out.laser.err_id)
          && readNamedPose(xml, "Found_Start", &out.laser.found_start)
          && readNamedPose(xml, "Found_End", &out.laser.found_end)
          && extractReal(xml, "Laser_dY", &out.laser.dy)
          && extractReal(xml, "Laser_dZ", &out.laser.dz))) {
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
