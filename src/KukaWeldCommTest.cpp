#include "KukaWeldComm.h"

#include <cmath>
#include <iostream>
#include <set>
#include <string>

namespace {

int g_failed = 0;

void expect(bool cond, const char* msg)
{
    if (!cond) {
        std::cerr << "FAIL: " << msg << '\n';
        ++g_failed;
    }
}

bool near(float a, float b, float eps = 1e-4f)
{
    return std::fabs(a - b) <= eps;
}

bool xyzEq(const Xyz& a, const Xyz& b)
{
    return near(a.x, b.x) && near(a.y, b.y) && near(a.z, b.z);
}

}  // namespace

int main()
{
    const std::vector<CommSignal>& signals = kukaWeldCommSignals();
    expect(signals.size() == 39, "signal count is 39");

    std::set<int> indices;
    std::set<std::string> names;
    int host_n = 0;
    int kuka_n = 0;
    for (const CommSignal& s : signals) {
        expect(s.index > 0, "positive index");
        expect(indices.insert(s.index).second, "unique index");
        expect(names.insert(s.name).second, "unique name");
        expect(s.meaning != nullptr && s.meaning[0] != '\0', "meaning");
        if (s.direction == CommDirection::HostToKuka) {
            ++host_n;
        } else {
            ++kuka_n;
        }
    }
    expect(host_n == 23, "23 host signals");
    expect(kuka_n == 16, "16 kuka signals");
    expect(kukaWeldPackedBytes(CommDirection::KukaToHost) == kKukaToHostPackedBytes,
           "kuka read packed bytes is 64");
    expect(kukaWeldPackedBytes(CommDirection::HostToKuka) == kHostToKukaPackedBytes,
           "host write packed bytes is 92");
    expect(static_cast<int>(sizeof(KukaCyclic)) == kKukaToHostPackedBytes, "sizeof KukaCyclic");
    expect(static_cast<int>(sizeof(HostCyclic)) == kHostToKukaPackedBytes, "sizeof HostCyclic");
    expect(commSignalPackedBytes("INT32") == 4 && commSignalPackedBytes("BOOL") == 4
               && commSignalPackedBytes("FLOAT32") == 4,
           "word size 4");
    expect(commSignalPackedBytes(nullptr) == 0, "null type");

    const CommSignal& first_kuka = signals[static_cast<std::size_t>(host_n)];
    const CommSignal& last_kuka = signals.back();
    expect(first_kuka.direction == CommDirection::KukaToHost, "first kuka signal");
    expect(kukaWeldPackedOffset(first_kuka) == 0, "kuka frame starts at 0");
    expect(kukaWeldPackedOffset(last_kuka) + commSignalPackedBytes(last_kuka.type)
               == kKukaToHostPackedBytes,
           "last kuka signal ends at 64");
    expect(names.count("Extern_Speed") == 0, "no extra motion words");
    expect(names.count("Robot_J1") == 0, "no joint streaming");
    expect(names.count("Pass_Layer") == 0, "no multi-pass extras");
    expect(names.count("Traj_Index") == 0, "no traj streaming");
    expect(names.count("Laser_FindEnable") == 0, "no duplicate laser enable");
    expect(names.count("Laser_TrackEnable") == 0, "no duplicate track enable");
    expect(names.count("Laser_Finding") == 0, "finding is Kuka_Phase");
    expect(names.count("Seam_StartX") == 1 && names.count("Found_StartX") == 1, "ref and found start");
    expect(names.count("Torch_A") == 1 && names.count("Seam_WeldSpeed") == 1, "torch and speed");
    expect(names.count("Seam_WeaveMode") == 1 && names.count("Seam_Chord") == 1, "weave");

    HostCyclic host;
    host.heartbeat = 3;
    host.cmd = HostCmd::FindStart;
    host.cmd_seq = 8;
    host.seam_id = 2;
    host.ref_start = {10.f, 20.f, 30.f};
    host.ref_end = {110.f, 20.f, 30.f};
    host.torch_a = 0.f;
    host.torch_b = 90.f;
    host.torch_c = 180.f;
    host.weld_speed_mm_s = 12.f;
    host.weave_mode = WeaveMode::Weave;
    host.weave_type = WeaveType::Triangle;
    host.amplitude_mm = 4.f;
    host.chord_mm = 18.f;
    host.laser_mode = LaserMode::FindAndTrack;
    host.laser_look_ahead_mm = 35.f;
    host.laser_search_radius_mm = 15.f;
    host.laser_timeout_ms = 4000;
    host.arc_enable = 1;

    HostCyclic host2;
    const std::string host_xml = encodeHostCyclicXml(host);
    expect(decodeHostCyclicXml(host_xml, &host2), "decode host");
    expect(host2.cmd == HostCmd::FindStart && host2.seam_id == 2, "host cmd");
    expect(xyzEq(host2.ref_start, host.ref_start) && xyzEq(host2.ref_end, host.ref_end), "ref xyz");
    expect(near(host2.torch_b, 90.f) && host2.weave_type == WeaveType::Triangle, "torch weave");
    expect(host2.laser_mode == LaserMode::FindAndTrack && host2.arc_enable == 1, "laser arc");
    expect(!decodeHostCyclicXml(host_xml, nullptr), "null host");
    expect(!decodeHostCyclicXml("<Host></Host>", &host2), "empty host");

    KukaCyclic kuka;
    kuka.heartbeat = 4;
    kuka.cmd_ack_seq = 8;
    kuka.phase = KukaPhase::FoundStart;
    kuka.laser_ready = 1;
    kuka.laser_start_valid = 1;
    kuka.found_start = {11.f, 21.f, 31.f};
    kuka.found_end = {109.f, 21.f, 31.f};
    KukaCyclic kuka2;
    expect(decodeKukaCyclicXml(encodeKukaCyclicXml(kuka), &kuka2), "decode kuka");
    expect(kuka2.phase == KukaPhase::FoundStart, "phase");
    expect(xyzEq(kuka2.found_start, kuka.found_start), "found start");
    expect(hostCmdName(HostCmd::MoveToFoundStart) == "MoveToFoundStart", "cmd name");
    expect(kukaPhaseName(KukaPhase::WeldToFoundEnd) == "WeldToFoundEnd", "phase name");

    const std::string md = commTableMarkdown();
    expect(md.find("| 字节偏移 | 字节数 | 数据类型 | 信号名 | 含义 |") != std::string::npos, "header");
    expect(md.find("| FLOAT32 | Seam_StartX |") != std::string::npos, "ref start");
    expect(md.find("| FLOAT32 | Found_EndZ |") != std::string::npos, "found end");
    expect(md.find("读取数据字节大小") != std::string::npos, "read size note");
    expect(md.find("**64**") != std::string::npos, "64 bytes");
    expect(md.find("Extern_Speed") == std::string::npos, "md has no extras");
    const std::string csv = commTableCsv();
    expect(csv.find("字节偏移,字节数,数据类型,信号名,含义,方向") == 0, "csv");
    expect(ekiConfigXml("10.0.0.8", 54600).find("Tag=\"Laser_LookAhead\"") != std::string::npos,
           "eki laser");

    if (g_failed != 0) {
        std::cerr << g_failed << " assertion(s) failed\n";
        return 1;
    }
    std::cout << "KukaWeldComm tests passed (" << signals.size() << " signals)\n";
    return 0;
}
