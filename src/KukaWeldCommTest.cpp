#include "KukaWeldComm.h"

#include <cmath>
#include <iostream>
#include <set>
#include <string>
#include <vector>

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

bool poseEq(const KukaPose& a, const KukaPose& b)
{
    return near(a.x, b.x) && near(a.y, b.y) && near(a.z, b.z)
        && near(a.a, b.a) && near(a.b, b.b) && near(a.c, b.c);
}

bool motionEq(const MotionBlock& a, const MotionBlock& b)
{
    return near(a.extern_speed, b.extern_speed) && near(a.extern_acc, b.extern_acc)
        && near(a.robot_speed, b.robot_speed) && near(a.robot_acc, b.robot_acc)
        && near(a.e1, b.e1) && near(a.e2, b.e2) && near(a.e3, b.e3)
        && near(a.x, b.x) && near(a.y, b.y) && near(a.z, b.z)
        && near(a.a, b.a) && near(a.b, b.b) && near(a.c, b.c)
        && near(a.j1, b.j1) && near(a.j2, b.j2) && near(a.j3, b.j3)
        && near(a.j4, b.j4) && near(a.j5, b.j5) && near(a.j6, b.j6);
}

const char* kScreenshotNames[] = {
    "Extern_Speed", "Extern_Acc", "Robot_Speed", "Robot_Acc",
    "Extern_E1", "Extern_E2", "Extern_E3",
    "Robot_X", "Robot_Y", "Robot_Z", "Robot_A", "Robot_B", "Robot_C",
    "Robot_J1", "Robot_J2", "Robot_J3", "Robot_J4", "Robot_J5", "Robot_J6",
};

}  // namespace

int main()
{
    const std::vector<CommSignal>& signals = kukaWeldCommSignals();
    expect(signals.size() == 120, "signal count is 120");

    std::set<std::string> names;
    int host_n = 0;
    int kuka_n = 0;
    for (const CommSignal& s : signals) {
        expect(s.index > 0, "positive index");
        expect(names.insert(s.name).second, "unique signal name");
        const std::string t = s.type;
        expect(t == "FLOAT32" || t == "INT32" || t == "BOOL", "plc type");
        if (s.direction == CommDirection::HostToKuka) {
            ++host_n;
        } else {
            ++kuka_n;
        }
    }
    expect(host_n == 84, "84 host-to-kuka signals");
    expect(kuka_n == 36, "36 kuka-to-host signals");

    for (int i = 0; i < 19; ++i) {
        expect(std::string(signals[static_cast<size_t>(i)].type) == "FLOAT32", "motion FLOAT32");
        expect(std::string(signals[static_cast<size_t>(i)].name) == kScreenshotNames[i],
               "screenshot motion name order");
        expect(signals[static_cast<size_t>(i)].direction == CommDirection::HostToKuka,
               "screenshot block is host to kuka");
    }

    bool found_inset = false;
    bool found_weave = false;
    bool found_multi = false;
    for (const CommSignal& s : signals) {
        found_inset = found_inset || std::string(s.name) == "Seam_Inset";
        found_weave = found_weave || std::string(s.name) == "Seam_WeaveType";
        found_multi = found_multi || std::string(s.name) == "Seam_Thickness";
    }
    expect(found_inset && found_weave && found_multi, "weld-list columns mapped");

    HostCyclic host;
    host.heartbeat = 42;
    host.cmd = HostCmd::Start;
    host.cmd_seq = 7;
    host.job.job_id = 1001;
    host.job.seam_count = 2;
    host.job.pass_count = 5;
    host.job.tool_no = 1;
    host.job.base_no = 2;
    host.job.override_pct = 80.f;
    host.job.approach_mm = 40.f;
    host.job.retract_mm = 60.f;
    host.motion.extern_speed = 200.f;
    host.motion.extern_acc = 50.f;
    host.motion.robot_speed = 12.5f;
    host.motion.robot_acc = 8.f;
    host.motion.e1 = 100.f;
    host.motion.e2 = 0.f;
    host.motion.e3 = 0.f;
    host.motion.x = 15.f;
    host.motion.y = 22.f;
    host.motion.z = 30.f;
    host.motion.a = 0.f;
    host.motion.b = 90.f;
    host.motion.c = 180.f;
    host.motion.j1 = 1.f;
    host.motion.j2 = 2.f;
    host.motion.j3 = 3.f;
    host.motion.j4 = 4.f;
    host.motion.j5 = 5.f;
    host.motion.j6 = 6.f;
    host.seam.seam_id = 1;
    host.seam.start = {10.f, 20.f, 30.f, 0.f, 90.f, 180.f, 100.f};
    host.seam.end = {110.f, 20.f, 30.f, 0.f, 90.f, 180.f, 100.f};
    host.seam.inset_mm = 2.5f;
    host.seam.speed_mm_s = 12.5f;
    host.seam.weave_mode = WeaveMode::Weave;
    host.seam.weave_type = WeaveType::Triangle;
    host.seam.amplitude_mm = 5.f;
    host.seam.chord_mm = 20.f;
    host.seam.multi_mode = MultiMode::Multi;
    host.seam.thickness_mm = 16.f;
    host.seam.groove_deg = 30.f;
    host.seam.fitup_gap_mm = 1.f;
    host.seam.penetration_mm = 2.f;
    host.pass.seam_id = 1;
    host.pass.layer = 2;
    host.pass.local_index = 3;
    host.pass.sequence = 4;
    host.pass.kind = PassKind::Fill;
    host.pass.start = {12.f, 20.f, 31.f, 1.f, 89.f, 179.f, 100.f};
    host.pass.end = {108.f, 20.f, 31.f, 1.f, 89.f, 179.f, 100.f};
    host.pass.speed_mm_s = 8.f;
    host.traj.index = 3;
    host.traj.count = 40;
    host.traj.speed_mm_s = 9.f;
    host.traj.flag = 1;
    host.welder.arc_enable = 1;
    host.welder.gas_enable = 1;
    host.welder.current_a = 180.f;
    host.welder.voltage_v = 22.5f;
    host.welder.wire_m_min = 6.2f;
    host.welder.gas_preflow_ms = 250;
    host.welder.gas_postflow_ms = 400;
    host.welder.crater_ms = 150;

    const std::string host_xml = encodeHostCyclicXml(host);
    expect(host_xml.find("<Extern_Speed>") != std::string::npos, "host xml motion");
    expect(host_xml.find("<Robot_J6>") != std::string::npos, "host xml joints");
    expect(host_xml.find("<Seam_Inset>") != std::string::npos, "host xml inset");

    HostCyclic host2;
    expect(decodeHostCyclicXml(host_xml, &host2), "decode host xml");
    expect(host2.heartbeat == 42 && host2.cmd == HostCmd::Start, "host cmd");
    expect(motionEq(host2.motion, host.motion), "host motion");
    expect(poseEq(host2.seam.start, host.seam.start), "seam start");
    expect(near(host2.seam.inset_mm, 2.5f), "inset");
    expect(host2.seam.weave_type == WeaveType::Triangle, "weave type");
    expect(near(host2.pass.speed_mm_s, 8.f), "pass speed");
    expect(near(host2.welder.current_a, 180.f), "welder current");
    expect(near(host2.motion.j6, 6.f), "joint 6");
    expect(!decodeHostCyclicXml(host_xml, nullptr), "null host decode");
    expect(!decodeHostCyclicXml("<Host></Host>", &host2), "empty host xml");

    KukaCyclic kuka;
    kuka.heartbeat = 9;
    kuka.cmd_ack_seq = 7;
    kuka.phase = KukaPhase::Welding;
    kuka.op_mode = KukaOpMode::Ext;
    kuka.pro_active = 1;
    kuka.drives_on = 1;
    kuka.seam_id = 1;
    kuka.layer = 2;
    kuka.pass_seq = 4;
    kuka.traj_index = 3;
    kuka.motion = host.motion;
    kuka.arc_on = 1;
    kuka.progress_pct = 33.5f;
    const std::string kuka_xml = encodeKukaCyclicXml(kuka);
    expect(kuka_xml.find("<Act_Robot_X>") != std::string::npos, "kuka actual pose");
    KukaCyclic kuka2;
    expect(decodeKukaCyclicXml(kuka_xml, &kuka2), "decode kuka xml");
    expect(kuka2.phase == KukaPhase::Welding, "phase");
    expect(motionEq(kuka2.motion, kuka.motion), "kuka motion");
    expect(near(kuka2.progress_pct, 33.5f), "progress");

    const KukaPose inset_s = insetPose(host.seam.start, host.seam.end, 10.f, true);
    const KukaPose inset_e = insetPose(host.seam.start, host.seam.end, 10.f, false);
    expect(near(inset_s.x, 20.f) && near(inset_e.x, 100.f), "inset along seam X");

    std::vector<TrajPoint> traj(2);
    traj[0].pose = {1.f, 2.f, 3.f, 4.f, 5.f, 6.f, 7.f};
    traj[0].speed_mm_s = 10.f;
    traj[0].flag = 1;
    traj[1].pose = {8.f, 9.f, 10.f, 11.f, 12.f, 13.f, 14.f};
    traj[1].speed_mm_s = 11.f;
    traj[1].flag = 2;
    const std::string csv = formatTrajCsv(traj);
    expect(csv.find("1,2,3,4,5,6,7,10,1;") == 0, "traj csv first group");

    const std::string md = commTableMarkdown();
    expect(md.find("| 数据类型 | 信号名 |") != std::string::npos, "two-column header");
    expect(md.find("| FLOAT32 | Extern_Speed |") != std::string::npos, "screenshot first row");
    expect(md.find("| FLOAT32 | Robot_J6 |") != std::string::npos, "screenshot last joint");
    const std::string table_csv = commTableCsv();
    expect(table_csv.find("数据类型,信号名,方向") == 0, "csv header");
    expect(table_csv.find("FLOAT32,Extern_Speed,") != std::string::npos, "csv motion");

    const std::string eki = ekiConfigXml("10.0.0.8", 54600);
    expect(eki.find("Tag=\"Extern_Speed\" Type=\"REAL\"") != std::string::npos, "eki motion tag");
    expect(eki.find("Tag=\"Seam_Inset\"") != std::string::npos, "eki seam");
    expect(eki.find("Tag=\"Act_Robot_J6\"") != std::string::npos, "eki actual joints");

    if (g_failed != 0) {
        std::cerr << g_failed << " assertion(s) failed\n";
        return 1;
    }
    std::cout << "KukaWeldComm tests passed (" << signals.size() << " signals)\n";
    return 0;
}
