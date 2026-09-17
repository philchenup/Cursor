#include "KukaWeldComm.h"

#include <cmath>
#include <cstdlib>
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

bool poseEq(const KukaPose& a, const KukaPose& b)
{
    return near(a.x, b.x) && near(a.y, b.y) && near(a.z, b.z)
        && near(a.a, b.a) && near(a.b, b.b) && near(a.c, b.c) && near(a.e1, b.e1);
}

}  // namespace

int main()
{
    const std::vector<CommSignal>& signals = kukaWeldCommSignals();
    expect(signals.size() == 101, "signal count is 101");

    std::set<int> indices;
    std::set<std::string> names;
    std::set<std::string> xpaths;
    int host_n = 0;
    int kuka_n = 0;
    for (const CommSignal& s : signals) {
        expect(s.index > 0, "positive index");
        expect(indices.insert(s.index).second, "unique index");
        expect(names.insert(s.name).second, "unique signal name");
        expect(xpaths.insert(s.xpath).second, "unique xpath");
        expect(s.size_bytes == 4, "4-byte KRL field");
        if (s.direction == CommDirection::HostToKuka) {
            ++host_n;
            expect(std::string(s.xpath).rfind("Host/", 0) == 0, "host xpath prefix");
        } else {
            ++kuka_n;
            expect(std::string(s.xpath).rfind("Kuka/", 0) == 0, "kuka xpath prefix");
        }
    }
    expect(host_n == 76, "76 host-to-kuka signals");
    expect(kuka_n == 25, "25 kuka-to-host signals");

    bool found_inset = false;
    bool found_weave = false;
    bool found_multi = false;
    for (const CommSignal& s : signals) {
        if (std::string(s.name) == "InsetMm") {
            found_inset = std::string(s.process_step) == "下载焊缝";
        }
        if (std::string(s.name) == "WeaveType") {
            found_weave = std::string(s.range).find("正弦") != std::string::npos;
        }
        if (std::string(s.name) == "ThicknessMm") {
            found_multi = std::string(s.group) == "焊缝工艺";
        }
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
    host.traj.pose = {15.f, 22.f, 30.f, 0.f, 90.f, 180.f, 100.f};
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
    expect(host_xml.find("<Host>") == 0, "host xml root");
    expect(host_xml.find("<Seam>") != std::string::npos, "host xml has Seam");
    expect(host_xml.find("<Welder>") != std::string::npos, "host xml has Welder");

    HostCyclic host2;
    expect(decodeHostCyclicXml(host_xml, &host2), "decode host xml");
    expect(host2.heartbeat == 42, "host heartbeat");
    expect(host2.cmd == HostCmd::Start, "host cmd");
    expect(host2.cmd_seq == 7, "host cmd seq");
    expect(host2.job.job_id == 1001 && host2.job.pass_count == 5, "job header");
    expect(near(host2.job.approach_mm, 40.f), "approach");
    expect(poseEq(host2.seam.start, host.seam.start), "seam start");
    expect(poseEq(host2.seam.end, host.seam.end), "seam end");
    expect(near(host2.seam.inset_mm, 2.5f), "inset");
    expect(host2.seam.weave_mode == WeaveMode::Weave, "weave mode");
    expect(host2.seam.weave_type == WeaveType::Triangle, "weave type");
    expect(host2.seam.multi_mode == MultiMode::Multi, "multi mode");
    expect(near(host2.seam.groove_deg, 30.f), "groove");
    expect(host2.pass.kind == PassKind::Fill && host2.pass.layer == 2, "pass");
    expect(poseEq(host2.pass.start, host.pass.start), "pass start");
    expect(near(host2.pass.speed_mm_s, 8.f), "pass speed distinct from seam");
    expect(near(host2.traj.pose.x, 15.f) && host2.traj.flag == 1, "traj");
    expect(near(host2.welder.current_a, 180.f), "welder current");
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
    kuka.tcp = {15.f, 22.f, 30.f, 0.f, 90.f, 180.f, 100.f};
    kuka.act_speed_mm_s = 8.8f;
    kuka.arc_on = 1;
    kuka.progress_pct = 33.5f;
    const std::string kuka_xml = encodeKukaCyclicXml(kuka);
    KukaCyclic kuka2;
    expect(decodeKukaCyclicXml(kuka_xml, &kuka2), "decode kuka xml");
    expect(kuka2.phase == KukaPhase::Welding, "phase");
    expect(kuka2.op_mode == KukaOpMode::Ext, "ext mode");
    expect(poseEq(kuka2.tcp, kuka.tcp), "tcp");
    expect(near(kuka2.progress_pct, 33.5f), "progress");
    expect(kukaPhaseName(KukaPhase::RailMove) == "RailMove", "phase name");
    expect(hostCmdName(HostCmd::DownloadSeam) == "DownloadSeam", "cmd name");

    const KukaPose inset_s = insetPose(host.seam.start, host.seam.end, 10.f, true);
    const KukaPose inset_e = insetPose(host.seam.start, host.seam.end, 10.f, false);
    expect(near(inset_s.x, 20.f) && near(inset_e.x, 100.f), "inset along seam X");
    expect(near(inset_s.y, 20.f) && near(inset_e.y, 20.f), "inset keeps Y");

    KukaPose same = host.seam.start;
    const KukaPose same_out = insetPose(same, same, 5.f, true);
    expect(near(same_out.x, same.x), "zero-length seam inset");

    std::vector<TrajPoint> traj(2);
    traj[0].pose = {1.f, 2.f, 3.f, 4.f, 5.f, 6.f, 7.f};
    traj[0].speed_mm_s = 10.f;
    traj[0].flag = 1;
    traj[1].pose = {8.f, 9.f, 10.f, 11.f, 12.f, 13.f, 14.f};
    traj[1].speed_mm_s = 11.f;
    traj[1].flag = 2;
    const std::string csv = formatTrajCsv(traj);
    expect(csv.find("1,2,3,4,5,6,7,10,1;") == 0, "traj csv first group");
    expect(csv.find(";8,9,10,11,12,13,14,11,2.") != std::string::npos, "traj csv end sign");

    const std::string md = commTableMarkdown();
    expect(md.find("| 序号 |") == 0, "markdown header");
    expect(md.find("Host/Seam/InsetMm") != std::string::npos, "markdown xpath");
    const std::string table_csv = commTableCsv();
    expect(table_csv.find("序号,分组") == 0, "csv header");
    expect(table_csv.find("Kuka/ProgressPct") != std::string::npos, "csv last xpath");

    const std::string eki = ekiConfigXml("10.0.0.8", 54600);
    expect(eki.find("<IP>10.0.0.8</IP>") != std::string::npos, "eki ip");
    expect(eki.find("Tag=\"Host/Seam/InsetMm\"") != std::string::npos, "eki receive inset");
    expect(eki.find("Tag=\"Kuka/Phase\"") != std::string::npos, "eki send phase");
    expect(eki.find("<RECEIVE>") != std::string::npos && eki.find("<SEND>") != std::string::npos,
           "eki receive/send");

    if (g_failed != 0) {
        std::cerr << g_failed << " assertion(s) failed\n";
        return 1;
    }
    std::cout << "KukaWeldComm tests passed (" << signals.size() << " signals)\n";
    return 0;
}
