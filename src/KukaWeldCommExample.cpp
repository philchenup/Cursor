#include "KukaWeldComm.h"

#include <fstream>
#include <iostream>
#include <string>

namespace {

bool writeFile(const std::string& path, const std::string& body)
{
    std::ofstream out(path.c_str());
    if (!out) {
        std::cerr << "cannot write " << path << '\n';
        return false;
    }
    out << body;
    return true;
}

}  // namespace

int main(int argc, char** argv)
{
    std::string csv_path;
    std::string eki_path;
    if (argc > 1) {
        csv_path = argv[1];
    }
    if (argc > 2) {
        eki_path = argv[2];
    }

    std::cout << "# 上位机 ↔ KUKA 焊接通讯数据结构表\n\n";
    std::cout << "共 " << kukaWeldCommSignals().size() << " 个信号。"
              << "命令 Start=" << static_cast<int>(HostCmd::Start)
              << "，阶段 Welding=" << static_cast<int>(KukaPhase::Welding)
              << "。\n\n";
    std::cout << commTableMarkdown() << '\n';

    HostCyclic demo;
    demo.heartbeat = 1;
    demo.cmd = HostCmd::DownloadSeam;
    demo.cmd_seq = 1;
    demo.job.job_id = 1;
    demo.job.seam_count = 1;
    demo.job.pass_count = 1;
    demo.seam.seam_id = 1;
    demo.seam.start = {0.f, 0.f, 0.f, 0.f, 90.f, 180.f, 0.f};
    demo.seam.end = {100.f, 0.f, 0.f, 0.f, 90.f, 180.f, 0.f};
    demo.seam.speed_mm_s = 10.f;
    demo.seam.weave_mode = WeaveMode::Weave;
    demo.seam.weave_type = WeaveType::Sine;
    demo.pass.seam_id = 1;
    demo.pass.sequence = 1;
    demo.traj.count = 2;
    demo.traj.flag = 1;
    demo.welder.arc_enable = 1;
    demo.welder.gas_enable = 1;
    std::cout << "## 上位机→KUKA 报文示例\n\n```xml\n"
              << encodeHostCyclicXml(demo) << "\n```\n\n";

    KukaCyclic fb;
    fb.heartbeat = 1;
    fb.phase = KukaPhase::Ready;
    fb.op_mode = KukaOpMode::Ext;
    fb.pro_active = 1;
    fb.drives_on = 1;
    fb.download_ok = 1;
    std::cout << "## KUKA→上位机 报文示例\n\n```xml\n"
              << encodeKukaCyclicXml(fb) << "\n```\n";

    if (!csv_path.empty() && !writeFile(csv_path, commTableCsv())) {
        return 1;
    }
    if (!eki_path.empty() && !writeFile(eki_path, ekiConfigXml())) {
        return 1;
    }
    return 0;
}
