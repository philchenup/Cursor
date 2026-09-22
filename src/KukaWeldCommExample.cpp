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
    std::cout << "共 " << kukaWeldCommSignals().size() << " 个信号，仅含总流程必要项。\n";
    std::cout << "读取数据（KUKA→上位机）打包 **" << kukaWeldPackedBytes(CommDirection::KukaToHost)
              << "** 字节；下发数据（上位机→KUKA）打包 "
              << kukaWeldPackedBytes(CommDirection::HostToKuka) << " 字节。\n\n";
    std::cout << commTableMarkdown() << '\n';

    HostCyclic demo;
    demo.heartbeat = 1;
    demo.cmd = HostCmd::FindStart;
    demo.cmd_seq = 1;
    demo.seam_id = 1;
    demo.ref_end = {100.f, 0.f, 0.f};
    demo.weld_speed_mm_s = 10.f;
    demo.weave_mode = WeaveMode::Weave;
    demo.laser_mode = LaserMode::Find;
    demo.arc_enable = 1;
    std::cout << "## 上位机→KUKA 报文示例\n\n```xml\n"
              << encodeHostCyclicXml(demo) << "\n```\n\n";

    KukaCyclic fb;
    fb.heartbeat = 1;
    fb.phase = KukaPhase::Ready;
    fb.laser_ready = 1;
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
