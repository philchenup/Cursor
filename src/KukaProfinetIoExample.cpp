#include "KukaProfinetIo.h"

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
    if (argc > 1) {
        csv_path = argv[1];
    }

    std::cout << "# KUKA PROFINET 轴/位姿过程数据\n\n";
    std::cout << "读取 3 个外部轴 + 6 个关节角 + 6 个 TCP 位姿，共 **"
              << kKukaPnPayloadBytes << "** 字节，CIFX 帧 **" << kKukaPnFrameBytes
              << "** 字节。\n\n";
    std::cout << kukaPnMapMarkdown();

    std::cout << "\n## CIFX 读写（替换 ChannelDemo 里的裸字节循环）\n\n";
    std::cout << "```cpp\n"
              << "unsigned char abRecvData[kKukaPnFrameBytes] = {0};\n"
              << "unsigned char abSendData[kKukaPnFrameBytes] = {0};\n"
              << "xChannelIORead(hChannel, 0, 0, sizeof(abRecvData), abRecvData, IO_WAIT_TIMEOUT);\n"
              << "KukaAxisPose fb;\n"
              << "decodeKukaPnAxisPose(abRecvData, sizeof(abRecvData), &fb);  // 读 E1-E3,A1-A6,XYZABC\n"
              << "KukaAxisPose cmd = fb;  // 示例：把当前位姿作为下发目标\n"
              << "encodeKukaPnAxisPose(cmd, abSendData, sizeof(abSendData));\n"
              << "xChannelIOWrite(hChannel, 0, 0, sizeof(abSendData), abSendData, IO_WAIT_TIMEOUT);\n"
              << "```\n";

    if (!csv_path.empty() && !writeFile(csv_path, kukaPnMapCsv())) {
        return 1;
    }
    return 0;
}
