#include "CommFormat.h"

#include <QString>
#include <cstdio>
#include <vector>

// 用法示例：把分组浮点数据编成通信报文字符串。
// 可直接用于 SocketWorker::sendMsg(FormatCommData(groups, cfg));
int main()
{
    CommConfig cfg;
    cfg.group_inner = ",";
    cfg.group_inter = ";";
    cfg.end_sign = ".";
    cfg.valid_num = 3;

    const std::vector<std::vector<float>> groups = {
        {1.23456f, 2.0f, 3.14159f},
        {10.5f, 0.001234f}
    };

    const QString payload = FormatCommData(groups, cfg);
    std::printf("payload: %s\n", qPrintable(payload));
    return 0;
}
