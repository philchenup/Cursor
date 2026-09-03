#ifndef COMM_FORMAT_H
#define COMM_FORMAT_H

#include <QString>

#include <string>
#include <vector>

/**
 * @brief 通信报文格式配置。
 *
 * 内层 vector 为一组（组内），外层 vector 为多组（组间）。
 * 每个 float 按 valid_num 位有效数字转为十进制字符串。
 */
struct CommConfig
{
    std::string group_inner = ",";
    std::string group_inter = ";";
    std::string end_sign = ".";
    int valid_num = 3;
};

/**
 * @brief 按 CommConfig 将分组浮点数据格式化为通信字符串。
 *
 * 组内元素用 group_inner 连接，组间用 group_inter 连接，整段末尾追加 end_sign。
 * 浮点转换使用 Qt 的 'g' 格式（有效数字），小数点固定为 '.'，与 locale 无关。
 *
 * 示例（默认配置）：
 *   {{1.234f, 5.678f}, {9.01f}}  →  "1.23,5.68;9.01."
 *
 * @param data   外层为组间、内层为组内的浮点数据
 * @param config 分隔符与有效数字配置，默认 "," / ";" / "." / 3
 * @return 格式化后的 QString，可直接交给 SocketWorker::sendMsg 等接口
 */
QString FormatCommData(const std::vector<std::vector<float>>& data,
                       const CommConfig& config = CommConfig());

#endif // COMM_FORMAT_H
