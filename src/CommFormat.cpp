#include "CommFormat.h"

#include <QStringList>

#include <algorithm>

namespace {

QString FloatToValidString(float value, int validNum)
{
    // Qt 'g'：最多 validNum 位有效数字；始终使用 C locale（小数点为 '.'）。
    const int digits = std::max(1, validNum);
    return QString::number(static_cast<double>(value), 'g', digits);
}

} // namespace

QString FormatCommData(const std::vector<std::vector<float>>& data,
                       const CommConfig& config)
{
    const QString innerSep = QString::fromStdString(config.group_inner);
    const QString interSep = QString::fromStdString(config.group_inter);
    const QString endSign = QString::fromStdString(config.end_sign);

    QStringList groups;
    groups.reserve(static_cast<int>(data.size()));

    for (const std::vector<float>& group : data) {
        QStringList items;
        items.reserve(static_cast<int>(group.size()));
        for (float value : group) {
            items.append(FloatToValidString(value, config.valid_num));
        }
        groups.append(items.join(innerSep));
    }

    return groups.join(interSep) + endSign;
}
