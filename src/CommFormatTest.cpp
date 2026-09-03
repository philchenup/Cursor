#include "CommFormat.h"

#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

namespace {

int g_failed = 0;

void ExpectEq(const char* name, const QString& actual, const QString& expected)
{
    if (actual == expected) {
        std::printf("PASS  %s  [%s]\n", name, qPrintable(actual));
        return;
    }
    std::printf("FAIL  %s\n  actual:   [%s]\n  expected: [%s]\n",
                name, qPrintable(actual), qPrintable(expected));
    ++g_failed;
}

} // namespace

int main()
{
    // 默认配置：组内 ','，组间 ';'，结尾 '.'，3 位有效数字
    {
        const std::vector<std::vector<float>> data = {
            {1.234f, 5.678f},
            {9.012f}
        };
        ExpectEq("default two groups",
                 FormatCommData(data),
                 QStringLiteral("1.23,5.68;9.01."));
    }

    {
        const std::vector<std::vector<float>> data = {
            {1.23456f, 2.0f, 3.14159f},
            {10.5f, 0.001234f}
        };
        ExpectEq("mixed magnitudes",
                 FormatCommData(data),
                 QStringLiteral("1.23,2,3.14;10.5,0.00123."));
    }

    {
        CommConfig cfg;
        cfg.group_inner = " ";
        cfg.group_inter = "|";
        cfg.end_sign = "#";
        cfg.valid_num = 4;
        const std::vector<std::vector<float>> data = {
            {1.23456f, -5.6789f},
            {100.0f}
        };
        ExpectEq("custom separators and 4 digits",
                 FormatCommData(data, cfg),
                 QStringLiteral("1.235 -5.679|100#"));
    }

    {
        ExpectEq("empty data is only end_sign",
                 FormatCommData({}),
                 QStringLiteral("."));
    }

    {
        const std::vector<std::vector<float>> data = {
            {1.0f},
            {},
            {2.0f}
        };
        ExpectEq("empty inner group",
                 FormatCommData(data),
                 QStringLiteral("1;;2."));
    }

    {
        const std::vector<std::vector<float>> data = {{0.0f}};
        ExpectEq("single zero",
                 FormatCommData(data),
                 QStringLiteral("0."));
    }

    {
        CommConfig cfg;
        cfg.valid_num = 0; // 非法值，实现侧钳到至少 1 位
        const std::vector<std::vector<float>> data = {{12.3f}};
        ExpectEq("valid_num clamped to 1",
                 FormatCommData(data, cfg),
                 QStringLiteral("1e+01."));
    }

    if (g_failed != 0) {
        std::printf("\n%d test(s) failed\n", g_failed);
        return EXIT_FAILURE;
    }

    std::printf("\nall tests passed\n");
    return EXIT_SUCCESS;
}
