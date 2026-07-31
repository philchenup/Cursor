#include "chinese_number.h"

#include <iostream>
#include <string>
#include <utility>
#include <vector>

int main()
{
    const std::vector<std::pair<std::string, std::string>> cases = {
        {"十二毫米", "12毫米"},
        {"二十三毫米", "23毫米"},
        {"一百零八", "108"},
        {"十点五度", "10.5度"},
        {"两千零五", "2005"},
        {"三万二千一百", "32100"},
        {"识别到十二毫米和三十五毫米", "识别到12毫米和35毫米"},
        {"二零二三", "2023"},
        {"两毫米", "2毫米"},
        {"12毫米", "12毫米"},
        // arbitrary / extended
        {"负二十三点五度", "-23.5度"},
        {"一亿零三百零四万", "103040000"},
        {"十 二 毫 米", "12毫米"},
        {"二零二三年度", "2023年度"},
        {"壹佰贰拾叁", "123"},
        {"廿五毫米", "25毫米"},
        {"卅二", "32"},
        {"九千九百九十九", "9999"},
        {"一百亿", "10000000000"},
        {"第三十二号", "第32号"},
        {"点五毫米", "0.5毫米"},
        {"１２３毫米", "123毫米"},
        {"一万零一", "10001"},
        {"三千零二十", "3020"},
    };

    int failed = 0;
    for (const auto& c : cases)
    {
        const std::string got = normalize_chinese_numbers(c.first);
        if (got != c.second)
        {
            std::cerr << "FAIL: \"" << c.first << "\" -> \"" << got
                      << "\" (expected \"" << c.second << "\")\n";
            ++failed;
        }
        else
        {
            std::cout << "OK: \"" << c.first << "\" -> \"" << got << "\"\n";
        }
    }

    return failed == 0 ? 0 : 1;
}
