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
