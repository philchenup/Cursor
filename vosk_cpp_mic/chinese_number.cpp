#include "chinese_number.h"

#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace {

struct CnToken
{
    enum Kind
    {
        Digit,
        Ten,
        Hundred,
        Thousand,
        Wan,
        Yi,
        Dot,
        Other
    };

    Kind kind = Other;
    int value = 0;
    std::string utf8;
};

bool starts_with(const std::string& s, size_t pos, const char* lit)
{
    const size_t n = std::char_traits<char>::length(lit);
    return pos + n <= s.size() && s.compare(pos, n, lit) == 0;
}

size_t utf8_char_len(unsigned char c)
{
    if ((c & 0x80) == 0)
        return 1;
    if ((c & 0xE0) == 0xC0)
        return 2;
    if ((c & 0xF0) == 0xE0)
        return 3;
    if ((c & 0xF8) == 0xF0)
        return 4;
    return 1;
}

CnToken match_token(const std::string& s, size_t pos)
{
    static const std::pair<const char*, int> kDigits[] = {
        {"零", 0}, {"〇", 0}, {"○", 0}, {"0", 0},
        {"一", 1}, {"壹", 1}, {"1", 1},
        {"二", 2}, {"贰", 2}, {"兩", 2}, {"两", 2}, {"俩", 2}, {"2", 2},
        {"三", 3}, {"叁", 3}, {"3", 3},
        {"四", 4}, {"肆", 4}, {"4", 4},
        {"五", 5}, {"伍", 5}, {"5", 5},
        {"六", 6}, {"陆", 6}, {"6", 6},
        {"七", 7}, {"柒", 7}, {"7", 7},
        {"八", 8}, {"捌", 8}, {"8", 8},
        {"九", 9}, {"玖", 9}, {"9", 9},
    };

    for (const auto& d : kDigits)
    {
        if (starts_with(s, pos, d.first))
        {
            CnToken t;
            t.kind = CnToken::Digit;
            t.value = d.second;
            t.utf8 = d.first;
            return t;
        }
    }

    struct Unit
    {
        const char* lit;
        CnToken::Kind kind;
    };
    static const Unit kUnits[] = {
        {"十", CnToken::Ten},     {"拾", CnToken::Ten},
        {"百", CnToken::Hundred}, {"佰", CnToken::Hundred},
        {"千", CnToken::Thousand},{"仟", CnToken::Thousand},
        {"万", CnToken::Wan},     {"萬", CnToken::Wan},
        {"亿", CnToken::Yi},      {"億", CnToken::Yi},
        {"点", CnToken::Dot},     {"點", CnToken::Dot},
        {".", CnToken::Dot},
    };

    for (const auto& u : kUnits)
    {
        if (starts_with(s, pos, u.lit))
        {
            CnToken t;
            t.kind = u.kind;
            t.utf8 = u.lit;
            return t;
        }
    }

    CnToken other;
    other.kind = CnToken::Other;
    other.utf8 = s.substr(pos, utf8_char_len(static_cast<unsigned char>(s[pos])));
    return other;
}

bool is_num_token(CnToken::Kind k)
{
    return k == CnToken::Digit || k == CnToken::Ten || k == CnToken::Hundred
        || k == CnToken::Thousand || k == CnToken::Wan || k == CnToken::Yi
        || k == CnToken::Dot;
}

bool is_digit_run(const std::vector<CnToken>& toks, size_t begin, size_t end)
{
    if (begin >= end)
        return false;
    for (size_t i = begin; i < end; ++i)
    {
        if (toks[i].kind != CnToken::Digit)
            return false;
    }
    return true;
}

long long parse_digit_run(const std::vector<CnToken>& toks, size_t begin, size_t end)
{
    long long v = 0;
    for (size_t i = begin; i < end; ++i)
        v = v * 10 + toks[i].value;
    return v;
}

long long parse_cn_integer(const std::vector<CnToken>& toks, size_t begin, size_t end)
{
    if (begin >= end)
        return 0;

    // 二零二三 / 一二 → pure digit sequence
    if (is_digit_run(toks, begin, end))
        return parse_digit_run(toks, begin, end);

    long long result = 0;
    long long section = 0;
    long long number = -1; // pending digit; -1 = none

    for (size_t i = begin; i < end; ++i)
    {
        const auto kind = toks[i].kind;
        if (kind == CnToken::Digit)
        {
            number = toks[i].value;
            continue;
        }

        auto apply_unit = [&](long long unit) {
            long long n = number;
            number = -1;
            if (n < 0)
                n = 1; // 十二 / 百八 / 千三
            section += n * unit;
        };

        if (kind == CnToken::Ten)
            apply_unit(10);
        else if (kind == CnToken::Hundred)
            apply_unit(100);
        else if (kind == CnToken::Thousand)
            apply_unit(1000);
        else if (kind == CnToken::Wan)
        {
            const long long n = number < 0 ? 0 : number;
            number = -1;
            result += (section + n) * 10000LL;
            section = 0;
        }
        else if (kind == CnToken::Yi)
        {
            const long long n = number < 0 ? 0 : number;
            number = -1;
            result += (section + n) * 100000000LL;
            section = 0;
        }
    }

    result += section + (number < 0 ? 0 : number);
    return result;
}

std::string tokens_to_arabic(const std::vector<CnToken>& toks)
{
    size_t dot = toks.size();
    for (size_t i = 0; i < toks.size(); ++i)
    {
        if (toks[i].kind == CnToken::Dot)
        {
            dot = i;
            break;
        }
    }

    if (dot == toks.size())
        return std::to_string(parse_cn_integer(toks, 0, toks.size()));

    const long long ip = (dot == 0) ? 0 : parse_cn_integer(toks, 0, dot);
    std::string frac;
    for (size_t i = dot + 1; i < toks.size(); ++i)
    {
        if (toks[i].kind != CnToken::Digit)
            break;
        frac.push_back(static_cast<char>('0' + toks[i].value));
    }

    std::string out = std::to_string(ip);
    if (!frac.empty())
    {
        out.push_back('.');
        out += frac;
    }
    return out;
}

bool segment_only_ascii(const std::vector<CnToken>& all, size_t begin, size_t end)
{
    for (size_t k = begin; k < end; ++k)
    {
        for (unsigned char c : all[k].utf8)
        {
            if (c >= 0x80)
                return false;
        }
    }
    return true;
}

} // namespace

std::string normalize_chinese_numbers(const std::string& text)
{
    if (text.empty())
        return text;

    std::vector<CnToken> all;
    all.reserve(text.size());
    for (size_t i = 0; i < text.size();)
    {
        CnToken t = match_token(text, i);
        i += t.utf8.size();
        all.push_back(std::move(t));
    }

    std::string out;
    out.reserve(text.size());

    size_t i = 0;
    while (i < all.size())
    {
        if (!is_num_token(all[i].kind))
        {
            out += all[i].utf8;
            ++i;
            continue;
        }

        size_t j = i;
        bool saw_dot = false;
        bool saw_value = false;
        while (j < all.size() && is_num_token(all[j].kind))
        {
            if (all[j].kind == CnToken::Dot)
            {
                if (saw_dot)
                    break;
                saw_dot = true;
                ++j;
                continue;
            }
            saw_value = true;
            ++j;
        }

        if (!saw_value)
        {
            out += all[i].utf8;
            ++i;
            continue;
        }

        // Keep already-Arabic runs unchanged (e.g. "12.5毫米").
        if (segment_only_ascii(all, i, j))
        {
            for (size_t k = i; k < j; ++k)
                out += all[k].utf8;
            i = j;
            continue;
        }

        std::vector<CnToken> segment(all.begin() + static_cast<std::ptrdiff_t>(i),
                                     all.begin() + static_cast<std::ptrdiff_t>(j));
        out += tokens_to_arabic(segment);
        i = j;
    }

    return out;
}
