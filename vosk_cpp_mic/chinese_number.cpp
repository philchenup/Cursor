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
        Twenty,
        Thirty,
        Forty,
        Hundred,
        Thousand,
        Wan,
        Yi,
        Zhao,
        Dot,
        SignNeg,
        SignPos,
        Skip,
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

bool is_space_char(const std::string& s, size_t pos)
{
    if (pos >= s.size())
        return false;
    const unsigned char c = static_cast<unsigned char>(s[pos]);
    if (c == ' ' || c == '\t' || c == '\n' || c == '\r')
        return true;
    return starts_with(s, pos, "\u3000");
}

CnToken match_token(const std::string& s, size_t pos)
{
    if (is_space_char(s, pos))
    {
        CnToken t;
        t.kind = CnToken::Skip;
        if (starts_with(s, pos, "\u3000"))
            t.utf8 = "\u3000";
        else
            t.utf8.assign(1, s[pos]);
        return t;
    }

    static const std::pair<const char*, int> kDigits[] = {
        {"零", 0}, {"〇", 0}, {"○", 0}, {"Ｏ", 0}, {"０", 0}, {"0", 0},
        {"一", 1}, {"壹", 1}, {"１", 1}, {"1", 1},
        {"二", 2}, {"贰", 2}, {"貳", 2}, {"兩", 2}, {"两", 2}, {"俩", 2}, {"倆", 2}, {"２", 2}, {"2", 2},
        {"三", 3}, {"叁", 3}, {"參", 3}, {"３", 3}, {"3", 3},
        {"四", 4}, {"肆", 4}, {"４", 4}, {"4", 4},
        {"五", 5}, {"伍", 5}, {"５", 5}, {"5", 5},
        {"六", 6}, {"陆", 6}, {"陸", 6}, {"６", 6}, {"6", 6},
        {"七", 7}, {"柒", 7}, {"７", 7}, {"7", 7},
        {"八", 8}, {"捌", 8}, {"８", 8}, {"8", 8},
        {"九", 9}, {"玖", 9}, {"９", 9}, {"9", 9},
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

    struct Named
    {
        const char* lit;
        CnToken::Kind kind;
    };
    static const Named kNamed[] = {
        {"廿", CnToken::Twenty},
        {"卅", CnToken::Thirty},
        {"卌", CnToken::Forty},
        {"十", CnToken::Ten},
        {"拾", CnToken::Ten},
        {"百", CnToken::Hundred},
        {"佰", CnToken::Hundred},
        {"千", CnToken::Thousand},
        {"仟", CnToken::Thousand},
        {"万", CnToken::Wan},
        {"萬", CnToken::Wan},
        {"亿", CnToken::Yi},
        {"億", CnToken::Yi},
        {"兆", CnToken::Zhao},
        {"点", CnToken::Dot},
        {"點", CnToken::Dot},
        {".", CnToken::Dot},
        {"负", CnToken::SignNeg},
        {"負", CnToken::SignNeg},
        {"-", CnToken::SignNeg},
        {"正", CnToken::SignPos},
        {"+", CnToken::SignPos},
    };

    for (const auto& u : kNamed)
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

bool is_value_token(CnToken::Kind k)
{
    return k == CnToken::Digit || k == CnToken::Ten || k == CnToken::Twenty
        || k == CnToken::Thirty || k == CnToken::Forty || k == CnToken::Hundred
        || k == CnToken::Thousand || k == CnToken::Wan || k == CnToken::Yi
        || k == CnToken::Zhao || k == CnToken::Dot;
}

bool is_num_token(CnToken::Kind k)
{
    return is_value_token(k) || k == CnToken::SignNeg || k == CnToken::SignPos
        || k == CnToken::Skip;
}

bool is_digit_run(const std::vector<CnToken>& toks, size_t begin, size_t end)
{
    bool any = false;
    for (size_t i = begin; i < end; ++i)
    {
        if (toks[i].kind == CnToken::Skip)
            continue;
        if (toks[i].kind != CnToken::Digit)
            return false;
        any = true;
    }
    return any;
}

long long parse_digit_run(const std::vector<CnToken>& toks, size_t begin, size_t end)
{
    long long v = 0;
    for (size_t i = begin; i < end; ++i)
    {
        if (toks[i].kind == CnToken::Skip)
            continue;
        v = v * 10 + toks[i].value;
    }
    return v;
}

long long parse_cn_integer(const std::vector<CnToken>& toks, size_t begin, size_t end)
{
    if (begin >= end)
        return 0;

    if (is_digit_run(toks, begin, end))
        return parse_digit_run(toks, begin, end);

    long long result = 0;
    long long section = 0;
    long long number = -1;

    auto flush_large = [&](long long unit) {
        const long long n = number < 0 ? 0 : number;
        number = -1;
        result += (section + n) * unit;
        section = 0;
    };

    auto apply_small_unit = [&](long long unit) {
        long long n = number;
        number = -1;
        if (n < 0)
            n = 1;
        section += n * unit;
    };

    for (size_t i = begin; i < end; ++i)
    {
        const auto kind = toks[i].kind;
        if (kind == CnToken::Skip)
            continue;

        if (kind == CnToken::Digit)
        {
            number = toks[i].value;
            continue;
        }
        if (kind == CnToken::Twenty)
        {
            section += 20;
            number = -1;
            continue;
        }
        if (kind == CnToken::Thirty)
        {
            section += 30;
            number = -1;
            continue;
        }
        if (kind == CnToken::Forty)
        {
            section += 40;
            number = -1;
            continue;
        }

        if (kind == CnToken::Ten)
            apply_small_unit(10);
        else if (kind == CnToken::Hundred)
            apply_small_unit(100);
        else if (kind == CnToken::Thousand)
            apply_small_unit(1000);
        else if (kind == CnToken::Wan)
            flush_large(10000LL);
        else if (kind == CnToken::Yi)
            flush_large(100000000LL);
        else if (kind == CnToken::Zhao)
            flush_large(1000000000000LL);
    }

    result += section + (number < 0 ? 0 : number);
    return result;
}

std::string tokens_to_arabic(const std::vector<CnToken>& toks)
{
    size_t i = 0;
    while (i < toks.size() && toks[i].kind == CnToken::Skip)
        ++i;

    std::string sign;
    if (i < toks.size() && toks[i].kind == CnToken::SignNeg)
    {
        sign = "-";
        ++i;
    }
    else if (i < toks.size() && toks[i].kind == CnToken::SignPos)
    {
        ++i;
    }

    while (i < toks.size() && toks[i].kind == CnToken::Skip)
        ++i;

    if (i >= toks.size())
        return sign + "0";

    size_t dot = toks.size();
    for (size_t k = i; k < toks.size(); ++k)
    {
        if (toks[k].kind == CnToken::Dot)
        {
            dot = k;
            break;
        }
    }

    std::string out = sign;
    if (dot == toks.size())
    {
        out += std::to_string(parse_cn_integer(toks, i, toks.size()));
        return out;
    }

    const long long ip = (dot == i) ? 0 : parse_cn_integer(toks, i, dot);
    out += std::to_string(ip);

    std::string frac;
    for (size_t k = dot + 1; k < toks.size(); ++k)
    {
        if (toks[k].kind == CnToken::Skip)
            continue;
        if (toks[k].kind != CnToken::Digit)
            break;
        frac.push_back(static_cast<char>('0' + toks[k].value));
    }
    if (!frac.empty())
    {
        out.push_back('.');
        out += frac;
    }
    return out;
}

bool has_non_ascii_numeral(const std::vector<CnToken>& toks)
{
    for (const auto& t : toks)
    {
        if (t.kind == CnToken::Skip)
            continue;
        for (unsigned char c : t.utf8)
        {
            if (c >= 0x80)
                return true;
        }
    }
    return false;
}

bool is_cjk_or_digit_byte_start(unsigned char c)
{
    // ASCII digit, or likely start of a multi-byte CJK / fullwidth codepoint.
    return (c >= '0' && c <= '9') || c >= 0x80;
}

// ASR often inserts spaces between Chinese characters: "十 二 毫 米" -> "12毫米".
std::string collapse_spaces_in_cjk(const std::string& s)
{
    std::string out;
    out.reserve(s.size());
    for (size_t i = 0; i < s.size();)
    {
        if (is_space_char(s, i))
        {
            const size_t n = starts_with(s, i, "\u3000") ? 3 : 1;
            // Drop space if both neighbors are digit/CJK.
            const size_t next = i + n;
            if (!out.empty() && next < s.size())
            {
                size_t last_start = out.size() - 1;
                while (last_start > 0
                       && (static_cast<unsigned char>(out[last_start]) & 0xC0) == 0x80)
                {
                    --last_start;
                }
                if (is_cjk_or_digit_byte_start(static_cast<unsigned char>(out[last_start]))
                    && is_cjk_or_digit_byte_start(static_cast<unsigned char>(s[next])))
                {
                    i += n;
                    continue;
                }
            }
            out.append(s, i, n);
            i += n;
            continue;
        }

        const size_t n = utf8_char_len(static_cast<unsigned char>(s[i]));
        out.append(s, i, n);
        i += n;
    }
    return out;
}

} // namespace

std::string normalize_chinese_numbers(const std::string& text)
{
    if (text.empty())
        return text;

    std::vector<CnToken> all;
    all.reserve(text.size());
    for (size_t p = 0; p < text.size();)
    {
        CnToken t = match_token(text, p);
        p += t.utf8.size();
        all.push_back(std::move(t));
    }

    std::string out;
    out.reserve(text.size());

    size_t i = 0;
    while (i < all.size())
    {
        if (all[i].kind == CnToken::Other || all[i].kind == CnToken::Skip)
        {
            out += all[i].utf8;
            ++i;
            continue;
        }

        if (all[i].kind == CnToken::SignNeg || all[i].kind == CnToken::SignPos)
        {
            size_t k = i + 1;
            while (k < all.size() && all[k].kind == CnToken::Skip)
                ++k;
            if (k >= all.size() || !is_value_token(all[k].kind))
            {
                out += all[i].utf8;
                ++i;
                continue;
            }
        }
        else if (!is_value_token(all[i].kind))
        {
            out += all[i].utf8;
            ++i;
            continue;
        }

        const size_t start = i;
        size_t j = i;
        bool saw_dot = false;
        bool saw_value = false;
        bool saw_sign = false;

        while (j < all.size() && is_num_token(all[j].kind))
        {
            const auto kind = all[j].kind;

            if (kind == CnToken::Skip)
            {
                size_t k = j + 1;
                while (k < all.size() && all[k].kind == CnToken::Skip)
                    ++k;
                if (k >= all.size())
                    break;
                // Stop before a new signed number or non-number.
                if (all[k].kind == CnToken::SignNeg || all[k].kind == CnToken::SignPos
                    || !is_num_token(all[k].kind))
                {
                    break;
                }
                ++j;
                continue;
            }

            if (kind == CnToken::SignNeg || kind == CnToken::SignPos)
            {
                if (saw_sign || saw_value || saw_dot)
                    break;
                saw_sign = true;
                ++j;
                continue;
            }

            if (kind == CnToken::Dot)
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
            out += all[start].utf8;
            i = start + 1;
            continue;
        }

        // Trim trailing skips from segment end (they are emitted as normal spaces later).
        size_t end = j;
        while (end > start && all[end - 1].kind == CnToken::Skip)
            --end;

        std::vector<CnToken> segment(all.begin() + static_cast<std::ptrdiff_t>(start),
                                     all.begin() + static_cast<std::ptrdiff_t>(end));

        if (!has_non_ascii_numeral(segment))
        {
            for (size_t k = start; k < end; ++k)
                out += all[k].utf8;
        }
        else
        {
            out += tokens_to_arabic(segment);
        }

        i = end;
    }

    return collapse_spaces_in_cjk(out);
}
