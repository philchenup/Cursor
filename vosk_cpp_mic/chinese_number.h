#pragma once

#include <string>

// Convert arbitrary Chinese numerals in UTF-8 text to Arabic digits.
// Keeps surrounding non-number text (units, words) unchanged.
//
// Examples:
//   "十二毫米"                 -> "12毫米"
//   "负二十三点五度"           -> "-23.5度"
//   "一亿零三百零四万"         -> "103040000"
//   "十 二 毫 米"              -> "12毫米"   (ASR spaces ignored inside numbers)
//   "二零二三年度"             -> "2023年度"
std::string normalize_chinese_numbers(const std::string& text);
