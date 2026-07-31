#pragma once

#include <string>

// Convert Chinese numerals in UTF-8 text to Arabic digits.
// Examples:
//   "十二毫米"     -> "12毫米"
//   "二十三点五度" -> "23.5度"
//   "一百零八"     -> "108"
std::string normalize_chinese_numbers(const std::string& text);
