// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file StringUtil.hpp
/// @brief Utility functions for working with std::strings
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2020-09-16

#pragma once

#include <algorithm>
#include <cctype>
#include <locale>
#include <vector>
#include <string>
#include <string_view>

namespace NAV::str
{
/// @brief Trim from start (in place)
/// @param[in, out] s The string to trim
static inline void ltrim(std::string& s)
{
    if (!s.empty() && s[0] == '\n')
    {
        s.erase(0, 1);
    }
    s.erase(s.begin(), std::find_if(s.begin(), s.end(), [](int ch) {
                return !std::isspace(ch);
            }));
}

/// @brief Trim from end (in place)
/// @param[in, out] s The string to trim
static inline void rtrim(std::string& s)
{
    if (!s.empty() && s[s.length() - 1] == '\n')
    {
        s.erase(s.length() - 1);
    }
    s.erase(std::find_if(s.rbegin(), s.rend(), [](int ch) {
                return !std::isspace(ch);
            }).base(),
            s.end());
}

/// @brief Trim from both ends (in place)
/// @param[in, out] s The string to trim
static inline void trim(std::string& s)
{
    ltrim(s);
    rtrim(s);
}

/// @brief Trim from start (in place)
/// @param[in, out] sv The string view to trim
static inline void ltrim(std::string_view& sv)
{
    sv.remove_prefix(std::min(sv.find_first_not_of(' '), sv.size()));
}

/// @brief Trim from end (in place)
/// @param[in, out] sv The string view to trim
static inline void rtrim(std::string_view& sv)
{
    sv.remove_suffix(std::min(sv.size() - sv.find_last_not_of(' ') - 1, sv.size()));
}

/// @brief Trim from both ends (in place)
/// @param[in, out] sv The string view to trim
static inline void trim(std::string_view& sv)
{
    ltrim(sv);
    rtrim(sv);
}

/// @brief Trim from start (copying)
/// @param[in] s The string to trim
/// @return The trimmed string
static inline std::string ltrim_copy(std::string s)
{
    ltrim(s);
    return s;
}

/// @brief Trim from end (copying)
/// @param[in] s The string to trim
/// @return The trimmed string
static inline std::string rtrim_copy(std::string s)
{
    rtrim(s);
    return s;
}

/// @brief Trim from both ends (copying)
/// @param[in] s The string to trim
/// @return The trimmed string
static inline std::string trim_copy(std::string s)
{
    trim(s);
    return s;
}

/// @brief Trim from start (copying)
/// @param[in] sv The string view to trim
/// @return The trimmed string
static inline std::string_view ltrim_copy(std::string_view sv)
{
    ltrim(sv);
    return sv;
}

/// @brief Trim from end (copying)
/// @param[in] sv The string view to trim
/// @return The trimmed string
static inline std::string_view rtrim_copy(std::string_view sv)
{
    rtrim(sv);
    return sv;
}

/// @brief Trim from both ends (copying)
/// @param[in] sv The string view to trim
/// @return The trimmed string
static inline std::string_view trim_copy(std::string_view sv)
{
    trim(sv);
    return sv;
}

/// @brief Enum for case sensitive tasks
enum CaseSensitivity
{
    RespectCase, ///< Respect the case
    IgnoreCase,  ///< Ignore case
};

/// @brief Replaces the first occurrence of a search pattern with another sequence
/// @param[in, out] str String to search in and return value
/// @param[in] from String pattern to search for
/// @param[in] to Replacement string
/// @param[in] cs Case sensitivity
/// @return True if something was replaced
static inline bool replace(std::string& str, const std::string& from, const std::string& to, CaseSensitivity cs = RespectCase)
{
    if (from.empty())
    {
        return false;
    }
    auto it = std::search(str.begin(), str.end(),
                          from.begin(), from.end(),
                          [cs](char ch1, char ch2) { return cs == RespectCase
                                                                ? ch1 == ch2
                                                                : std::toupper(ch1) == std::toupper(ch2); });

    if (it == str.end())
    {
        return false;
    }
    auto start_pos = static_cast<size_t>(it - str.begin());
    str.replace(start_pos, from.length(), to);
    return true;
}

/// @brief Replaces all occurrence of a search pattern with another sequence
/// @param[in, out] str String to search in and return value
/// @param[in] from String pattern to search for
/// @param[in] to Replacement string
/// @param[in] cs Case sensitivity
static inline void replaceAll(std::string& str, const std::string& from, const std::string& to, CaseSensitivity cs = RespectCase)
{
    while (replace(str, from, to, cs)) {}
}

/// @brief Replaces all occurrence of a search pattern with another sequence
/// @param[in, out] str String to search in and return value
/// @param[in] from String pattern to search for
/// @param[in] to Replacement string
/// @param[in] cs Case sensitivity
/// @return The string with the replacements
static inline std::string replaceAll_copy(std::string str, const std::string& from, const std::string& to, CaseSensitivity cs = RespectCase)
{
    replaceAll(str, from, to, cs);
    return str;
}

/// @brief Splits a string into parts at a delimiter
/// @param[in] str String to split
/// @param[in] delimiter Sequence of characters to split at
/// @return List with splitted parts
static inline std::vector<std::string> split(const std::string& str, const std::string& delimiter)
{
    size_t pos_start = 0;
    size_t pos_end = 0;
    size_t delim_len = delimiter.length();
    std::vector<std::string> res;

    while ((pos_end = str.find(delimiter, pos_start)) != std::string::npos)
    {
        if (pos_start != pos_end)
        {
            res.push_back(str.substr(pos_start, pos_end - pos_start));
        }
        pos_start = pos_end + delim_len;
        while (pos_start < str.size() && str.find(delimiter, pos_start) == pos_start)
        {
            pos_start += delim_len;
        }
    }
    if (pos_start != str.size())
    {
        res.push_back(str.substr(pos_start));
    }
    return res;
}

/// @brief Splits a string into parts at a delimiter
/// @param[in] str String to split
/// @param[in] delimiter Character to split at
/// @return List with splitted parts
static inline std::vector<std::string> split(const std::string& str, char delimiter)
{
    return split(str, std::string(1, delimiter));
}

} // namespace NAV::str
