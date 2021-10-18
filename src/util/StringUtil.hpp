/// @file StringUtil.hpp
/// @brief Utility functions for working with std::strings
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2020-09-16

#pragma once

#include <algorithm>
#include <cctype>
#include <locale>

namespace NAV::str
{
/// @brief Trim from start (in place)
/// @param[in, out] s The string to trim
static inline void ltrim(std::string& s)
{
    s.erase(s.begin(), std::find_if(s.begin(), s.end(), [](int ch) {
                return !std::isspace(ch);
            }));
}

/// @brief Trim from end (in place)
/// @param[in, out] s The string to trim
static inline void rtrim(std::string& s)
{
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

/// @brief Replaces the first occurrence of a search pattern with another sequence
/// @param[in, out] str String to search in and return value
/// @param[in] from String pattern to search for
/// @param[in] to Replacement string
/// @return True if something was replaced
static inline bool replace(std::string& str, const std::string& from, const std::string& to)
{
    if (from.empty())
    {
        return false;
    }
    size_t start_pos = str.find(from);
    if (start_pos == std::string::npos)
    {
        return false;
    }
    str.replace(start_pos, from.length(), to);
    return true;
}

/// @brief Replaces all occurrence of a search pattern with another sequence
/// @param[in, out] str String to search in and return value
/// @param[in] from String pattern to search for
/// @param[in] to Replacement string
static inline void replaceAll(std::string& str, const std::string& from, const std::string& to)
{
    while (replace(str, from, to))
        ;
}

/// @brief Replaces all occurrence of a search pattern with another sequence
/// @param[in, out] str String to search in and return value
/// @param[in] from String pattern to search for
/// @param[in] to Replacement string
/// @return The string with the replacements
static inline std::string replaceAll_copy(std::string str, const std::string& from, const std::string& to)
{
    replaceAll(str, from, to);
    return str;
}
} // namespace NAV::str
