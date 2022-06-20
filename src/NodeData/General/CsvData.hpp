/// @file CsvData.hpp
/// @brief CSV Data container
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2022-06-18

#pragma once

#include <vector>
#include <string>
#include <variant>

namespace NAV
{
/// CSV Data container
class CsvData
{
  public:
    /// @brief Returns the type of the data class
    /// @return The data type
    [[nodiscard]] static std::string type() { return "CsvData"; }

    /// CSV Elements (number or if not convertible to number as std::string)
    using CsvElement = std::variant<double, std::string>;

    /// CSV Line with splitted entries
    using CsvLine = std::vector<CsvElement>;

    /// Data description
    std::vector<std::string> description;
    /// Data container
    std::vector<CsvLine> lines;
};

} // namespace NAV
