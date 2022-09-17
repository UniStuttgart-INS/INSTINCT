/// @file CsvFile.hpp
/// @brief CSV File reader
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2022-06-18

#pragma once

#include "internal/Node/Node.hpp"
#include "Nodes/DataProvider/Protocol/FileReader.hpp"
#include "NodeData/General/CsvData.hpp"

namespace NAV
{
/// CSV File reader
class CsvFile : public Node, FileReader
{
  public:
    /// @brief Default constructor
    CsvFile();
    /// @brief Destructor
    ~CsvFile() override;
    /// @brief Copy constructor
    CsvFile(const CsvFile&) = delete;
    /// @brief Move constructor
    CsvFile(CsvFile&&) = delete;
    /// @brief Copy assignment operator
    CsvFile& operator=(const CsvFile&) = delete;
    /// @brief Move assignment operator
    CsvFile& operator=(CsvFile&&) = delete;

    /// @brief String representation of the Class Type
    [[nodiscard]] static std::string typeStatic();

    /// @brief String representation of the Class Type
    [[nodiscard]] std::string type() const override;

    /// @brief String representation of the Class Category
    [[nodiscard]] static std::string category();

    /// @brief ImGui config window which is shown on double click
    /// @attention Don't forget to set _hasConfig to true in the constructor of the node
    void guiConfig() override;

    /// @brief Saves the node into a json object
    [[nodiscard]] json save() const override;

    /// @brief Restores the node from a json object
    /// @param[in] j Json object with the node state
    void restore(const json& j) override;

  private:
    /// @brief Initialize the node
    bool initialize() override;

    /// @brief Deinitialize the node
    void deinitialize() override;

    /// @brief Resets the node. It is guaranteed that the node is initialized when this is called.
    bool resetNode() override;

    /// @brief Determines the type of the file
    /// @return The File Type
    [[nodiscard]] FileType determineFileType() override;

    /// @brief Read the Header of the file
    void readHeader() override;

    /// Data container
    CsvData _data;

    /// Delimiter character
    char _delimiter = ',';

    /// Comment character
    char _comment = '#';

    /// Amount of lines to skip at the start
    int _skipLines = 0;

    /// Flag whether there is a header line at the start
    bool _hasHeaderLine = true;
};

} // namespace NAV
