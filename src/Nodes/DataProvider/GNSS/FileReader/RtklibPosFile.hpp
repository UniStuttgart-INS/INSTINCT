/// @file RtklibPosFile.hpp
/// @brief File Reader for RTKLIB Pos files
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2020-06-02

#pragma once

#include "internal/Node/Node.hpp"
#include "Nodes/DataProvider/Protocol/FileReader.hpp"

namespace NAV
{
/// File Reader for RTKLIB pos log files
class RtklibPosFile : public Node, public FileReader
{
  public:
    /// @brief Default constructor
    RtklibPosFile();
    /// @brief Destructor
    ~RtklibPosFile() override;
    /// @brief Copy constructor
    RtklibPosFile(const RtklibPosFile&) = delete;
    /// @brief Move constructor
    RtklibPosFile(RtklibPosFile&&) = delete;
    /// @brief Copy assignment operator
    RtklibPosFile& operator=(const RtklibPosFile&) = delete;
    /// @brief Move assignment operator
    RtklibPosFile& operator=(RtklibPosFile&&) = delete;

    /// @brief String representation of the Class Type
    [[nodiscard]] static std::string typeStatic();

    /// @brief String representation of the Class Type
    [[nodiscard]] std::string type() const override;

    /// @brief String representation of the Class Category
    [[nodiscard]] static std::string category();

    /// @brief ImGui config window which is shown on double click
    /// @attention Don't forget to set hasConfig to true in the constructor of the node
    void guiConfig() override;

    /// @brief Saves the node into a json object
    [[nodiscard]] json save() const override;

    /// @brief Restores the node from a json object
    /// @param[in] j Json object with the node state
    void restore(const json& j) override;

    /// @brief Resets the node. Moves the read cursor to the start
    bool resetNode() override;

  private:
    constexpr static size_t OutputPortIndex_RtklibPosObs = 0; ///< @brief Flow (RtklibPosObs)

    /// @brief Initialize the node
    bool initialize() override;

    /// @brief Deinitialize the node
    void deinitialize() override;

    /// @brief Polls data from the file
    /// @param[in] peek Specifies if the data should be peeked (without moving the read cursor) or read
    /// @return The read observation
    [[nodiscard]] std::shared_ptr<NodeData> pollData(bool peek = false);

    /// @brief Determines the type of the file
    /// @return The File Type
    [[nodiscard]] FileType determineFileType() override;

    /// @brief Read the Header of the file
    void readHeader() override;
};

} // namespace NAV
