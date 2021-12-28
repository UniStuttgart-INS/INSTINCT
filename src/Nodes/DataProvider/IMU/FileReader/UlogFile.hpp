/// @file UlogFile.hpp
/// @brief File Reader for ULog files
/// @author M. Maier (marcel.maier@ins.uni-stuttgart.de)
/// @date 2021-12-28

#pragma once

#include "Nodes/DataProvider/IMU/Imu.hpp"
#include "Nodes/DataProvider/Protocol/FileReader.hpp"

namespace NAV
{
class UlogFile : public Imu, public FileReader
{
  public:
    /// @brief Default constructor
    UlogFile();
    /// @brief Destructor
    ~UlogFile() override;
    /// @brief Copy constructor
    UlogFile(const UlogFile&) = delete;
    /// @brief Move constructor
    UlogFile(UlogFile&&) = delete;
    /// @brief Copy assignment operator
    UlogFile& operator=(const UlogFile&) = delete;
    /// @brief Move assignment operator
    UlogFile& operator=(UlogFile&&) = delete;

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
    constexpr static size_t OutputPortIndex_UlogOutput = 0; ///< @brief Flow (UlogOutput)

    /// @brief Initialize the node
    bool initialize() override;

    /// @brief Deinitialize the node
    void deinitialize() override;

    /// @brief Read the Header of the file
    void readHeader() override;

    /// @brief Polls data from the file
    /// @param[in] peek Specifies if the data should be peeked (without moving the read cursor) or read
    /// @return The read observation
    [[nodiscard]] std::shared_ptr<const NodeData> pollData([[maybe_unused]] bool peek = false); //TODO: remove [[maybe_unused]] when enabling the callbacks

    /// @brief Number of messages read
    uint32_t messageCount = 0;
};
} // namespace NAV