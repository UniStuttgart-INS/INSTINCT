/// @file ImuFile.hpp
/// @brief File Reader for Imu log files
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2020-03-16

#pragma once

#include "Nodes/DataProvider/IMU/Imu.hpp"
#include "Nodes/DataProvider/Protocol/FileReader.hpp"

namespace NAV
{
/// File Reader for Imu log files
class ImuFile : public Imu, public FileReader
{
  public:
    /// @brief Default constructor
    ImuFile();
    /// @brief Destructor
    ~ImuFile() override;
    /// @brief Copy constructor
    ImuFile(const ImuFile&) = delete;
    /// @brief Move constructor
    ImuFile(ImuFile&&) = delete;
    /// @brief Copy assignment operator
    ImuFile& operator=(const ImuFile&) = delete;
    /// @brief Move assignment operator
    ImuFile& operator=(ImuFile&&) = delete;

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

    /// @brief Initialize the node
    bool initialize() override;

    /// @brief Deinitialize the node
    void deinitialize() override;

    /// @brief Resets the node. Moves the read cursor to the start
    void resetNode() override;

  private:
    constexpr static size_t OutputPortIndex_ImuFile = 0;       ///< @brief Delegate
    constexpr static size_t OutputPortIndex_ImuObs = 1;        ///< @brief Flow (ImuObs)
    constexpr static size_t OutputPortIndex_HeaderColumns = 2; ///< @brief Object (std::vector<std::string>)

    /// @brief Polls data from the file
    /// @param[in] peek Specifies if the data should be peeked (without moving the read cursor) or read
    /// @return The read observation
    [[nodiscard]] std::shared_ptr<NodeData> pollData(bool peek = false);
};

} // namespace NAV
