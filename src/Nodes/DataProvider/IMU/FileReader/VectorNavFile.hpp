/// @file VectorNavFile.hpp
/// @brief File Reader for Vector Nav log files
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2020-12-27

#pragma once

#include "Nodes/DataProvider/IMU/Imu.hpp"
#include "Nodes/DataProvider/Protocol/FileReader.hpp"

namespace NAV
{
/// File Reader for Vector Nav log files
class VectorNavFile : public Imu, public FileReader
{
  public:
    /// @brief Default constructor
    VectorNavFile();
    /// @brief Destructor
    ~VectorNavFile() override;
    /// @brief Copy constructor
    VectorNavFile(const VectorNavFile&) = delete;
    /// @brief Move constructor
    VectorNavFile(VectorNavFile&&) = delete;
    /// @brief Copy assignment operator
    VectorNavFile& operator=(const VectorNavFile&) = delete;
    /// @brief Move assignment operator
    VectorNavFile& operator=(VectorNavFile&&) = delete;

    /// @brief String representation of the Class Type
    [[nodiscard]] static std::string typeStatic();

    /// @brief String representation of the Class Type
    [[nodiscard]] std::string type() const override;

    /// @brief String representation of the Class Category
    [[nodiscard]] static std::string category();

    /// @brief ImGui config window which is shown on double click
    /// @attention Don't forget to set hasConfig to true in the constructor of the node
    void config() override;

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
    constexpr static size_t OutputPortIndex_VectorNavFile = 0; ///< @brief Delegate
    constexpr static size_t OutputPortIndex_VectorNavObs = 1;  ///< @brief Flow (VectorNavObs)
    constexpr static size_t OutputPortIndex_HeaderColumns = 2; ///< @brief Object (std::vector<std::string>)

    /// @brief Polls data from the file
    /// @param[in] peek Specifies if the data should be peeked (without moving the read cursor) or read
    /// @return The read observation
    [[nodiscard]] std::shared_ptr<NodeData> pollData(bool peek = false);

    std::vector<std::string> readHeaderCallback(bool okay);
};

} // namespace NAV
