/// @file VectorNavFile.hpp
/// @brief File Reader for Vector Nav log files
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2020-03-16

#pragma once

#include "Nodes/DataProvider/IMU/Imu.hpp"
#include "Nodes/DataProvider/Protocol/FileReader.hpp"

#include "vn/sensors.h"

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
    void guiConfig() override;

    /// @brief Saves the node into a json object
    [[nodiscard]] json save() const override;

    /// @brief Restores the node from a json object
    /// @param[in] j Json object with the node state
    void restore(const json& j) override;

    /// @brief Resets the node. Moves the read cursor to the start
    bool resetNode() override;

  private:
    constexpr static size_t OutputPortIndex_VectorNavBinaryOutput = 0; ///< @brief Flow (VectorNavBinaryOutput)

    /// @brief Initialize the node
    bool initialize() override;

    /// @brief Deinitialize the node
    void deinitialize() override;

    /// @brief Virtual Function to determine the File Type
    /// @return The File path which was recognized
    [[nodiscard]] FileType determineFileType() override;

    /// @brief Read the Header of the file
    void readHeader() override;

    /// @brief Binary Output Register 1 - 3.
    ///
    /// This register allows the user to construct a custom binary output message that
    /// contains a collection of desired estimated states and sensor measurements.
    /// @note See User manual VN-310 - 8.2.11-13 (p 100ff) / VN-100 - 5.2.11-13 (p 73ff)
    vn::sensors::BinaryOutputRegister binaryOutputRegister;

    /// @brief Polls data from the file
    /// @param[in] peek Specifies if the data should be peeked (without moving the read cursor) or read
    /// @return The read observation
    [[nodiscard]] std::shared_ptr<NodeData> pollData(bool peek = false);

    /// @brief Amount of messages read
    uint32_t messageCount = 0;
};

} // namespace NAV
