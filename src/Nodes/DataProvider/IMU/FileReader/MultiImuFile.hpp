/// @file MultiImuFile.hpp
/// @brief File reader for Multi-IMU data log files
/// @author M. Maier (marcel.maier@ins.uni-stuttgart.de)
/// @date 2022-06-24

#pragma once

#include "internal/Node/Node.hpp"
#include "Nodes/DataProvider/Protocol/FileReader.hpp"
#include "NodeData/IMU/ImuPos.hpp"

#include "NodeData/IMU/ImuObs.hpp"

namespace NAV
{
/// File reader for Multi-IMU data log files
class MultiImuFile : public Node, public FileReader
{
  public:
    /// @brief Default constructor
    MultiImuFile();
    /// @brief Destructor
    ~MultiImuFile() override;
    /// @brief Copy constructor
    MultiImuFile(const MultiImuFile&) = delete;
    /// @brief Move constructor
    MultiImuFile(MultiImuFile&&) = delete;
    /// @brief Copy assignment operator
    MultiImuFile& operator=(const MultiImuFile&) = delete;
    /// @brief Move assignment operator
    MultiImuFile& operator=(MultiImuFile&&) = delete;

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

    /// @brief Resets the node. Moves the read cursor to the start
    bool resetNode() override;

    /// Position and rotation information for conversion from platform to body frame
    [[nodiscard]] const ImuPos& imuPosition() const { return _imuPos; }

  protected:
    /// Position and rotation information for conversion from platform to body frame
    ImuPos _imuPos;

  private:
    constexpr static size_t OUTPUT_PORT_INDEX_IMU_OBS = 0; ///< @brief Flow (ImuObs)

    /// @brief Initialize the node
    bool initialize() override;

    /// @brief Deinitialize the node
    void deinitialize() override;

    /// @brief Adds/Deletes Output Pins depending on the variable _nOutputPins
    void updateNumberOfOutputPins();

    /// @brief Function to determine the File Type
    /// @return The File path which was recognized
    NAV::FileReader::FileType determineFileType() override;

    /// @brief Function to read the Header of a file
    void readHeader() override;

    /// @brief Polls data from the file
    /// @param[in] peek Specifies if the data should be peeked (without moving the read cursor) or read
    /// @return The read observation
    [[nodiscard]] std::shared_ptr<const NodeData> pollData(bool peek = false);

    /// Number of connected sensors
    size_t _nSensors = 5;

    /// @brief First 'gpsSecond', s.t. measurements start at time = 0
    double _startTime{};

    /// @brief Container of column names
    std::vector<std::string> _columns{ "sensorId", "gpsSecond", "timeNumerator", "timeDenominator", "accelX", "accelY", "accelZ", "gyroX", "gyroY", "gyroZ" };

    /// @brief Container for individual sensor orientations of a Multi-IMU
    std::vector<ImuPos> _imuPosAll;
};

} // namespace NAV
