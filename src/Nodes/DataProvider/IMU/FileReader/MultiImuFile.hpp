// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file MultiImuFile.hpp
/// @brief File reader for Multi-IMU data log files
/// @author M. Maier (marcel.maier@ins.uni-stuttgart.de)
/// @date 2022-06-24

#pragma once

#include "internal/Node/Node.hpp"
#include "Nodes/DataProvider/Protocol/FileReader.hpp"
#include "NodeData/IMU/ImuPos.hpp"
#include "internal/gui/widgets/TimeEdit.hpp"

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
    /// @param[in] pinIdx Index of the pin the data is requested on
    /// @param[in] peek Specifies if the data should be peeked (without moving the read cursor) or read
    /// @return The read observation
    [[nodiscard]] std::shared_ptr<const NodeData> pollData(size_t pinIdx, bool peek);

    /// Number of connected sensors
    size_t _nSensors = 5;

    /// @brief Read messages.
    /// Vector idx: Sensor Id,
    /// - Map Key: InsTime
    /// - Map Value: IMU Observation
    std::vector<std::map<InsTime, std::shared_ptr<ImuObs>>> _messages;

    /// @brief Counter for messages
    std::vector<size_t> _messageCnt;

    /// @brief Delimiter: ',' for GPZDA and ' ' for GPGGA messages (NMEA)
    char _delim = ',';

    /// @brief First 'gpsSecond', s.t. measurements start at time = 0
    double _startupGpsSecond{};

    /// Time Format to input the start time with
    gui::widgets::TimeEditFormat _startTimeEditFormat = gui::widgets::TimeEditFormat::GPSWeekToW;

    /// @brief Absolute start time
    InsTime _startTime{ 2000, 1, 1, 0, 0, 0 };

    /// @brief Container of column names
    std::vector<std::string> _columns{ "sensorId", "gpsSecond", "timeNumerator", "timeDenominator", "accelX", "accelY", "accelZ", "gyroX", "gyroY", "gyroZ" };

    /// @brief Container of header column names
    std::vector<std::string> _headerColumns{ "nmeaMsgType", "UTC_HMS", "day", "month", "year" };

    /// @brief Container for individual sensor orientations of a Multi-IMU
    std::vector<ImuPos> _imuPosAll;

    /// @brief Previous observation (for timestamp)
    InsTime _lastFiltObs{};

    /// Types of NMEA messages available
    enum class NmeaType
    {
        GPGGA, ///< NMEA message type
        GPZDA, ///< NMEA message type
        COUNT, ///< Number of items in the enum
    };
    /// @brief Converts the enum to a string
    /// @param[in] value Enum value to convert into text
    /// @return String representation of the enum
    friend constexpr const char* to_string(NmeaType value);

    /// Selected NMEA type in the GUI
    NmeaType _nmeaType = NmeaType::GPZDA;
};

/// @brief Converts the enum to a string
/// @param[in] value Enum value to convert into text
/// @return String representation of the enum
constexpr const char* to_string(NAV::MultiImuFile::NmeaType value)
{
    switch (value)
    {
    case NAV::MultiImuFile::NmeaType::GPGGA:
        return "GPGGA";
    case NAV::MultiImuFile::NmeaType::GPZDA:
        return "GPZDA";
    case NAV::MultiImuFile::NmeaType::COUNT:
        return "";
    }
    return "";
}

} // namespace NAV
