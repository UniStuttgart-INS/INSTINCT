// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file RinexObsFile.hpp
/// @brief File reader for RINEX Observation messages
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @author Rui Wang (rui.wang@ins.uni-stuttgart.de)
/// @date 2022-04-26

#pragma once

#include <set>
#include <unordered_map>

#include "internal/Node/Node.hpp"
#include "Nodes/DataProvider/Protocol/FileReader.hpp"

#include "Navigation/GNSS/Core/Frequency.hpp"
#include "Navigation/GNSS/Core/Code.hpp"

#include "NodeData/GNSS/GnssObs.hpp"

#include "util/Vendor/RINEX/RINEXUtilities.hpp"

namespace NAV
{
/// File reader Node for RINEX Observation messages
class RinexObsFile : public Node, public FileReader
{
  public:
    /// @brief Default constructor
    RinexObsFile();
    /// @brief Destructor
    ~RinexObsFile() override;
    /// @brief Copy constructor
    RinexObsFile(const RinexObsFile&) = delete;
    /// @brief Move constructor
    RinexObsFile(RinexObsFile&&) = delete;
    /// @brief Copy assignment operator
    RinexObsFile& operator=(const RinexObsFile&) = delete;
    /// @brief Move assignment operator
    RinexObsFile& operator=(RinexObsFile&&) = delete;

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

  private:
    constexpr static size_t OUTPUT_PORT_INDEX_GNSS_OBS = 0; ///< @brief Flow (GnssObs)

    /// @brief Initialize the node
    bool initialize() override;

    /// @brief Deinitialize the node
    void deinitialize() override;

    /// @brief Determines the type of the file
    /// @return The File Type
    [[nodiscard]] FileType determineFileType() override;

    /// @brief Read the Header of the file
    void readHeader() override;

    /// @brief Polls the data from the file
    /// @return The read observation
    [[nodiscard]] std::shared_ptr<const NodeData> pollData();

    /// @brief Supported RINEX versions
    static inline const std::set<double> _supportedVersions = { 3.04, 3.03, 3.02 };

    /// @brief Version of the RINEX file
    double _version = 0.0;

    /// Observation description. [Key]: Satellite System, [Value]: List with descriptions
    std::unordered_map<SatelliteSystem, std::vector<NAV::vendor::RINEX::ObservationDescription>> _obsDescription;

    /// Time system of all observations in the file
    TimeSystem _timeSystem = TimeSys_None;

    /// @brief Receiver clock offset app
    bool _rcvClockOffsAppl = false;

    /// @brief Whether to remove less precise codes (e.g. if G1X (L1C combined) is present, don't use G1L (L1C pilot) and G1S (L1C data))
    bool _eraseLessPreciseCodes = true;

    /// @brief Removes less precise codes (e.g. if G1X (L1C combined) is present, don't use G1L (L1C pilot) and G1S (L1C data))
    /// @param[in] gnssObs GnssObs to search for less precise codes
    /// @param[in] freq Signal frequency (also identifies the satellite system)
    /// @param[in] satNum Number of the satellite
    void eraseLessPreciseCodes(const std::shared_ptr<GnssObs>& gnssObs, const Frequency& freq, uint16_t satNum);
};

} // namespace NAV
