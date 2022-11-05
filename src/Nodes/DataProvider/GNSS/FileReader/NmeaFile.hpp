// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file NmeaFile.hpp
/// @brief File Reader for NMEA files
/// @author T. Hobiger (thomas.hobiger@ins.uni-stuttgart.de)
/// @date 2022-11-03

#pragma once

#include "internal/Node/Node.hpp"
#include "Nodes/DataProvider/Protocol/FileReader.hpp"

namespace NAV
{
/// File Reader for NMEA log files
class NmeaFile : public Node, public FileReader
{
  public:
    /// @brief Default constructor
    NmeaFile();
    /// @brief Destructor
    ~NmeaFile() override;
    /// @brief Copy constructor
    NmeaFile(const NmeaFile&) = delete;
    /// @brief Move constructor
    NmeaFile(NmeaFile&&) = delete;
    /// @brief Copy assignment operator
    NmeaFile& operator=(const NmeaFile&) = delete;
    /// @brief Move assignment operator
    NmeaFile& operator=(NmeaFile&&) = delete;

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
    constexpr static size_t OUTPUT_PORT_INDEX_NMEA_POS_OBS = 0; ///< @brief Flow (RtklibPosObs)

    /// @brief Initialize the node
    bool initialize() override;

    /// @brief Deinitialize the node
    void deinitialize() override;

    /// @brief Polls data from the file
    /// @param[in] peek Specifies if the data should be peeked (without moving the read cursor) or read
    /// @return The read observation
    [[nodiscard]] std::shared_ptr<const NodeData> pollData(bool peek = false);

    /// @brief Determines the type of the file
    /// @return The File Type
    [[nodiscard]] FileType determineFileType() override;

    /// @brief Read the Header of the file
    void readHeader() override;
	
	 /// @brief checks whether a ZDA time tag was read so that UTC can be reconstructed together with the GGA tag
	bool haveValidDate = false ;
	
	 /// @brief second of day (SOD) from last GGA stream. This variable is used to check if SOD is increasing, if not, wait for next ZDA stream to get date info
	double oldSOD = -1.0;
	
	/// @brief checks whether a ZDA time tag was read so that UTC can be reconstructed together with the GGA tag
	std::vector<int> ddmmyyyy={0,0,0};
	
    /// @brief Set date info from ZDA steam
    /// @param[in] line Line that contains a potential $--ZDA stream
	void setdatefromzda(const std::string & line);
	
	
};

} // namespace NAV
