/// @file UbloxUtilities.hpp
/// @brief Helper Functions to work with Ublox Sensors
/// @author T. Topp (thomas.topp@nav.uni-stuttgart.de)
/// @date 2020-07-22

#pragma once

#include <cstdint>
#include <vector>
#include <memory>

#include "NodeData/GNSS/UbloxObs.hpp"

namespace NAV::sensors::ublox
{
/// @brief Decrypts the provided Ublox observation
/// @param[in, out] obs Ublox Observation to decrypt
/// @param[in, out] currentInsTime Current Ins Time
/// @param[in] peek Specifies if the data should be peeked or read
void decryptUbloxObs(std::shared_ptr<NAV::UbloxObs>& obs, std::optional<NAV::InsTime>& currentInsTime, bool peek = false);

/// @brief Calculates the two UBX checksums for the provided data vector
/// @param[in] data Data Vector for which the checksum should be calculated
/// @return The checksums CK_A and CK_B
std::pair<uint8_t, uint8_t> checksumUBX(const std::vector<uint8_t>& data);

/// @brief Calculates the NMEA checksum for the provided data vector
/// @param[in] data Data Vector for which the checksum should be calculated
/// @return The calculated checksum
uint8_t checksumNMEA(const std::vector<uint8_t>& data);

} // namespace NAV::sensors::ublox