/// @file EmlidUtilities.hpp
/// @brief Helper Functions to work with Emlid Sensors
/// @author T. Topp (thomas.topp@nav.uni-stuttgart.de)
/// @date 2020-07-22

#pragma once

#include <cstdint>
#include <vector>
#include <memory>

#include "NodeData/GNSS/EmlidObs.hpp"

namespace NAV::sensors::emlid
{
/// @brief Decrypts the provided Emlid observation
///
/// @param[in, out] obs Emlid Observation to decrypt
/// @param[in, out] currentInsTime Current Ins Time
/// @param[in] peek Specifies if the data should be peeked or read
void decryptEmlidObs(std::shared_ptr<NAV::EmlidObs>& obs, std::optional<NAV::InsTime>& currentInsTime, bool peek = false);

/// @brief Calculates the two UBX checksums for the provided data vector
///
/// @param[in] data Data Vector for which the checksum should be calculated
/// @return The checksums CK_A and CK_B
std::pair<uint8_t, uint8_t> checksumUBX(const std::vector<uint8_t>& data);

} // namespace NAV::sensors::emlid