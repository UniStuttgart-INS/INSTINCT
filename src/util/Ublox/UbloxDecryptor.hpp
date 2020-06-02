/**
 * @file UbloxDecryptor.hpp
 * @brief Decrypts Ublox Messages
 * @author T. Topp (thomas.topp@nav.uni-stuttgart.de)
 * @date 2020-05-26
 */

#pragma once

#include <memory>
#include "NodeData/GNSS/UbloxObs.hpp"

namespace NAV::ublox
{
/**
 * @brief Decrypts the provided Ublox observation
 * 
 * @param[in, out] obs Ublox Observation to decrypt
 * @param[in, out] currentInsTime Current Ins Time
 * @param[in] peek Specifies if the data should be peeked  or read
 */
void decryptUbloxObs(std::shared_ptr<NAV::UbloxObs>& obs, std::optional<NAV::InsTime>& currentInsTime, bool peek = false);

} // namespace NAV::ublox
