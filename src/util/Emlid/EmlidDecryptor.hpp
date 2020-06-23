/**
 * @file EmlidDecryptor.hpp
 * @brief Decrypts Emlid Messages
 * @author T. Topp (thomas.topp@nav.uni-stuttgart.de)
 * @date 2020-05-26
 */

#pragma once

#include <memory>
#include "NodeData/GNSS/EmlidObs.hpp"

namespace NAV::Emlid
{
/**
 * @brief Decrypts the provided Emlid observation
 * 
 * @param[in, out] obs Emlid Observation to decrypt
 * @param[in, out] currentInsTime Current Ins Time
 * @param[in] peek Specifies if the data should be peeked  or read
 */
void decryptEmlidObs(std::shared_ptr<NAV::EmlidObs>& obs, std::optional<NAV::InsTime>& currentInsTime, bool peek = false);

} // namespace NAV::Emlid
