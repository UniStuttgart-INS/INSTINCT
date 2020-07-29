/// @file KvhUtilities.hpp
/// @brief Helper Functions to work with Kvh Sensors
/// @author T. Topp (thomas.topp@nav.uni-stuttgart.de)
/// @date 2020-07-28

#pragma once

#include <cstdint>
#include <vector>

namespace NAV::sensors::kvh
{
uint32_t ui32CalcImuCRC(const std::vector<uint8_t>& rawData);

} // namespace NAV::sensors::kvh
