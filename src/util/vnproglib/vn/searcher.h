// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file searcher.h
/// @brief Extract from the vnproglib
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2022-09-20

#pragma once

#ifndef HAS_VECTORNAV_LIBRARY

    #include <string>
    #include <vector>
    #include <cstdint>

// NOLINTBEGIN

namespace vn
{
namespace sensors
{

/// \brief Helpful class for finding VectorNav sensors.
class Searcher
{
  public:
    /// \brief Searches the serial port at all valid baudrates for a VectorNav
    ///     sensor.
    ///
    /// \param[in] portlist The serial ports to search.
    static void findPorts(std::vector<std::string>& portlist);

    /// \brief Searches the serial port at all valid baudrates for a VectorNav
    ///     sensor.
    ///
    /// \param[in] portName The serial port to search.
    /// \param[out] foundBaudrate If a sensor is found, this will be set to the
    ///     baudrate the sensor is communicating at.
    /// \returns <c>true</c> if a sensor if found; otherwise <c>false</c>.
    static bool search(const std::string& portName, int32_t* foundBaudrate);

    /// \brief Checks all available serial ports on the system for any
    ///     VectorNav sensors.
    ///
    /// \return Collection of serial ports and baudrates for all found sensors.
    static std::vector<std::pair<std::string, uint32_t>> search(void);

    /// \brief Checks the provided list of serial ports for any connected
    ///     VectorNav sensors.
    ///
    /// \param[in] portsToCheck List of serial ports to check for sensors.
    /// \return Collection of serial ports and baudrates for all found sensors.
    static std::vector<std::pair<std::string, uint32_t>> search(std::vector<std::string>& portsToCheck);

    /// \brief Tests if a sensor is connected to the serial port at the
    ///     specified baudrate.
    ///
    /// \param[in] portName The serial port to test.
    /// \param[in] baudrate The baudrate to test at.
    /// \returns <c>true</c> if a sensor if found; otherwise <c>false</c>.
    static bool test(std::string portName, uint32_t baudrate);
};

} // namespace sensors
} // namespace vn

// NOLINTEND

#endif