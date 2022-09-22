// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file timestamp.hpp
/// @brief Extract from the timestamp implementation
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2022-09-20

#pragma once

#ifndef HAS_UARTSENSOR_LIBRARY

    #include <cstdint>

namespace uart::xplat
{
/// Timestamp class
struct TimeStamp
{
  public:
    /// Default constructor
    TimeStamp() = default;

  private:
    /// @brief Constructor
    /// @param[in] sec Seconds
    /// @param[in] usec Microseconds
    TimeStamp(int64_t sec, uint64_t usec);

  public:
    /// @brief Returns a timestamp.
    /// @return The timestamp.
    static TimeStamp get();

    int64_t _sec{ 0 };   ///< Seconds.
    uint64_t _usec{ 0 }; ///< Microseconds.
};

} // namespace uart::xplat

#endif