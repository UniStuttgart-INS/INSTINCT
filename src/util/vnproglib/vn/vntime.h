// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file vntime.h
/// @brief Extract from the vnproglib
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2022-09-20

#pragma once

#ifndef HAS_VECTORNAV_LIBRARY

    #include <cstdint>

// NOLINTBEGIN

namespace vn
{
namespace xplat
{

/// Timestamp class
struct TimeStamp
{
  public:
    /// Default constructor
    TimeStamp();

  private:
    /// @brief Constructor
    /// @param[in] sec Seconds
    /// @param[in] usec Microseconds
    TimeStamp(int64_t sec, uint64_t usec);

  public:
    // \brief Returns a timestamp.
    // \return The timestamp.
    static TimeStamp get();

    // HACK: Current values are made public until the TimeStamp interface
    // is fully worked out.
    // private:
  public:
    int64_t _sec;   ///< Seconds.
    uint64_t _usec; ///< Microseconds.
};

} // namespace xplat
} // namespace vn

// NOLINTEND

#endif