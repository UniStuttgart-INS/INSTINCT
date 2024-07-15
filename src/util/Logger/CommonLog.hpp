// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file CommonLog.hpp
/// @brief Common logging variables like time into run and local positions
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2022-06-23

#pragma once

#include <mutex>
#include <vector>

#include "util/Eigen.hpp"
#include "Navigation/Time/InsTime.hpp"

namespace NAV
{
/// Common logging variables like time into run and local positions
class CommonLog
{
  public:
    /// @brief Destructor
    virtual ~CommonLog();
    /// @brief Copy constructor
    CommonLog(const CommonLog&) = delete;
    /// @brief Move constructor
    CommonLog(CommonLog&&) = delete;
    /// @brief Copy assignment operator
    CommonLog& operator=(const CommonLog&) = delete;
    /// @brief Move assignment operator
    CommonLog& operator=(CommonLog&&) = delete;

    /// @brief Initialize the common log variables
    void initialize() const;

    /// @brief Calculates the relative time into he run
    /// @param[in] insTime Absolute Time
    /// @return Relative
    static double calcTimeIntoRun(const InsTime& insTime);

    /// Local position offset from a reference point
    struct LocalPosition
    {
        double northSouth = 0.0; ///< North/South deviation from the reference point [m]
        double eastWest = 0.0;   ///< East/West deviation from the reference point [m]
    };

    /// @brief Calculate the local position offset from a reference point
    /// @param[in] lla_position [𝜙, λ, h] Latitude, Longitude, Altitude in [rad, rad, m]
    /// @return Local positions in north/south and east/west directions in [m]
    static LocalPosition calcLocalPosition(const Eigen::Vector3d& lla_position);

  protected:
    /// @brief Default constructor
    CommonLog();

    /// Start Time for calculation of relative time
    static inline InsTime _startTime;
    /// Start Latitude [rad] for calculation of relative North-South
    static inline double _originLatitude = std::nan("");
    /// Start Longitude [rad] for calculation of relative East-West
    static inline double _originLongitude = std::nan("");

  private:
    /// Mutex to lock before writing
    static inline std::mutex _mutex;
    /// Vector on which nodes want to initialize
    static inline std::vector<bool> _wantsInit;
    /// Index which common log node this is
    size_t _index = 0;
};

} // namespace NAV
