/// @file CommonLog.hpp
/// @brief Common logging variables like time into run and local positions
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2022-06-23

#pragma once

#include <Eigen/Core>
#include "Navigation/Time/InsTime.hpp"
#include "Navigation/Ellipsoid/Ellipsoid.hpp"

namespace NAV
{
/// Common logging variables like time into run and local positions
class CommonLog
{
  public:
    /// @brief Initialize the common log variables
    void initialize();

    /// @brief Calculates the relative time into he run
    /// @param[in] insTime Absolute Time
    /// @return Relative
    double calcTimeIntoRun(const InsTime& insTime);

    /// Local position offset from a reference point
    struct LocalPosition
    {
        double northSouth = std::nan(""); ///< North/South deviation from the reference point [m]
        double eastWest = std::nan("");   ///< East/West deviation from the reference point [m]
    };

    /// @brief Calculate the local position offset from a reference point
    /// @param[in] lla_position [𝜙, λ, h] Latitude, Longitude, Altitude in [rad, rad, m]
    /// @return Local positions in north/south and east/west directions in [m]
    LocalPosition calcLocalPosition(const Eigen::Vector3d& lla_position);

  protected:
    /// @brief Default constructor
    CommonLog() = default;

    /// Start Time for calculation of relative time
    static inline InsTime _startTime;
    /// Start Latitude [rad] for calculation of relative North-South
    static inline double _originLatitude = std::nan("");
    /// Start Longitude [rad] for calculation of relative East-West
    static inline double _originLongitude = std::nan("");

  private:
    /// Counter of how many nodes already depend on the values
    static inline size_t _referenceCounter = 0;
    /// Set to true if the node increased the reference counter
    bool _locked = false;
};

} // namespace NAV
