/// @file KalmanFilter.hpp
/// @brief Generalized Kalman Filter class
/// @author T. Topp (thomas.topp@nav.uni-stuttgart.de)
/// @date 2020-10-15

#pragma once

namespace NAV
{
class KalmanFilter
{
  public:
    /// @brief Default constructor
    KalmanFilter() = default;
    /// @brief Destructor
    ~KalmanFilter() = default;
    /// @brief Copy constructor
    KalmanFilter(const KalmanFilter&) = delete;
    /// @brief Move constructor
    KalmanFilter(KalmanFilter&&) = delete;
    /// @brief Copy assignment operator
    KalmanFilter& operator=(const KalmanFilter&) = delete;
    /// @brief Move assignment operator
    KalmanFilter& operator=(KalmanFilter&&) = delete;
};

} // namespace NAV
