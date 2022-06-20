/// @file ZenithDelay.hpp
/// @brief Zenith hydrostatic and wet delay
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2022-05-26

#pragma once

namespace NAV
{
/// Zenith delays
struct ZenithDelay
{
    double ZHD{}; ///< Zenith hydrostatic delay [m]
    double ZWD{}; ///< Zenith wet delay [m]
};

} // namespace NAV
