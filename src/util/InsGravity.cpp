#include "InsGravity.hpp"

#include "InsConstants.hpp"

double NAV::gravity::gravityMagnitude_Gleason(const double& latitude)
{
    return 9.7803253359 * (1.0 + 1.931853e-3 * std::pow(std::sin(latitude), 2))
           / std::sqrt(1.0 - InsConst::WGS84_e_squared * std::sin(latitude));
}