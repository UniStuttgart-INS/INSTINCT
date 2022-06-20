#include "Cosecant.hpp"

#include "Navigation/Math/Math.hpp"

namespace NAV
{

double calcTropoMapFunc_cosecant(double elevation)
{
    return math::csc(elevation);
}

double calcTropoMapFunc_secant(double zenithDistance)
{
    return math::sec(zenithDistance);
}

} // namespace NAV
