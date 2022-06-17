#include "Functions.hpp"

#include <cmath>
#include "Navigation/Constants.hpp"
#include "util/Logger.hpp"

namespace NAV
{

Eigen::Vector3d e_calcLineOfSightUnitVector(const Eigen::Vector3d& e_posAnt, const Eigen::Vector3d& e_posSat)
{
    return (e_posSat - e_posAnt) / (e_posSat - e_posAnt).norm();
}

double calcSatElevation(const Eigen::Vector3d& n_lineOfSightUnitVector)
{
    return -std::asin(n_lineOfSightUnitVector(2));
}

double calcSatAzimuth(const Eigen::Vector3d& n_lineOfSightUnitVector)
{
    return std::atan2(n_lineOfSightUnitVector(1), n_lineOfSightUnitVector(0));
}

double doppler2psrRate(double doppler, Frequency freq, int8_t num)
{
    return -InsConst::C / freq.getFrequency(num) * doppler;
}

double ratioFreqSquared(Frequency f1, Frequency f2)
{
    return std::pow(f1.getFrequency() / f2.getFrequency(), 2);
}

} // namespace NAV