#include "InsMath.hpp"

double NAV::measureDistance(double lat1, double lon1, double lat2, double lon2)
{
    double R = InsConst::WGS84_a;
    double dLat = lat2 - lat1;
    double dLon = lon2 - lon1;
    double a = std::sin(dLat / 2.0) * std::sin(dLat / 2.0) + std::cos(lat1) * std::cos(lat2) * std::sin(dLon / 2.0) * std::sin(dLon / 2.0);
    double c = 2.0 * std::atan2(std::sqrt(a), std::sqrt(1.0 - a));
    double d = R * c;
    return d; // meters
}