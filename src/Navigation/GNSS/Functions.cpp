// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

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

uint8_t galSisaVal2Idx(double val)
{
    if (val < 0.0 || val > 6.0) { return 255; } // No Accuracy Prediction Available (NAPA)
    if (val <= 0.5) { return static_cast<uint8_t>(static_cast<unsigned int>(val) / 0.01); }
    if (val <= 1.0) { return static_cast<uint8_t>(static_cast<unsigned int>((val - 0.5)) / 0.02) + 50; }
    if (val <= 2.0) { return static_cast<uint8_t>(static_cast<unsigned int>((val - 1.0)) / 0.04) + 75; }
    return static_cast<uint8_t>(static_cast<unsigned int>((val - 2.0)) / 0.16) + 100;
}

double galSisaIdx2Val(uint8_t idx)
{
    if (idx <= 49) { return static_cast<double>(idx) * 0.01; }
    if (idx <= 74) { return 0.5 + (static_cast<double>(idx) - 50.0) * 0.02; }
    if (idx <= 99) { return 1.0 + (static_cast<double>(idx) - 75.0) * 0.04; }
    if (idx <= 125) { return 2.0 + (static_cast<double>(idx) - 100.0) * 0.16; }
    return 500.0;
}

uint8_t gpsUraVal2Idx(double val)
{
    constexpr std::array<double, 15> URA = { 2.4, 3.4, 4.85, 6.85, 9.65, 13.65, 24.0, 48.0, 96.0, 192.0, 384.0, 768.0, 1536.0, 3072.0, 6144.0 };
    return val < 0.0 ? static_cast<uint8_t>(URA.size())
                     : static_cast<uint8_t>(std::lower_bound(URA.begin(), URA.end(), val) - URA.begin());
}

double gpsUraIdx2Val(uint8_t idx)
{
    constexpr std::array<double, 15> URA = { 2.4, 3.4, 4.85, 6.85, 9.65, 13.65, 24.0, 48.0, 96.0, 192.0, 384.0, 768.0, 1536.0, 3072.0, 6144.0 };
    return URA.at(std::min(static_cast<size_t>(idx), URA.size()));
}

} // namespace NAV