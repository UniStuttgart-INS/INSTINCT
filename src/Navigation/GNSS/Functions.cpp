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

double calcSagnacCorrection(const Eigen::Vector3d& e_posAnt, const Eigen::Vector3d& e_satPos)
{
    return 1.0 / InsConst<>::C * (e_posAnt - e_satPos).dot(InsConst<>::e_omega_ie.cross(e_posAnt));
}

double calcSagnacRateCorrection(const Eigen::Vector3d& e_posAnt, const Eigen::Vector3d& e_satPos, const Eigen::Vector3d& e_velAnt, const Eigen::Vector3d& e_satVel)
{
    return InsConst<>::omega_ie / InsConst<>::C
           * (e_satVel.y() * e_posAnt.x() + e_satPos.y() * e_velAnt.x()
              - e_satVel.x() * e_posAnt.y() - e_satPos.x() * e_velAnt.y());
}

double doppler2rangeRate(double doppler, Frequency freq, int8_t num)
{
    return -InsConst<>::C / freq.getFrequency(num) * doppler;
}

double rangeRate2doppler(double rangeRate, Frequency freq, int8_t num)
{
    return -freq.getFrequency(num) / InsConst<>::C * rangeRate;
}

double ratioFreqSquared(Frequency f1, Frequency f2, int8_t num1, int8_t num2)
{
    return std::pow(f1.getFrequency(num1) / f2.getFrequency(num2), 2);
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
    if (idx == 1) { return 2.8; }
    if (idx == 3) { return 5.7; }
    if (idx == 5) { return 11.3; }
    if (idx <= 6) { return std::pow(2, 1.0 + idx / 2.0); }
    if (idx < 15) { return std::pow(2, idx - 2); }
    return 6144.0;
}

} // namespace NAV