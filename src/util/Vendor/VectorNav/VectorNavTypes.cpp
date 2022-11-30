// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "VectorNavTypes.hpp"

std::ostream& NAV::vendor::vectornav::operator<<(std::ostream& os, const NAV::vendor::vectornav::SatSys& satSys)
{
    switch (satSys)
    {
    case SatSys::GPS:
        os << "GPS";
        break;
    case SatSys::SBAS:
        os << "SBAS";
        break;
    case SatSys::Galileo:
        os << "Galileo";
        break;
    case SatSys::BeiDou:
        os << "BeiDou";
        break;
    case SatSys::IMES:
        os << "IMES";
        break;
    case SatSys::QZSS:
        os << "QZSS";
        break;
    case SatSys::GLONASS:
        os << "GLONASS";
        break;
    }
    return os;
}