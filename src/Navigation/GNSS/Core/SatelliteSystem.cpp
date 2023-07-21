// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "SatelliteSystem.hpp"

#include "util/Logger.hpp"

namespace NAV
{

SatelliteSystem SatelliteSystem::fromString(const std::string& typeString)
{
    if (typeString == "GPS" || typeString == "GP")
    {
        return GPS;
    }
    if (typeString == "GLONASS" || typeString == "GLO" || typeString == "GL")
    {
        return GLO;
    }
    if (typeString == "GALILEO" || typeString == "GAL" || typeString == "GA")
    {
        return GAL;
    }
    if (typeString == "QZSS" || typeString == "QZS" || typeString == "QZ")
    {
        return QZSS;
    }
    if (typeString == "BDS" || typeString == "BD")
    {
        return BDS;
    }
    if (typeString == "IRNSS" || typeString == "IRN" || typeString == "IR")
    {
        return IRNSS;
    }
    if (typeString == "SBAS" || typeString == "SB")
    {
        return SBAS;
    }

    return SatSys_None;
}

SatelliteSystem SatelliteSystem::fromChar(char typeChar)
{
    switch (typeChar)
    {
    case 'G':
        return GPS;
    case 'R':
        return GLO;
    case 'E':
        return GAL;
    case 'J':
    case 'Q':
        return QZSS;
    case 'C':
        return BDS;
    case 'I':
        return IRNSS;
    case 'S':
        return SBAS;
    default:
        return SatSys_None;
    }
}

SatelliteSystem SatelliteSystem::fromEnum(size_t enumeration)
{
    switch (enumeration)
    {
    case 0:
        return SatSys_None;
    case 1:
        return GPS;
    case 2:
        return GAL;
    case 3:
        return GLO;
    case 4:
        return BDS;
    case 5:
        return QZSS;
    case 6:
        return IRNSS;
    case 7:
        return SBAS;
    default:
        return SatSys_None;
    }
}

SatelliteSystem::operator std::string() const
{
    std::string str;
    if (value & GPS) { str += ((!str.empty() ? " | " : "") + std::string("GPS")); }
    if (value & GLO) { str += ((!str.empty() ? " | " : "") + std::string("GLO")); }
    if (value & GAL) { str += ((!str.empty() ? " | " : "") + std::string("GAL")); }
    if (value & QZSS) { str += ((!str.empty() ? " | " : "") + std::string("QZSS")); }
    if (value & BDS) { str += ((!str.empty() ? " | " : "") + std::string("BDS")); }
    if (value & IRNSS) { str += ((!str.empty() ? " | " : "") + std::string("IRNSS")); }
    if (value & SBAS) { str += ((!str.empty() ? " | " : "") + std::string("SBAS")); }

    return str.empty() ? "None" : str;
}

SatelliteSystem::operator char() const
{
    switch (value)
    {
    case GPS:
        return 'G';
    case GLO:
        return 'R';
    case GAL:
        return 'E';
    case QZSS:
        return 'J';
    case BDS:
        return 'C';
    case IRNSS:
        return 'I';
    case SBAS:
        return 'S';
    default:
        return '\0';
    }
}

TimeSystem SatelliteSystem::GetTimeSystemForSatelliteSystem(SatelliteSystem satSys)
{
    switch (SatelliteSystem_(satSys))
    {
    case GPS:
        return GPST;
    case GLO:
        return GLNT;
    case GAL:
        return GST;
    case QZSS:
        return QZSST;
    case BDS:
        return BDT;
    case IRNSS:
        return IRNSST;
    case SBAS:
        return GPST;
    case SatSys_None:
        break;
    }
    return TimeSys_None;
}

TimeSystem SatelliteSystem::getTimeSystem() const
{
    return GetTimeSystemForSatelliteSystem(value);
}

std::vector<uint16_t> SatelliteSystem::GetSatellitesForSatelliteSystem(SatelliteSystem satSys)
{
    switch (SatelliteSystem_(satSys))
    {
    case GPS:
        return { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32 };
    case GLO:
        return { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24 };
    case GAL:
        return { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32,
                 33, 34, 35, 36 };
    case QZSS:
        return { 1, 2, 3, 4, 5 };
    case BDS:
        return { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32,
                 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62 };
    case IRNSS:
        return { 1, 2, 3, 4, 5, 6, 7, 8, 9 };
    case SBAS:
        return {
            120, 234, 236,      // EGNOS
            131, 133, 135, 138, // WAAS
            129, 137,           // MSAS
            127, 128, 132,      // GAGAN
        };
    case SatSys_None:
        break;
    }
    return {};
}

std::vector<uint16_t> SatelliteSystem::getSatellites() const
{
    return GetSatellitesForSatelliteSystem(value);
}

size_t SatelliteSystem::ToEnumeration(SatelliteSystem satSys)
{
    switch (SatelliteSystem_(satSys))
    {
    case SatSys_None:
        return 0;
    case GPS:
        return 1;
    case GAL:
        return 2;
    case GLO:
        return 3;
    case BDS:
        return 4;
    case QZSS:
        return 5;
    case IRNSS:
        return 6;
    case SBAS:
        return 7;
    }
    return 0;
}

size_t SatelliteSystem::toEnumeration() const
{
    return ToEnumeration(value);
}

std::vector<SatelliteSystem> SatelliteSystem::ToVector(SatelliteSystem satSys)
{
    std::vector<SatelliteSystem> v;
    for (const SatelliteSystem& sys : GetAll())
    {
        if (sys.value & satSys.value) { v.push_back(sys); }
    }
    return v;
}

std::vector<SatelliteSystem> SatelliteSystem::toVector() const
{
    return ToVector(value);
}

std::vector<SatelliteSystem> SatelliteSystem::GetAll()
{
    return { GPS, GAL, GLO, BDS, QZSS, IRNSS, SBAS };
}

void to_json(json& j, const SatelliteSystem& data)
{
    j = std::string(data);
}
void from_json(const json& j, SatelliteSystem& data)
{
    data = SatelliteSystem::fromString(j.get<std::string>());
}

std::ostream& operator<<(std::ostream& os, const SatelliteSystem& satSys)
{
    os << fmt::format("{}", satSys);
    return os;
}

} // namespace NAV