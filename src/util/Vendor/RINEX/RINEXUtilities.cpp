// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "RINEXUtilities.hpp"

#include <sstream>
#include <locale>

#include <boost/date_time/posix_time/posix_time.hpp> // TODO: Remove 'boost.dateTime' from conan as soon as 'std::chrono::utc_clock' becomes available
#include <fmt/chrono.h>
#include <fmt/format.h>
using namespace fmt::literals; // NOLINT(google-build-using-namespace)

#include "internal/Version.hpp"
#include "Navigation/Math/Math.hpp"
#include "util/Logger.hpp"

namespace NAV
{
namespace vendor::RINEX
{

void ObsHeader::reset()
{
    satSys = SatSys_None;
    systemObsTypes.clear();
    interval = 1e9;
    timeFirstObs.reset();
    timeLastObs.reset();
    satellites.clear();
    timeSys = TimeSys_None;
}

bool ObsHeader::addObsType(const Code& code, ObsType type)
{
    auto& obsDescriptions = systemObsTypes[code.getFrequency().getSatSys()];
    if (std::find_if(obsDescriptions.begin(), obsDescriptions.end(), [&code, &type](const auto& obsDesc) {
            return obsDesc.code == code && obsDesc.type == type;
        })
        == obsDescriptions.end())
    {
        obsDescriptions.push_back(ObservationDescription{ type, code });
        return true;
    }
    return false;
}

std::string ObsHeader::generateHeader() const
{
    std::string str;

    str += headerLineRinexVersionType();
    str += headerLinePgmRunByDate();
    str += headerLineComments();
    str += headerLineMarkerName();
    str += headerLineMarkerNumber();
    str += headerLineMarkerType();
    str += headerLineObserverAgency();
    str += headerLineRecNumTypeVers();
    str += headerLineAntNumType();
    str += headerLineApproxPositionXYZ();
    str += headerLineAntennaDeltaHEN();
    // ANTENNA: DELTA X/Y/Z (optional)
    // ANTENNA:PHASECENTER (optional)
    // ANTENNA: B.SIGHT XYZ (optional)
    // ANTENNA: ZERODIR AZI (optional)
    // ANTENNA: ZERODIR XYZ (optional)
    // CENTER OF MASS: XYZ (optional)
    str += headerLineSysNumObsType();
    str += headerLineSignalStrengthUnit();
    str += headerLineInterval();
    str += headerLineTimeOfFirstObs();
    str += headerLineTimeOfLastObs();
    // RCV CLOCK OFFS APPL (optional)
    // SYS / DCBS APPLIED (optional)
    // SYS / PCVS APPLIED (optional)
    // SYS / SCALE FACTOR (optional)
    str += headerLineSysPhaseShift();
    // GLONASS SLOT / FRQ # (is mandatory, but a lot of receivers also do not write it. So we leave it out. Would require NAV information)
    str += headerLineGlonassCodPhsBis();
    str += headerLineLeapSeconds();
    str += headerLineNumSatellites();
    // PRN / # OF OBS (optional)
    str += headerLineEndOfHeader();

    return str;
}
std::string ObsHeader::headerLineRinexVersionType() const
{
    std::string longSatSysString;
    switch (static_cast<SatelliteSystem_>(satSys))
    {
    case GPS:
        longSatSysString = "GPS";
        break;
    case GAL:
        longSatSysString = "GALILEO";
        break;
    case GLO:
        longSatSysString = "GLONASS";
        break;
    case BDS:
        longSatSysString = "BEIDOU";
        break;
    case QZSS:
        longSatSysString = "QZSS";
        break;
    case IRNSS:
        longSatSysString = "IRNSS";
        break;
    default:
        longSatSysString = "MIXED";
        break;
    }

    return fmt::format("{version: 9.2f}{X:11}{fileType:<20}{satSys:1}: {satSysTxt:<17}{label:<20}\n", "X"_a = "", "label"_a = "RINEX VERSION / TYPE",
                       "version"_a = version,
                       "fileType"_a = "OBSERVATION DATA",
                       "satSys"_a = satSys.toChar(),
                       "satSysTxt"_a = longSatSysString);
}
std::string ObsHeader::headerLinePgmRunByDate() const
{
    std::stringstream ss;
    ss.imbue(std::locale{ ss.getloc(), new boost::posix_time::time_facet{ "%Y%m%d %H%M%S" } });
    ss << boost::posix_time::second_clock::universal_time();
    return fmt::format("{pgm:20}{runBy:20}{date} UTC {label:20}\n", "label"_a = "PGM / RUN BY / DATE",
                       "pgm"_a = "INSTINCT " + PROJECT_VERSION_STRING,
                       "runBy"_a = runBy.substr(0, 20),
                       "date"_a = ss.str()); // std::chrono::floor<std::chrono::seconds>(std::chrono::utc_clock::now())
}
std::string ObsHeader::headerLineComments() const
{
    std::string str;
    for (const auto& comment : comments)
    {
        str += fmt::format("{comment:60}{label:<20}\n", "label"_a = "COMMENT",
                           "comment"_a = comment.substr(0, 60));
    }
    return str;
}
std::string ObsHeader::headerLineMarkerName() const
{
    return fmt::format("{markerName:60}{label:<20}\n", "label"_a = "MARKER NAME",
                       "markerName"_a = markerName.substr(0, 60));
}
std::string ObsHeader::headerLineMarkerNumber() const
{
    if (!markerNumber.empty())
    {
        return fmt::format("{markerNumber:60}{label:<20}\n", "label"_a = "MARKER NUMBER",
                           "markerNumber"_a = markerNumber.substr(0, 60));
    }
    return "";
}
std::string ObsHeader::headerLineMarkerType() const
{
    return fmt::format("{markerType:20}{X:40}{label:<20}\n", "X"_a = "", "label"_a = "MARKER TYPE",
                       "markerType"_a = (markerType == MarkerTypes::USER_DEFINED
                                             ? markerTypeUser.substr(0, 20)
                                             : NAV::to_string(markerType)));
}
std::string ObsHeader::headerLineObserverAgency() const
{
    return fmt::format("{observer:20}{agency:40}{label:<20}\n", "label"_a = "OBSERVER / AGENCY",
                       "observer"_a = observer.substr(0, 20),
                       "agency"_a = agency.substr(0, 40));
}
std::string ObsHeader::headerLineRecNumTypeVers() const
{
    return fmt::format("{rec:20}{type:20}{vers:20}{label:<20}\n", "label"_a = "REC # / TYPE / VERS",
                       "rec"_a = receiverNumber.substr(0, 20),
                       "type"_a = receiverType.substr(0, 20),
                       "vers"_a = receiverVersion.substr(0, 20));
}
std::string ObsHeader::headerLineAntNumType() const
{
    return fmt::format("{ant:20}{type:20}{X:20}{label:<20}\n", "X"_a = "", "label"_a = "ANT # / TYPE",
                       "ant"_a = antennaNumber.substr(0, 20),
                       "type"_a = antennaType.substr(0, 20));
}
std::string ObsHeader::headerLineApproxPositionXYZ() const
{
    if (approxPositionEnabled)
    {
        return fmt::format("{x:14.4f}{y:14.4f}{z:14.4f}{X:18}{label:<20}\n", "X"_a = "", "label"_a = "APPROX POSITION XYZ",
                           "x"_a = approxPositionXYZ.x(),
                           "y"_a = approxPositionXYZ.y(),
                           "z"_a = approxPositionXYZ.z());
    }
    return "";
}
std::string ObsHeader::headerLineAntennaDeltaHEN() const
{
    return fmt::format("{antH:14.4f}{antE:14.4f}{antN:14.4f}{X:18}{label:<20}\n", "X"_a = "", "label"_a = "ANTENNA: DELTA H/E/N",
                       "antH"_a = antennaDeltaHeightEastNorth[0],
                       "antE"_a = antennaDeltaHeightEastNorth[1],
                       "antN"_a = antennaDeltaHeightEastNorth[2]);
}
std::string ObsHeader::headerLineSysNumObsType() const
{
    std::string str;
    for (const auto& [satSys, types] : systemObsTypes) // SYS / # / OBS TYPES
    {
        bool firstLine = true;
        for (size_t i = 0; i < types.size(); ++i)
        {
            const auto& obsDesc = types.at(i);
            if (i % 13 == 0)
            {
                if (firstLine)
                {
                    str += fmt::format("{sys:1}{X:2}{num:3}", "X"_a = "",
                                       "sys"_a = satSys.toChar(),
                                       "num"_a = types.size());
                    firstLine = false;
                }
                else { str += fmt::format("{X:6}", "X"_a = ""); }
            }

            str += fmt::format("{X:1}{desc:3}", "X"_a = "",
                               "desc"_a = vendor::RINEX::obsTypeToChar(obsDesc.type)
                                          + fmt::format("{}", obsDesc.code).substr(1, 2));

            if ((i + 1) % 13 == 0 || i == types.size() - 1)
            {
                str += fmt::format("{X:{w}}{label:<20}\n", "X"_a = "", "label"_a = "SYS / # / OBS TYPES",
                                   "w"_a = 54 - 4 * ((i % 13) + 1));
            }
        }
    }
    return str;
}
std::string ObsHeader::headerLineSignalStrengthUnit()
{
    return fmt::format("{sigUnit:20}{X:40}{label:<20}\n", "X"_a = "", "label"_a = "SIGNAL STRENGTH UNIT",
                       "sigUnit"_a = "DBHZ");
}
std::string ObsHeader::headerLineInterval() const
{
    if (interval == 1e9) { return fmt::format("{X:60}{label:<20}\n", "X"_a = "", "label"_a = "INTERVAL"); }
    return fmt::format("{interval:10.3f}{X:50}{label:<20}\n", "X"_a = "", "label"_a = "INTERVAL",
                       "interval"_a = interval);
}
std::string ObsHeader::headerLineTimeOfFirstObs() const
{
    auto firstObsYMDHMS = timeFirstObs.toYMDHMS(timeSys, 7);
    return fmt::format("{y:6d}{m:6d}{d:6d}{h:6d}{min:6d}{sec:13.7f}{X:5}{sys:3}{X:9}{label:<20}\n", "X"_a = "", "label"_a = "TIME OF FIRST OBS",
                       "y"_a = firstObsYMDHMS.year,
                       "m"_a = firstObsYMDHMS.month,
                       "d"_a = firstObsYMDHMS.day,
                       "h"_a = firstObsYMDHMS.hour,
                       "min"_a = firstObsYMDHMS.min,
                       "sec"_a = static_cast<double>(firstObsYMDHMS.sec),
                       "sys"_a = vendor::RINEX::timeSystemString(timeSys));
}
std::string ObsHeader::headerLineTimeOfLastObs() const
{
    auto lastObsYMDHMS = timeLastObs.toYMDHMS(timeSys, 7);
    return fmt::format("{y:6d}{m:6d}{d:6d}{h:6d}{min:6d}{sec:13.7f}{X:5}{sys:3}{X:9}{label:<20}\n", "X"_a = "", "label"_a = "TIME OF LAST OBS",
                       "y"_a = lastObsYMDHMS.year,
                       "m"_a = lastObsYMDHMS.month,
                       "d"_a = lastObsYMDHMS.day,
                       "h"_a = lastObsYMDHMS.hour,
                       "min"_a = lastObsYMDHMS.min,
                       "sec"_a = static_cast<double>(lastObsYMDHMS.sec),
                       "sys"_a = vendor::RINEX::timeSystemString(timeSys));
}
std::string ObsHeader::headerLineSysPhaseShift()
{
    return fmt::format("{X:60}{label:<20}\n", "X"_a = "", "label"_a = "SYS / PHASE SHIFT");
}
std::string ObsHeader::headerLineGlonassCodPhsBis()
{
    return fmt::format("{X:60}{label:<20}\n", "X"_a = "", "label"_a = "GLONASS COD/PHS/BIS");
}
std::string ObsHeader::headerLineLeapSeconds() const
{
    return fmt::format("{leap:6d}{X:54}{label:<20}\n", "X"_a = "", "label"_a = "LEAP SECONDS",
                       "leap"_a = timeFirstObs.leapGps2UTC());
}
std::string ObsHeader::headerLineNumSatellites() const
{
    return fmt::format("{nSat:6d}{X:54}{label:<20}\n", "X"_a = "", "label"_a = "# OF SATELLITES",
                       "nSat"_a = satellites.size());
}
std::string ObsHeader::headerLineEndOfHeader()
{
    return fmt::format("{X:60}{label:<20}\n", "X"_a = "", "label"_a = "END OF HEADER");
}

std::string ObsHeader::epochRecordLine(const InsTime& epochTime) const
{
    auto timeYMDHMS = epochTime.toYMDHMS(timeSys, 7);
    return fmt::format("> {y:4d} {m:2d} {d:2d} {h:2d} {min:2d}{sec:11.7f}  0{numSat:3d}{X:6}{X:15}\n", "X"_a = "",
                       "y"_a = timeYMDHMS.year,
                       "m"_a = timeYMDHMS.month,
                       "d"_a = timeYMDHMS.day,
                       "h"_a = timeYMDHMS.hour,
                       "min"_a = timeYMDHMS.min,
                       "sec"_a = static_cast<double>(timeYMDHMS.sec),
                       "numSat"_a = satellites.size());
}

void to_json(json& j, const ObsHeader& obj)
{
    j = json{
        { "version", obj.version },
        { "runBy", obj.runBy },
        { "comments", obj.comments },
        { "markerName", obj.markerName },
        { "markerNumber", obj.markerNumber },
        { "markerType", obj.markerType },
        { "markerTypeUser", obj.markerTypeUser },
        { "observer", obj.observer },
        { "agency", obj.agency },
        { "receiverNumber", obj.receiverNumber },
        { "receiverType", obj.receiverType },
        { "receiverVersion", obj.receiverVersion },
        { "antennaNumber", obj.antennaNumber },
        { "antennaType", obj.antennaType },
        { "approxPositionEnabled", obj.approxPositionEnabled },
        { "approxPositionXYZ", obj.approxPositionXYZ },
        { "antennaDeltaHeightEastNorth", obj.antennaDeltaHeightEastNorth },
    };
}

void from_json(const json& j, ObsHeader& obj)
{
    if (j.contains("version")) { j.at("version").get_to(obj.version); }
    if (j.contains("runBy")) { j.at("runBy").get_to(obj.runBy); }
    if (j.contains("comments")) { j.at("comments").get_to(obj.comments); }
    if (j.contains("markerName")) { j.at("markerName").get_to(obj.markerName); }
    if (j.contains("markerNumber")) { j.at("markerNumber").get_to(obj.markerNumber); }
    if (j.contains("markerType")) { j.at("markerType").get_to(obj.markerType); }
    if (j.contains("markerTypeUser")) { j.at("markerTypeUser").get_to(obj.markerTypeUser); }
    if (j.contains("observer")) { j.at("observer").get_to(obj.observer); }
    if (j.contains("agency")) { j.at("agency").get_to(obj.agency); }
    if (j.contains("receiverNumber")) { j.at("receiverNumber").get_to(obj.receiverNumber); }
    if (j.contains("receiverType")) { j.at("receiverType").get_to(obj.receiverType); }
    if (j.contains("receiverVersion")) { j.at("receiverVersion").get_to(obj.receiverVersion); }
    if (j.contains("antennaNumber")) { j.at("antennaNumber").get_to(obj.antennaNumber); }
    if (j.contains("antennaType")) { j.at("antennaType").get_to(obj.antennaType); }
    if (j.contains("approxPositionEnabled")) { j.at("approxPositionEnabled").get_to(obj.approxPositionEnabled); }
    if (j.contains("approxPositionXYZ")) { j.at("approxPositionXYZ").get_to(obj.approxPositionXYZ); }
    if (j.contains("antennaDeltaHeightEastNorth")) { j.at("antennaDeltaHeightEastNorth").get_to(obj.antennaDeltaHeightEastNorth); }
}
ObsType obsTypeFromChar(char c)
{
    switch (c)
    {
    case 'C':
        return ObsType::C;
    case 'L':
        return ObsType::L;
    case 'D':
        return ObsType::D;
    case 'S':
        return ObsType::S;
    case 'I':
        return ObsType::I;
    case 'X':
        return ObsType::X;
    default:
        return ObsType::Error;
    }
}

char obsTypeToChar(ObsType type)
{
    switch (type)
    {
    case ObsType::C:
        return 'C';
    case ObsType::L:
        return 'L';
    case ObsType::D:
        return 'D';
    case ObsType::S:
        return 'S';
    case ObsType::I:
        return 'I';
    case ObsType::X:
        return 'X';
    default:
        return '\0';
    }
}

std::string timeSystemString(TimeSystem timeSys)
{
    switch (static_cast<TimeSystem_>(timeSys))
    {
    case GPST:
        return "GPS";
    case GST:
        return "GAL";
    case GLNT:
        return "GLO";
    case BDT:
        return "BDT";
    case QZSST:
        return "QZS";
    case IRNSST:
        return "IRN";
    case UTC:
    case TimeSys_None:
    default:
        break;
    }
    return "GPS";
}

TimeSystem timeSystem(SatelliteSystem satSys)
{
    switch (static_cast<SatelliteSystem_>(satSys))
    {
    case GPS:
        return GPST;
    case GAL:
        return GST;
    case GLO:
        return GLNT;
    case BDS:
        return BDT;
    case QZSS:
        return QZSST;
    case IRNSS:
        return IRNSST;
    default:
        break;
    }
    return GPST;
}

Frequency getFrequencyFromBand(SatelliteSystem satSys, int band)
{
    switch (band)
    {
    case 1:
        // 1 = L1       (GPS, QZSS, SBAS, BDS)
        //     G1       (GLO)
        //     E1       (GAL)
        //     B1       (BDS)
        if (satSys == GPS)
        {
            return Frequency_::G01;
        }
        if (satSys == QZSS)
        {
            return Frequency_::J01;
        }
        if (satSys == SBAS)
        {
            return Frequency_::S01;
        }
        if (satSys == BDS)
        {
            return Frequency_::B01;
        }
        if (satSys == GLO)
        {
            return Frequency_::R01;
        }
        if (satSys == GAL)
        {
            return Frequency_::E01;
        }
        break;
    case 2:
        // 2 = L2       (GPS, QZSS)
        //     G2       (GLO)
        //     B1-2     (BDS)
        if (satSys == GPS)
        {
            return Frequency_::G02;
        }
        if (satSys == QZSS)
        {
            return Frequency_::J02;
        }
        if (satSys == GLO)
        {
            return Frequency_::R02;
        }
        if (satSys == BDS)
        {
            return Frequency_::B02;
        }
        break;
    case 3:
        // 3 = G3       (GLO)
        if (satSys == GLO)
        {
            return Frequency_::R03;
        }
        break;
    case 4:
        // 4 = G1a      (GLO)
        if (satSys == GLO)
        {
            return Frequency_::R04;
        }
        break;
    case 5:
        // 5 = L5       (GPS, QZSS, SBAS, IRNSS)
        //     E5a      (GAL)
        //     B2/B2a   (BDS)
        if (satSys == GPS)
        {
            return Frequency_::G05;
        }
        if (satSys == QZSS)
        {
            return Frequency_::J05;
        }
        if (satSys == SBAS)
        {
            return Frequency_::S05;
        }
        if (satSys == IRNSS)
        {
            return Frequency_::I05;
        }
        if (satSys == GAL)
        {
            return Frequency_::E05;
        }
        if (satSys == BDS)
        {
            return Frequency_::B05;
        }
        break;
    case 6:
        // 6 = E6       (GAL)
        //     L6       (QZSS)
        //     B3       (BDS)
        //     G2a      (GLO)
        if (satSys == GAL)
        {
            return Frequency_::E06;
        }
        if (satSys == QZSS)
        {
            return Frequency_::J06;
        }
        if (satSys == BDS)
        {
            return Frequency_::B06;
        }
        if (satSys == GLO)
        {
            return Frequency_::R06;
        }
        break;
    case 7:
        // 7 = E5b      (GAL)
        //     B2/B2b   (BDS)
        if (satSys == GAL)
        {
            return Frequency_::E07;
        }
        if (satSys == BDS)
        {
            return Frequency_::B07;
        }
        break;
    case 8:
        // 8 = E5a+b    (GAL)
        //     B2a+b    (BDS)
        if (satSys == GAL)
        {
            return Frequency_::E08;
        }
        if (satSys == BDS)
        {
            return Frequency_::B08;
        }
        break;
    case 9:
        // 9 = S        (IRNSS)
        if (satSys == IRNSS)
        {
            return Frequency_::I09;
        }
        break;
    case 0:
        // 0 for type X (all)
        break;
    default:
        break;
    }

    LOG_ERROR("Cannot find frequency for satellite system '{}' and band '{}'", satSys, band);
    return Freq_None;
}

} // namespace vendor::RINEX

const char* to_string(vendor::RINEX::ObsHeader::MarkerTypes markerType)
{
    using MarkerTypes = NAV::vendor::RINEX::ObsHeader::MarkerTypes;
    switch (markerType)
    {
    case MarkerTypes::GEODETIC:
        return "GEODETIC";
    case MarkerTypes::NON_GEODETIC:
        return "NON_GEODETIC";
    case MarkerTypes::NON_PHYSICAL:
        return "NON_PHYSICAL";
    case MarkerTypes::SPACEBORNE:
        return "SPACEBORNE";
    case MarkerTypes::GROUND_CRAFT:
        return "GROUND_CRAFT";
    case MarkerTypes::WATER_CRAFT:
        return "WATER_CRAFT";
    case MarkerTypes::AIRBORNE:
        return "AIRBORNE";
    case MarkerTypes::FIXED_BUOY:
        return "FIXED_BUOY";
    case MarkerTypes::FLOATING_BUOY:
        return "FLOATING_BUOY";
    case MarkerTypes::FLOATING_ICE:
        return "FLOATING_ICE";
    case MarkerTypes::GLACIER:
        return "GLACIER";
    case MarkerTypes::BALLISTIC:
        return "BALLISTIC";
    case MarkerTypes::ANIMAL:
        return "ANIMAL";
    case MarkerTypes::HUMAN:
        return "HUMAN";
    case MarkerTypes::USER_DEFINED:
        return "USER_DEFINED";
    case MarkerTypes::COUNT:
        return "";
    }
    return "";
}

const char* tooltip(vendor::RINEX::ObsHeader::MarkerTypes markerType)
{
    using MarkerTypes = NAV::vendor::RINEX::ObsHeader::MarkerTypes;
    switch (markerType)
    {
    case MarkerTypes::GEODETIC:
        return "Earth-fixed, high-precision monument";
    case MarkerTypes::NON_GEODETIC:
        return "Earth-fixed, low-precision monument";
    case MarkerTypes::NON_PHYSICAL:
        return "Generated from network processing";
    case MarkerTypes::SPACEBORNE:
        return "Orbiting space vehicle";
    case MarkerTypes::GROUND_CRAFT:
        return "Mobile terrestrial vehicle";
    case MarkerTypes::WATER_CRAFT:
        return "Mobile water craft";
    case MarkerTypes::AIRBORNE:
        return "Aircraft, balloon, etc.";
    case MarkerTypes::FIXED_BUOY:
        return "'Fixed' on water surface";
    case MarkerTypes::FLOATING_BUOY:
        return "Floating on water surface";
    case MarkerTypes::FLOATING_ICE:
        return "Floating ice sheet, etc.";
    case MarkerTypes::GLACIER:
        return "'Fixed' on a glacier";
    case MarkerTypes::BALLISTIC:
        return "Rockets, shells, etc";
    case MarkerTypes::ANIMAL:
        return "Animal carrying a receiver";
    case MarkerTypes::HUMAN:
        return "Human being";
    case MarkerTypes::USER_DEFINED:
        return "Users may define other project-dependent keywords";
    case MarkerTypes::COUNT:
        return "";
    }
    return "";
}

} // namespace NAV