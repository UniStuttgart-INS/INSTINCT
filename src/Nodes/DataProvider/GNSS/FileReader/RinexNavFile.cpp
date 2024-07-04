// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "RinexNavFile.hpp"

#include <bitset>

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"

#include "util/StringUtil.hpp"

#include "Navigation/GNSS/Core/SatelliteSystem.hpp"
#include "Navigation/Atmosphere/Ionosphere/IonosphericCorrections.hpp"
#include "Navigation/GNSS/Satellite/Ephemeris/GPSEphemeris.hpp"
#include "Navigation/GNSS/Satellite/Ephemeris/GalileoEphemeris.hpp"
#include "Navigation/GNSS/Satellite/Ephemeris/GLONASSEphemeris.hpp"
#include "Navigation/GNSS/Satellite/Ephemeris/BDSEphemeris.hpp"

namespace NAV
{
RinexNavFile::RinexNavFile()
    : Node(typeStatic())
{
    LOG_TRACE("{}: called", name);

    _hasConfig = true;
    _guiConfigDefaultWindowSize = { 517, 87 };

    nm::CreateOutputPin(this, GnssNavInfo::type().c_str(), Pin::Type::Object, { GnssNavInfo::type() }, &_gnssNavInfo);
}

RinexNavFile::~RinexNavFile()
{
    LOG_TRACE("{}: called", nameId());
}

std::string RinexNavFile::typeStatic()
{
    return "RinexNavFile";
}

std::string RinexNavFile::type() const
{
    return typeStatic();
}

std::string RinexNavFile::category()
{
    return "Data Provider";
}

void RinexNavFile::guiConfig()
{
    if (auto res = FileReader::guiConfig(R"(Rinex Nav (.nav .rnx .gal .geo .glo .*N .*P){.nav,.rnx,.gal,.geo,.glo,(.+[.]\d\d?N),(.+[.]\d\d?L),(.+[.]\d\d?P)},.*)",
                                         { ".nav", ".rnx", ".gal", ".geo", ".glo", "(.+[.]\\d\\d?N)", "(.+[.]\\d\\d?L)", "(.+[.]\\d\\d?P)" }, size_t(id), nameId()))
    {
        LOG_DEBUG("{}: Path changed to {}", nameId(), _path);
        flow::ApplyChanges();
        if (res == FileReader::PATH_CHANGED)
        {
            doReinitialize();
        }
        else
        {
            doDeinitialize();
        }
    }
    ImGui::Text("Supported versions: ");
    std::for_each(_supportedVersions.cbegin(), _supportedVersions.cend(), [](double x) {
        ImGui::SameLine();
        ImGui::Text("%0.2f", x);
    });
}

[[nodiscard]] json RinexNavFile::save() const
{
    LOG_TRACE("{}: called", nameId());

    json j;

    j["FileReader"] = FileReader::save();

    return j;
}

void RinexNavFile::restore(json const& j)
{
    LOG_TRACE("{}: called", nameId());

    if (j.contains("FileReader"))
    {
        FileReader::restore(j.at("FileReader"));
    }
}

bool RinexNavFile::initialize()
{
    LOG_TRACE("{}: called", nameId());

    {
        // The guards needs to be released before FileReader::initialize()
        auto guard = requestOutputValueLock(OUTPUT_PORT_INDEX_GNSS_NAV_INFO);
        _gnssNavInfo.reset();
    }
    _version = 0.0;

    if (!FileReader::initialize())
    {
        return false;
    }

    readOrbits();

    return true;
}

void RinexNavFile::deinitialize()
{
    LOG_TRACE("{}: called", nameId());

    FileReader::deinitialize();
}

bool RinexNavFile::resetNode()
{
    LOG_TRACE("{}: called", nameId());

    FileReader::resetReader();

    return true;
}

FileReader::FileType RinexNavFile::determineFileType()
{
    auto extHeaderLabel = [](const std::string& line) {
        return str::trim_copy(std::string_view(line).substr(60, 20));
    };

    std::filesystem::path filepath = getFilepath();

    auto filestreamHeader = std::ifstream(filepath);
    if (filestreamHeader.good())
    {
        std::string line;
        // --------------------------------------- RINEX VERSION / TYPE ------------------------------------------
        while (line.find("RINEX VERSION / TYPE") == std::string::npos && !filestreamHeader.eof())
        {
            std::getline(filestreamHeader, line);
        }
        if (filestreamHeader.eof())
        {
            LOG_ERROR("{}: Not a valid RINEX NAV file. Could not read 'RINEX VERSION / TYPE' line.", nameId());
            return FileReader::FileType::NONE;
        }
        str::rtrim(line);
        if (str::rtrim_copy(line).size() != 80)
        {
            LOG_ERROR("{}: Not a valid RINEX NAV file. Lines should be 80 characters long but the file has {}.", nameId(), line.size() - 1);
            return FileReader::FileType::NONE;
        }

        if (extHeaderLabel(line) != "RINEX VERSION / TYPE")
        {
            LOG_ERROR("{}: Not a valid RINEX NAV file. Could not read 'RINEX VERSION / TYPE' line.", nameId());
            return FileReader::FileType::NONE;
        }

        _version = std::stod(str::trim_copy(line.substr(0, 20))); // FORMAT: F9.2,11X
        if (!_supportedVersions.contains(_version))
        {
            LOG_ERROR("{}: RINEX version {} is not supported. Supported versions are [{}]", nameId(),
                      _version, fmt::join(_supportedVersions.begin(), _supportedVersions.end(), ", "));
            _version = 0.0;
            return FileReader::FileType::NONE;
        }

        std::string fileType = str::trim_copy(line.substr(20, 20)); // FORMAT: A1,19X
        if (fileType.at(0) != 'N'                                   // N: GNSS NAV DATA
            && fileType != "NAVIGATION DATA"
            && fileType != "GLONASS NAV DATA")
        {
            LOG_ERROR("{}: Not a valid RINEX NAV file. File type '{}' not recognized.", nameId(), fileType);
            doDeinitialize();
            return FileReader::FileType::NONE;
        }
        std::string satSystem;
        if (_version < 3.0)
        {
            if (fileType == "NAVIGATION DATA")
            {
                satSystem = 'G';
            }
            else if (fileType == "GLONASS NAV DATA")
            {
                satSystem = 'R';
            }
            else if (fileType.at(0) == 'N')
            {
                satSystem = fileType.substr(3, 3);
            }
        }
        else
        {
            satSystem = str::trim_copy(line.substr(40, 20)); // FORMAT: A1,19X
        }
        if (satSystem.empty() || (SatelliteSystem::fromChar(satSystem.at(0)) == SatSys_None && satSystem.at(0) != 'M'))
        {
            LOG_ERROR("{}: Not a valid RINEX NAV file. Satellite System '{}' not recognized.", nameId(), satSystem);
            doDeinitialize();
            return FileReader::FileType::NONE;
        }
        // ---------------------------------------- PGM / RUN BY / DATE ------------------------------------------
        std::getline(filestreamHeader, line);
        str::rtrim(line);
        if (extHeaderLabel(line) != "PGM / RUN BY / DATE")
        {
            LOG_ERROR("{}: Not a valid RINEX NAV file. Could not read 'PGM / RUN BY / DATE' line.", nameId());
            return FileReader::FileType::NONE;
        }

        // ----------------------------------------- END OF HEADER -------------------------------------------
        while (std::getline(filestreamHeader, line) && !filestreamHeader.eof())
        {
            str::rtrim(line);
            if (extHeaderLabel(line) == "END OF HEADER")
            {
                return FileReader::FileType::ASCII;
            }
        }
        LOG_ERROR("{}: Not a valid RINEX NAV file. Could not read 'END OF HEADER' line.", nameId());
        return FileReader::FileType::NONE;
    }

    LOG_ERROR("{}: Could not determine file type because file could not be opened '{}' line.", nameId(), filepath.string());
    return FileReader::FileType::NONE;
}

std::string_view extHeaderLabel(const std::string& line)
{
    return line.size() >= 60 ? str::trim_copy(std::string_view(line).substr(60, 20))
                             : std::string_view{};
};

void RinexNavFile::executeHeaderParser(double version)
{
    if (version < 3.0)
    {
        parseHeader2();
    }
    else if (version < 4.0)
    {
        parseHeader3();
    }
    else if (version == 4.0)
    {
        parseHeader4();
    }
    else
    {
        LOG_ERROR("{}: Unsupported RINEX version {}.", nameId(), version);
        // return nullptr;
    }
}

void RinexNavFile::executeOrbitParser(double version)
{
    if (version < 3.0)
    {
        parseOrbit2();
    }
    else if (version < 4.0)
    {
        parseOrbit3();
    }
    else if (version == 4.0)
    {
        parseOrbit4();
    }
    else
    {
        LOG_ERROR("{}: Unsupported RINEX version {}.", nameId(), version);
        // return nullptr;
    }
}

void RinexNavFile::parseHeader2()
{
    std::string line;
    // --------------------------------------- RINEX VERSION / TYPE ------------------------------------------
    while (line.find("RINEX VERSION / TYPE") == std::string::npos && !eof())
    {
        getline(line);
    }
    SatelliteSystem satSys = SatSys_None;
    LOG_DEBUG("{}: Version: {:3.2f}", nameId(), _version); // FORMAT: F9.2,11X
    if ((line.substr(20, 15) == "N: GPS NAV DATA" || line.substr(20, 15) == "NAVIGATION DATA")
        && (line.substr(40, 3) == "GPS" || str::trim_copy(line.substr(40, 3)).empty()))
    {
        LOG_DEBUG("{}: SatSys : {}", nameId(), "G: GPS"); // FORMAT: A1,19X
        satSys = GPS;
        _gnssNavInfo.satelliteSystems |= satSys;
    }
    else if (line.substr(20, 16) == "GLONASS NAV DATA")
    {
        LOG_DEBUG("{}: SatSys : {}", nameId(), "R: GLONASS"); // FORMAT: A1,19X
        satSys = GLO;
        _gnssNavInfo.satelliteSystems |= satSys;
    }
    // #######################################################################################################
    while (getline(line) && !eof())
    {
        str::rtrim(line);
        auto headerLabel = extHeaderLabel(line);
        if (headerLabel == "PGM / RUN BY / DATE")
        {
            // Name of program creating current file
            LOG_DATA("{}: Program: {}", nameId(), str::trim_copy(line.substr(0, 20)));  // FORMAT: A20
                                                                                        // Name of agency creating current file
            LOG_DATA("{}: Run by : {}", nameId(), str::trim_copy(line.substr(20, 20))); // FORMAT: A20
                                                                                        // Date and time of file creation
            LOG_DATA("{}: Date   : {}", nameId(), str::trim_copy(line.substr(40, 20))); // FORMAT: A20
        }
        else if (headerLabel == "COMMENT")
        {
            LOG_DATA("{}: Comment: {}", nameId(), line.substr(0, 60)); // FORMAT: A60
        }
        else if (headerLabel == "IONOSPHERIC CORR")
        {
            auto correctionType = str::trim_copy(line.substr(0, 4)); // FORMAT: A4,1X,
            std::array<double, 4> values{};
            values[0] = str::stod(str::replaceAll_copy(line.substr(5, 12), "d", "e", str::IgnoreCase), 0.0); // FORMAT: 4D12.4
            values[1] = str::stod(str::replaceAll_copy(line.substr(17, 12), "d", "e", str::IgnoreCase), 0.0);
            values[2] = str::stod(str::replaceAll_copy(line.substr(29, 12), "d", "e", str::IgnoreCase), 0.0);
            values[3] = str::stod(str::replaceAll_copy(line.substr(41, 12), "d", "e", str::IgnoreCase), 0.0);

            auto satSys = SatelliteSystem::fromString(correctionType.substr(0, 3));
            IonosphericCorrections::AlphaBeta alphaBeta = (correctionType.size() == 4 && correctionType.at(3) == 'B')
                                                              ? IonosphericCorrections::Beta
                                                              : IonosphericCorrections::Alpha;
            _gnssNavInfo.ionosphericCorrections.insert(satSys, alphaBeta, values);
            LOG_DATA("{}: Ionospheric Correction: {}-{}: [{}]", nameId(),
                     std::string(satSys), alphaBeta == IonosphericCorrections::Alpha ? "Alpha" : "Beta",
                     fmt::join(values.begin(), values.end(), ", "));
        }
        else if (headerLabel == "ION ALPHA" || headerLabel == "ION BETA")
        {
            auto correctionType = headerLabel.substr(4, 1);
            auto valuesStr = str::split_wo_empty(str::replaceAll_copy(line.substr(4, 60 - 4), "d", "e", str::IgnoreCase), " ");
            std::array<double, 4> values{};
            for (size_t i = 0; i < valuesStr.size(); i++)
            {
                values.at(i) = std::stod(valuesStr.at(i));
            }
            IonosphericCorrections::AlphaBeta alphaBeta = correctionType == "B" ? IonosphericCorrections::Beta
                                                                                : IonosphericCorrections::Alpha;
            _gnssNavInfo.ionosphericCorrections.insert(satSys, alphaBeta, values);
            LOG_DATA("{}: Ionospheric Correction: {}-{}: [{}]", nameId(),
                     std::string(satSys), alphaBeta == IonosphericCorrections::Alpha ? "Alpha" : "Beta",
                     fmt::join(values.begin(), values.end(), ", "));
        }
        else if (headerLabel == "TIME SYSTEM CORR"
                 || headerLabel == "CORR TO SYSTEM TIME")
        {
            auto tau_c = str::stod(str::replaceAll_copy(line.substr(21, 19), "d", "e", str::IgnoreCase), 0.0);
            LOG_DATA("{}:     tau_c {}", nameId(), tau_c);
            if (tau_c != 0)
            {
                _gnssNavInfo.timeSysCorr[{ satSys.getTimeSystem(), UTC }] = { tau_c, 0 };
            }
        }
        else if (headerLabel == "DELTA-UTC: A0,A1,T,W")
        {
            auto a0 = std::stod(str::replaceAll_copy(line.substr(3, 19), "d", "e", str::IgnoreCase));
            LOG_DATA("{}:     a0 {}", nameId(), a0);
            auto a1 = std::stod(str::replaceAll_copy(line.substr(22, 19), "d", "e", str::IgnoreCase));
            LOG_DATA("{}:     a1 {}", nameId(), a1);

            if (a0 != 0 || a1 != 0)
            {
                _gnssNavInfo.timeSysCorr[{ satSys.getTimeSystem(), UTC }] = { a0, a1 };
            }
        }
        else if (headerLabel == "LEAP SECONDS")
        {
            LOG_DATA("{}: Leap seconds: {}", nameId(), std::stoi(line.substr(0, 6)));
        }
        else if (headerLabel == "END OF HEADER")
        {
            break;
        }
    }
}

void RinexNavFile::parseHeader3()
{
    std::string line;
    // --------------------------------------- RINEX VERSION / TYPE ------------------------------------------
    while (line.find("RINEX VERSION / TYPE") == std::string::npos && !eof())
    {
        getline(line);
    }
    SatelliteSystem satSys = SatSys_None;
    LOG_DEBUG("{}: Version: {:3.2f}", nameId(), _version);                       // FORMAT: F9.2,11X
    LOG_DEBUG("{}: SatSys : {}", nameId(), str::trim_copy(line.substr(40, 20))); // FORMAT: A1,19X
    // #######################################################################################################
    while (getline(line) && !eof())
    {
        str::rtrim(line);
        auto headerLabel = extHeaderLabel(line);
        if (headerLabel == "PGM / RUN BY / DATE")
        {
            // Name of program creating current file
            LOG_DATA("{}: Program: {}", nameId(), str::trim_copy(line.substr(0, 20)));  // FORMAT: A20
                                                                                        // Name of agency creating current file
            LOG_DATA("{}: Run by : {}", nameId(), str::trim_copy(line.substr(20, 20))); // FORMAT: A20
                                                                                        // Date and time of file creation
            LOG_DATA("{}: Date   : {}", nameId(), str::trim_copy(line.substr(40, 20))); // FORMAT: A20
        }
        else if (headerLabel == "COMMENT")
        {
            LOG_DATA("{}: Comment: {}", nameId(), line.substr(0, 60)); // FORMAT: A60
        }
        else if (headerLabel == "IONOSPHERIC CORR")
        {
            auto correctionType = str::trim_copy(line.substr(0, 4)); // FORMAT: A4,1X,
            std::array<double, 4> values{};
            values[0] = str::stod(str::replaceAll_copy(line.substr(5, 12), "d", "e", str::IgnoreCase), 0.0); // FORMAT: 4D12.4
            values[1] = str::stod(str::replaceAll_copy(line.substr(17, 12), "d", "e", str::IgnoreCase), 0.0);
            values[2] = str::stod(str::replaceAll_copy(line.substr(29, 12), "d", "e", str::IgnoreCase), 0.0);
            values[3] = str::stod(str::replaceAll_copy(line.substr(41, 12), "d", "e", str::IgnoreCase), 0.0);

            auto satSys = SatelliteSystem::fromString(correctionType.substr(0, 3));
            IonosphericCorrections::AlphaBeta alphaBeta = (correctionType.size() == 4 && correctionType.at(3) == 'B')
                                                              ? IonosphericCorrections::Beta
                                                              : IonosphericCorrections::Alpha;
            _gnssNavInfo.ionosphericCorrections.insert(satSys, alphaBeta, values);
            LOG_DATA("{}: Ionospheric Correction: {}-{}: [{}]", nameId(),
                     std::string(satSys), alphaBeta == IonosphericCorrections::Alpha ? "Alpha" : "Beta",
                     fmt::join(values.begin(), values.end(), ", "));
        }
        else if (headerLabel == "ION ALPHA" || headerLabel == "ION BETA")
        {
            auto correctionType = headerLabel.substr(4, 1);
            auto valuesStr = str::split_wo_empty(str::replaceAll_copy(line.substr(4, 60 - 4), "d", "e", str::IgnoreCase), " ");
            std::array<double, 4> values{};
            for (size_t i = 0; i < valuesStr.size(); i++)
            {
                values.at(i) = std::stod(valuesStr.at(i));
            }
            IonosphericCorrections::AlphaBeta alphaBeta = correctionType == "B" ? IonosphericCorrections::Beta
                                                                                : IonosphericCorrections::Alpha;
            _gnssNavInfo.ionosphericCorrections.insert(satSys, alphaBeta, values);
            LOG_DATA("{}: Ionospheric Correction: {}-{}: [{}]", nameId(),
                     std::string(satSys), alphaBeta == IonosphericCorrections::Alpha ? "Alpha" : "Beta",
                     fmt::join(values.begin(), values.end(), ", "));
        }
        else if (headerLabel == "TIME SYSTEM CORR"
                 || headerLabel == "CORR TO SYSTEM TIME")
        {
            auto correctionType = str::trim_copy(line.substr(0, 4)); // FORMAT: A4,1X,
            LOG_DATA("{}: Time System Correction: {}", nameId(), correctionType);

            std::array<size_t, 2> x{ 5, 22 };
            std::array<size_t, 2> n{ 17, 16 };

            if (correctionType == "SBUT")
            {
                x = { 7, 26 };
                n = { 19, 19 };
            }
            else if (correctionType == "GLUT")
            {
                x = { 5, 24 };
                n = { 19, 19 };
            }

            auto a0 = str::stod(str::replaceAll_copy(line.substr(x[0], n[0]), "d", "e", str::IgnoreCase), std::nan(""));
            LOG_DATA("{}:     a0 {}", nameId(), a0);
            auto a1 = str::stod(str::replaceAll_copy(line.substr(x[1], n[1]), "d", "e", str::IgnoreCase), std::nan(""));
            LOG_DATA("{}:     a1 {}", nameId(), a1);

            if (!std::isnan(a0) && !std::isnan(a1))
            {
                std::pair<TimeSystem, TimeSystem> timeSystems;
                if (correctionType == "GPUT") { timeSystems = { GPST, UTC }; }
                else if (correctionType == "GLUT") { timeSystems = { GLNT, UTC }; }
                else if (correctionType == "GAUT") { timeSystems = { GST, UTC }; }
                else if (correctionType == "BDUT") { timeSystems = { BDT, UTC }; }
                else if (correctionType == "QZUT") { timeSystems = { QZSST, UTC }; }
                else if (correctionType == "IRUT") { timeSystems = { IRNSST, UTC }; }
                // else if (correctionType == "SBUT") { timeSystems = { SBAST, UTC }; }
                else if (correctionType == "GLGP") { timeSystems = { GLNT, GPST }; }
                else if (correctionType == "GAGP") { timeSystems = { GST, GPST }; }
                else if (correctionType == "GPGA") { timeSystems = { GST, GPST }; } // Pre RINEX 3.04
                else if (correctionType == "QZGP") { timeSystems = { QZSST, GPST }; }
                else if (correctionType == "IRGP") { timeSystems = { IRNSST, GPST }; }
                else
                {
                    LOG_TRACE("{}:     Time System Correction '{}' not implemented yet", nameId(), correctionType);
                    continue;
                }
                _gnssNavInfo.timeSysCorr[timeSystems] = { a0, a1 };
            }
        }
        else if (headerLabel == "DELTA-UTC: A0,A1,T,W")
        {
            auto a0 = std::stod(str::replaceAll_copy(line.substr(3, 19), "d", "e", str::IgnoreCase));
            LOG_DATA("{}:     a0 {}", nameId(), a0);
            auto a1 = std::stod(str::replaceAll_copy(line.substr(22, 19), "d", "e", str::IgnoreCase));
            LOG_DATA("{}:     a1 {}", nameId(), a1);

            if (a0 != 0 || a1 != 0)
            {
                _gnssNavInfo.timeSysCorr[{ satSys.getTimeSystem(), UTC }] = { a0, a1 };
            }
        }
        else if (headerLabel == "LEAP SECONDS")
        {
            LOG_DATA("{}: Leap seconds: {}", nameId(), std::stoi(line.substr(0, 6)));
        }
        else if (headerLabel == "END OF HEADER")
        {
            break;
        }
    }
}

void RinexNavFile::parseHeader4()
{
    std::string line;
    // --------------------------------------- RINEX VERSION / TYPE ------------------------------------------
    while (line.find("RINEX VERSION / TYPE") == std::string::npos && !eof())
    {
        getline(line);
    }
    // SatelliteSystem satSys = SatSys_None;
    LOG_DEBUG("{}: Version: {:3.2f}", nameId(), _version);                       // FORMAT: F9.2,11X
    LOG_DEBUG("{}: SatSys : {}", nameId(), str::trim_copy(line.substr(40, 20))); // FORMAT: A1,19X
    // #######################################################################################################
    while (getline(line) && !eof())
    {
        str::rtrim(line);
        auto headerLabel = extHeaderLabel(line);
        if (headerLabel == "PGM / RUN BY / DATE")
        {
            // Name of program creating current file
            LOG_DATA("{}: Program: {}", nameId(), str::trim_copy(line.substr(0, 20)));  // FORMAT: A20
                                                                                        // Name of agency creating current file
            LOG_DATA("{}: Run by : {}", nameId(), str::trim_copy(line.substr(20, 20))); // FORMAT: A20
                                                                                        // Date and time of file creation
            LOG_DATA("{}: Date   : {}", nameId(), str::trim_copy(line.substr(40, 20))); // FORMAT: A20
        }
        else if (headerLabel == "COMMENT")
        {
            LOG_DATA("{}: Comment: {}", nameId(), line.substr(0, 60)); // FORMAT: A60
        }
        else if (headerLabel == "REC # / TYPE / VERS") // Not in mixed station files
        {
            // Receiver number of station
            LOG_DATA("{}: Receiver number   : {}", nameId(), str::trim_copy(line.substr(0, 20))); // FORMAT: A20
            // Receiver type of station
            LOG_DATA("{}: Receiver type   : {}", nameId(), str::trim_copy(line.substr(20, 20))); // FORMAT: A20
            // Receiver software version of station
            LOG_DATA("{}: Receiver version   : {}", nameId(), str::trim_copy(line.substr(40, 20))); // FORMAT: A20
        }
        else if (headerLabel == "Merged File")
        {
            if (!str::trim_copy(line.substr(0, 9)).empty())
            {
                // Numbers of files merged
                LOG_DATA("{}: Number of files merged   : {}", nameId(), str::trim_copy(line.substr(0, 9))); // FORMAT: I9
            }
        }
        else if (headerLabel == "DOI")
        {
            // Digital Object Identifier
            LOG_DATA("{}: Comment: {}", nameId(), line.substr(0, 60)); // FORMAT: A60
        }
        else if (headerLabel == "LICENSE OF USE")
        {
            // License of Use
            LOG_DATA("{}: Comment: {}", nameId(), line.substr(0, 60)); // FORMAT: A60
        }
        else if (headerLabel == "LICENSE OF USE")
        {
            // License of Use
            LOG_DATA("{}: Comment: {}", nameId(), line.substr(0, 60)); // FORMAT: A60
        }
        else if (headerLabel == "LEAP SECONDS")
        {
            LOG_DATA("{}: Leap seconds: {}", nameId(), std::stoi(line.substr(0, 6))); // I6
        }
        else if (headerLabel == "END OF HEADER")
        {
            break;
        }
    }
}

void RinexNavFile::parseOrbit2()
{
    std::string line;

    try
    {
        while (getline(line) && !eof())
        {
            if (line.size() > 82) { return abortReading(); } // 80 + \n\r

            // -------------------------------------- SV / EPOCH / SV CLK ----------------------------------------

            // Satellite system (G), sat number (PRN) - A1,I2.2,
            // Satellite system (E), satellite number - A1,I2.2,
            // Satellite system (R), satellite number (slot number in sat. constellation) - A1,I2.2,
            // Satellite system (J), Satellite PRN-192 - A1,I2,
            // Satellite system (C), sat number (PRN) - A1,I2.2,
            // Satellite system (S), satellite number (slot number in sat. constellation) - A1,I2.2,
            // Satellite system (I), sat number (PRN) - A1,I2.2,
            SatelliteSystem satSys = SatSys_None;

            // Offset for RINEX 2.xx versions. Has only 3 leading spaces
            satSys = _gnssNavInfo.satelliteSystems;

            if (satSys == SatSys_None) { continue; } // To intercept comment lines

            auto satNumStr = line.substr(0, 2);
            if (satNumStr == "  ") { continue; }

            auto satNum = static_cast<uint8_t>(str::stoi(satNumStr, 0));
            if (satNum == 0) { continue; }
            // Epoch: Toc - Time of Clock (GPS) year (4 digits) - 1X,I4,
            // month, day, hour, minute, second - 5(1X,I2.2),
            auto timeSplit = str::split_wo_empty(line.substr(2, 20), " ");
            auto timeSystem = satSys == GLO ? UTC : satSys.getTimeSystem();

            int year = std::stoi(timeSplit.at(0));

            // RINEX 2.xx has 2-Digit Years
            year += (year >= 80 ? 1900 : 2000);

            InsTime epoch{ static_cast<uint16_t>(year),
                           static_cast<uint16_t>(std::stoi(timeSplit.at(1))),
                           static_cast<uint16_t>(std::stoi(timeSplit.at(2))),
                           static_cast<uint16_t>(std::stoi(timeSplit.at(3))),
                           static_cast<uint16_t>(std::stoi(timeSplit.at(4))),
                           std::stold(timeSplit.at(5)),
                           timeSystem };

            LOG_DATA("{}: {}-{} {} (read as {} time)", nameId(), satSys, satNum, epoch.toYMDHMS(), std::string(timeSystem));

            if (satSys == GPS || satSys == GAL || satSys == QZSS || satSys == BDS) // NOLINT(misc-redundant-expression) // bugged warning
            {
                // Polynomial coefficients for clock correction
                std::array<double, 3> a{};
                // SV clock bias [seconds]
                a[0] = std::stod(str::replaceAll_copy(line.substr(22, 19), "d", "e", str::IgnoreCase));
                // SV clock drift [sec/sec]
                a[1] = std::stod(str::replaceAll_copy(line.substr(41, 19), "d", "e", str::IgnoreCase));
                // SV clock drift rate [sec/sec^2]
                a[2] = std::stod(str::replaceAll_copy(line.substr(60, 19), "d", "e", str::IgnoreCase));

                LOG_DATA("{}:    clkBias {}, clkDrift {}, clkDriftRate {}", nameId(), a[0], a[1], a[2]);

                // ------------------------------------ BROADCAST ORBIT - 1 --------------------------------------
                getline(line);
                if (line.size() > 82) { return abortReading(); } // 80 + \n\r

                // GPS/QZSS: Issue of Data, Ephemeris (IODE)
                // GAL:      IODnav Issue of Data of the nav batch
                // BDS:      AODE Age of Data, Ephemeris (as specified in BeiDou ICD Table Section 5.2.4.11 Table 5-8)
                //           and field range is: 0-31.
                // IRNSS:    IODEC Issue of Data, Ephemeris and Clock
                auto IODE_IODnav_AODE_IODEC = static_cast<size_t>(std::stod(str::replaceAll_copy(line.substr(3, 19), "d", "e", str::IgnoreCase)));
                // Crs (meters)
                double Crs = std::stod(str::replaceAll_copy(line.substr(22, 19), "d", "e", str::IgnoreCase));
                // Delta n (radians/sec)
                double delta_n = std::stod(str::replaceAll_copy(line.substr(41, 19), "d", "e", str::IgnoreCase));
                // M0 (radians)
                double M_0 = std::stod(str::replaceAll_copy(line.substr(60, 19), "d", "e", str::IgnoreCase));

                LOG_DATA("{}:    IODE_IODnav_AODE_IODEC {}, Crs {}, Delta_n {}, M0 {}", nameId(), IODE_IODnav_AODE_IODEC, Crs, delta_n, M_0);

                // ------------------------------------ BROADCAST ORBIT - 2 --------------------------------------
                getline(line);
                if (line.size() > 82) { return abortReading(); } // 80 + \n\r

                // Cuc (radians)
                double Cuc = std::stod(str::replaceAll_copy(line.substr(3, 19), "d", "e", str::IgnoreCase));
                // e Eccentricity
                double e = std::stod(str::replaceAll_copy(line.substr(22, 19), "d", "e", str::IgnoreCase));
                // Cus (radians)
                double Cus = std::stod(str::replaceAll_copy(line.substr(41, 19), "d", "e", str::IgnoreCase));
                // sqrt(A) (sqrt(m))
                double sqrt_A = std::stod(str::replaceAll_copy(line.substr(60, 19), "d", "e", str::IgnoreCase));

                LOG_DATA("{}:    Cuc {}, e {}, Cus {}, sqrt_A {}", nameId(), Cuc, e, Cus, sqrt_A);

                // ------------------------------------ BROADCAST ORBIT - 3 --------------------------------------
                getline(line);
                if (line.size() > 82) { return abortReading(); } // 80 + \n\r

                // Toe Time of Ephemeris (sec of GPS week)
                double toeSec = std::stod(str::replaceAll_copy(line.substr(3, 19), "d", "e", str::IgnoreCase));
                // Cic (radians)
                double Cic = std::stod(str::replaceAll_copy(line.substr(22, 19), "d", "e", str::IgnoreCase));
                // OMEGA0 (radians)
                double Omega_0 = std::stod(str::replaceAll_copy(line.substr(41, 19), "d", "e", str::IgnoreCase));
                // Cis (radians)
                double Cis = std::stod(str::replaceAll_copy(line.substr(60, 19), "d", "e", str::IgnoreCase));

                LOG_DATA("{}:    toe {}, Cic {}, Omega_0 {}, Cis {}", nameId(), toeSec, Cic, Omega_0, Cis);

                // ------------------------------------ BROADCAST ORBIT - 4 --------------------------------------
                getline(line);
                if (line.size() > 82) { return abortReading(); } // 80 + \n\r

                // i0 (radians)
                double i_0 = std::stod(str::replaceAll_copy(line.substr(3, 19), "d", "e", str::IgnoreCase));
                // Crc (meters)
                double Crc = std::stod(str::replaceAll_copy(line.substr(22, 19), "d", "e", str::IgnoreCase));
                // omega (radians)
                double omega = std::stod(str::replaceAll_copy(line.substr(41, 19), "d", "e", str::IgnoreCase));
                // OMEGA DOT (radians/sec)
                double Omega_dot = std::stod(str::replaceAll_copy(line.substr(60, 19), "d", "e", str::IgnoreCase));

                LOG_DATA("{}:    i_0 {}, Crc {}, omega {}, Omega_dot {}", nameId(), i_0, Crc, omega, Omega_dot);

                // ------------------------------------ BROADCAST ORBIT - 5 --------------------------------------
                getline(line);
                if (line.size() > 82) { return abortReading(); } // 80 + \n\r

                // IDOT (radians/sec)
                double i_dot = std::stod(str::replaceAll_copy(line.substr(3, 19), "d", "e", str::IgnoreCase));
                // GPS:       Codes on L2 channel
                // QZSS:      Codes on L2 channel (fixed to 2, see IS-QZSS-PNT 4.1.2.7)
                // GAL:       Data sources (FLOAT --> INTEGER)
                //            Bit 0 set: I/NAV E1-B
                //            Bit 1 set: F/NAV E5a-I
                //            Bit 2 set: I/NAV E5b-I
                //            Bits 0 and 2 : Both can be set if the navigation messages were merged, however, bits 0-2
                //                           cannot all be set, as the I/NAV and F/NAV messages contain different information
                //            Bit 3 reserved for Galileo internal use
                //            Bit 4 reserved for Galileo internal use
                //            Bit 8 set: af0-af2, Toc, SISA are for E5a,E1
                //            Bit 9 set: af0-af2, Toc, SISA are for E5b,E1
                //            Bits 8-9 : exclusive (only one bit can be set)
                // BDS/IRNSS: Spare
                auto codesOnL2Channel_dataSources = static_cast<uint16_t>(str::stod(str::replaceAll_copy(line.substr(22, 19), "d", "e", str::IgnoreCase), 0));

                // GPS Week # (to go with TOE) Continuous number, not mod(1024)!
                auto gpsWeek = static_cast<int32_t>(str::stod(str::replaceAll_copy(line.substr(41, 19), "d", "e", str::IgnoreCase), epoch.toGPSweekTow().gpsWeek));
                if (satSys == BDS) { gpsWeek += InsTimeUtil::DIFF_BDT_WEEK_TO_GPST_WEEK; }
                auto toe = InsTime(0, gpsWeek, toeSec, satSys.getTimeSystem());

                // GPS:  L2 P data flag
                // QZSS: L2P data flag set to 1 since QZSS does not track L2P
                // GAL/BDS/IRNSS:  Spare
                bool L2PdataFlag = static_cast<bool>(str::stod(str::replaceAll_copy(line.substr(60, 19), "d", "e", str::IgnoreCase), 0.0));

                LOG_DATA("{}:    i_dot {}, codesOnL2Channel/dataSources {} ({}), gpsWeek {}, L2PdataFlag {}", nameId(),
                         i_dot, codesOnL2Channel_dataSources, std::bitset<10>(codesOnL2Channel_dataSources).to_string(), gpsWeek, L2PdataFlag);

                // ------------------------------------ BROADCAST ORBIT - 6 --------------------------------------
                getline(line);
                if (line.size() > 82) { return abortReading(); } // 80 + \n\r

                // GPS:  SV accuracy (meters) See GPS ICD 200H Section 20.3.3.3.1.3 use specified equations to define
                //       nominal values, N = 0-6: use 2(1+N/2) (round to one decimal place i.e. 2.8, 5.7 and 11.3) ,
                //       N= 7-15:use 2 (N-2), 8192 specifies use at own risk
                // QZSS: SV accuracy (meters) (IS -QZSS-PNT, Section 5.4.3.1) which refers to: IS GPS 200H Section 20.3.3.3.1.3
                //       use specified equations to define nominal values, N = 0-6: use 2(1+N/2) (round to one decimal
                //       place i.e. 2.8, 5.7 and 11.3) , N= 7-15:use 2 (N-2), 8192 specifies use at own risk
                // IRNSS: User Range Accuracy(m), See IRNSS ICD Section 6.2.1.4 , use specified equations to define
                //        nominal values, N = 0-6: use 2(1+N/2) (round to one decimal place i.e. 2.8, 5.7 and 11.3) ,
                //        N= 7-15:use 2 (N-2), 8192 specifies use at own risk
                double signalAccuracy = std::stod(str::replaceAll_copy(line.substr(3, 19), "d", "e", str::IgnoreCase));
                // GPS/QZSS: SV health (bits 17-22 w 3 sf 1)
                // GAL:      SV health (FLOAT converted to INTEGER) See Galileo ICD Section 5.1.9.3
                //           Bit 0: E1B DVS       Bits 1-2: E1B HS          Bit 3: E5a DVS
                //           Bits 4-5: E5a HS     Bit 6: E5b DVS            Bits 7-8: E5b HS
                // BDS:      SatH1
                // IRNSS:    Health (Sub frame 1,bits 155(most significant) and 156(least significant)),
                //           where 0 = L5 and S healthy, 1 = L5 healthy and S unhealthy, 2= L5 unhealthy
                //           and S healthy, 3= both L5 and S unhealthy
                auto svHealth = static_cast<uint16_t>(std::stod(str::replaceAll_copy(line.substr(22, 19), "d", "e", str::IgnoreCase)));
                // GPS:  TGD (seconds)
                // QZSS: TGD (seconds) The QZSS ICD specifies a do not use bit pattern "10000000" this condition is represented by a blank field.
                // GAL:  BGD E5a/E1 (seconds)
                // BDS:  TGD1 B1/B3 (seconds)
                double tgd_bgd5a_TGD1 = str::stod(str::replaceAll_copy(line.substr(41, 19), "d", "e", str::IgnoreCase), 0.0);

                // GPS/QZSS: IODC Issue of Data, Clock
                // GAL:      BGD E5b/E1 (seconds)
                // BDS:      TGD2 B2/B3 (seconds)
                // IRNSS:    Blank
                double IODC_bgd5b_TGD2 = str::stod(str::replaceAll_copy(line.substr(60, 19), "d", "e", str::IgnoreCase), 0.0);

                LOG_DATA("{}:    svAccuracy {}, svHealth {}, TGD {}, IODC {}", nameId(), signalAccuracy, svHealth, tgd_bgd5a_TGD1, IODC_bgd5b_TGD2);

                // ------------------------------------ BROADCAST ORBIT - 7 --------------------------------------
                getline(line);
                if (line.size() > 82) { return abortReading(); } // 80 + \n\r

                // Transmission time of message (sec of GPS week, derived e.g.from Z-count in Hand Over Word (HOW))
                [[maybe_unused]] auto transmissionTime = str::stod(str::replaceAll_copy(line.substr(3, 19), "d", "e", str::IgnoreCase), 0.0);
                // GPS/QZSS: Fit Interval in hours see section 6.11. (BNK).
                // GAL:      Spare
                // BDS:      AODC Age of Data Clock (as specified in BeiDou ICD Table Section 5.2.4.9 Table 5-6) and field range is: 0-31.
                auto fitInterval_AODC = str::stod(str::replaceAll_copy(line.substr(22, 19), "d", "e", str::IgnoreCase), 0.0);
                // Spare
                // std::stod(str::replaceAll_copy(line.substr(41, 19), "d", "e", str::IgnoreCase));
                // Spare
                // std::stod(str::replaceAll_copy(line.substr(60, 19), "d", "e", str::IgnoreCase));

                LOG_DATA("{}:    transmissionTime {}, fitInterval/AODC {}", nameId(), transmissionTime, fitInterval_AODC);

                if (satSys == GPS)
                {
                    _gnssNavInfo.addSatelliteNavData({ satSys, satNum }, std::make_shared<GPSEphemeris>(epoch, toe,
                                                                                                        IODE_IODnav_AODE_IODEC, IODC_bgd5b_TGD2, a,
                                                                                                        sqrt_A, e, i_0, Omega_0, omega, M_0,
                                                                                                        delta_n, Omega_dot, i_dot, Cus, Cuc,
                                                                                                        Cis, Cic, Crs, Crc,
                                                                                                        signalAccuracy, svHealth,
                                                                                                        codesOnL2Channel_dataSources, L2PdataFlag,
                                                                                                        tgd_bgd5a_TGD1, fitInterval_AODC));
                }
                else if (satSys == GAL)
                {
                    // The same satellite can appear multiple times with different dataSource bits
                    // We want to prefer 'I/NAV E1-B' (Bit 0 set) over 'F/NAV E5a-I' (Bit 1 set) or 'I/NAV E5b-I' (Bit 2 set)

                    if (_gnssNavInfo.satellites().contains({ satSys, satNum }) // We have this satellite already
                        && !std::bitset<10>(codesOnL2Channel_dataSources)[0])  // This message is not 'I/NAV E1-B'
                    {
                        const auto& navData = _gnssNavInfo.satellites().at({ satSys, satNum }).getNavigationData();
                        auto existingEph = std::find_if(navData.begin(), navData.end(),
                                                        [&](const std::shared_ptr<SatNavData>& satNavData) {
                                                            return satNavData->type == SatNavData::GalileoEphemeris && satNavData->refTime == epoch
                                                                   && std::dynamic_pointer_cast<GalileoEphemeris>(satNavData)->dataSource[0];
                                                        });
                        if (existingEph != navData.end()) // There is already a 'I/NAV E1-B' message
                        {
                            LOG_DATA("{}:    Skipping ephemeris data because of dataSource priority", nameId());
                            continue;
                        }
                    }

                    GalileoEphemeris::SvHealth health = { .E5a_DataValidityStatus = static_cast<GalileoEphemeris::SvHealth::DataValidityStatus>((svHealth & 0b000001000) >> 3),
                                                          .E5b_DataValidityStatus = static_cast<GalileoEphemeris::SvHealth::DataValidityStatus>((svHealth & 0b001000000) >> 6),
                                                          .E1B_DataValidityStatus = static_cast<GalileoEphemeris::SvHealth::DataValidityStatus>((svHealth & 0b000000001) >> 0),
                                                          .E5a_SignalHealthStatus = static_cast<GalileoEphemeris::SvHealth::SignalHealthStatus>((svHealth & 0b000110000) >> 4),
                                                          .E5b_SignalHealthStatus = static_cast<GalileoEphemeris::SvHealth::SignalHealthStatus>((svHealth & 0b110000000) >> 7),
                                                          .E1B_SignalHealthStatus = static_cast<GalileoEphemeris::SvHealth::SignalHealthStatus>((svHealth & 0b000000110) >> 1) };
                    _gnssNavInfo.addSatelliteNavData({ satSys, satNum }, std::make_shared<GalileoEphemeris>(epoch, toe, IODE_IODnav_AODE_IODEC, a,
                                                                                                            sqrt_A, e, i_0, Omega_0, omega, M_0,
                                                                                                            delta_n, Omega_dot, i_dot, Cus, Cuc,
                                                                                                            Cis, Cic, Crs, Crc,
                                                                                                            codesOnL2Channel_dataSources, signalAccuracy, health,
                                                                                                            tgd_bgd5a_TGD1, IODC_bgd5b_TGD2));
                }
                else if (satSys == BDS)
                {
                    _gnssNavInfo.addSatelliteNavData({ satSys, satNum }, std::make_shared<BDSEphemeris>(epoch, toe,
                                                                                                        IODE_IODnav_AODE_IODEC, fitInterval_AODC, a,
                                                                                                        sqrt_A, e, i_0, Omega_0, omega, M_0,
                                                                                                        delta_n, Omega_dot, i_dot, Cus, Cuc,
                                                                                                        Cis, Cic, Crs, Crc,
                                                                                                        signalAccuracy, svHealth,
                                                                                                        tgd_bgd5a_TGD1, IODC_bgd5b_TGD2));
                }
                else if (satSys == QZSS)
                {
                    LOG_WARN("QZSS is not yet supported. Therefore the Navigation file data will be skipped.");
                }
            }
            else if (satSys == GLO || satSys == SBAS) // NOLINT(misc-redundant-expression) // bugged warning
            {
                // TODO: Offset

                Eigen::Vector3d pos;
                Eigen::Vector3d vel;
                Eigen::Vector3d accelLuniSolar;

                double tau_c{};
                // Coefficient of linear polynomial of time system difference [s]
                if (_gnssNavInfo.timeSysCorr.contains({ satSys.getTimeSystem(), UTC }))
                {
                    tau_c = _gnssNavInfo.timeSysCorr.at({ satSys.getTimeSystem(), UTC }).a0;
                }

                // GLO:  SV clock bias (sec) (-TauN)
                // SBAS: SV clock bias (sec) (aGf0)
                double m_tau_n = std::stod(str::replaceAll_copy(line.substr(22, 19), "d", "e", str::IgnoreCase));
                // GLO:  SV relative frequency bias (+GammaN)
                // SBAS: SV relative frequency bias (aGf1)
                double gamma_n = std::stod(str::replaceAll_copy(line.substr(41, 19), "d", "e", str::IgnoreCase));
                // GLO:  Message frame time (tk+nd*86400) in seconds of the UTC week
                // SBAS: Transmission time of message (start of the message) in GPS seconds of the week
                [[maybe_unused]] auto msgFrameTime = std::stod(str::replaceAll_copy(line.substr(60, 19), "d", "e", str::IgnoreCase));
                LOG_DATA("{}:    clkBias {}, relFreqBias {}, msgFrameTime {}", nameId(), m_tau_n, gamma_n, msgFrameTime);

                // ------------------------------------ BROADCAST ORBIT - 1 --------------------------------------
                getline(line);
                if (line.size() > 82) { return abortReading(); } // 80 + \n\r

                // Satellite position X (km)
                pos.x() = std::stod(str::replaceAll_copy(line.substr(3, 19), "d", "e", str::IgnoreCase)) * 1e3;
                // velocity X dot (km/sec)
                vel.x() = std::stod(str::replaceAll_copy(line.substr(22, 19), "d", "e", str::IgnoreCase)) * 1e3;
                // X acceleration (km/sec2)
                accelLuniSolar.x() = std::stod(str::replaceAll_copy(line.substr(41, 19), "d", "e", str::IgnoreCase)) * 1e3;
                // GLO:  health (0=healthy, 1=unhealthy) (MSB of 3-bit Bn)
                // SBAS: Health: SBAS: See Section 8.3.3 bit mask for: health, health availability and User Range Accuracy.
                [[maybe_unused]] auto health = std::stod(str::replaceAll_copy(line.substr(60, 19), "d", "e", str::IgnoreCase));

                LOG_DATA("{}:    satPosX {}, velX {}, accelX {}, health {}", nameId(), pos.x(), vel.x(), accelLuniSolar.x(), health);

                // ------------------------------------ BROADCAST ORBIT - 2 --------------------------------------
                getline(line);
                if (line.size() > 82) { return abortReading(); } // 80 + \n\r

                // Satellite position Y (km)
                pos.y() = std::stod(str::replaceAll_copy(line.substr(3, 19), "d", "e", str::IgnoreCase)) * 1e3;
                // velocity Y dot (km/sec)
                vel.y() = std::stod(str::replaceAll_copy(line.substr(22, 19), "d", "e", str::IgnoreCase)) * 1e3;
                // Y acceleration (km/sec2)
                accelLuniSolar.y() = std::stod(str::replaceAll_copy(line.substr(41, 19), "d", "e", str::IgnoreCase)) * 1e3;
                // GLO:  frequency number(-7...+13) (-7...+6 ICD 5.1)
                // SBAS: Accuracy code (URA, meters)
                double frequencyNumber_accuracyCode = std::stod(str::replaceAll_copy(line.substr(60, 19), "d", "e", str::IgnoreCase));

                LOG_DATA("{}:    satPosY {}, velY {}, accelY {}, freqNum/accuracyCode {}", nameId(), pos.y(), vel.y(), accelLuniSolar.y(), frequencyNumber_accuracyCode);

                // ------------------------------------ BROADCAST ORBIT - 3 --------------------------------------
                getline(line);
                if (line.size() > 82) { return abortReading(); } // 80 + \n\r

                // Satellite position Z (km)
                pos.z() = std::stod(str::replaceAll_copy(line.substr(3, 19), "d", "e", str::IgnoreCase)) * 1e3;
                // velocity Z dot (km/sec)
                vel.z() = std::stod(str::replaceAll_copy(line.substr(22, 19), "d", "e", str::IgnoreCase)) * 1e3;
                // Z acceleration (km/sec2)
                accelLuniSolar.z() = std::stod(str::replaceAll_copy(line.substr(41, 19), "d", "e", str::IgnoreCase)) * 1e3;
                // GLO:  Age of oper. information (days) (E)
                // SBAS: IODN (Issue of Data Navigation, DO229, 8 first bits after Message Type if MT9)
                [[maybe_unused]] auto ageOfOperation_IODN = str::stod(str::replaceAll_copy(line.substr(60, 19), "d", "e", str::IgnoreCase), 0.0);

                LOG_DATA("{}:    satPosZ {}, velZ {}, accelZ {}, ageOfOperation/IODN {}", nameId(), pos.z(), vel.z(), accelLuniSolar.z(), ageOfOperation_IODN);

                if (satSys == GLO)
                {
                    _gnssNavInfo.addSatelliteNavData({ satSys, satNum }, std::make_shared<GLONASSEphemeris>(epoch, tau_c,
                                                                                                            -m_tau_n, gamma_n, static_cast<bool>(health),
                                                                                                            pos, vel, accelLuniSolar,
                                                                                                            frequencyNumber_accuracyCode));
                }
                else if (satSys == SBAS)
                {
                    LOG_WARN("SBAS is not yet supported. Therefore the Navigation file data will be skipped.");
                }
            }
        }
    }
    catch (const std::exception& e)
    {
        abortReading();
    }
}

void RinexNavFile::parseOrbit3()
{
    std::string line;
    try
    {
        while (getline(line) && !eof())
        {
            if (line.size() > 82) { return abortReading(); } // 80 + \n\r

            // -------------------------------------- SV / EPOCH / SV CLK ----------------------------------------

            // Satellite system (G), sat number (PRN) - A1,I2.2,
            // Satellite system (E), satellite number - A1,I2.2,
            // Satellite system (R), satellite number (slot number in sat. constellation) - A1,I2.2,
            // Satellite system (J), Satellite PRN-192 - A1,I2,
            // Satellite system (C), sat number (PRN) - A1,I2.2,
            // Satellite system (S), satellite number (slot number in sat. constellation) - A1,I2.2,
            // Satellite system (I), sat number (PRN) - A1,I2.2,
            SatelliteSystem satSys = SatSys_None;

            satSys = SatelliteSystem::fromChar(line.at(0));
            _gnssNavInfo.satelliteSystems |= satSys;

            if (satSys == SatSys_None) { continue; } // To intercept comment lines

            auto satNumStr = line.substr(1, 2);
            if (satNumStr == "  ") { continue; }

            auto satNum = static_cast<uint8_t>(str::stoi(satNumStr, 0));
            if (satNum == 0) { continue; }

            if (parseEphemeris(line, satSys, satNum))
            {
                continue;
            }
        }
    }
    catch (const std::exception& e)
    {
        abortReading();
    }
}

void RinexNavFile::parseOrbit4()
{
    std::string line;
    try
    {
        while (getline(line) && !eof())
        {
            if (line.size() > 82) { return abortReading(); } // 80 + \n\r

            if (line.at(0) == '>')
            {
                // -------------------------------------- Type / SV / MSSG ----------------------------------------

                // Satellite system (G), sat number (PRN) - A1,I2.2,
                // Satellite system (E), satellite number - A1,I2.2,
                // Satellite system (R), satellite number (slot number in sat. constellation) - A1,I2.2,
                // Satellite system (J), Satellite PRN-192 - A1,I2,
                // Satellite system (C), sat number (PRN) - A1,I2.2,
                // Satellite system (S), satellite number (slot number in sat. constellation) - A1,I2.2,
                // Satellite system (I), sat number (PRN) - A1,I2.2,

                SatelliteSystem satSys = SatSys_None;
                satSys = SatelliteSystem::fromChar(line.at(6));
                _gnssNavInfo.satelliteSystems |= satSys;
                if (satSys == SatSys_None) { continue; } // To intercept comment lines

                auto satNumStr = line.substr(7, 2);
                if (satNumStr == "  ") { continue; }

                auto satNum = static_cast<uint8_t>(str::stoi(satNumStr, 0));
                if (satNum == 0) { continue; }

                std::string msgType = line.substr(2, 3);
                NavMsgType navMsgType = getNavMsgType(msgType);

                // std::string satMsgType = line.substr(10, 4);

                switch (navMsgType)
                {
                case NavMsgType::EPH:
                {
                    // Code for EPH
                    getline(line);
                    if (line.size() > 82) { return abortReading(); }

                    if (satSys != SatelliteSystem::fromChar(line.at(0)) || satNumStr != line.substr(1, 2)) { continue; }

                    if (parseEphemeris(line, satSys, satNum))
                    {
                        continue;
                    }
                }
                break;
                case NavMsgType::STO:
                {
                    // Code for STO
                    getline(line);
                    if (line.size() > 82) { return abortReading(); }

                    // Epoch: Toc - Time of Clock (GPS) year (4 digits) - 4X,I4,
                    // month, day, hour, minute, second - 5(1X,I2.2),
                    auto timeSplit = str::split_wo_empty(line.substr(4, 20), " ");
                    auto timeSystem = satSys == GLO ? UTC : satSys.getTimeSystem();

                    int year = std::stoi(timeSplit.at(0));

                    InsTime epoch{ static_cast<uint16_t>(year),
                                   static_cast<uint16_t>(std::stoi(timeSplit.at(1))),
                                   static_cast<uint16_t>(std::stoi(timeSplit.at(2))),
                                   static_cast<uint16_t>(std::stoi(timeSplit.at(3))),
                                   static_cast<uint16_t>(std::stoi(timeSplit.at(4))),
                                   std::stold(timeSplit.at(5)),
                                   timeSystem };

                    LOG_DATA("{}: {}-{} {} (read as {} time)", nameId(), satSys, satNum, epoch.toYMDHMS(), std::string(timeSystem));
                    auto correctionType = str::trim_copy(line.substr(24, 4)); // FORMAT: 1X,A18
                    LOG_DATA("{}: Time System Correction: {}", nameId(), correctionType);

                    getline(line);
                    if (line.size() > 82) { return abortReading(); }

                    // auto t_tm = str::stod(str::trim_copy(line.substr(4, 19)), std::nan(""));
                    // LOG_DATA("{}:     t_tm {}", nameId(), t_tm);
                    auto a0 = str::stod(str::trim_copy(line.substr(23, 19)), std::nan(""));
                    LOG_DATA("{}:     a0 {}", nameId(), a0);
                    auto a1 = str::stod(str::trim_copy(line.substr(42, 19)), std::nan(""));
                    LOG_DATA("{}:     a1 {}", nameId(), a1);
                    // auto a2 = str::stod(str::trim_copy(line.substr(61, 19)), std::nan(""));
                    // LOG_DATA("{}:     a2 {}", nameId(), a2);

                    if (!std::isnan(a0) && !std::isnan(a1))
                    {
                        std::pair<TimeSystem, TimeSystem> timeSystems;
                        if (correctionType == "GPUT")
                        {
                            timeSystems = { GPST, UTC };
                        }
                        else if (correctionType == "GLUT")
                        {
                            timeSystems = { GLNT, UTC };
                        }
                        else if (correctionType == "GAUT")
                        {
                            timeSystems = { GST, UTC };
                        }
                        else if (correctionType == "BDUT")
                        {
                            timeSystems = { BDT, UTC };
                        }
                        else if (correctionType == "QZUT")
                        {
                            timeSystems = { QZSST, UTC };
                        }
                        else if (correctionType == "IRUT")
                        {
                            timeSystems = { IRNSST, UTC };
                        }
                        // else if (correctionType == "SBUT") { timeSystems = { SBAST, UTC }; }
                        else if (correctionType == "GLGP")
                        {
                            timeSystems = { GLNT, GPST };
                        }
                        else if (correctionType == "GAGP")
                        {
                            timeSystems = { GST, GPST };
                        }
                        else if (correctionType == "GPGA")
                        {
                            timeSystems = { GST, GPST };
                        } // Pre RINEX 3.04
                        else if (correctionType == "QZGP")
                        {
                            timeSystems = { QZSST, GPST };
                        }
                        else if (correctionType == "IRGP")
                        {
                            timeSystems = { IRNSST, GPST };
                        }
                        else
                        {
                            LOG_TRACE("{}:     Time System Correction '{}' not implemented yet", nameId(), correctionType);
                            continue;
                        }
                        _gnssNavInfo.timeSysCorr[timeSystems] = { a0, a1 };
                    }
                }
                break;
                case NavMsgType::EOP:
                    // Code for EOP
                    // TODO: Implement EOP
                    break;
                case NavMsgType::ION:
                {
                    // Code for ION
                    getline(line);
                    if (line.size() > 82) { return abortReading(); }

                    // Epoch: t_tm - Transmission timne of ION data, year (4 digits) - 4X,I4,
                    // month, day, hour, minute, second - 5(1X,I2.2),
                    auto timeSplit = str::split_wo_empty(line.substr(4, 20), " ");
                    auto timeSystem = satSys == GLO ? UTC : satSys.getTimeSystem();

                    int year = std::stoi(timeSplit.at(0));

                    InsTime t_tm{ static_cast<uint16_t>(year),
                                  static_cast<uint16_t>(std::stoi(timeSplit.at(1))),
                                  static_cast<uint16_t>(std::stoi(timeSplit.at(2))),
                                  static_cast<uint16_t>(std::stoi(timeSplit.at(3))),
                                  static_cast<uint16_t>(std::stoi(timeSplit.at(4))),
                                  std::stold(timeSplit.at(5)),
                                  timeSystem };

                    LOG_DATA("{}: {}-{} {} (read as {} time)", nameId(), satSys, satNum, t_tm.toYMDHMS(), std::string(timeSystem));

                    // TODO: Destinction of BDS unclear in RINEX versin 4.00 BDGIM or Klobuchar model
                    // Klobuchar
                    if (satSys == GPS || satSys == QZSS || satSys == IRNSS || satSys == BDS) // NOLINT(misc-redundant-expression) // bugged warning
                    {
                        std::array<double, 4> alpha{};
                        std::array<double, 4> beta{};
                        alpha[0] = str::stod(str::trim_copy(line.substr(23, 19)), 0.0);
                        alpha[1] = str::stod(str::trim_copy(line.substr(42, 19)), 0.0);
                        alpha[2] = str::stod(str::trim_copy(line.substr(61, 19)), 0.0);
                        getline(line);
                        if (line.size() > 82) { return abortReading(); }
                        alpha[3] = str::stod(str::trim_copy(line.substr(4, 19)), 0.0);
                        beta[0] = str::stod(str::trim_copy(line.substr(23, 19)), 0.0);
                        beta[1] = str::stod(str::trim_copy(line.substr(42, 19)), 0.0);
                        beta[2] = str::stod(str::trim_copy(line.substr(61, 19)), 0.0);
                        getline(line);
                        if (line.size() > 82) { return abortReading(); }
                        beta[3] = str::stod(str::trim_copy(line.substr(4, 19)), 0.0);
                        _gnssNavInfo.ionosphericCorrections.insert(satSys, IonosphericCorrections::Alpha, alpha);
                        _gnssNavInfo.ionosphericCorrections.insert(satSys, IonosphericCorrections::Beta, beta);
                        LOG_DATA("{}: Ionospheric Correction: {}-Alpha: [{}]", nameId(),
                                 std::string(satSys), fmt::join(alpha.begin(), alpha.end(), ", "));
                        LOG_DATA("{}: Ionospheric Correction: {}-Beta: [{}]", nameId(),
                                 std::string(satSys), fmt::join(beta.begin(), beta.end(), ", "));
                        [[maybe_unused]] auto regionCode = str::stod(str::trim_copy(line.substr(23, 19)), std::nan(""));
                    }
                    else if (satSys == GAL) // Nequick-G
                    {
                        std::array<double, 4> alpha{};
                        alpha[0] = str::stod(str::trim_copy(line.substr(23, 19)), 0.0);
                        alpha[1] = str::stod(str::trim_copy(line.substr(42, 19)), 0.0);
                        alpha[2] = str::stod(str::trim_copy(line.substr(61, 19)), 0.0);
                        getline(line);
                        if (line.size() > 82) { return abortReading(); }
                        // TODO: Change Correction format to support Nequick-G
                        // This is a hack this data is not an alpha value but a disturbance flag
                        alpha[3] = str::stod(str::trim_copy(line.substr(4, 19)), 0.0);
                        _gnssNavInfo.ionosphericCorrections.insert(satSys, IonosphericCorrections::Alpha, alpha);
                        LOG_DATA("{}: Ionospheric Correction: {}-Alpha: [{}]", nameId(),
                                 std::string(satSys), fmt::join(alpha.begin(), alpha.end(), ", "));
                    } /*
                     else if (satSys == BDS) // BDGIM
                     {
                         // TODO: Implement Beidou ION message
                     }*/
                    else
                    {
                        continue;
                    }
                }
                break;
                default:
                    // Abort
                    break;
                }
            }
        }
    }
    catch (const std::exception& e)
    {
        abortReading();
    }
}

bool RinexNavFile::parseEphemeris(std::string& line, SatelliteSystem satSys, uint8_t satNum)
{
    // Epoch: Toc - Time of Clock (GPS) year (4 digits) - 1X,I4,
    // month, day, hour, minute, second - 5(1X,I2.2),
    auto timeSplit = str::split_wo_empty(line.substr(3, 20), " ");
    auto timeSystem = satSys == GLO ? UTC : satSys.getTimeSystem();

    int year = std::stoi(timeSplit.at(0));

    InsTime epoch{ static_cast<uint16_t>(year),
                   static_cast<uint16_t>(std::stoi(timeSplit.at(1))),
                   static_cast<uint16_t>(std::stoi(timeSplit.at(2))),
                   static_cast<uint16_t>(std::stoi(timeSplit.at(3))),
                   static_cast<uint16_t>(std::stoi(timeSplit.at(4))),
                   std::stold(timeSplit.at(5)),
                   timeSystem };

    LOG_DATA("{}: {}-{} {} (read as {} time)", nameId(), satSys, satNum, epoch.toYMDHMS(), std::string(timeSystem));

    if (satSys == GPS || satSys == GAL || satSys == QZSS || satSys == BDS) // NOLINT(misc-redundant-expression) // bugged warning
    {
        // Polynomial coefficients for clock correction
        std::array<double, 3> a{};
        // SV clock bias [seconds]
        a[0] = std::stod(str::replaceAll_copy(line.substr(23, 19), "d", "e", str::IgnoreCase));
        // SV clock drift [sec/sec]
        a[1] = std::stod(str::replaceAll_copy(line.substr(42, 19), "d", "e", str::IgnoreCase));
        // SV clock drift rate [sec/sec^2]
        a[2] = std::stod(str::replaceAll_copy(line.substr(61, 19), "d", "e", str::IgnoreCase));

        LOG_DATA("{}:    clkBias {}, clkDrift {}, clkDriftRate {}", nameId(), a[0], a[1], a[2]);

        // ------------------------------------ BROADCAST ORBIT - 1 --------------------------------------
        getline(line);
        if (line.size() > 82)
        {
            abortReading();
            return false;
        } // 80 + \n\r

        // GPS/QZSS: Issue of Data, Ephemeris (IODE)
        // GAL:      IODnav Issue of Data of the nav batch
        // BDS:      AODE Age of Data, Ephemeris (as specified in BeiDou ICD Table Section 5.2.4.11 Table 5-8)
        //           and field range is: 0-31.
        // IRNSS:    IODEC Issue of Data, Ephemeris and Clock
        auto IODE_IODnav_AODE_IODEC = static_cast<size_t>(std::stod(str::replaceAll_copy(line.substr(4, 19), "d", "e", str::IgnoreCase)));
        // Crs (meters)
        double Crs = std::stod(str::replaceAll_copy(line.substr(23, 19), "d", "e", str::IgnoreCase));
        // Delta n (radians/sec)
        double delta_n = std::stod(str::replaceAll_copy(line.substr(42, 19), "d", "e", str::IgnoreCase));
        // M0 (radians)
        double M_0 = std::stod(str::replaceAll_copy(line.substr(61, 19), "d", "e", str::IgnoreCase));

        LOG_DATA("{}:    IODE_IODnav_AODE_IODEC {}, Crs {}, Delta_n {}, M0 {}", nameId(), IODE_IODnav_AODE_IODEC, Crs, delta_n, M_0);

        // ------------------------------------ BROADCAST ORBIT - 2 --------------------------------------
        getline(line);
        if (line.size() > 82)
        {
            abortReading();
            return false;
        } // 80 + \n\r

        // Cuc (radians)
        double Cuc = std::stod(str::replaceAll_copy(line.substr(4, 19), "d", "e", str::IgnoreCase));
        // e Eccentricity
        double e = std::stod(str::replaceAll_copy(line.substr(23, 19), "d", "e", str::IgnoreCase));
        // Cus (radians)
        double Cus = std::stod(str::replaceAll_copy(line.substr(42, 19), "d", "e", str::IgnoreCase));
        // sqrt(A) (sqrt(m))
        double sqrt_A = std::stod(str::replaceAll_copy(line.substr(61, 19), "d", "e", str::IgnoreCase));

        LOG_DATA("{}:    Cuc {}, e {}, Cus {}, sqrt_A {}", nameId(), Cuc, e, Cus, sqrt_A);

        // ------------------------------------ BROADCAST ORBIT - 3 --------------------------------------
        getline(line);
        if (line.size() > 82)
        {
            abortReading();
            return false;
        } // 80 + \n\r

        // Toe Time of Ephemeris (sec of GPS week)
        double toeSec = std::stod(str::replaceAll_copy(line.substr(4, 19), "d", "e", str::IgnoreCase));
        // Cic (radians)
        double Cic = std::stod(str::replaceAll_copy(line.substr(23, 19), "d", "e", str::IgnoreCase));
        // OMEGA0 (radians)
        double Omega_0 = std::stod(str::replaceAll_copy(line.substr(42, 19), "d", "e", str::IgnoreCase));
        // Cis (radians)
        double Cis = std::stod(str::replaceAll_copy(line.substr(61, 19), "d", "e", str::IgnoreCase));

        LOG_DATA("{}:    toe {}, Cic {}, Omega_0 {}, Cis {}", nameId(), toeSec, Cic, Omega_0, Cis);

        // ------------------------------------ BROADCAST ORBIT - 4 --------------------------------------
        getline(line);
        if (line.size() > 82)
        {
            abortReading();
            return false;
        } // 80 + \n\r

        // i0 (radians)
        double i_0 = std::stod(str::replaceAll_copy(line.substr(4, 19), "d", "e", str::IgnoreCase));
        // Crc (meters)
        double Crc = std::stod(str::replaceAll_copy(line.substr(23, 19), "d", "e", str::IgnoreCase));
        // omega (radians)
        double omega = std::stod(str::replaceAll_copy(line.substr(42, 19), "d", "e", str::IgnoreCase));
        // OMEGA DOT (radians/sec)
        double Omega_dot = std::stod(str::replaceAll_copy(line.substr(61, 19), "d", "e", str::IgnoreCase));

        LOG_DATA("{}:    i_0 {}, Crc {}, omega {}, Omega_dot {}", nameId(), i_0, Crc, omega, Omega_dot);

        // ------------------------------------ BROADCAST ORBIT - 5 --------------------------------------
        getline(line);
        if (line.size() > 82)
        {
            abortReading();
            return false;
        } // 80 + \n\r

        // IDOT (radians/sec)
        double i_dot = std::stod(str::replaceAll_copy(line.substr(4, 19), "d", "e", str::IgnoreCase));
        // GPS:       Codes on L2 channel
        // QZSS:      Codes on L2 channel (fixed to 2, see IS-QZSS-PNT 4.1.2.7)
        // GAL:       Data sources (FLOAT --> INTEGER)
        //            Bit 0 set: I/NAV E1-B
        //            Bit 1 set: F/NAV E5a-I
        //            Bit 2 set: I/NAV E5b-I
        //            Bits 0 and 2 : Both can be set if the navigation messages were merged, however, bits 0-2
        //                           cannot all be set, as the I/NAV and F/NAV messages contain different information
        //            Bit 3 reserved for Galileo internal use
        //            Bit 4 reserved for Galileo internal use
        //            Bit 8 set: af0-af2, Toc, SISA are for E5a,E1
        //            Bit 9 set: af0-af2, Toc, SISA are for E5b,E1
        //            Bits 8-9 : exclusive (only one bit can be set)
        // BDS/IRNSS: Spare
        auto codesOnL2Channel_dataSources = static_cast<uint16_t>(str::stod(str::replaceAll_copy(line.substr(23, 19), "d", "e", str::IgnoreCase), 0));

        // GPS Week # (to go with TOE) Continuous number, not mod(1024)!
        auto gpsWeek = static_cast<int32_t>(str::stod(str::replaceAll_copy(line.substr(42, 19), "d", "e", str::IgnoreCase), epoch.toGPSweekTow().gpsWeek));
        if (satSys == BDS) { gpsWeek += InsTimeUtil::DIFF_BDT_WEEK_TO_GPST_WEEK; }
        auto toe = InsTime(0, gpsWeek, toeSec, satSys.getTimeSystem());

        // GPS:  L2 P data flag
        // QZSS: L2P data flag set to 1 since QZSS does not track L2P
        // GAL/BDS/IRNSS:  Spare
        bool L2PdataFlag = static_cast<bool>(str::stod(str::replaceAll_copy(line.substr(61, 19), "d", "e", str::IgnoreCase), 0.0));

        LOG_DATA("{}:    i_dot {}, codesOnL2Channel/dataSources {} ({}), gpsWeek {}, L2PdataFlag {}", nameId(),
                 i_dot, codesOnL2Channel_dataSources, std::bitset<10>(codesOnL2Channel_dataSources).to_string(), gpsWeek, L2PdataFlag);

        // ------------------------------------ BROADCAST ORBIT - 6 --------------------------------------
        getline(line);
        if (line.size() > 82)
        {
            abortReading();
            return false;
        } // 80 + \n\r

        // GPS:  SV accuracy (meters) See GPS ICD 200H Section 20.3.3.3.1.3 use specified equations to define
        //       nominal values, N = 0-6: use 2(1+N/2) (round to one decimal place i.e. 2.8, 5.7 and 11.3) ,
        //       N= 7-15:use 2 (N-2), 8192 specifies use at own risk
        // QZSS: SV accuracy (meters) (IS -QZSS-PNT, Section 5.4.3.1) which refers to: IS GPS 200H Section 20.3.3.3.1.3
        //       use specified equations to define nominal values, N = 0-6: use 2(1+N/2) (round to one decimal
        //       place i.e. 2.8, 5.7 and 11.3) , N= 7-15:use 2 (N-2), 8192 specifies use at own risk
        // IRNSS: User Range Accuracy(m), See IRNSS ICD Section 6.2.1.4 , use specified equations to define
        //        nominal values, N = 0-6: use 2(1+N/2) (round to one decimal place i.e. 2.8, 5.7 and 11.3) ,
        //        N= 7-15:use 2 (N-2), 8192 specifies use at own risk
        double signalAccuracy = std::stod(str::replaceAll_copy(line.substr(4, 19), "d", "e", str::IgnoreCase));
        // GPS/QZSS: SV health (bits 17-22 w 3 sf 1)
        // GAL:      SV health (FLOAT converted to INTEGER) See Galileo ICD Section 5.1.9.3
        //           Bit 0: E1B DVS       Bits 1-2: E1B HS          Bit 3: E5a DVS
        //           Bits 4-5: E5a HS     Bit 6: E5b DVS            Bits 7-8: E5b HS
        // BDS:      SatH1
        // IRNSS:    Health (Sub frame 1,bits 155(most significant) and 156(least significant)),
        //           where 0 = L5 and S healthy, 1 = L5 healthy and S unhealthy, 2= L5 unhealthy
        //           and S healthy, 3= both L5 and S unhealthy
        auto svHealth = static_cast<uint16_t>(std::stod(str::replaceAll_copy(line.substr(23, 19), "d", "e", str::IgnoreCase)));
        // GPS:  TGD (seconds)
        // QZSS: TGD (seconds) The QZSS ICD specifies a do not use bit pattern "10000000" this condition is represented by a blank field.
        // GAL:  BGD E5a/E1 (seconds)
        // BDS:  TGD1 B1/B3 (seconds)
        double tgd_bgd5a_TGD1 = str::stod(str::replaceAll_copy(line.substr(42, 19), "d", "e", str::IgnoreCase), 0.0);

        // GPS/QZSS: IODC Issue of Data, Clock
        // GAL:      BGD E5b/E1 (seconds)
        // BDS:      TGD2 B2/B3 (seconds)
        // IRNSS:    Blank
        double IODC_bgd5b_TGD2 = str::stod(str::replaceAll_copy(line.substr(61, 19), "d", "e", str::IgnoreCase), 0.0);

        LOG_DATA("{}:    svAccuracy {}, svHealth {}, TGD {}, IODC {}", nameId(), signalAccuracy, svHealth, tgd_bgd5a_TGD1, IODC_bgd5b_TGD2);

        // ------------------------------------ BROADCAST ORBIT - 7 --------------------------------------
        getline(line);
        if (line.size() > 82)
        {
            abortReading();
            return false;
        } // 80 + \n\r

        // Transmission time of message (sec of GPS week, derived e.g.from Z-count in Hand Over Word (HOW))
        [[maybe_unused]] auto transmissionTime = str::stod(str::replaceAll_copy(line.substr(4, 19), "d", "e", str::IgnoreCase), 0.0);
        // GPS/QZSS: Fit Interval in hours see section 6.11. (BNK).
        // GAL:      Spare
        // BDS:      AODC Age of Data Clock (as specified in BeiDou ICD Table Section 5.2.4.9 Table 5-6) and field range is: 0-31.
        auto fitInterval_AODC = str::stod(str::replaceAll_copy(line.substr(23, 19), "d", "e", str::IgnoreCase), 0.0);
        // Spare
        // std::stod(str::replaceAll_copy(line.substr(42, 19), "d", "e", str::IgnoreCase));
        // Spare
        // std::stod(str::replaceAll_copy(line.substr(61, 19), "d", "e", str::IgnoreCase));

        LOG_DATA("{}:    transmissionTime {}, fitInterval/AODC {}", nameId(), transmissionTime, fitInterval_AODC);

        if (satSys == GPS)
        {
            _gnssNavInfo.addSatelliteNavData({ satSys, satNum }, std::make_shared<GPSEphemeris>(epoch, toe,
                                                                                                IODE_IODnav_AODE_IODEC, IODC_bgd5b_TGD2, a,
                                                                                                sqrt_A, e, i_0, Omega_0, omega, M_0,
                                                                                                delta_n, Omega_dot, i_dot, Cus, Cuc,
                                                                                                Cis, Cic, Crs, Crc,
                                                                                                signalAccuracy, svHealth,
                                                                                                codesOnL2Channel_dataSources, L2PdataFlag,
                                                                                                tgd_bgd5a_TGD1, fitInterval_AODC));
        }
        else if (satSys == GAL)
        {
            // The same satellite can appear multiple times with different dataSource bits
            // We want to prefer 'I/NAV E1-B' (Bit 0 set) over 'F/NAV E5a-I' (Bit 1 set) or 'I/NAV E5b-I' (Bit 2 set)

            if (_gnssNavInfo.satellites().contains({ satSys, satNum }) // We have this satellite already
                && !std::bitset<10>(codesOnL2Channel_dataSources)[0])  // This message is not 'I/NAV E1-B'
            {
                const auto& navData = _gnssNavInfo.satellites().at({ satSys, satNum }).getNavigationData();
                auto existingEph = std::find_if(navData.begin(), navData.end(),
                                                [&](const std::shared_ptr<SatNavData>& satNavData) {
                                                    return satNavData->type == SatNavData::GalileoEphemeris && satNavData->refTime == epoch
                                                           && std::dynamic_pointer_cast<GalileoEphemeris>(satNavData)->dataSource[0];
                                                });
                if (existingEph != navData.end()) // There is already a 'I/NAV E1-B' message
                {
                    LOG_DATA("{}:    Skipping ephemeris data because of dataSource priority", nameId());
                    return false;
                }
            }

            GalileoEphemeris::SvHealth health = { .E5a_DataValidityStatus = static_cast<GalileoEphemeris::SvHealth::DataValidityStatus>((svHealth & 0b000001000) >> 3),
                                                  .E5b_DataValidityStatus = static_cast<GalileoEphemeris::SvHealth::DataValidityStatus>((svHealth & 0b001000000) >> 6),
                                                  .E1B_DataValidityStatus = static_cast<GalileoEphemeris::SvHealth::DataValidityStatus>((svHealth & 0b000000001) >> 0),
                                                  .E5a_SignalHealthStatus = static_cast<GalileoEphemeris::SvHealth::SignalHealthStatus>((svHealth & 0b000110000) >> 4),
                                                  .E5b_SignalHealthStatus = static_cast<GalileoEphemeris::SvHealth::SignalHealthStatus>((svHealth & 0b110000000) >> 7),
                                                  .E1B_SignalHealthStatus = static_cast<GalileoEphemeris::SvHealth::SignalHealthStatus>((svHealth & 0b000000110) >> 1) };
            _gnssNavInfo.addSatelliteNavData({ satSys, satNum }, std::make_shared<GalileoEphemeris>(epoch, toe, IODE_IODnav_AODE_IODEC, a,
                                                                                                    sqrt_A, e, i_0, Omega_0, omega, M_0,
                                                                                                    delta_n, Omega_dot, i_dot, Cus, Cuc,
                                                                                                    Cis, Cic, Crs, Crc,
                                                                                                    codesOnL2Channel_dataSources, signalAccuracy, health,
                                                                                                    tgd_bgd5a_TGD1, IODC_bgd5b_TGD2));
        }
        else if (satSys == BDS)
        {
            _gnssNavInfo.addSatelliteNavData({ satSys, satNum }, std::make_shared<BDSEphemeris>(epoch, toe,
                                                                                                IODE_IODnav_AODE_IODEC, fitInterval_AODC, a,
                                                                                                sqrt_A, e, i_0, Omega_0, omega, M_0,
                                                                                                delta_n, Omega_dot, i_dot, Cus, Cuc,
                                                                                                Cis, Cic, Crs, Crc,
                                                                                                signalAccuracy, svHealth,
                                                                                                tgd_bgd5a_TGD1, IODC_bgd5b_TGD2));
        }
        else if (satSys == QZSS)
        {
            LOG_WARN("QZSS is not yet supported. Therefore the Navigation file data will be skipped.");
        }
    }
    else if (satSys == GLO || satSys == SBAS) // NOLINT(misc-redundant-expression) // bugged warning
    {
        // TODO: Offset

        Eigen::Vector3d pos;
        Eigen::Vector3d vel;
        Eigen::Vector3d accelLuniSolar;

        double tau_c{};
        // Coefficient of linear polynomial of time system difference [s]
        if (_gnssNavInfo.timeSysCorr.contains({ satSys.getTimeSystem(), UTC }))
        {
            tau_c = _gnssNavInfo.timeSysCorr.at({ satSys.getTimeSystem(), UTC }).a0;
        }

        // GLO:  SV clock bias (sec) (-TauN)
        // SBAS: SV clock bias (sec) (aGf0)
        double m_tau_n = std::stod(str::replaceAll_copy(line.substr(23, 19), "d", "e", str::IgnoreCase));
        // GLO:  SV relative frequency bias (+GammaN)
        // SBAS: SV relative frequency bias (aGf1)
        double gamma_n = std::stod(str::replaceAll_copy(line.substr(42, 19), "d", "e", str::IgnoreCase));
        // GLO:  Message frame time (tk+nd*86400) in seconds of the UTC week
        // SBAS: Transmission time of message (start of the message) in GPS seconds of the week
        [[maybe_unused]] auto msgFrameTime = std::stod(str::replaceAll_copy(line.substr(61, 19), "d", "e", str::IgnoreCase));
        LOG_DATA("{}:    clkBias {}, relFreqBias {}, msgFrameTime {}", nameId(), m_tau_n, gamma_n, msgFrameTime);

        // ------------------------------------ BROADCAST ORBIT - 1 --------------------------------------
        getline(line);
        if (line.size() > 82)
        {
            abortReading();
            return false;
        } // 80 + \n\r

        // Satellite position X (km)
        pos.x() = std::stod(str::replaceAll_copy(line.substr(4, 19), "d", "e", str::IgnoreCase)) * 1e3;
        // velocity X dot (km/sec)
        vel.x() = std::stod(str::replaceAll_copy(line.substr(23, 19), "d", "e", str::IgnoreCase)) * 1e3;
        // X acceleration (km/sec2)
        accelLuniSolar.x() = std::stod(str::replaceAll_copy(line.substr(42, 19), "d", "e", str::IgnoreCase)) * 1e3;
        // GLO:  health (0=healthy, 1=unhealthy) (MSB of 3-bit Bn)
        // SBAS: Health: SBAS: See Section 8.3.3 bit mask for: health, health availability and User Range Accuracy.
        [[maybe_unused]] auto health = std::stod(str::replaceAll_copy(line.substr(61, 19), "d", "e", str::IgnoreCase));

        LOG_DATA("{}:    satPosX {}, velX {}, accelX {}, health {}", nameId(), pos.x(), vel.x(), accelLuniSolar.x(), health);

        // ------------------------------------ BROADCAST ORBIT - 2 --------------------------------------
        getline(line);
        if (line.size() > 82)
        {
            abortReading();
            return false;
        } // 80 + \n\r

        // Satellite position Y (km)
        pos.y() = std::stod(str::replaceAll_copy(line.substr(4, 19), "d", "e", str::IgnoreCase)) * 1e3;
        // velocity Y dot (km/sec)
        vel.y() = std::stod(str::replaceAll_copy(line.substr(23, 19), "d", "e", str::IgnoreCase)) * 1e3;
        // Y acceleration (km/sec2)
        accelLuniSolar.y() = std::stod(str::replaceAll_copy(line.substr(42, 19), "d", "e", str::IgnoreCase)) * 1e3;
        // GLO:  frequency number(-7...+13) (-7...+6 ICD 5.1)
        // SBAS: Accuracy code (URA, meters)
        double frequencyNumber_accuracyCode = std::stod(str::replaceAll_copy(line.substr(61, 19), "d", "e", str::IgnoreCase));

        LOG_DATA("{}:    satPosY {}, velY {}, accelY {}, freqNum/accuracyCode {}", nameId(), pos.y(), vel.y(), accelLuniSolar.y(), frequencyNumber_accuracyCode);

        // ------------------------------------ BROADCAST ORBIT - 3 --------------------------------------
        getline(line);
        if (line.size() > 82)
        {
            abortReading();
            return false;
        } // 80 + \n\r

        // Satellite position Z (km)
        pos.z() = std::stod(str::replaceAll_copy(line.substr(4, 19), "d", "e", str::IgnoreCase)) * 1e3;
        // velocity Z dot (km/sec)
        vel.z() = std::stod(str::replaceAll_copy(line.substr(23, 19), "d", "e", str::IgnoreCase)) * 1e3;
        // Z acceleration (km/sec2)
        accelLuniSolar.z() = std::stod(str::replaceAll_copy(line.substr(42, 19), "d", "e", str::IgnoreCase)) * 1e3;
        // GLO:  Age of oper. information (days) (E)
        // SBAS: IODN (Issue of Data Navigation, DO229, 8 first bits after Message Type if MT9)
        [[maybe_unused]] auto ageOfOperation_IODN = str::stod(str::replaceAll_copy(line.substr(61, 19), "d", "e", str::IgnoreCase), 0.0);

        LOG_DATA("{}:    satPosZ {}, velZ {}, accelZ {}, ageOfOperation/IODN {}", nameId(), pos.z(), vel.z(), accelLuniSolar.z(), ageOfOperation_IODN);

        // ------------------------------------ BROADCAST ORBIT - 4 --------------------------------------
        // since version 3.05 GLONASS has an additional line
        if (satSys == GLO && _version >= 3.05)
        {
            getline(line);
            if (line.size() > 82)
            {
                abortReading();
                return false;
            } // 80 + \n\r

            // status flags (FLOAT converted to INTEGER) (can be blank)
            //        M  Bit 7-8: GLO type indicator (00=GLO, 01=GLO-M/K)
            //        P4 Bit 6:   GLO-M/K only, 1=data updated, 0=data not updated
            //        P3 Bit 5:   num of satellites in current frame alamanc (0=4 sats, 1=5 sats)
            //        P2 Bit 4:   indicate even (0) or odd (1) of time interval
            //        P1 Bit 2-3: update and validity interval (00=0 min, 01=30 min, 10=45 min, 11=60 min)
            //        P  Bit 0-1: GLO-M/K only, time offset parameters tau_c, tau_GPS source (00=ground, 01 = tau_c ground, tau_GPS on-board, 10 = tau_c on-board, tau_GPS ground, 11 = on-board)
            [[maybe_unused]] double statusFlags = str::stod(str::replaceAll_copy(str::trim_copy(line.substr(4, 19)), "d", "e", str::IgnoreCase), std::nan(""));
            // L1/L2 group delay difference Δτ [sec]
            [[maybe_unused]] double L1L2groupDelayDifference = std::stod(str::replaceAll_copy(line.substr(23, 19), "d", "e", str::IgnoreCase));
            // URAI raw accuracy index F_T (GLO-M/K only)r
            [[maybe_unused]] double URAI = std::stod(str::replaceAll_copy(line.substr(42, 19), "d", "e", str::IgnoreCase));
            // health flags (FLOAT converted to INTEGER) (can be blank)
            //       l(3) Bit 2: GLO-M/K only, health bit of string 3
            //       A_C  Bit 1: 1=almanac health reported in ephemerides record, 0=not reported
            //       C    Bit 0: almanac health bit (1=healthy, 0=not healthy)
            [[maybe_unused]] double healthFlags = str::stod(str::replaceAll_copy(str::trim_copy(line.substr(61, 19)), "d", "e", str::IgnoreCase), std::nan(""));

            LOG_DATA("{}:    statusFlags {}, L1L2groupDelayDifference {}, URAI {}, healthFlags {}", nameId(), statusFlags, L1L2groupDelayDifference, URAI, healthFlags);
        }

        if (satSys == GLO)
        {
            _gnssNavInfo.addSatelliteNavData({ satSys, satNum }, std::make_shared<GLONASSEphemeris>(epoch, tau_c,
                                                                                                    -m_tau_n, gamma_n, static_cast<bool>(health),
                                                                                                    pos, vel, accelLuniSolar,
                                                                                                    frequencyNumber_accuracyCode));
        }
        else if (satSys == SBAS)
        {
            LOG_WARN("SBAS is not yet supported. Therefore the Navigation file data will be skipped.");
        }
    }
    return true;
}

void RinexNavFile::readHeader()
{
    LOG_TRACE("{}: called", nameId());
    auto guard = requestOutputValueLock(OUTPUT_PORT_INDEX_GNSS_NAV_INFO);
    executeHeaderParser(_version);
}

void RinexNavFile::readOrbits()
{
    LOG_TRACE("{}: called", nameId());
    auto guard = requestOutputValueLock(OUTPUT_PORT_INDEX_GNSS_NAV_INFO);
    executeOrbitParser(_version);
}

} // namespace NAV
