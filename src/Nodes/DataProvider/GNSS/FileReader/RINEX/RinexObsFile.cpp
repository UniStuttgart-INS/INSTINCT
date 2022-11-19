// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "RinexObsFile.hpp"

#include <Eigen/Core>

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"

#include "util/StringUtil.hpp"

#include "Navigation/GNSS/Core/SatelliteSystem.hpp"
#include "Navigation/GNSS/Core/Code.hpp"
#include "NodeData/GNSS/GnssObs.hpp"

namespace NAV
{

RinexObsFile::RinexObsFile()
    : Node(typeStatic())
{
    LOG_TRACE("{}: called", name);

    _hasConfig = true;
    _guiConfigDefaultWindowSize = { 517, 67 };

    nm::CreateOutputPin(this, "GnssObs", Pin::Type::Flow, { NAV::GnssObs::type() }, &RinexObsFile::pollData);
}

RinexObsFile::~RinexObsFile()
{
    LOG_TRACE("{}: called", nameId());
}

std::string RinexObsFile::typeStatic()
{
    return "RinexObsFile";
}

std::string RinexObsFile::type() const
{
    return typeStatic();
}

std::string RinexObsFile::category()
{
    return "Data Provider";
}

void RinexObsFile::guiConfig()
{
    if (auto res = FileReader::guiConfig("Rinex Obs (.obs .rnx .*O){.obs,.rnx,(.+[.]\\d\\d?O)},.*", { ".obs", ".rnx", "(.+[.]\\d\\d?O)" }, size_t(id), nameId()))
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
}

[[nodiscard]] json RinexObsFile::save() const
{
    LOG_TRACE("{}: called", nameId());

    json j;

    j["FileReader"] = FileReader::save();

    return j;
}

void RinexObsFile::restore(json const& j)
{
    LOG_TRACE("{}: called", nameId());

    if (j.contains("FileReader"))
    {
        FileReader::restore(j.at("FileReader"));
    }
}

bool RinexObsFile::initialize()
{
    LOG_TRACE("{}: called", nameId());

    return FileReader::initialize();
}

void RinexObsFile::deinitialize()
{
    LOG_TRACE("{}: called", nameId());

    FileReader::deinitialize();

    _version = 0.0;
    _timeSystem = TimeSys_None;
    _obsDescription.clear();
    _rcvClockOffsAppl = false;
}

bool RinexObsFile::resetNode()
{
    LOG_TRACE("{}: called", nameId());

    FileReader::resetReader();

    return true;
}

FileReader::FileType RinexObsFile::determineFileType()
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
        std::getline(filestreamHeader, line);
        str::rtrim(line);
        if (line.size() != 80)
        {
            LOG_ERROR("{}: Not a valid RINEX OBS file. Lines should be 80 characters long but the file has {}.", nameId(), line.size() - 1);
            return FileReader::FileType::NONE;
        }

        if (extHeaderLabel(line) != "RINEX VERSION / TYPE")
        {
            LOG_ERROR("{}: Not a valid RINEX OBS file. Could not read 'RINEX VERSION / TYPE' line.", nameId());
            return FileReader::FileType::NONE;
        }

        double version = std::stod(str::trim_copy(line.substr(0, 20))); // FORMAT: F9.2,11X
        if (!_supportedVersions.contains(version))
        {
            LOG_ERROR("{}: RINEX version {} is not supported. Supported versions are [{}]", nameId(),
                      version, fmt::join(_supportedVersions.begin(), _supportedVersions.end(), ", "));
            return FileReader::FileType::NONE;
        }

        std::string fileType = str::trim_copy(line.substr(20, 20)); // FORMAT: A1,19X
        if (fileType.at(0) != 'O')
        {
            LOG_ERROR("{}: Not a valid RINEX OBS file. File type '{}' not recognized.", nameId(), fileType);
            doDeinitialize();
            return FileReader::FileType::NONE;
        }
        std::string satSystem = str::trim_copy(line.substr(40, 20)); // FORMAT: A1,19X
        if (SatelliteSystem::fromChar(satSystem.at(0)) == SatSys_None && satSystem.at(0) != 'M')
        {
            LOG_ERROR("{}: Not a valid RINEX OBS file. Satellite System '{}' not recognized.", nameId(), satSystem.at(0));
            doDeinitialize();
            return FileReader::FileType::NONE;
        }
        // ---------------------------------------- PGM / RUN BY / DATE ------------------------------------------
        std::getline(filestreamHeader, line);
        if (extHeaderLabel(line) != "PGM / RUN BY / DATE")
        {
            LOG_ERROR("{}: Not a valid RINEX OBS file. Could not read 'PGM / RUN BY / DATE' line.", nameId());
            return FileReader::FileType::NONE;
        }

        // ----------------------------------------- END OF HEADER -------------------------------------------
        while (std::getline(filestreamHeader, line))
        {
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

void RinexObsFile::readHeader()
{
    LOG_TRACE("{}: called", nameId());

    std::string line;

    // --------------------------------------- RINEX VERSION / TYPE ------------------------------------------
    std::getline(_filestream, line);
    _version = std::stod(str::trim_copy(line.substr(0, 20)));
    LOG_DEBUG("{}: Version: {:3.2f}", nameId(), _version);                       // FORMAT: F9.2,11X
    LOG_DEBUG("{}: SatSys : {}", nameId(), str::trim_copy(line.substr(40, 20))); // FORMAT: A1,19X

    // #######################################################################################################
    while (std::getline(_filestream, line) && !_filestream.eof())
    {
        if (line.size() < 60)
        {
            LOG_WARN("{}: Skipping header line because it does not include a header label: '{}'", nameId(), line);
            continue;
        }
        auto headerLabel = str::trim_copy(line.substr(60, 20));
        if (headerLabel == "PGM / RUN BY / DATE")
        {
            // Name of program creating current file
            LOG_DATA("{}: Program: {}", nameId(), str::trim_copy(line.substr(0, 20))); // FORMAT: A20
            // Name of agency creating current file
            LOG_DATA("{}: Run by : {}", nameId(), str::trim_copy(line.substr(20, 20))); // FORMAT: A20
            // Date and time of file creation
            LOG_DATA("{}: Date   : {}", nameId(), str::trim_copy(line.substr(40, 20))); // FORMAT: A20
        }
        else if (headerLabel == "COMMENT")
        {
            LOG_DATA("{}: Comment: {}", nameId(), line.substr(0, 60)); // FORMAT: A60
        }
        else if (headerLabel == "MARKER NAME")
        {
            auto markerName = str::trim_copy(line.substr(0, 60)); // FORMAT: A60
            if (!markerName.empty())
            {
                LOG_DATA("{}: Marker name: {}", nameId(), markerName);
            }
        }
        else if (headerLabel == "MARKER NUMBER")
        {
            auto markerNumber = str::trim_copy(line.substr(0, 20)); // FORMAT: A20
            if (!markerNumber.empty())
            {
                LOG_DATA("{}: Marker number: {}", nameId(), markerNumber);
            }
        }
        else if (headerLabel == "MARKER TYPE")
        {
            auto markerType = str::trim_copy(line.substr(0, 60)); // FORMAT: A20,40X
            if (!markerType.empty())
            {
                LOG_DATA("{}: Marker type: {}", nameId(), markerType);
            }
        }
        else if (headerLabel == "OBSERVER / AGENCY")
        {
            auto observer = str::trim_copy(line.substr(0, 20)); // FORMAT: A20,A40
            auto agency = str::trim_copy(line.substr(20, 40));
            if (!observer.empty() || !agency.empty())
            {
                LOG_DATA("{}: Observer '{}', Agency '{}'", nameId(), observer, agency);
            }
        }
        else if (headerLabel == "REC # / TYPE / VERS")
        {
            auto receiverNumber = str::trim_copy(line.substr(0, 20)); // FORMAT: 3A20
            auto receiverType = str::trim_copy(line.substr(20, 20));
            auto receiverVersion = str::trim_copy(line.substr(40, 20));
            if (!receiverNumber.empty() || !receiverType.empty() || !receiverVersion.empty())
            {
                LOG_DATA("{}: RecNum '{}', recType '{}', recVersion '{}'", nameId(),
                         receiverNumber, receiverType, receiverVersion);
            }
        }
        else if (headerLabel == "ANT # / TYPE")
        {
            auto antennaNumber = str::trim_copy(line.substr(0, 20)); // FORMAT: 2A20
            auto antennaType = str::trim_copy(line.substr(20, 20));
            if (!antennaNumber.empty() || !antennaType.empty())
            {
                LOG_DATA("{}: antNum '{}', antType '{}'", nameId(), antennaNumber, antennaType);
            }
        }
        else if (headerLabel == "APPROX POSITION XYZ")
        {
            // Geocentric approximate marker position (Units: Meters, System: ITRS recommended)
            // Optional for moving platforms
            [[maybe_unused]] Eigen::Vector3d position_xyz{ std::stod(str::trim_copy(line.substr(0, 14))),
                                                           std::stod(str::trim_copy(line.substr(14, 14))),
                                                           std::stod(str::trim_copy(line.substr(28, 14))) }; // FORMAT: 3F14.4

            LOG_DATA("{}: Approx Position XYZ: {} (not used yet)", nameId(), position_xyz.transpose());
        }
        else if (headerLabel == "ANTENNA: DELTA H/E/N")
        {
            // Antenna height: Height of the antenna reference point (ARP) above the marker [m]
            [[maybe_unused]] double antennaHeight = std::stod(str::trim_copy(line.substr(0, 14))); // FORMAT: F14.4,
            // Horizontal eccentricity of ARP relative to the marker (east) [m]
            [[maybe_unused]] double antennaEccentricityEast = std::stod(str::trim_copy(line.substr(14, 14))); // FORMAT: 2F14.4,
            // Horizontal eccentricity of ARP relative to the marker (north) [m]
            [[maybe_unused]] double antennaEccentricityNorth = std::stod(str::trim_copy(line.substr(28, 14)));

            LOG_DATA("{}: Antenna delta H/E/N: {}, {}, {} (not used yet)", nameId(),
                     antennaHeight, antennaEccentricityEast, antennaEccentricityNorth);
        }
        else if (headerLabel == "ANTENNA: DELTA X/Y/Z")
        {
            //  Position of antenna reference point for antenna on vehicle (m): XYZ vector in body-fixed coordinate system
            [[maybe_unused]] Eigen::Vector3d antennaDeltaXYZ{ std::stod(str::trim_copy(line.substr(0, 14))),
                                                              std::stod(str::trim_copy(line.substr(14, 14))),
                                                              std::stod(str::trim_copy(line.substr(28, 14))) }; // FORMAT: 3F14.4

            LOG_DATA("{}: Antenna Delta XYZ: {} (not used yet)", nameId(), antennaDeltaXYZ.transpose());
        }
        else if (headerLabel == "ANTENNA: PHASECENTER")
        {
            LOG_TRACE("{}: '{}' not implemented yet", nameId(), "ANTENNA: PHASECENTER");
        }
        else if (headerLabel == "ANTENNA: B.SIGHT XYZ")
        {
            LOG_TRACE("{}: '{}' not implemented yet", nameId(), "ANTENNA: B.SIGHT XYZ");
        }
        else if (headerLabel == "ANTENNA: ZERODIR AZI")
        {
            LOG_TRACE("{}: '{}' not implemented yet", nameId(), "ANTENNA: ZERODIR AZI");
        }
        else if (headerLabel == "ANTENNA: ZERODIR XYZ")
        {
            LOG_TRACE("{}: '{}' not implemented yet", nameId(), "ANTENNA: ZERODIR XYZ");
        }
        else if (headerLabel == "CENTER OF MASS: XYZ")
        {
            LOG_TRACE("{}: '{}' not implemented yet", nameId(), "CENTER OF MASS: XYZ");
        }
        else if (headerLabel == "SYS / # / OBS TYPES")
        {
            // Satellite system code (G/R/E/J/C/I/S) - FORMAT: A1,
            auto satSys = SatelliteSystem::fromChar(line.at(0));

            // Number of different observation types for the specified satellite system - Format: 2X,I3,
            size_t numSpecifications = std::stoul(line.substr(3, 3));

            std::string debugOutput;
            for (size_t n = 0, nLine = 1, i = 7; n < numSpecifications; n++, nLine++, i += 4)
            {
                // Observation descriptors: Type, Band, Attribute - FORMAT 13(1X,A3)

                ObsType type = obsTypeFromChar(line.at(i));
                Frequency freq = getFrequencyFromBand(satSys, line.at(i + 1) - '0');
                Code code = Code::fromFreqAttr(freq, line.at(i + 2));

                _obsDescription[satSys].push_back(ObservationDescription{ .type = type, .frequency = freq, .code = code });

                debugOutput += fmt::format("({},{},{})", obsTypeToChar(type), freq, code);

                if (nLine == 13)
                {
                    std::getline(_filestream, line);
                    nLine = 0;
                    i = 3;
                }
            }

            LOG_DATA("{}: Obs Type {} with {} specifications [{}]", nameId(),
                     satSys, numSpecifications, debugOutput);
        }
        else if (headerLabel == "SIGNAL STRENGTH UNIT")
        {
            LOG_TRACE("{}: '{}' not implemented yet", nameId(), "SIGNAL STRENGTH UNIT");
        }
        else if (headerLabel == "INTERVAL")
        {
            LOG_TRACE("{}: '{}' not implemented yet", nameId(), "INTERVAL");
        }
        else if (headerLabel == "TIME OF FIRST OBS")
        {
            [[maybe_unused]] auto year = std::stoi(line.substr(0, 6));
            [[maybe_unused]] auto month = std::stoi(line.substr(6, 6));
            [[maybe_unused]] auto day = std::stoi(line.substr(12, 6));
            [[maybe_unused]] auto hour = std::stoi(line.substr(18, 6));
            [[maybe_unused]] auto min = std::stoi(line.substr(24, 6));
            [[maybe_unused]] auto sec = std::stold(line.substr(30, 13));
            _timeSystem = TimeSystem::fromString(line.substr(30 + 13 + 5, 3));
            LOG_DATA("{}: Time of first obs: {} UTC (originally in '{}' time)", nameId(),
                     InsTime{ static_cast<uint16_t>(year),
                              static_cast<uint16_t>(month),
                              static_cast<uint16_t>(day),
                              static_cast<uint16_t>(hour),
                              static_cast<uint16_t>(min),
                              sec,
                              _timeSystem }
                         .toYMDHMS(),
                     std::string(_timeSystem));
        }
        else if (headerLabel == "TIME OF LAST OBS")
        {
            LOG_TRACE("{}: '{}' not implemented yet", nameId(), "TIME OF LAST OBS");
        }
        else if (headerLabel == "RCV CLOCK OFFS APPL")
        {
            if (std::stoi(line.substr(0, 5)))
            {
                _rcvClockOffsAppl = true;
                LOG_INFO("{}: Data (epoch, pseudorange, phase) corrected by the reported clock offset.", nameId());
            }
            LOG_TRACE("{}: Receiver clock offset applies: {}", nameId(), _rcvClockOffsAppl);
        }
        else if (headerLabel == "SYS / DCBS APPLIED")
        {
            LOG_TRACE("{}: '{}' not implemented yet", nameId(), "SYS / DCBS APPLIED");
        }
        else if (headerLabel == "SYS / PCVS APPLIED")
        {
            LOG_TRACE("{}: '{}' not implemented yet", nameId(), "SYS / PCVS APPLIED");
        }
        else if (headerLabel == "SYS / SCALE FACTOR")
        {
            LOG_TRACE("{}: '{}' not implemented yet", nameId(), "SYS / SCALE FACTOR");
        }
        else if (headerLabel == "SYS / PHASE SHIFT")
        {
            LOG_TRACE("{}: '{}' not implemented yet", nameId(), "SYS / PHASE SHIFT");
        }
        else if (headerLabel == "GLONASS SLOT / FRQ #")
        {
            LOG_TRACE("{}: '{}' not implemented yet", nameId(), "GLONASS SLOT / FRQ #");
        }
        else if (headerLabel == "GLONASS COD/PHS/BIS")
        {
            LOG_TRACE("{}: '{}' not implemented yet", nameId(), "GLONASS COD/PHS/BIS");
        }
        else if (headerLabel == "LEAP SECONDS")
        {
            LOG_TRACE("{}: '{}' not implemented yet", nameId(), "LEAP SECONDS");
        }
        else if (headerLabel == "# OF SATELLITES")
        {
            LOG_TRACE("{}: '{}' not implemented yet", nameId(), "# OF SATELLITES");
        }
        else if (headerLabel == "PRN / # OF OBS")
        {
            LOG_TRACE("{}: '{}' not implemented yet", nameId(), "PRN / # OF OBS");
        }
        else if (headerLabel == "END OF HEADER")
        {
            break;
        }
        else
        {
            LOG_ERROR("{}: Unknown header label '{}' in line '{}'", nameId(), headerLabel, line);
        }
    }

    if (_timeSystem == TimeSys_None) // If time system not set, try to apply default value
    {
        if (_obsDescription.size() == 1)
        {
            switch (SatelliteSystem_(_obsDescription.begin()->first))
            {
            case GPS:
                _timeSystem = GPST;
                break;
            case GLO:
                _timeSystem = GLNT;
                break;
            case GAL:
                _timeSystem = UTC;
                break;
            case QZSS:
                _timeSystem = QZSST;
                break;
            case BDS:
                _timeSystem = BDT;
                break;
            case IRNSS:
                _timeSystem = IRNSST;
                break;
            default:
                LOG_CRITICAL("{}: Could not determine time system of the file because satellite system '{}' has no default.",
                             nameId(), SatelliteSystem(_obsDescription.begin()->first));
                break;
            }
        }
        else
        {
            LOG_CRITICAL("{}: Could not determine time system of the file.", nameId());
        }
    }
}

std::shared_ptr<const NodeData> RinexObsFile::pollData(bool peek)
{
    //  Get current position
    auto pos = _filestream.tellg();

    std::string line;

    InsTime epochTime;

    // 0: OK | 1: power failure between previous and current epoch | > 1 : Special event
    int epochFlag = -1;
    while (epochFlag != 0 && !_filestream.eof() && std::getline(_filestream, line)) // Read lines till epoch record with valid epoch flag
    {
        if (line.empty())
        {
            continue;
        }
        if (line.at(0) == '>') // EPOCH record - Record identifier: > - Format: A1,
        {
            auto year = std::stoi(line.substr(2, 4));   // Format: 1X,I4,
            auto month = std::stoi(line.substr(7, 2));  // Format: 1X,I2.2,
            auto day = std::stoi(line.substr(10, 2));   // Format: 1X,I2.2,
            auto hour = std::stoi(line.substr(13, 2));  // Format: 1X,I2.2,
            auto min = std::stoi(line.substr(16, 2));   // Format: 1X,I2.2,
            auto sec = std::stold(line.substr(18, 11)); // Format: F11.7,

            epochTime = InsTime{ static_cast<uint16_t>(year), static_cast<uint16_t>(month), static_cast<uint16_t>(day),
                                 static_cast<uint16_t>(hour), static_cast<uint16_t>(min), sec,
                                 _timeSystem };

            epochFlag = std::stoi(line.substr(31, 1)); // Format: 2X,I1,

            [[maybe_unused]] auto numSats = std::stoi(line.substr(32, 3)); // Format: I3,
                                                                           // Reserved - Format 6X,
            [[maybe_unused]] double recClkOffset = 0.0;
            try
            {
                recClkOffset = line.size() >= 41 + 3 ? std::stod(line.substr(41, 15)) : 0.0; // Format: F15.12
            }
            catch (const std::exception& /* exception */)
            {
                LOG_DATA("{}: 'recClkOffset' not mentioned in file --> recClkOffset = {}", nameId(), recClkOffset);
            }

            LOG_DATA("{}: {}, epochFlag {}, numSats {}, recClkOffset {}", nameId(),
                     epochTime.toYMDHMS(), epochFlag, numSats, recClkOffset);

            if (peek)
            {
                auto obs = std::make_shared<NodeData>();
                obs->insTime = epochTime;
                _filestream.seekg(pos, std::ios_base::beg);
                return obs;
            }
        }
    }
    if (epochTime.empty())
    {
        return nullptr;
    }

    auto gnssObs = std::make_shared<GnssObs>();
    gnssObs->insTime = epochTime;

    // TODO: while loop till eof() or epochFlag == 0 (in case some other flags in the file)

    while (!_filestream.eof() && _filestream.peek() != '>' && std::getline(_filestream, line)) // Read observation records till line with '>'
    {
        if (line.empty())
        {
            continue;
        }
        auto satSys = SatelliteSystem::fromChar(line.at(0));              // Format: A1,
        auto satNum = static_cast<uint8_t>(std::stoi(line.substr(1, 2))); // Format: I2.2,

        LOG_DATA("{}: {}{}:", nameId(), char(satSys), satNum);

        size_t curExtractLoc = 3;
        for (const auto& obsDesc : _obsDescription.at(satSys))
        {
            if (line.size() < curExtractLoc + 14) // Remaining elements are all blank
            {
                break;
            }

            auto strObs = str::trim_copy(line.substr(curExtractLoc, 14)); // Format: F14.3
            curExtractLoc += 14;
            if (strObs.empty())
            {
                curExtractLoc += 2;
                continue;
            }
            // Observation value depending on definition type
            double observation = std::stod(strObs);

            // TODO: Springer Handbook of Global Navigation, p. 1211 prefer attributes over others and let user decide also which ones to take into the calculation

            // Loss of lock indicator
            // Bit 0 set: Lost lock between previous and current observation: Cycle slip possible.
            //            For phase observations only. Note: Bit 0 is the least significant bit.
            // Bit 1 set: Half-cycle ambiguity/slip possible. Software not capable of handling half
            //            cycles should skip this observation. Valid for the current epoch only.
            // Bit 2 set: Galileo BOC-tracking of an MBOC-modulated signal (may suffer from increased noise).
            int8_t LLI = 0;
            if (line.size() > curExtractLoc)
            {
                char LLIc = line.at(curExtractLoc);
                if (LLIc == ' ')
                {
                    LLIc = '0';
                }
                LLI = static_cast<int8_t>(LLIc - '0');
            }
            curExtractLoc++; // Go over Loss of lock indicator (LLI)

            // Signal Strength Indicator (SSI)
            //
            //   Carrier to Noise ratio(RINEX)      |  Carrier to Noise ratio(dbHz)
            // 1 (minimum possible signal strength) |             < 12
            //                  2                   |             12-17
            //                  3                   |             18-23
            //                  4                   |             24-29
            // 5 (average/good S/N ratio)           |             30-35
            //                  6                   |             36-41
            //                  7                   |             42-47
            //                  8                   |             48-53
            // 9 (maximum possible signal strength) |             ≥ 54
            // 0 or blank: not known, don't care    |               -
            [[maybe_unused]] uint8_t SSI = 0;
            if (line.size() > curExtractLoc)
            {
                char SSIc = line.at(curExtractLoc);
                if (SSIc == ' ')
                {
                    SSIc = '0';
                }
                SSI = static_cast<uint8_t>(SSIc - '0');
            }
            curExtractLoc++; // Go over Signal Strength Indicator (SSI)

            if (obsDesc.type == ObsType::C) // Code / Pseudorange
            {
                (*gnssObs)(obsDesc.frequency, satNum, obsDesc.code).pseudorange = observation;
            }
            else if (obsDesc.type == ObsType::L) // Phase
            {
                (*gnssObs)(obsDesc.frequency, satNum, obsDesc.code).carrierPhase = observation;
                if (LLI != 0)
                {
                    (*gnssObs)(obsDesc.frequency, satNum, obsDesc.code).LLI = LLI;
                }
            }
            else if (obsDesc.type == ObsType::D) // Doppler
            {
                (*gnssObs)(obsDesc.frequency, satNum, obsDesc.code).doppler = observation;
            }
            else if (obsDesc.type == ObsType::S) // Raw signal strength(carrier to noise ratio)
            {
                (*gnssObs)(obsDesc.frequency, satNum, obsDesc.code).CN0 = observation;
            }

            LOG_DATA("{}:     {}-{}-{}-{}: {}, LLI {}, SSI {}", nameId(),
                     obsTypeToChar(obsDesc.type), obsDesc.frequency, obsDesc.code, satNum,
                     observation, LLI, SSI);
        }

        if (!std::isnan(gnssObs->data.back().pseudorange))
        {
            if (std::isnan(gnssObs->data.back().carrierPhase))
            {
                LOG_WARN("{}: A data record at epoch {} (plus leap seconds) contains Pseudorange, but is missing carrier phase.", nameId(), epochTime.toYMDHMS());
            }
            if (std::isnan(gnssObs->data.back().doppler))
            {
                LOG_WARN("{}: A data record at epoch {} (plus leap seconds) contains Pseudorange, but is missing doppler.", nameId(), epochTime.toYMDHMS());
            }
            if (std::isnan(gnssObs->data.back().CN0))
            {
                LOG_WARN("{}: A data record at epoch {} (plus leap seconds) contains Pseudorange, but is missing raw signal strength(carrier to noise ratio).", nameId(), epochTime.toYMDHMS());
            }
        }
    }

    if (peek)
    {
        _filestream.seekg(pos, std::ios_base::beg);
    }
    else
    {
        invokeCallbacks(OutputPortIndex_GnssObs, gnssObs);
    }

    return gnssObs;
}

RinexObsFile::ObsType RinexObsFile::obsTypeFromChar(char c)
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

char RinexObsFile::obsTypeToChar(ObsType type)
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

Frequency RinexObsFile::getFrequencyFromBand(SatelliteSystem satSys, int band)
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

} // namespace NAV
