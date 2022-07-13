#include "RinexNavFile.hpp"

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"

#include "util/StringUtil.hpp"

#include "Navigation/GNSS/Core/SatelliteSystem.hpp"
#include "Navigation/GNSS/NavigationMessage/IonosphericCorrections.hpp"

namespace NAV
{

RinexNavFile::RinexNavFile()
{
    name = typeStatic();

    LOG_TRACE("{}: called", name);

    _hasConfig = true;
    _guiConfigDefaultWindowSize = { 517, 67 };

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
    if (auto res = FileReader::guiConfig("Rinex Nav (.nav .rnx .gal .geo .glo .*N){.nav,.rnx,.gal,.geo,.glo,(.+[.]\\d\\d?N)},.*", { ".nav", ".rnx", ".gal", ".geo", ".glo", "(.+[.]\\d\\d?N)" }, size_t(id), nameId()))
    {
        LOG_DEBUG("{}: Path changed to {}", nameId(), _path);
        flow::ApplyChanges();
        if (res == FileReader::PATH_CHANGED)
        {
            nm::InitializeNode(*this);
        }
        else
        {
            nm::DeinitializeNode(*this);
        }
    }
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

    _gnssNavInfo.ionosphericCorrections.clear();
    _gnssNavInfo.broadcastEphemeris.clear();
    _version = 0.0;
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
        std::getline(filestreamHeader, line);
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

        double version = std::stod(str::trim_copy(line.substr(0, 20))); // FORMAT: F9.2,11X
        if (!_supportedVersions.contains(version))
        {
            LOG_ERROR("{}: RINEX version {} is not supported. Supported versions are [{}]", nameId(),
                      version, fmt::join(_supportedVersions.begin(), _supportedVersions.end(), ", "));
            return FileReader::FileType::NONE;
        }

        std::string fileType = str::trim_copy(line.substr(20, 20)); // FORMAT: A1,19X
        if (fileType.at(0) != 'N')
        {
            LOG_ERROR("{}: Not a valid RINEX NAV file. File type '{}' not recognized.", nameId(), fileType);
            nm::DeinitializeNode(*this);
            return FileReader::FileType::NONE;
        }
        std::string satSystem = str::trim_copy(line.substr(40, 20)); // FORMAT: A1,19X
        if (SatelliteSystem::fromChar(satSystem.at(0)) == SatSys_None && satSystem.at(0) != 'M')
        {
            LOG_ERROR("{}: Not a valid RINEX NAV file. Satellite System '{}' not recognized.", nameId(), satSystem.at(0));
            nm::DeinitializeNode(*this);
            return FileReader::FileType::NONE;
        }
        // ---------------------------------------- PGM / RUN BY / DATE ------------------------------------------
        std::getline(filestreamHeader, line);
        if (extHeaderLabel(line) != "PGM / RUN BY / DATE")
        {
            LOG_ERROR("{}: Not a valid RINEX NAV file. Could not read 'PGM / RUN BY / DATE' line.", nameId());
            return FileReader::FileType::NONE;
        }

        // ----------------------------------------- END OF HEADER -------------------------------------------
        while (std::getline(filestreamHeader, line) && !_filestream.eof())
        {
            if (extHeaderLabel(line) == "END OF HEADER")
            {
                return FileReader::FileType::CSV;
            }
        }
        LOG_ERROR("{}: Not a valid RINEX NAV file. Could not read 'END OF HEADER' line.", nameId());
        return FileReader::FileType::NONE;
    }

    LOG_ERROR("{}: Could not determine file type because file could not be opened '{}' line.", nameId(), filepath.string());
    return FileReader::FileType::NONE;
}

void RinexNavFile::readHeader()
{
    LOG_TRACE("{}: called", nameId());

    std::string line;
    auto extHeaderLabel = [](const std::string& line) {
        return line.size() >= 60 ? str::trim_copy(std::string_view(line).substr(60, 20))
                                 : std::string_view{};
    };

    // --------------------------------------- RINEX VERSION / TYPE ------------------------------------------
    std::getline(_filestream, line);
    _version = std::stod(str::trim_copy(line.substr(0, 20)));
    LOG_DEBUG("{}: Version: {:3.2f}", nameId(), _version);                       // FORMAT: F9.2,11X
    LOG_DEBUG("{}: SatSys : {}", nameId(), str::trim_copy(line.substr(40, 20))); // FORMAT: A1,19X

    // #######################################################################################################
    while (std::getline(_filestream, line) && !_filestream.eof())
    {
        auto headerLabel = extHeaderLabel(line);
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
        else if (headerLabel == "IONOSPHERIC CORR")
        {
            auto correctionType = str::trim_copy(line.substr(0, 4)); // FORMAT: A4,1X,
            auto valuesStr = str::split(str::replaceAll_copy(line.substr(4, 60 - 4), "d", "e", str::IgnoreCase), " ");
            std::vector<double> values(valuesStr.size());
            for (size_t i = 0; i < valuesStr.size(); i++)
            {
                values.at(i) = std::stod(valuesStr.at(i));
            }

            auto satSys = SatelliteSystem::fromString(correctionType.substr(0, 3));
            IonosphericCorrections::AlphaBeta alphaBeta = correctionType.size() >= 4
                                                              ? (correctionType.at(3) == 'B'
                                                                     ? IonosphericCorrections::B
                                                                     : IonosphericCorrections::A)
                                                              : IonosphericCorrections::A;
            _gnssNavInfo.ionosphericCorrections.insert(satSys, alphaBeta, values);

            LOG_DATA("{}: Ionospheric Correction: {}-{}: [{}]", nameId(),
                     std::string(satSys), alphaBeta == IonosphericCorrections::A ? "A" : "B",
                     fmt::join(values.begin(), values.end(), ", "));
        }
        else if (headerLabel == "TIME SYSTEM CORR")
        {
            auto correctionType = str::trim_copy(line.substr(0, 4)); // FORMAT: A4,1X,
            LOG_DATA("{}: Time System Correction: {}", nameId(), correctionType);

            if (correctionType == "GPUT" || correctionType == "GAUT" || correctionType == "BDUT" || correctionType == "QZUT"
                || correctionType == "IRUT" || correctionType == "SBUT")
            {
                auto a0 = std::stod(str::replaceAll_copy(line.substr(5, 17), "d", "e", str::IgnoreCase));
                LOG_DATA("{}:     a0 {}", nameId(), a0);
                auto a1 = std::stod(str::replaceAll_copy(line.substr(22, 16), "d", "e", str::IgnoreCase));
                LOG_DATA("{}:     a1 {}", nameId(), a1);

                SatelliteSystem satSys = SatelliteSystem::fromString(correctionType.substr(0, 2));

                _gnssNavInfo.timeSysCorr[satSys] = { a0, a1 };
            }
            else if (correctionType == "GLUT")
            {
                auto tau_c = std::stod(str::replaceAll_copy(line.substr(5, 19), "d", "e", str::IgnoreCase));
                LOG_DATA("{}:     tau_c {}", nameId(), tau_c);
                auto a1 = std::stod(str::replaceAll_copy(line.substr(24, 19), "d", "e", str::IgnoreCase));
                LOG_DATA("{}:     a1    {}", nameId(), a1);

                _gnssNavInfo.timeSysCorr[GLO] = { tau_c, a1 };
            }
            else
            {
                LOG_TRACE("{}:     Not implemented yet", nameId());
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

void RinexNavFile::readOrbits()
{
    LOG_TRACE("{}: called", nameId());

    std::string line;

    while (std::getline(_filestream, line) && !_filestream.eof())
    {
        // -------------------------------------- SV / EPOCH / SV CLK ----------------------------------------

        // Satellite system (G), sat number (PRN) - A1,I2.2,
        // Satellite system (E), satellite number - A1,I2.2,
        // Satellite system (R), satellite number (slot number in sat. constellation) - A1,I2.2,
        // Satellite system (J), Satellite PRN-192 - A1,I2,
        // Satellite system (C), sat number (PRN) - A1,I2.2,
        // Satellite system (S), satellite number (slot number in sat. constellation) - A1,I2.2,
        // Satellite system (I), sat number (PRN) - A1,I2.2,
        SatelliteSystem satSys = SatelliteSystem::fromChar(line.at(0));
        _gnssNavInfo.satelliteSystems |= satSys;
        auto satNum = static_cast<uint8_t>(std::stoi(line.substr(1, 2)));

        // Epoch: Toc - Time of Clock (GPS) year (4 digits) - 1X,I4,
        // month, day, hour, minute, second - 5(1X,I2.2),
        auto timeSplit = str::split(line.substr(3, 20), " ");
        auto timeSystem = satSys.getTimeSystem();
        InsTime epoch{ static_cast<uint16_t>(std::stoi(timeSplit.at(0))),
                       static_cast<uint16_t>(std::stoi(timeSplit.at(1))),
                       static_cast<uint16_t>(std::stoi(timeSplit.at(2))),
                       static_cast<uint16_t>(std::stoi(timeSplit.at(3))),
                       static_cast<uint16_t>(std::stoi(timeSplit.at(4))),
                       std::stold(timeSplit.at(5)),
                       timeSystem };

        LOG_DATA("{}: {}-{} {} (read as {} time)", nameId(), satSys, satNum,
                 epoch.toYMDHMS(), std::string(timeSystem));

        if (satSys == GPS || satSys == GAL || satSys == QZSS || satSys == BDS) // NOLINT(misc-redundant-expression) // bugged warning
        {
            GPSEphemeris ephemeris{};
            // Toc - Time of clock
            ephemeris.toc = epoch;

            // SV clock bias [seconds]
            ephemeris.a[0] = std::stod(str::replaceAll_copy(line.substr(23, 19), "d", "e", str::IgnoreCase));
            // SV clock drift [sec/sec]
            ephemeris.a[1] = std::stod(str::replaceAll_copy(line.substr(42, 19), "d", "e", str::IgnoreCase));
            // SV clock drift rate [sec/sec^2]
            ephemeris.a[2] = std::stod(str::replaceAll_copy(line.substr(61, 19), "d", "e", str::IgnoreCase));
            LOG_DATA("{}:    clkBias {}, clkDrift {}, clkDriftRate {}", nameId(),
                     ephemeris.a[0], ephemeris.a[1], ephemeris.a[2]);

            // ------------------------------------ BROADCAST ORBIT - 1 --------------------------------------
            std::getline(_filestream, line);

            // GPS/QZSS: Issue of Data, Ephemeris (IODE)
            // GAL:      IODnav Issue of Data of the nav batch
            // BDS:      AODE Age of Data, Ephemeris (as specified in BeiDou ICD Table Section 5.2.4.11 Table 5-8)
            //           and field range is: 0-31.
            // IRNSS:    IODEC Issue of Data, Ephemeris and Clock
            [[maybe_unused]] auto IODE = std::stod(str::replaceAll_copy(line.substr(4, 19), "d", "e", str::IgnoreCase));
            // Crs (meters)
            ephemeris.Crs = std::stod(str::replaceAll_copy(line.substr(23, 19), "d", "e", str::IgnoreCase));
            // Delta n (radians/sec)
            ephemeris.delta_n = std::stod(str::replaceAll_copy(line.substr(42, 19), "d", "e", str::IgnoreCase));
            // M0 (radians)
            ephemeris.M_0 = std::stod(str::replaceAll_copy(line.substr(61, 19), "d", "e", str::IgnoreCase));

            if (satSys == GAL)
            {
                LOG_DATA("{}:    IODnav {}, Crs {}, Delta_n {}, M0 {}", nameId(),
                         IODE, ephemeris.Crs, ephemeris.delta_n, ephemeris.M_0);
            }
            else if (satSys == BDS)
            {
                LOG_DATA("{}:    AODE {}, Crs {}, Delta_n {}, M0 {}", nameId(),
                         IODE, ephemeris.Crs, ephemeris.delta_n, ephemeris.M_0);
            }
            else if (satSys == IRNSS)
            {
                LOG_DATA("{}:    IODEC {}, Crs {}, Delta_n {}, M0 {}", nameId(),
                         IODE, ephemeris.Crs, ephemeris.delta_n, ephemeris.M_0);
            }
            else
            {
                LOG_DATA("{}:    IODE {}, Crs {}, Delta_n {}, M0 {}", nameId(),
                         IODE, ephemeris.Crs, ephemeris.delta_n, ephemeris.M_0);
            }

            // ------------------------------------ BROADCAST ORBIT - 2 --------------------------------------
            std::getline(_filestream, line);

            // Cuc (radians)
            ephemeris.Cuc = std::stod(str::replaceAll_copy(line.substr(4, 19), "d", "e", str::IgnoreCase));
            // e Eccentricity
            ephemeris.e = std::stod(str::replaceAll_copy(line.substr(23, 19), "d", "e", str::IgnoreCase));
            // Cus (radians)
            ephemeris.Cus = std::stod(str::replaceAll_copy(line.substr(42, 19), "d", "e", str::IgnoreCase));
            // sqrt(A) (sqrt(m))
            ephemeris.sqrt_A = std::stod(str::replaceAll_copy(line.substr(61, 19), "d", "e", str::IgnoreCase));

            LOG_DATA("{}:    Cuc {}, e {}, Cus {}, sqrt_A {}", nameId(),
                     ephemeris.Cuc, ephemeris.e, ephemeris.Cus, ephemeris.sqrt_A);

            // ------------------------------------ BROADCAST ORBIT - 3 --------------------------------------
            std::getline(_filestream, line);

            // Toe Time of Ephemeris (sec of GPS week)
            auto toe = std::stod(str::replaceAll_copy(line.substr(4, 19), "d", "e", str::IgnoreCase));
            // Cic (radians)
            ephemeris.Cic = std::stod(str::replaceAll_copy(line.substr(23, 19), "d", "e", str::IgnoreCase));
            // OMEGA0 (radians)
            ephemeris.Omega_0 = std::stod(str::replaceAll_copy(line.substr(42, 19), "d", "e", str::IgnoreCase));
            // Cis (radians)
            ephemeris.Cis = std::stod(str::replaceAll_copy(line.substr(61, 19), "d", "e", str::IgnoreCase));

            LOG_DATA("{}:    toe {}, Cic {}, Omega_0 {}, Cis {}", nameId(),
                     toe, ephemeris.Cic, ephemeris.Omega_0, ephemeris.Cis);

            // ------------------------------------ BROADCAST ORBIT - 4 --------------------------------------
            std::getline(_filestream, line);

            // i0 (radians)
            ephemeris.i_0 = std::stod(str::replaceAll_copy(line.substr(4, 19), "d", "e", str::IgnoreCase));
            // Crc (meters)
            ephemeris.Crc = std::stod(str::replaceAll_copy(line.substr(23, 19), "d", "e", str::IgnoreCase));
            // omega (radians)
            ephemeris.omega = std::stod(str::replaceAll_copy(line.substr(42, 19), "d", "e", str::IgnoreCase));
            // OMEGA DOT (radians/sec)
            ephemeris.Omega_dot = std::stod(str::replaceAll_copy(line.substr(61, 19), "d", "e", str::IgnoreCase));

            LOG_DATA("{}:    i_0 {}, Crc {}, omega {}, Omega_dot {}", nameId(),
                     ephemeris.i_0, ephemeris.Crc, ephemeris.omega, ephemeris.Omega_dot);

            // ------------------------------------ BROADCAST ORBIT - 5 --------------------------------------
            std::getline(_filestream, line);

            // IDOT (radians/sec)
            ephemeris.i_dot = std::stod(str::replaceAll_copy(line.substr(4, 19), "d", "e", str::IgnoreCase));
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
            [[maybe_unused]] auto codesOnL2Channel = std::stod(str::replaceAll_copy(line.substr(23, 19), "d", "e", str::IgnoreCase));
            // GPS Week # (to go with TOE) Continuous number, not mod(1024)!
            auto gpsWeek = static_cast<int32_t>(std::stod(str::replaceAll_copy(line.substr(42, 19), "d", "e", str::IgnoreCase)));
            ephemeris.toe = InsTime(0, gpsWeek, toe);

            // GPS:  L2 P data flag
            // QZSS: L2P data flag set to 1 since QZSS does not track L2P
            // GAL:  Spare
            [[maybe_unused]] auto L2PdataFlag = std::stod(str::replaceAll_copy(line.substr(61, 19), "d", "e", str::IgnoreCase));

            if (satSys == GAL)
            {
                LOG_DATA("{}:    i_dot {}, dataSources {}, gpsWeek {}", nameId(),
                         ephemeris.i_dot, codesOnL2Channel, gpsWeek);
            }
            else if (satSys == BDS || satSys == IRNSS) // NOLINT(misc-redundant-expression) // bugged warning
            {
                LOG_DATA("{}:    i_dot {}, gpsWeek {}", nameId(),
                         ephemeris.i_dot, gpsWeek);
            }
            else
            {
                LOG_DATA("{}:    i_dot {}, codesOnL2Channel {}, gpsWeek {}, L2PdataFlag {}", nameId(),
                         ephemeris.i_dot, codesOnL2Channel, gpsWeek, L2PdataFlag);
            }

            // ------------------------------------ BROADCAST ORBIT - 6 --------------------------------------
            std::getline(_filestream, line);

            // GPS:  SV accuracy (meters) See GPS ICD 200H Section 20.3.3.3.1.3 use specified equations to define
            //       nominal values, N = 0-6: use 2(1+N/2) (round to one decimal place i.e. 2.8, 5.7 and 11.3) ,
            //       N= 7-15:use 2 (N-2), 8192 specifies use at own risk
            // QZSS: SV accuracy (meters) (IS -QZSS-PNT, Section 5.4.3.1) which refers to: IS GPS 200H Section 20.3.3.3.1.3
            //       use specified equations to define nominal values, N = 0-6: use 2(1+N/2) (round to one decimal
            //       place i.e. 2.8, 5.7 and 11.3) , N= 7-15:use 2 (N-2), 8192 specifies use at own risk
            // IRNSS: User Range Accuracy(m), See IRNSS ICD Section 6.2.1.4 , use specified equations to define
            //        nominal values, N = 0-6: use 2(1+N/2) (round to one decimal place i.e. 2.8, 5.7 and 11.3) ,
            //        N= 7-15:use 2 (N-2), 8192 specifies use at own risk
            ephemeris.signalAccuracy = std::stod(str::replaceAll_copy(line.substr(4, 19), "d", "e", str::IgnoreCase));
            // GPS/QZSS: SV health (bits 17-22 w 3 sf 1)
            // GAL:      SV health (FLOAT converted to INTEGER) See Galileo ICD Section 5.1.9.3
            //           Bit 0: E1B DVS       Bits 1-2: E1B HS          Bit 3: E5a DVS
            //           Bits 4-5: E5a HS     Bit 6: E5b DVS            Bits 7-8: E5b HS
            // BDS:      SatH1
            // IRNSS:    Health (Sub frame 1,bits 155(most significant) and 156(least significant)),
            //           where 0 = L5 and S healthy, 1 = L5 healthy and S unhealthy, 2= L5 unhealthy
            //           and S healthy, 3= both L5 and S unhealthy
            ephemeris.svHealth = static_cast<uint16_t>(std::stod(str::replaceAll_copy(line.substr(23, 19), "d", "e", str::IgnoreCase)));
            // GPS:  TGD (seconds)
            // QZSS: TGD (seconds) The QZSS ICD specifies a do not use bit pattern "10000000" this condition is represented by a blank field.
            // GAL:  BGD E5a/E1 (seconds)
            // BDS:  TGD1 B1/B3 (seconds)
            auto tgd = std::stod(str::replaceAll_copy(line.substr(42, 19), "d", "e", str::IgnoreCase));
            if (satSys == GPS)
            {
                ephemeris.T_GD = tgd;
            }
            else if (satSys == GAL)
            {
                ephemeris.BGD_E1_E5a = tgd;
            }
            // GPS/QZSS: IODC Issue of Data, Clock
            // GAL:      BGD E5b/E1 (seconds)
            // BDS:      TGD2 B2/B3 (seconds)
            // IRNSS:    Blank
            auto IODC = std::stod(str::replaceAll_copy(line.substr(61, 19), "d", "e", str::IgnoreCase));
            if (satSys == GAL)
            {
                ephemeris.BGD_E1_E5b = IODC;
            }

            if (satSys == GAL)
            {
                LOG_DATA("{}:    sisaAccuracy {}, svHealth {}, bgd_e5a {}, bgd_e5b {}", nameId(),
                         ephemeris.signalAccuracy, ephemeris.svHealth, ephemeris.BGD_E1_E5a, ephemeris.BGD_E1_E5b);
            }
            else if (satSys == BDS)
            {
                LOG_DATA("{}:    svAccuracy {}, SatH1 {}, TGD1 B1/B3 {}, TGD2 B2/B3 {}", nameId(),
                         ephemeris.signalAccuracy, ephemeris.svHealth, tgd, IODC);
            }
            else if (satSys == IRNSS)
            {
                LOG_DATA("{}:    userRangeAccuracy {}, health {}, TGD {}", nameId(),
                         ephemeris.signalAccuracy, ephemeris.svHealth, tgd);
            }
            else
            {
                LOG_DATA("{}:    svAccuracy {}, svHealth {}, TGD {}, IODC {}", nameId(),
                         ephemeris.signalAccuracy, ephemeris.svHealth, ephemeris.T_GD, IODC);
            }

            // ------------------------------------ BROADCAST ORBIT - 7 --------------------------------------
            std::getline(_filestream, line);

            // Transmission time of message (sec of GPS week, derived e.g.from Z-count in Hand Over Word (HOW))
            [[maybe_unused]] auto transmissionTime = std::stod(str::replaceAll_copy(line.substr(4, 19), "d", "e", str::IgnoreCase));
            // GPS/QZSS: Fit Interval in hours see section 6.11. (BNK).
            // GAL:      Spare
            // BDS:      AODC Age of Data Clock (as specified in BeiDou ICD Table Section 5.2.4.9 Table 5-6) and field range is: 0-31.
            [[maybe_unused]] auto fitInterval = std::stod(str::replaceAll_copy(line.substr(23, 19), "d", "e", str::IgnoreCase));
            // Spare
            // std::stod(str::replaceAll_copy(line.substr(42, 19), "d", "e", str::IgnoreCase));
            // Spare
            // std::stod(str::replaceAll_copy(line.substr(61, 19), "d", "e", str::IgnoreCase));

            if (satSys == GAL || satSys == IRNSS) // NOLINT(misc-redundant-expression) // bugged warning
            {
                LOG_DATA("{}:    transmissionTime {}", nameId(),
                         transmissionTime);
            }
            else if (satSys == BDS)
            {
                LOG_DATA("{}:    transmissionTime {}, AODC {}", nameId(),
                         transmissionTime, fitInterval);
            }
            else
            {
                LOG_DATA("{}:    transmissionTime {}, fitInterval {}", nameId(),
                         transmissionTime, fitInterval);
            }

            _gnssNavInfo.broadcastEphemeris[{ satSys, satNum }].push_back(std::make_pair(epoch, ephemeris));
        }
        else if (satSys == GLO || satSys == SBAS) // NOLINT(misc-redundant-expression) // bugged warning
        {
            GLONASSEphemeris ephemeris{};
            // Toc - Time of clock
            ephemeris.toc = epoch;
            // Coefficient of linear polynomial of time system difference [s]
            ephemeris.tau_c = _gnssNavInfo.timeSysCorr.at(GLO).a0;

            // GLO:  SV clock bias (sec) (-TauN)
            // SBAS: SV clock bias (sec) (aGf0)
            ephemeris.tau_n = std::stod(str::replaceAll_copy(line.substr(23, 19), "d", "e", str::IgnoreCase));
            // GLO:  SV relative frequency bias (+GammaN)
            // SBAS: SV relative frequency bias (aGf1)
            ephemeris.gamma_n = std::stod(str::replaceAll_copy(line.substr(42, 19), "d", "e", str::IgnoreCase));
            // GLO:  Message frame time (tk+nd*86400) in seconds of the UTC week
            // SBAS: Transmission time of message (start of the message) in GPS seconds of the week
            [[maybe_unused]] auto msgFrameTime = std::stod(str::replaceAll_copy(line.substr(61, 19), "d", "e", str::IgnoreCase));
            LOG_DATA("{}:    clkBias {}, relFreqBias {}, msgFrameTime {}", nameId(),
                     ephemeris.tau_n, ephemeris.gamma_n, msgFrameTime);

            // ------------------------------------ BROADCAST ORBIT - 1 --------------------------------------
            std::getline(_filestream, line);

            // Satellite position X (km)
            ephemeris.pos.x() = std::stod(str::replaceAll_copy(line.substr(4, 19), "d", "e", str::IgnoreCase)) * 1e3;
            // velocity X dot (km/sec)
            ephemeris.vel.x() = std::stod(str::replaceAll_copy(line.substr(23, 19), "d", "e", str::IgnoreCase)) * 1e3;
            // X acceleration (km/sec2)
            ephemeris.accelLuniSolar.x() = std::stod(str::replaceAll_copy(line.substr(42, 19), "d", "e", str::IgnoreCase)) * 1e3;
            // GLO:  health (0=healthy, 1=unhealthy) (MSB of 3-bit Bn)
            // SBAS: Health: SBAS: See Section 8.3.3 bit mask for: health, health availability and User Range Accuracy.
            [[maybe_unused]] auto health = std::stod(str::replaceAll_copy(line.substr(61, 19), "d", "e", str::IgnoreCase));

            LOG_DATA("{}:    satPosX {}, velX {}, accelX {}, health {}", nameId(),
                     ephemeris.pos.x(), ephemeris.vel.x(), ephemeris.accelLuniSolar.x(), health);

            // ------------------------------------ BROADCAST ORBIT - 2 --------------------------------------
            std::getline(_filestream, line);

            // Satellite position Y (km)
            ephemeris.pos.y() = std::stod(str::replaceAll_copy(line.substr(4, 19), "d", "e", str::IgnoreCase)) * 1e3;
            // velocity Y dot (km/sec)
            ephemeris.vel.y() = std::stod(str::replaceAll_copy(line.substr(23, 19), "d", "e", str::IgnoreCase)) * 1e3;
            // Y acceleration (km/sec2)
            ephemeris.accelLuniSolar.y() = std::stod(str::replaceAll_copy(line.substr(42, 19), "d", "e", str::IgnoreCase)) * 1e3;
            // GLO:  frequency number(-7...+13) (-7...+6 ICD 5.1)
            // SBAS: Accuracy code (URA, meters)
            ephemeris.frequencyNumber = static_cast<int8_t>(std::stod(str::replaceAll_copy(line.substr(61, 19), "d", "e", str::IgnoreCase)));

            if (satSys == GLO)
            {
                LOG_DATA("{}:    satPosY {}, velY {}, accelY {}, freqNum {}", nameId(),
                         ephemeris.pos.y(), ephemeris.vel.y(), ephemeris.accelLuniSolar.y(), static_cast<int>(ephemeris.frequencyNumber));
            }
            else
            {
                LOG_DATA("{}:    satPosY {}, velY {}, accelY {}, accuracyCode {}", nameId(),
                         ephemeris.pos.y(), ephemeris.vel.y(), ephemeris.accelLuniSolar.y(), static_cast<int>(ephemeris.frequencyNumber));
            }

            // ------------------------------------ BROADCAST ORBIT - 3 --------------------------------------
            std::getline(_filestream, line);

            // Satellite position Z (km)
            ephemeris.pos.z() = std::stod(str::replaceAll_copy(line.substr(4, 19), "d", "e", str::IgnoreCase)) * 1e3;
            // velocity Z dot (km/sec)
            ephemeris.vel.z() = std::stod(str::replaceAll_copy(line.substr(23, 19), "d", "e", str::IgnoreCase)) * 1e3;
            // Z acceleration (km/sec2)
            ephemeris.accelLuniSolar.z() = std::stod(str::replaceAll_copy(line.substr(42, 19), "d", "e", str::IgnoreCase)) * 1e3;
            // GLO:  Age of oper. information (days) (E)
            // SBAS: IODN (Issue of Data Navigation, DO229, 8 first bits after Message Type if MT9)
            [[maybe_unused]] auto ageOfOperInfo = std::stod(str::replaceAll_copy(line.substr(61, 19), "d", "e", str::IgnoreCase));

            if (satSys == GLO)
            {
                LOG_DATA("{}:    satPosZ {}, velZ {}, accelZ {}, ageOfOperInfo {}", nameId(),
                         ephemeris.pos.z(), ephemeris.vel.z(), ephemeris.accelLuniSolar.z(), ageOfOperInfo);
            }
            else
            {
                LOG_DATA("{}:    satPosZ {}, velZ {}, accelZ {}, IODN {}", nameId(),
                         ephemeris.pos.z(), ephemeris.vel.z(), ephemeris.accelLuniSolar.z(), ageOfOperInfo);
            }

            _gnssNavInfo.broadcastEphemeris[{ satSys, satNum }].push_back(std::make_pair(epoch, ephemeris));
        }
        // else if (satSys == )
        // {
        //     // TODO: Support RINEX Nav messages for QZSS
        //     LOG_WARN("{}: QZSS is not supported yet!!!", nameId());
        // }
    }
}

} // namespace NAV
