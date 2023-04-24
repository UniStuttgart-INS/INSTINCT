// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "NmeaFile.hpp"

#include "util/Logger.hpp"
#include "Navigation/Transformations/CoordinateFrames.hpp"
#include "Navigation/Transformations/Units.hpp"
#include "util/Time/TimeBase.hpp"
#include "util/StringUtil.hpp"

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"
#include "NodeData/State/PosVel.hpp"

NAV::NmeaFile::NmeaFile()
    : Node(typeStatic())
{
    LOG_TRACE("{}: called", name);

    _hasConfig = true;
    _guiConfigDefaultWindowSize = { 100, 290 };

    nm::CreateOutputPin(this, "PosVel", Pin::Type::Flow, { NAV::PosVel::type() }, &NmeaFile::pollData);
}

NAV::NmeaFile::~NmeaFile()
{
    LOG_TRACE("{}: called", nameId());
}

std::string NAV::NmeaFile::typeStatic()
{
    return "NmeaFile";
}

std::string NAV::NmeaFile::type() const
{
    return typeStatic();
}

std::string NAV::NmeaFile::category()
{
    return "Data Provider";
}

void NAV::NmeaFile::guiConfig()
{
    if (auto res = FileReader::guiConfig(".*", { ".*" }, size_t(id), nameId()))
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

[[nodiscard]] json NAV::NmeaFile::save() const
{
    LOG_TRACE("{}: called", nameId());

    json j;

    j["FileReader"] = FileReader::save();

    return j;
}

void NAV::NmeaFile::restore(json const& j)
{
    LOG_TRACE("{}: called", nameId());

    if (j.contains("FileReader"))
    {
        FileReader::restore(j.at("FileReader"));
    }
}

bool NAV::NmeaFile::initialize()
{
    LOG_TRACE("{}: called", nameId());

    return FileReader::initialize();
}

void NAV::NmeaFile::deinitialize()
{
    LOG_TRACE("{}: called", nameId());

    FileReader::deinitialize();
}

bool NAV::NmeaFile::resetNode()
{
    FileReader::resetReader();

    _hasValidDate = false;
    _oldSoD = -1.0;

    return true;
}

bool NAV::NmeaFile::setDateFromZDA(const std::string& line)
{
    // decode ZDA string according to http://www.nmea.de/nmea0183datensaetze.html#zda

    //   0    1         2  3  4    5  6
    //   |    |         |  |  |    |  |
    // $--ZDA,hhmmss.ss,xx,xx,xxxx,xx,xx*hh<CR><LF>
    std::vector<std::string> splittedString = str::split(line, ",");
    if (splittedString.size() == 7)
    {
        std::size_t pos_star = splittedString.back().find('*');
        if (pos_star != std::string::npos)
        {
            int64_t crc = std::strtol(splittedString[6].substr(pos_star + 1).c_str(), nullptr, 16);
            // checksum calculation similar to https://gist.github.com/devendranaga/fce8e166f4335fa777650493cb9246db
            int64_t mycrc = 0;
            pos_star = line.find('*');
            for (unsigned int i = 1; i < pos_star; i++)
            {
                mycrc ^= line.at(i);
            }
            if (mycrc == crc)
            {
                _currentDate.day = std::stoi(splittedString[2]);
                _currentDate.month = std::stoi(splittedString[3]);
                _currentDate.year = std::stoi(splittedString[4]);

                _hasValidDate = true;
                return true;
            }
        }
    }
    return false;
}

bool NAV::NmeaFile::setDateFromRMC(const std::string& line)
{
    // decode RMC string according to http://www.nmea.de/nmea0183datensaetze.html#rmc

    //   0    1         2 3       4 5        6 7   8   9    10  11
    //   |    |         | |       | |        | |   |   |    |   |
    // $--RMC,hhmmss.ss,A,llll.ll,a,yyyyy.yy,a,x.x,x.x,xxxx,x.x,a*hh<CR><LF>
    std::vector<std::string> splittedString = str::split(line, ",");
    if (splittedString.size() == 12)
    {
        std::size_t pos_star = splittedString[11].find('*');
        if (pos_star != std::string::npos)
        {
            int64_t crc = std::strtol(splittedString[11].substr(pos_star + 1).c_str(), nullptr, 16);
            // checksum calculation similar to https://gist.github.com/devendranaga/fce8e166f4335fa777650493cb9246db
            int64_t mycrc = 0;
            pos_star = line.find('*');
            for (unsigned int i = 1; i < pos_star; i++)
            {
                mycrc ^= line.at(i);
            }
            if (mycrc == crc)
            {
                _currentDate.day = std::stoi(splittedString[9].substr(0, 2));
                _currentDate.month = std::stoi(splittedString[9].substr(2, 2));
                _currentDate.year = std::stoi(splittedString[9].substr(4, 2));
                if (_currentDate.year > 60)
                {
                    _currentDate.year += 1900;
                }
                else
                {
                    _currentDate.year += 2000;
                }

                _hasValidDate = true;
                return true;
            }
        }
    }
    return false;
}

std::shared_ptr<const NAV::NodeData> NAV::NmeaFile::pollData()
{
    auto obs = std::make_shared<PosVel>();

    // Read line
    std::string line;

    std::vector<std::string> splittedData;

    int hour = 0;
    int minute = 0;
    double second = 0.0;

    double lat_rad = 0.0;
    double lon_rad = 0.0;
    double hgt = 0.0;

    while (true)
    {
        getline(line);

        if (eof())
        {
            return nullptr; // when done with file reading
        }

        // Remove any starting non text characters
        line.erase(line.begin(), std::find_if(line.begin(), line.end(), [](int ch) { return std::isgraph(ch); }));

        splittedData = str::split(line, ",");

        if (splittedData[0].substr(0, 1) == "$")
        {
            if (_hasValidDate && splittedData[0].substr(3, 3) == "GGA")
            {
                // decode GGA stream according to http://www.nmea.de/nmea0183datensaetze.html#gga

                //   0    1         2       3 4        5 6 7  8   9  10 11 12 13  14
                //   |    |         |       | |        | | |  |   |   |  |  | |   |
                // $--GGA,hhmmss.ss,llll.ll,a,yyyyy.yy,a,x,xx,x.x,x.x,M,x.x,M,x.x,xxxx*hh<CR><LF>
                if (splittedData.size() != 15) { continue; }

                std::size_t pos_star = splittedData[14].find('*');
                int64_t crc = std::strtol(splittedData[14].substr(pos_star + 1).c_str(), nullptr, 16);
                // checksum calculation similar to https://gist.github.com/devendranaga/fce8e166f4335fa777650493cb9246db
                int64_t mycrc = 0;
                pos_star = line.find('*');
                for (unsigned int i = 1; i < pos_star; i++)
                {
                    mycrc ^= line.at(i);
                }
                if (mycrc == crc)
                {
                    hour = std::stoi(splittedData[1].substr(0, 2));
                    minute = std::stoi(splittedData[1].substr(2, 2));
                    second = std::stod(splittedData[1].substr(4));
                    double newSOD = hour * 24 * 60.0 + minute * 60.0 + second;

                    // only continue if second of day > than previous one
                    if (newSOD < _oldSoD)
                    {
                        _oldSoD = newSOD;      // store current second of day for next call of this routine
                        _hasValidDate = false; // force wait until next ZDA stream
                        continue;
                    }

                    double lat1 = std::stod(splittedData[2].substr(0, 2));
                    double lat2 = std::stod(splittedData[2].substr(2));

                    lat_rad = (lat1 + lat2 / 60.0) / 180.0 * M_PI; // convert to radian

                    if (splittedData[3] == "S") // flip sign if south latitude
                    {
                        lat_rad *= -1.0;
                    }

                    double lon1 = std::stoi(splittedData[4].substr(0, 3));
                    double lon2 = std::stod(splittedData[4].substr(3));

                    lon_rad = (lon1 + lon2 / 60.0) / 180.0 * M_PI; // convert to radian

                    if (splittedData[5] == "W") // flip sign if west longitude
                    {
                        lon_rad *= -1.0;
                    }

                    hgt = std::stod(splittedData[9]) + std::stod(splittedData[11]); // ellipsoidal height = height above geoid + geoid height

                    break;
                }
            }
            else if (splittedData[0].substr(3, 3) == "ZDA")
            {
                if (setDateFromZDA(line))
                {
                    _oldSoD = -1;
                }
                continue;
            }
            else if (splittedData[0].substr(3, 3) == "RMC")
            {
                if (setDateFromRMC(line))
                {
                    _oldSoD = -1;
                }
                continue;
            }
        }
    }

    Eigen::Vector3d lla_pos{ lat_rad, lon_rad, hgt };
    Eigen::Vector3d n_vel{ std::nan(""), std::nan(""), std::nan("") }; // GGA streams don't contain velocity, thus set this one invalid
    obs->insTime = InsTime(_currentDate.year, _currentDate.month, _currentDate.day,
                           hour, minute, second,
                           UTC); // per definition, time tags in GGA NMEA streams are in UTC

    obs->setPosition_lla(lla_pos);

    obs->setVelocity_n(n_vel);

    invokeCallbacks(OUTPUT_PORT_INDEX_NMEA_POS_OBS, obs);
    return obs;
}

NAV::FileReader::FileType NAV::NmeaFile::determineFileType()
{
    return FileReader::FileType::ASCII;
}

void NAV::NmeaFile::readHeader()
{
    // Empty because NMEA does not have a header
}