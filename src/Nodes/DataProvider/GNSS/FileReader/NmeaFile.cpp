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

    return true;
}

std::shared_ptr<const NAV::NodeData> NAV::NmeaFile::pollData(bool peek)
{

 
    auto obs = std::make_shared<PosVel>();
    // Get current position
    auto pos = _filestream.tellg();

    // Read line
    std::string line;
	


    while(true)
	{
		
        std::getline(_filestream, line);
	
		if (_filestream.eof())
		{
			return nullptr;
		}	
	
	
        // Remove any starting non text characters
        line.erase(line.begin(), std::find_if(line.begin(), line.end(), [](int ch) { return std::isgraph(ch); }));
        
        auto splittedData = str::split(line, ",");
        
        if (splittedData[0].substr(0,1)=="$" &&  splittedData[0].substr(3,3)=="RMC")
        {
            break;
        }
	}		
	std::cout << line << std::endl;
    
	
	std::string cell;

    TimeSystem timeSystem = UTC;
    std::optional<uint16_t> year;
    std::optional<uint16_t> month;
    std::optional<uint16_t> day;
    std::optional<uint16_t> hour;
    std::optional<uint16_t> minute;
    std::optional<long double> second = 0L;
    std::optional<uint16_t> gpsWeek;
    std::optional<long double> gpsToW;
    //Eigen::Vector3d lla_pos{ std::nan(""), std::nan(""), std::nan("") };
	
	Eigen::Vector3d lla_pos{ 0.5,0.05,200 };
    Eigen::Vector3d n_vel{ std::nan(""), std::nan(""), std::nan("") };
	
    static int i = 0;
    obs->insTime = InsTime(2000, 1, 1,
                               10, 20, 30+i++,
                               timeSystem);
							   
	obs->setPosition_lla(lla_pos);
        
	obs->setVelocity_n(n_vel);
  
    if (peek)
    {
        // Return to position before "Read line".
        _filestream.seekg(pos, std::ios_base::beg);
    }


    // Calls all the callbacks
    if (!peek)
    {
        invokeCallbacks(OUTPUT_PORT_INDEX_RTKLIB_POS_OBS, obs);
    }
	
    return obs;
}

NAV::FileReader::FileType NAV::NmeaFile::determineFileType()
{
    return FileReader::FileType::NMEA;
}

void NAV::NmeaFile::readHeader()
{
 
}