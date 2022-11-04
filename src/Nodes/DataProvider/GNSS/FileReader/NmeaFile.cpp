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

void NAV::NmeaFile::setdatefromzda(const std::string & line)
{
	
	std::vector<std::string> splittedString = str::split(line, ",");
	if (splittedString.size()==7)
	{
        std::size_t pos_star = splittedString[6].find("*");
        if (pos_star >=0)
        {
            long crc = std::strtol(splittedString[6].substr(pos_star+1).c_str(), NULL, 16);
			long mycrc=0;
			for (unsigned int i = 1; i< line.length() - 4; i ++) 
			{
				mycrc ^= line.at(i);
			}
			if (mycrc == crc)
			{
				ddmmyyyy[0] = std::stoi(splittedString[2]);
				ddmmyyyy[1] = std::stoi(splittedString[3]);
				ddmmyyyy[2] = std::stoi(splittedString[4]);
				
				haveValidDate = true;
			}
        }
    }			
}


std::shared_ptr<const NAV::NodeData> NAV::NmeaFile::pollData(bool peek)
{

 
    auto obs = std::make_shared<PosVel>();
    // Get current position
    auto pos = _filestream.tellg();

    // Read line
    std::string line;
	
    std::vector<std::string> splittedData;
	
    TimeSystem timeSystem = UTC;

    int  hour;
    int  minute;
    double second;
	
	double lat_rad = 0.0;
    double lon_rad = 0.0;
	double hgt = 0.0; 
	
    while(true)
	{
		
        std::getline(_filestream, line);
	
		if (_filestream.eof())
		{
			return nullptr;
		}	
	
	
        // Remove any starting non text characters
        line.erase(line.begin(), std::find_if(line.begin(), line.end(), [](int ch) { return std::isgraph(ch); }));
        
        splittedData = str::split(line, ",");
        
        if (splittedData[0].substr(0,1)=="$")
        {
			
			if (haveValidDate &&  splittedData[0].substr(3,3)=="GGA")
			{
				if (splittedData.size()!=15) continue;
				std::size_t pos_star = splittedData[14].find("*");
	            long crc = std::strtol(splittedData[14].substr(pos_star+1).c_str(), NULL, 16);
				long mycrc=0;
				for (unsigned int i = 1; i< line.length() - 4; i ++) 
				{
					mycrc ^= line.at(i);
				}
				if (mycrc == crc)
				{
				    hour = std::stoi(splittedData[1].substr(0,2));
				    minute= std::stoi(splittedData[1].substr(2,2));
				    second = std::stod(splittedData[1].substr(4));
					double newSOD =hour*24*60.0+minute*60.0+second;
					
					//only contine if second of day > than previous one
					if (newSOD<oldSOD)
					{
						oldSOD=newSOD;
						haveValidDate = false; //force wait until next ZDA stream
						continue;
					}	
					
					
					int lat1 = std::stoi(splittedData[2].substr(0,2));
					double lat2 = std::stod(splittedData[2].substr(2));
					
					lat_rad = (lat1+lat2/60.0)/180*M_PI ;
					
					if (splittedData[3]=="S")
					{
						lat_rad *= -1.0;
					}
					
					int lon1 = std::stoi(splittedData[4].substr(0,3));
					double lon2 = std::stod(splittedData[4].substr(3));
					
					lon_rad = (lon1+lon2/60.0)/180*M_PI;
					
					if (splittedData[5]=="W")
					{
						lon_rad *= -1.0;
					}
					
					hgt = std::stod(splittedData[9]) + std::stod(splittedData[11]);
					
				    break;	
				}

			}
			else if (splittedData[0].substr(3,3)=="ZDA")
		    {
		    	setdatefromzda(line);
			}
        }
	}
	
	
    
	
   
	
	Eigen::Vector3d lla_pos{ lat_rad,lon_rad,hgt };
    Eigen::Vector3d n_vel{ std::nan(""), std::nan(""), std::nan("") };

    obs->insTime = InsTime(ddmmyyyy[0], ddmmyyyy[1], ddmmyyyy[2],
                               hour, minute, second,
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