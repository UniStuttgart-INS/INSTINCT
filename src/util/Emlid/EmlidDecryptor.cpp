#include "EmlidDecryptor.hpp"

#include "util/Logger.hpp"

void NAV::Emlid::decryptEmlidObs(std::shared_ptr<NAV::EmlidObs>& obs, std::optional<NAV::InsTime>& currentInsTime, bool peek)
{
    if (obs->raw.type() == Emlid::EmlidPacket::Type::TYPE_BINARY)
    {
        obs->msgId = obs->raw.extractUint8();
        obs->payloadLength = obs->raw.extractUint16();

        // Navigation Results Messages: Position
        auto msgId = static_cast<Emlid::ErbMessageID>(obs->msgId);
        if (msgId == Emlid::ErbMessageID::ERB_Message_VER)
        {
            obs->data = Emlid::ErbVER();

            std::get<Emlid::ErbVER>(obs->data).iTOW = obs->raw.extractUint32();
            std::get<Emlid::ErbVER>(obs->data).verH = obs->raw.extractUint8();
            std::get<Emlid::ErbVER>(obs->data).verM = obs->raw.extractUint8();
            std::get<Emlid::ErbVER>(obs->data).verL = obs->raw.extractUint8();

            // Calculate the insTime with the iTOW
            if (currentInsTime.has_value())
            {
                 currentInsTime.emplace(currentInsTime.value().GetGPSTime().gpsWeek,
                                        static_cast<long double>(std::get<Emlid::ErbVER>(obs->data).iTOW) / 1000.0L,
                                        currentInsTime.value().GetGPSTime().gpsCycle);
                obs->insTime = currentInsTime;
            }

            if (!peek)
            {
                LOG_DATA("Erb:  VER, iTOW {}, verH {}, verH {}, verH {}", std::get<Emlid::ErbVER>(obs->data).iTOW,
                         std::get<Emlid::ErbVER>(obs->data).verH, std::get<Emlid::ErbVER>(obs->data).verM,
                         std::get<Emlid::ErbVER>(obs->data).verL);
            }
        }
        else if (msgId == Emlid::ErbMessageID::ERB_Message_POS)
        {
            obs->data = Emlid::ErbNavPosllh();

            std::get<Emlid::ErbNavPosllh>(obs->data).iTOW = obs->raw.extractUint32();
            std::get<Emlid::ErbNavPosllh>(obs->data).lon = obs->raw.extractDouble();
            std::get<Emlid::ErbNavPosllh>(obs->data).lat = obs->raw.extractDouble();    
            std::get<Emlid::ErbNavPosllh>(obs->data).height = obs->raw.extractDouble();
            std::get<Emlid::ErbNavPosllh>(obs->data).hMSL = obs->raw.extractDouble();
            std::get<Emlid::ErbNavPosllh>(obs->data).hAcc = obs->raw.extractUint32();
            std::get<Emlid::ErbNavPosllh>(obs->data).vAcc = obs->raw.extractUint32();

            // Calculate the insTime with the iTOW
            if (currentInsTime.has_value())
            {
                 currentInsTime.emplace(currentInsTime.value().GetGPSTime().gpsWeek,
                                        static_cast<long double>(std::get<Emlid::ErbNavPosllh>(obs->data).iTOW) / 1000.0L,
                                        currentInsTime.value().GetGPSTime().gpsCycle);
                obs->insTime = currentInsTime;
            }

            if (!peek)
            {
                LOG_DATA("Erb:  NAV-POSLLH, iTOW {}, lon {}, lat {}, height {}",
                         std::get<Emlid::ErbNavPosllh>(obs->data).iTOW, std::get<Emlid::ErbNavPosllh>(obs->data).lon,
                         std::get<Emlid::ErbNavPosllh>(obs->data).lat, std::get<Emlid::ErbNavPosllh>(obs->data).height);
            }
        }
        else if (msgId == Emlid::ErbMessageID::ERB_Message_STAT)
        {
            obs->data = Emlid::ErbSTAT();

            std::get<Emlid::ErbSTAT>(obs->data).iTOW = obs->raw.extractUint32();
            std::get<Emlid::ErbSTAT>(obs->data).weekGPS = obs->raw.extractUint16();
            std::get<Emlid::ErbSTAT>(obs->data).fixType = obs->raw.extractUint8();
            std::get<Emlid::ErbSTAT>(obs->data).fixStatus = obs->raw.extractUint8();
            std::get<Emlid::ErbSTAT>(obs->data).numSV = obs->raw.extractUint8();

            // Calculate the insTime with the iTOW
            if (currentInsTime.has_value())
            {
                 currentInsTime.emplace(currentInsTime.value().GetGPSTime().gpsWeek,
                                        static_cast<long double>(std::get<Emlid::ErbSTAT>(obs->data).iTOW) / 1000.0L,
                                        currentInsTime.value().GetGPSTime().gpsCycle);
                obs->insTime = currentInsTime;
            }

            if (!peek)
            {
                LOG_DATA("Erb:  STAT, iTOW {}, weekGPS {}, fixType {}, fixStatus {}, numSV {}",
                         std::get<Emlid::ErbSTAT>(obs->data).iTOW,
                         std::get<Emlid::ErbSTAT>(obs->data).weekGPS, std::get<Emlid::ErbSTAT>(obs->data).fixType,
                         std::get<Emlid::ErbSTAT>(obs->data).fixStatus, std::get<Emlid::ErbSTAT>(obs->data).numSV);
            }
        }
        else if (msgId == Emlid::ErbMessageID::ERB_Message_DPOS)
        {
            obs->data = Emlid::ErbDOPS();

            std::get<Emlid::ErbDOPS>(obs->data).iTOW = obs->raw.extractUint32();
            std::get<Emlid::ErbDOPS>(obs->data).dopGeo = obs->raw.extractUint16();
            std::get<Emlid::ErbDOPS>(obs->data).dopPos = obs->raw.extractUint16();
            std::get<Emlid::ErbDOPS>(obs->data).dopVer = obs->raw.extractUint16();
            std::get<Emlid::ErbDOPS>(obs->data).dopHor = obs->raw.extractUint16();

            // Calculate the insTime with the iTOW
            if (currentInsTime.has_value())
            {
                 currentInsTime.emplace(currentInsTime.value().GetGPSTime().gpsWeek,
                                        static_cast<long double>(std::get<Emlid::ErbDOPS>(obs->data).iTOW) / 1000.0L,
                                        currentInsTime.value().GetGPSTime().gpsCycle);
                obs->insTime = currentInsTime;
            }

            if (!peek)
            {
                LOG_DATA("Erb:  STAT, iTOW {}, dopGeo {}, dopPos {}, dopVer {}, dopHor {}",
                         std::get<Emlid::ErbDOPS>(obs->data).iTOW,
                         std::get<Emlid::ErbDOPS>(obs->data).dopGeo, std::get<Emlid::ErbDOPS>(obs->data).dopPos,
                         std::get<Emlid::ErbDOPS>(obs->data).dopVer, std::get<Emlid::ErbDOPS>(obs->data).dopHor);
            }
        }
        else if (msgId == Emlid::ErbMessageID::ERB_Message_VEL)
        {
            obs->data = Emlid::ErbNavVelned();

            std::get<Emlid::ErbNavVelned>(obs->data).iTOW = obs->raw.extractUint32();
            std::get<Emlid::ErbNavVelned>(obs->data).velN = obs->raw.extractInt32();
            std::get<Emlid::ErbNavVelned>(obs->data).velE = obs->raw.extractInt32();
            std::get<Emlid::ErbNavVelned>(obs->data).velD = obs->raw.extractInt32();
            std::get<Emlid::ErbNavVelned>(obs->data).speed = obs->raw.extractUint32();
            std::get<Emlid::ErbNavVelned>(obs->data).gSpeed = obs->raw.extractUint32();
            std::get<Emlid::ErbNavVelned>(obs->data).heading = obs->raw.extractInt32();
            std::get<Emlid::ErbNavVelned>(obs->data).sAcc = obs->raw.extractUint32();

            // Calculate the insTime with the iTOW
            if (currentInsTime.has_value())
            {
                currentInsTime.emplace(currentInsTime.value().GetGPSTime().gpsWeek,
                                       static_cast<long double>(std::get<Emlid::ErbNavVelned>(obs->data).iTOW) / 1000.0L,
                                       currentInsTime.value().GetGPSTime().gpsCycle);
                obs->insTime = currentInsTime;
            }

            if (!peek)
            {
                LOG_DATA("Erb:  NAV-VELNED, iTOW {}, gSpeed {} [cm/s], heading {} [deg*1e-5], velD {} [cm/s]",
                         std::get<Emlid::ErbNavVelned>(obs->data).iTOW, std::get<Emlid::ErbNavVelned>(obs->data).gSpeed,
                         std::get<Emlid::ErbNavVelned>(obs->data).heading, std::get<Emlid::ErbNavVelned>(obs->data).velD);
            }
        }
        else if (msgId == Emlid::ErbMessageID::ERB_Message_SVI)
        {
            obs->data = Emlid::ErbSVI();

            std::get<Emlid::ErbSVI>(obs->data).iTOW = obs->raw.extractUint32();
            std::get<Emlid::ErbSVI>(obs->data).nSV = obs->raw.extractUint8();
            std::get<Emlid::ErbSVI>(obs->data).idSV = obs->raw.extractUint8();
            std::get<Emlid::ErbSVI>(obs->data).typeSV = obs->raw.extractUint8();
            std::get<Emlid::ErbSVI>(obs->data).carPh = obs->raw.extractInt32();
            std::get<Emlid::ErbSVI>(obs->data).psRan = obs->raw.extractInt32();
            std::get<Emlid::ErbSVI>(obs->data).freqD = obs->raw.extractInt32();
            std::get<Emlid::ErbSVI>(obs->data).snr = obs->raw.extractUint16();
            std::get<Emlid::ErbSVI>(obs->data).azim = obs->raw.extractUint16();
            std::get<Emlid::ErbSVI>(obs->data).elev = obs->raw.extractUint16();

            // Calculate the insTime with the iTOW
            if (currentInsTime.has_value())
            {
                currentInsTime.emplace(currentInsTime.value().GetGPSTime().gpsWeek,
                                       static_cast<long double>(std::get<Emlid::ErbSVI>(obs->data).iTOW) / 1000.0L,
                                       currentInsTime.value().GetGPSTime().gpsCycle);
                obs->insTime = currentInsTime;
            }

            if (!peek)
            {
                LOG_DATA("Erb:  SVI iTOW {}, nSV {} , idSV {} , typeSV {} , carPh {}, psRan {} , freqD {} , snr {}, , azim {} , elev {}", 
                         std::get<Emlid::ErbSVI>(obs->data).iTOW, std::get<Emlid::ErbSVI>(obs->data).nSV, std::get<Emlid::ErbSVI>(obs->data).idSV,
                         std::get<Emlid::ErbSVI>(obs->data).typeSV, std::get<Emlid::ErbSVI>(obs->data).carPh,
                         std::get<Emlid::ErbSVI>(obs->data).psRan, std::get<Emlid::ErbSVI>(obs->data).freqD,
                         std::get<Emlid::ErbSVI>(obs->data).snr, std::get<Emlid::ErbSVI>(obs->data).azim,
                         std::get<Emlid::ErbSVI>(obs->data).elev);
            }
        }
        else if (msgId == Emlid::ErbMessageID::ERB_Message_RTK)
        {
            obs->data = Emlid::ErbRTK();

            std::get<Emlid::ErbRTK>(obs->data).numSV = obs->raw.extractUint8();
            std::get<Emlid::ErbRTK>(obs->data).age = obs->raw.extractUint16();
            std::get<Emlid::ErbRTK>(obs->data).baselineN = obs->raw.extractInt32();
            std::get<Emlid::ErbRTK>(obs->data).baselineE = obs->raw.extractInt32();
            std::get<Emlid::ErbRTK>(obs->data).baselineD = obs->raw.extractInt32();
            std::get<Emlid::ErbRTK>(obs->data).arRatio = obs->raw.extractUint16();
            std::get<Emlid::ErbRTK>(obs->data).weekGPS = obs->raw.extractUint16();
            std::get<Emlid::ErbRTK>(obs->data).timeGPS = obs->raw.extractUint32();

            if (!peek)
            {
                LOG_DATA("Erb:  RTK numSV {}, age {} [s * 1e-2] , baselineN {} , baselineE {} , baselineD {}, arRatio {} [*1e-1] , weekGPS {} , timeGPS {}", 
                         std::get<Emlid::ErbRTK>(obs->data).numSV, std::get<Emlid::ErbRTK>(obs->data).age, std::get<Emlid::ErbRTK>(obs->data).baselineN,
                         std::get<Emlid::ErbRTK>(obs->data).baselineE, std::get<Emlid::ErbRTK>(obs->data).baselineD,
                         std::get<Emlid::ErbRTK>(obs->data).arRatio, std::get<Emlid::ErbRTK>(obs->data).weekGPS,
                         std::get<Emlid::ErbRTK>(obs->data).timeGPS);
            }
        }
        else
        {
            if (!peek)
            {
                LOG_WARN("Erb:  NAV-{:x}, Size {}, not implemented yet!", msgId, obs->raw.getRawDataLength());
            }
        }
        
    }
    else if (obs->raw.type() == Emlid::EmlidPacket::Type::TYPE_ASCII)
    {
        if (!peek)
        {
            LOG_DATA("NMEA: {}", obs->raw.datastr().substr(0, obs->raw.datastr().size() - 2));
        }
    }
}