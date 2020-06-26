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
        if (msgId == Emlid::ErbMessageID::ERB_MessageId_VER)
        {
            obs->data = Emlid::ErbVer();

            std::get<Emlid::ErbVer>(obs->data).iTOW = obs->raw.extractUint32();
            std::get<Emlid::ErbVer>(obs->data).verH = obs->raw.extractUint8();
            std::get<Emlid::ErbVer>(obs->data).verM = obs->raw.extractUint8();
            std::get<Emlid::ErbVer>(obs->data).verL = obs->raw.extractUint8();

            // Calculate the insTime with the iTOW
            if (currentInsTime.has_value())
            {
                currentInsTime.emplace(currentInsTime.value().GetGPSTime().gpsWeek,
                                       static_cast<long double>(std::get<Emlid::ErbVer>(obs->data).iTOW) / 1000.0L,
                                       currentInsTime.value().GetGPSTime().gpsCycle);
                obs->insTime = currentInsTime;
            }

            if (!peek)
            {
                LOG_DATA("Erb: VER, iTOW {}, verH {}, verH {}, verH {}",
                         std::get<Emlid::ErbVer>(obs->data).iTOW,
                         std::get<Emlid::ErbVer>(obs->data).verH,
                         std::get<Emlid::ErbVer>(obs->data).verM,
                         std::get<Emlid::ErbVer>(obs->data).verL);
            }
        }
        else if (msgId == Emlid::ErbMessageID::ERB_MessageId_POS)
        {
            obs->data = Emlid::ErbPos();

            std::get<Emlid::ErbPos>(obs->data).iTOW = obs->raw.extractUint32();
            std::get<Emlid::ErbPos>(obs->data).lon = obs->raw.extractDouble();
            std::get<Emlid::ErbPos>(obs->data).lat = obs->raw.extractDouble();
            std::get<Emlid::ErbPos>(obs->data).height = obs->raw.extractDouble();
            std::get<Emlid::ErbPos>(obs->data).hMSL = obs->raw.extractDouble();
            std::get<Emlid::ErbPos>(obs->data).hAcc = obs->raw.extractUint32();
            std::get<Emlid::ErbPos>(obs->data).vAcc = obs->raw.extractUint32();

            // Calculate the insTime with the iTOW
            if (currentInsTime.has_value())
            {
                currentInsTime.emplace(currentInsTime.value().GetGPSTime().gpsWeek,
                                       static_cast<long double>(std::get<Emlid::ErbPos>(obs->data).iTOW) / 1000.0L,
                                       currentInsTime.value().GetGPSTime().gpsCycle);
                obs->insTime = currentInsTime;
            }

            if (!peek)
            {
                LOG_DATA("Erb: POS, iTOW {}, lon {}, lat {}, height {}",
                         std::get<Emlid::ErbPos>(obs->data).iTOW, std::get<Emlid::ErbPos>(obs->data).lon,
                         std::get<Emlid::ErbPos>(obs->data).lat, std::get<Emlid::ErbPos>(obs->data).height);
            }
        }
        else if (msgId == Emlid::ErbMessageID::ERB_MessageId_STAT)
        {
            obs->data = Emlid::ErbStat();

            std::get<Emlid::ErbStat>(obs->data).iTOW = obs->raw.extractUint32();
            std::get<Emlid::ErbStat>(obs->data).weekGPS = obs->raw.extractUint16();
            std::get<Emlid::ErbStat>(obs->data).fixType = obs->raw.extractUint8();
            std::get<Emlid::ErbStat>(obs->data).fixStatus = obs->raw.extractUint8();
            std::get<Emlid::ErbStat>(obs->data).numSV = obs->raw.extractUint8();

            // Calculate the insTime with the iTOW
            if (currentInsTime.has_value())
            {
                currentInsTime.emplace(currentInsTime.value().GetGPSTime().gpsWeek,
                                       static_cast<long double>(std::get<Emlid::ErbStat>(obs->data).iTOW) / 1000.0L,
                                       currentInsTime.value().GetGPSTime().gpsCycle);
                obs->insTime = currentInsTime;
            }

            if (!peek)
            {
                LOG_DATA("Erb: STAT, iTOW {}, weekGPS {}, fixType {}, fixStatus {}, numSV {}",
                         std::get<Emlid::ErbStat>(obs->data).iTOW,
                         std::get<Emlid::ErbStat>(obs->data).weekGPS, std::get<Emlid::ErbStat>(obs->data).fixType,
                         std::get<Emlid::ErbStat>(obs->data).fixStatus, std::get<Emlid::ErbStat>(obs->data).numSV);
            }
        }
        else if (msgId == Emlid::ErbMessageID::ERB_MessageId_DPOS)
        {
            obs->data = Emlid::ErbDops();

            std::get<Emlid::ErbDops>(obs->data).iTOW = obs->raw.extractUint32();
            std::get<Emlid::ErbDops>(obs->data).dopGeo = obs->raw.extractUint16();
            std::get<Emlid::ErbDops>(obs->data).dopPos = obs->raw.extractUint16();
            std::get<Emlid::ErbDops>(obs->data).dopVer = obs->raw.extractUint16();
            std::get<Emlid::ErbDops>(obs->data).dopHor = obs->raw.extractUint16();

            // Calculate the insTime with the iTOW
            if (currentInsTime.has_value())
            {
                currentInsTime.emplace(currentInsTime.value().GetGPSTime().gpsWeek,
                                       static_cast<long double>(std::get<Emlid::ErbDops>(obs->data).iTOW) / 1000.0L,
                                       currentInsTime.value().GetGPSTime().gpsCycle);
                obs->insTime = currentInsTime;
            }

            if (!peek)
            {
                LOG_DATA("Erb: DOPS, iTOW {}, dopGeo {}, dopPos {}, dopVer {}, dopHor {}",
                         std::get<Emlid::ErbDops>(obs->data).iTOW,
                         std::get<Emlid::ErbDops>(obs->data).dopGeo, std::get<Emlid::ErbDops>(obs->data).dopPos,
                         std::get<Emlid::ErbDops>(obs->data).dopVer, std::get<Emlid::ErbDops>(obs->data).dopHor);
            }
        }
        else if (msgId == Emlid::ErbMessageID::ERB_MessageId_VEL)
        {
            obs->data = Emlid::ErbVel();

            std::get<Emlid::ErbVel>(obs->data).iTOW = obs->raw.extractUint32();
            std::get<Emlid::ErbVel>(obs->data).velN = obs->raw.extractInt32();
            std::get<Emlid::ErbVel>(obs->data).velE = obs->raw.extractInt32();
            std::get<Emlid::ErbVel>(obs->data).velD = obs->raw.extractInt32();
            std::get<Emlid::ErbVel>(obs->data).speed = obs->raw.extractUint32();
            std::get<Emlid::ErbVel>(obs->data).gSpeed = obs->raw.extractUint32();
            std::get<Emlid::ErbVel>(obs->data).heading = obs->raw.extractInt32();
            std::get<Emlid::ErbVel>(obs->data).sAcc = obs->raw.extractUint32();

            // Calculate the insTime with the iTOW
            if (currentInsTime.has_value())
            {
                currentInsTime.emplace(currentInsTime.value().GetGPSTime().gpsWeek,
                                       static_cast<long double>(std::get<Emlid::ErbVel>(obs->data).iTOW) / 1000.0L,
                                       currentInsTime.value().GetGPSTime().gpsCycle);
                obs->insTime = currentInsTime;
            }

            if (!peek)
            {
                LOG_DATA("Erb: VEL, iTOW {}, gSpeed {} [cm/s], heading {} [deg*1e-5], velD {} [cm/s]",
                         std::get<Emlid::ErbVel>(obs->data).iTOW, std::get<Emlid::ErbVel>(obs->data).gSpeed,
                         std::get<Emlid::ErbVel>(obs->data).heading, std::get<Emlid::ErbVel>(obs->data).velD);
            }
        }
        else if (msgId == Emlid::ErbMessageID::ERB_MessageId_SVI)
        {
            obs->data = Emlid::ErbSvi();

            std::get<Emlid::ErbSvi>(obs->data).iTOW = obs->raw.extractUint32();
            std::get<Emlid::ErbSvi>(obs->data).nSV = obs->raw.extractUint8();
            std::get<Emlid::ErbSvi>(obs->data).idSV = obs->raw.extractUint8();
            std::get<Emlid::ErbSvi>(obs->data).typeSV = obs->raw.extractUint8();
            std::get<Emlid::ErbSvi>(obs->data).carPh = obs->raw.extractInt32();
            std::get<Emlid::ErbSvi>(obs->data).psRan = obs->raw.extractInt32();
            std::get<Emlid::ErbSvi>(obs->data).freqD = obs->raw.extractInt32();
            std::get<Emlid::ErbSvi>(obs->data).snr = obs->raw.extractUint16();
            std::get<Emlid::ErbSvi>(obs->data).azim = obs->raw.extractUint16();
            std::get<Emlid::ErbSvi>(obs->data).elev = obs->raw.extractUint16();

            // Calculate the insTime with the iTOW
            if (currentInsTime.has_value())
            {
                currentInsTime.emplace(currentInsTime.value().GetGPSTime().gpsWeek,
                                       static_cast<long double>(std::get<Emlid::ErbSvi>(obs->data).iTOW) / 1000.0L,
                                       currentInsTime.value().GetGPSTime().gpsCycle);
                obs->insTime = currentInsTime;
            }

            if (!peek)
            {
                LOG_DATA("Erb: SVI, iTOW {}, nSV {} , idSV {} , typeSV {} , carPh {}, psRan {} , freqD {} , snr {}, , azim {} , elev {}",
                         std::get<Emlid::ErbSvi>(obs->data).iTOW, std::get<Emlid::ErbSvi>(obs->data).nSV, std::get<Emlid::ErbSvi>(obs->data).idSV,
                         std::get<Emlid::ErbSvi>(obs->data).typeSV, std::get<Emlid::ErbSvi>(obs->data).carPh,
                         std::get<Emlid::ErbSvi>(obs->data).psRan, std::get<Emlid::ErbSvi>(obs->data).freqD,
                         std::get<Emlid::ErbSvi>(obs->data).snr, std::get<Emlid::ErbSvi>(obs->data).azim,
                         std::get<Emlid::ErbSvi>(obs->data).elev);
            }
        }
        else if (msgId == Emlid::ErbMessageID::ERB_MessageId_RTK)
        {
            obs->data = Emlid::ErbRtk();

            std::get<Emlid::ErbRtk>(obs->data).numSV = obs->raw.extractUint8();
            std::get<Emlid::ErbRtk>(obs->data).age = obs->raw.extractUint16();
            std::get<Emlid::ErbRtk>(obs->data).baselineN = obs->raw.extractInt32();
            std::get<Emlid::ErbRtk>(obs->data).baselineE = obs->raw.extractInt32();
            std::get<Emlid::ErbRtk>(obs->data).baselineD = obs->raw.extractInt32();
            std::get<Emlid::ErbRtk>(obs->data).arRatio = obs->raw.extractUint16();
            std::get<Emlid::ErbRtk>(obs->data).weekGPS = obs->raw.extractUint16();
            std::get<Emlid::ErbRtk>(obs->data).timeGPS = obs->raw.extractUint32();

            if (!peek)
            {
                LOG_DATA("Erb: RTK, numSV {}, age {} [s * 1e-2] , baselineN {} , baselineE {} , baselineD {}, arRatio {} [*1e-1] , weekGPS {} , timeGPS {}",
                         std::get<Emlid::ErbRtk>(obs->data).numSV, std::get<Emlid::ErbRtk>(obs->data).age, std::get<Emlid::ErbRtk>(obs->data).baselineN,
                         std::get<Emlid::ErbRtk>(obs->data).baselineE, std::get<Emlid::ErbRtk>(obs->data).baselineD,
                         std::get<Emlid::ErbRtk>(obs->data).arRatio, std::get<Emlid::ErbRtk>(obs->data).weekGPS,
                         std::get<Emlid::ErbRtk>(obs->data).timeGPS);
            }
        }
        else
        {
            if (!peek)
            {
                LOG_WARN("Erb: {:x}, Size {}, not implemented yet!", msgId, obs->raw.getRawDataLength());
            }
        }
    }
}