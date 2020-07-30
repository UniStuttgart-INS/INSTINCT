#include "EmlidUtilities.hpp"
#include "EmlidTypes.hpp"

#include "util/Logger.hpp"

void NAV::sensors::emlid::decryptEmlidObs(std::shared_ptr<NAV::EmlidObs>& obs, std::optional<NAV::InsTime>& currentInsTime, bool peek)
{
    if (obs->raw.type() == uart::protocol::Packet::Type::TYPE_BINARY)
    {
        obs->msgId = obs->raw.extractUint8();
        obs->payloadLength = obs->raw.extractUint16();

        // Navigation Results Messages: Position
        auto msgId = static_cast<ErbMessageID>(obs->msgId);
        if (msgId == ErbMessageID::ERB_MessageId_VER)
        {
            obs->data = ErbVer();

            std::get<ErbVer>(obs->data).iTOW = obs->raw.extractUint32();
            std::get<ErbVer>(obs->data).verH = obs->raw.extractUint8();
            std::get<ErbVer>(obs->data).verM = obs->raw.extractUint8();
            std::get<ErbVer>(obs->data).verL = obs->raw.extractUint8();

            // Calculate the insTime with the iTOW
            if (currentInsTime.has_value())
            {
                currentInsTime.emplace(currentInsTime.value().GetGPSTime().gpsWeek,
                                       static_cast<long double>(std::get<ErbVer>(obs->data).iTOW) / 1000.0L,
                                       currentInsTime.value().GetGPSTime().gpsCycle);
                obs->insTime = currentInsTime;
            }

            if (!peek)
            {
                LOG_DATA("Erb: VER, iTOW {}, verH {}, verH {}, verH {}",
                         std::get<ErbVer>(obs->data).iTOW,
                         std::get<ErbVer>(obs->data).verH,
                         std::get<ErbVer>(obs->data).verM,
                         std::get<ErbVer>(obs->data).verL);
            }
        }
        else if (msgId == ErbMessageID::ERB_MessageId_POS)
        {
            obs->data = ErbPos();

            std::get<ErbPos>(obs->data).iTOW = obs->raw.extractUint32();
            std::get<ErbPos>(obs->data).lon = obs->raw.extractDouble();
            std::get<ErbPos>(obs->data).lat = obs->raw.extractDouble();
            std::get<ErbPos>(obs->data).height = obs->raw.extractDouble();
            std::get<ErbPos>(obs->data).hMSL = obs->raw.extractDouble();
            std::get<ErbPos>(obs->data).hAcc = obs->raw.extractUint32();
            std::get<ErbPos>(obs->data).vAcc = obs->raw.extractUint32();

            // Calculate the insTime with the iTOW
            if (currentInsTime.has_value())
            {
                currentInsTime.emplace(currentInsTime.value().GetGPSTime().gpsWeek,
                                       static_cast<long double>(std::get<ErbPos>(obs->data).iTOW) / 1000.0L,
                                       currentInsTime.value().GetGPSTime().gpsCycle);
                obs->insTime = currentInsTime;
            }

            if (!peek)
            {
                LOG_DATA("Erb: POS, iTOW {}, lon {}, lat {}, height {}",
                         std::get<ErbPos>(obs->data).iTOW, std::get<ErbPos>(obs->data).lon,
                         std::get<ErbPos>(obs->data).lat, std::get<ErbPos>(obs->data).height);
            }
        }
        else if (msgId == ErbMessageID::ERB_MessageId_STAT)
        {
            obs->data = ErbStat();

            std::get<ErbStat>(obs->data).iTOW = obs->raw.extractUint32();
            std::get<ErbStat>(obs->data).weekGPS = obs->raw.extractUint16();
            std::get<ErbStat>(obs->data).fixType = obs->raw.extractUint8();
            std::get<ErbStat>(obs->data).fixStatus = obs->raw.extractUint8();
            std::get<ErbStat>(obs->data).numSV = obs->raw.extractUint8();

            // Calculate the insTime with the iTOW
            if (currentInsTime.has_value())
            {
                currentInsTime.emplace(currentInsTime.value().GetGPSTime().gpsWeek,
                                       static_cast<long double>(std::get<ErbStat>(obs->data).iTOW) / 1000.0L,
                                       currentInsTime.value().GetGPSTime().gpsCycle);
                obs->insTime = currentInsTime;
            }

            if (!peek)
            {
                LOG_DATA("Erb: STAT, iTOW {}, weekGPS {}, fixType {}, fixStatus {}, numSV {}",
                         std::get<ErbStat>(obs->data).iTOW,
                         std::get<ErbStat>(obs->data).weekGPS, std::get<ErbStat>(obs->data).fixType,
                         std::get<ErbStat>(obs->data).fixStatus, std::get<ErbStat>(obs->data).numSV);
            }
        }
        else if (msgId == ErbMessageID::ERB_MessageId_DPOS)
        {
            obs->data = ErbDops();

            std::get<ErbDops>(obs->data).iTOW = obs->raw.extractUint32();
            std::get<ErbDops>(obs->data).dopGeo = obs->raw.extractUint16();
            std::get<ErbDops>(obs->data).dopPos = obs->raw.extractUint16();
            std::get<ErbDops>(obs->data).dopVer = obs->raw.extractUint16();
            std::get<ErbDops>(obs->data).dopHor = obs->raw.extractUint16();

            // Calculate the insTime with the iTOW
            if (currentInsTime.has_value())
            {
                currentInsTime.emplace(currentInsTime.value().GetGPSTime().gpsWeek,
                                       static_cast<long double>(std::get<ErbDops>(obs->data).iTOW) / 1000.0L,
                                       currentInsTime.value().GetGPSTime().gpsCycle);
                obs->insTime = currentInsTime;
            }

            if (!peek)
            {
                LOG_DATA("Erb: DOPS, iTOW {}, dopGeo {}, dopPos {}, dopVer {}, dopHor {}",
                         std::get<ErbDops>(obs->data).iTOW,
                         std::get<ErbDops>(obs->data).dopGeo, std::get<ErbDops>(obs->data).dopPos,
                         std::get<ErbDops>(obs->data).dopVer, std::get<ErbDops>(obs->data).dopHor);
            }
        }
        else if (msgId == ErbMessageID::ERB_MessageId_VEL)
        {
            obs->data = ErbVel();

            std::get<ErbVel>(obs->data).iTOW = obs->raw.extractUint32();
            std::get<ErbVel>(obs->data).velN = obs->raw.extractInt32();
            std::get<ErbVel>(obs->data).velE = obs->raw.extractInt32();
            std::get<ErbVel>(obs->data).velD = obs->raw.extractInt32();
            std::get<ErbVel>(obs->data).speed = obs->raw.extractUint32();
            std::get<ErbVel>(obs->data).gSpeed = obs->raw.extractUint32();
            std::get<ErbVel>(obs->data).heading = obs->raw.extractInt32();
            std::get<ErbVel>(obs->data).sAcc = obs->raw.extractUint32();

            // Calculate the insTime with the iTOW
            if (currentInsTime.has_value())
            {
                currentInsTime.emplace(currentInsTime.value().GetGPSTime().gpsWeek,
                                       static_cast<long double>(std::get<ErbVel>(obs->data).iTOW) / 1000.0L,
                                       currentInsTime.value().GetGPSTime().gpsCycle);
                obs->insTime = currentInsTime;
            }

            if (!peek)
            {
                LOG_DATA("Erb: VEL, iTOW {}, gSpeed {} [cm/s], heading {} [deg*1e-5], velD {} [cm/s]",
                         std::get<ErbVel>(obs->data).iTOW, std::get<ErbVel>(obs->data).gSpeed,
                         std::get<ErbVel>(obs->data).heading, std::get<ErbVel>(obs->data).velD);
            }
        }
        else if (msgId == ErbMessageID::ERB_MessageId_SVI)
        {
            obs->data = ErbSvi();

            std::get<ErbSvi>(obs->data).iTOW = obs->raw.extractUint32();
            std::get<ErbSvi>(obs->data).nSV = obs->raw.extractUint8();
            std::get<ErbSvi>(obs->data).idSV = obs->raw.extractUint8();
            std::get<ErbSvi>(obs->data).typeSV = obs->raw.extractUint8();
            std::get<ErbSvi>(obs->data).carPh = obs->raw.extractInt32();
            std::get<ErbSvi>(obs->data).psRan = obs->raw.extractInt32();
            std::get<ErbSvi>(obs->data).freqD = obs->raw.extractInt32();
            std::get<ErbSvi>(obs->data).snr = obs->raw.extractUint16();
            std::get<ErbSvi>(obs->data).azim = obs->raw.extractUint16();
            std::get<ErbSvi>(obs->data).elev = obs->raw.extractUint16();

            // Calculate the insTime with the iTOW
            if (currentInsTime.has_value())
            {
                currentInsTime.emplace(currentInsTime.value().GetGPSTime().gpsWeek,
                                       static_cast<long double>(std::get<ErbSvi>(obs->data).iTOW) / 1000.0L,
                                       currentInsTime.value().GetGPSTime().gpsCycle);
                obs->insTime = currentInsTime;
            }

            if (!peek)
            {
                LOG_DATA("Erb: SVI, iTOW {}, nSV {} , idSV {} , typeSV {} , carPh {}, psRan {} , freqD {} , snr {}, , azim {} , elev {}",
                         std::get<ErbSvi>(obs->data).iTOW, std::get<ErbSvi>(obs->data).nSV, std::get<ErbSvi>(obs->data).idSV,
                         std::get<ErbSvi>(obs->data).typeSV, std::get<ErbSvi>(obs->data).carPh,
                         std::get<ErbSvi>(obs->data).psRan, std::get<ErbSvi>(obs->data).freqD,
                         std::get<ErbSvi>(obs->data).snr, std::get<ErbSvi>(obs->data).azim,
                         std::get<ErbSvi>(obs->data).elev);
            }
        }
        else if (msgId == ErbMessageID::ERB_MessageId_RTK)
        {
            obs->data = ErbRtk();

            std::get<ErbRtk>(obs->data).numSV = obs->raw.extractUint8();
            std::get<ErbRtk>(obs->data).age = obs->raw.extractUint16();
            std::get<ErbRtk>(obs->data).baselineN = obs->raw.extractInt32();
            std::get<ErbRtk>(obs->data).baselineE = obs->raw.extractInt32();
            std::get<ErbRtk>(obs->data).baselineD = obs->raw.extractInt32();
            std::get<ErbRtk>(obs->data).arRatio = obs->raw.extractUint16();
            std::get<ErbRtk>(obs->data).weekGPS = obs->raw.extractUint16();
            std::get<ErbRtk>(obs->data).timeGPS = obs->raw.extractUint32();

            if (!peek)
            {
                LOG_DATA("Erb: RTK, numSV {}, age {} [s * 1e-2] , baselineN {} , baselineE {} , baselineD {}, arRatio {} [*1e-1] , weekGPS {} , timeGPS {}",
                         std::get<ErbRtk>(obs->data).numSV, std::get<ErbRtk>(obs->data).age, std::get<ErbRtk>(obs->data).baselineN,
                         std::get<ErbRtk>(obs->data).baselineE, std::get<ErbRtk>(obs->data).baselineD,
                         std::get<ErbRtk>(obs->data).arRatio, std::get<ErbRtk>(obs->data).weekGPS,
                         std::get<ErbRtk>(obs->data).timeGPS);
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

std::pair<uint8_t, uint8_t> NAV::sensors::emlid::checksumUBX(const std::vector<uint8_t>& data)
{
    uint8_t cka = 0;
    uint8_t ckb = 0;

    for (size_t i = 2; i < data.size() - 2; i++)
    {
        cka += data.at(i);
        ckb += cka;
    }
    return std::make_pair(cka, ckb);
}