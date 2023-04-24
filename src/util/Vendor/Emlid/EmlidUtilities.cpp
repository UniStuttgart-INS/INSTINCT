// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "EmlidUtilities.hpp"
#include "EmlidTypes.hpp"

#include "util/Logger.hpp"
#include "util/Time/TimeBase.hpp"

void NAV::vendor::emlid::decryptEmlidObs(const std::shared_ptr<NAV::EmlidObs>& obs, uart::protocol::Packet& packet)
{
    if (packet.type() == uart::protocol::Packet::Type::TYPE_BINARY)
    {
        obs->msgId = packet.extractUint8();
        obs->payloadLength = packet.extractUint16();

        // Navigation Results Messages: Position
        auto msgId = static_cast<ErbMessageID>(obs->msgId);
        if (msgId == ErbMessageID::ERB_MessageId_VER)
        {
            obs->data = ErbVer();

            std::get<ErbVer>(obs->data).iTOW = packet.extractUint32();
            std::get<ErbVer>(obs->data).verH = packet.extractUint8();
            std::get<ErbVer>(obs->data).verM = packet.extractUint8();
            std::get<ErbVer>(obs->data).verL = packet.extractUint8();

            // Calculate the insTime with the iTOW
            auto currentTime = util::time::GetCurrentInsTime();
            if (!currentTime.empty())
            {
                auto gpst = currentTime.toGPSweekTow();
                currentTime = InsTime(gpst.gpsCycle,
                                      gpst.gpsWeek,
                                      static_cast<long double>(std::get<ErbVer>(obs->data).iTOW) / 1000.0L);
                obs->insTime = currentTime;
            }

            LOG_DATA("Erb: VER, iTOW {}, verH {}, verH {}, verH {}",
                     std::get<ErbVer>(obs->data).iTOW,
                     std::get<ErbVer>(obs->data).verH,
                     std::get<ErbVer>(obs->data).verM,
                     std::get<ErbVer>(obs->data).verL);
        }
        else if (msgId == ErbMessageID::ERB_MessageId_POS)
        {
            obs->data = ErbPos();

            std::get<ErbPos>(obs->data).iTOW = packet.extractUint32();
            std::get<ErbPos>(obs->data).lon = packet.extractDouble();
            std::get<ErbPos>(obs->data).lat = packet.extractDouble();
            std::get<ErbPos>(obs->data).height = packet.extractDouble();
            std::get<ErbPos>(obs->data).hMSL = packet.extractDouble();
            std::get<ErbPos>(obs->data).hAcc = packet.extractUint32();
            std::get<ErbPos>(obs->data).vAcc = packet.extractUint32();

            // Calculate the insTime with the iTOW
            auto currentTime = util::time::GetCurrentInsTime();
            if (!currentTime.empty())
            {
                auto gpst = currentTime.toGPSweekTow();
                currentTime = InsTime(gpst.gpsCycle,
                                      gpst.gpsWeek,
                                      static_cast<long double>(std::get<ErbPos>(obs->data).iTOW) / 1000.0L);
                obs->insTime = currentTime;
            }

            LOG_DATA("Erb: POS, iTOW {}, lon {}, lat {}, height {}",
                     std::get<ErbPos>(obs->data).iTOW, std::get<ErbPos>(obs->data).lon,
                     std::get<ErbPos>(obs->data).lat, std::get<ErbPos>(obs->data).height);
        }
        else if (msgId == ErbMessageID::ERB_MessageId_STAT)
        {
            obs->data = ErbStat();

            std::get<ErbStat>(obs->data).iTOW = packet.extractUint32();
            std::get<ErbStat>(obs->data).weekGPS = packet.extractUint16();
            std::get<ErbStat>(obs->data).fixType = packet.extractUint8();
            std::get<ErbStat>(obs->data).fixStatus = packet.extractUint8();
            std::get<ErbStat>(obs->data).numSV = packet.extractUint8();

            // Calculate the insTime with the iTOW
            auto currentTime = util::time::GetCurrentInsTime();
            if (!currentTime.empty())
            {
                auto gpst = currentTime.toGPSweekTow();
                currentTime = InsTime(gpst.gpsCycle,
                                      gpst.gpsWeek,
                                      static_cast<long double>(std::get<ErbStat>(obs->data).iTOW) / 1000.0L);
                obs->insTime = currentTime;
            }

            LOG_DATA("Erb: STAT, iTOW {}, weekGPS {}, fixType {}, fixStatus {}, numSV {}",
                     std::get<ErbStat>(obs->data).iTOW,
                     std::get<ErbStat>(obs->data).weekGPS, std::get<ErbStat>(obs->data).fixType,
                     std::get<ErbStat>(obs->data).fixStatus, std::get<ErbStat>(obs->data).numSV);
        }
        else if (msgId == ErbMessageID::ERB_MessageId_DPOS)
        {
            obs->data = ErbDops();

            std::get<ErbDops>(obs->data).iTOW = packet.extractUint32();
            std::get<ErbDops>(obs->data).dopGeo = packet.extractUint16();
            std::get<ErbDops>(obs->data).dopPos = packet.extractUint16();
            std::get<ErbDops>(obs->data).dopVer = packet.extractUint16();
            std::get<ErbDops>(obs->data).dopHor = packet.extractUint16();

            // Calculate the insTime with the iTOW
            auto currentTime = util::time::GetCurrentInsTime();
            if (!currentTime.empty())
            {
                auto gpst = currentTime.toGPSweekTow();
                currentTime = InsTime(gpst.gpsCycle,
                                      gpst.gpsWeek,
                                      static_cast<long double>(std::get<ErbDops>(obs->data).iTOW) / 1000.0L);
                obs->insTime = currentTime;
            }

            LOG_DATA("Erb: DOPS, iTOW {}, dopGeo {}, dopPos {}, dopVer {}, dopHor {}",
                     std::get<ErbDops>(obs->data).iTOW,
                     std::get<ErbDops>(obs->data).dopGeo, std::get<ErbDops>(obs->data).dopPos,
                     std::get<ErbDops>(obs->data).dopVer, std::get<ErbDops>(obs->data).dopHor);
        }
        else if (msgId == ErbMessageID::ERB_MessageId_VEL)
        {
            obs->data = ErbVel();

            std::get<ErbVel>(obs->data).iTOW = packet.extractUint32();
            std::get<ErbVel>(obs->data).velN = packet.extractInt32();
            std::get<ErbVel>(obs->data).velE = packet.extractInt32();
            std::get<ErbVel>(obs->data).velD = packet.extractInt32();
            std::get<ErbVel>(obs->data).speed = packet.extractUint32();
            std::get<ErbVel>(obs->data).gSpeed = packet.extractUint32();
            std::get<ErbVel>(obs->data).heading = packet.extractInt32();
            std::get<ErbVel>(obs->data).sAcc = packet.extractUint32();

            // Calculate the insTime with the iTOW
            auto currentTime = util::time::GetCurrentInsTime();
            if (!currentTime.empty())
            {
                auto gpst = currentTime.toGPSweekTow();
                currentTime = InsTime(gpst.gpsCycle,
                                      gpst.gpsWeek,
                                      static_cast<long double>(std::get<ErbVel>(obs->data).iTOW) / 1000.0L);
                obs->insTime = currentTime;
            }

            LOG_DATA("Erb: VEL, iTOW {}, gSpeed {} [cm/s], heading {} [deg*1e-5], velD {} [cm/s]",
                     std::get<ErbVel>(obs->data).iTOW, std::get<ErbVel>(obs->data).gSpeed,
                     std::get<ErbVel>(obs->data).heading, std::get<ErbVel>(obs->data).velD);
        }
        else if (msgId == ErbMessageID::ERB_MessageId_SVI)
        {
            obs->data = ErbSvi();

            std::get<ErbSvi>(obs->data).iTOW = packet.extractUint32();
            std::get<ErbSvi>(obs->data).nSV = packet.extractUint8();
            std::get<ErbSvi>(obs->data).idSV = packet.extractUint8();
            std::get<ErbSvi>(obs->data).typeSV = packet.extractUint8();
            std::get<ErbSvi>(obs->data).carPh = packet.extractInt32();
            std::get<ErbSvi>(obs->data).psRan = packet.extractInt32();
            std::get<ErbSvi>(obs->data).freqD = packet.extractInt32();
            std::get<ErbSvi>(obs->data).snr = packet.extractUint16();
            std::get<ErbSvi>(obs->data).azim = packet.extractUint16();
            std::get<ErbSvi>(obs->data).elev = packet.extractUint16();

            // Calculate the insTime with the iTOW
            auto currentTime = util::time::GetCurrentInsTime();
            if (!currentTime.empty())
            {
                auto gpst = currentTime.toGPSweekTow();
                currentTime = InsTime(gpst.gpsCycle,
                                      gpst.gpsWeek,
                                      static_cast<long double>(std::get<ErbSvi>(obs->data).iTOW) / 1000.0L);
                obs->insTime = currentTime;
            }

            LOG_DATA("Erb: SVI, iTOW {}, nSV {} , idSV {} , typeSV {} , carPh {}, psRan {} , freqD {} , snr {}, , azim {} , elev {}",
                     std::get<ErbSvi>(obs->data).iTOW, std::get<ErbSvi>(obs->data).nSV, std::get<ErbSvi>(obs->data).idSV,
                     std::get<ErbSvi>(obs->data).typeSV, std::get<ErbSvi>(obs->data).carPh,
                     std::get<ErbSvi>(obs->data).psRan, std::get<ErbSvi>(obs->data).freqD,
                     std::get<ErbSvi>(obs->data).snr, std::get<ErbSvi>(obs->data).azim,
                     std::get<ErbSvi>(obs->data).elev);
        }
        else if (msgId == ErbMessageID::ERB_MessageId_RTK)
        {
            obs->data = ErbRtk();

            std::get<ErbRtk>(obs->data).numSV = packet.extractUint8();
            std::get<ErbRtk>(obs->data).age = packet.extractUint16();
            std::get<ErbRtk>(obs->data).baselineN = packet.extractInt32();
            std::get<ErbRtk>(obs->data).baselineE = packet.extractInt32();
            std::get<ErbRtk>(obs->data).baselineD = packet.extractInt32();
            std::get<ErbRtk>(obs->data).arRatio = packet.extractUint16();
            std::get<ErbRtk>(obs->data).weekGPS = packet.extractUint16();
            std::get<ErbRtk>(obs->data).timeGPS = packet.extractUint32();

            LOG_DATA("Erb: RTK, numSV {}, age {} [s * 1e-2] , baselineN {} , baselineE {} , baselineD {}, arRatio {} [*1e-1] , weekGPS {} , timeGPS {}",
                     std::get<ErbRtk>(obs->data).numSV, std::get<ErbRtk>(obs->data).age, std::get<ErbRtk>(obs->data).baselineN,
                     std::get<ErbRtk>(obs->data).baselineE, std::get<ErbRtk>(obs->data).baselineD,
                     std::get<ErbRtk>(obs->data).arRatio, std::get<ErbRtk>(obs->data).weekGPS,
                     std::get<ErbRtk>(obs->data).timeGPS);
        }
        else
        {
            LOG_DATA("Erb: {:x}, Size {}, not implemented yet!", msgId, packet.getRawDataLength());
        }
    }
}

std::pair<uint8_t, uint8_t> NAV::vendor::emlid::checksumUBX(const std::vector<uint8_t>& data)
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