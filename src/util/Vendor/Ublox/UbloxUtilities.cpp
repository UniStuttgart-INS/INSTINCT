// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "UbloxUtilities.hpp"
#include "UbloxTypes.hpp"

#include "util/Eigen.hpp"
#include "Navigation/Transformations/CoordinateFrames.hpp"
#include "util/Logger.hpp"

#include "util/Time/TimeBase.hpp"

void NAV::vendor::ublox::decryptUbloxObs(const std::shared_ptr<NAV::UbloxObs>& obs, uart::protocol::Packet& packet)
{
    if (packet.type() == uart::protocol::Packet::Type::TYPE_BINARY)
    {
        obs->msgClass = static_cast<UbxClass>(packet.extractUint8());
        obs->msgId = packet.extractUint8();
        obs->payloadLength = packet.extractUint16();

        // Ack/Nak Messages: Acknowledge or Reject messages to UBX-CFG input messages
        if (obs->msgClass == UbxClass::UBX_CLASS_ACK)
        {
            auto msgId = static_cast<UbxAckMessages>(obs->msgId);
            if (msgId == UbxAckMessages::UBX_ACK_ACK)
            {
                obs->data = UbxAckAck();
                std::get<UbxAckAck>(obs->data).clsID = packet.extractUint8();
                std::get<UbxAckAck>(obs->data).msgID = packet.extractUint8();

                LOG_DATA("UBX:  ACK-ACK, clsID {}, msgID {}", std::get<UbxAckAck>(obs->data).clsID, std::get<UbxAckAck>(obs->data).msgID);
            }
            else if (msgId == UbxAckMessages::UBX_ACK_NAK)
            {
                obs->data = UbxAckNak();
                std::get<UbxAckNak>(obs->data).clsID = packet.extractUint8();
                std::get<UbxAckNak>(obs->data).msgID = packet.extractUint8();

                LOG_DATA("UBX:  ACK-NAK, Size {}", packet.getRawDataLength());
            }
            else
            {
                LOG_DATA("UBX:  ACK-{:x}, Size {}, not implemented yet!", obs->msgId, packet.getRawDataLength());
            }
        }
        // Configuration Input Messages: Configure the receiver
        else if (obs->msgClass == UbxClass::UBX_CLASS_CFG)
        {
            [[maybe_unused]] auto msgId = static_cast<UbxCfgMessages>(obs->msgId);
            LOG_DATA("UBX:  CFG-{:x}, Size {}, not implemented yet!", obs->msgId, packet.getRawDataLength());
        }
        // External Sensor Fusion Messages: External Sensor Measurements and Status Information
        else if (obs->msgClass == UbxClass::UBX_CLASS_ESF)
        {
            auto msgId = static_cast<UbxEsfMessages>(obs->msgId);
            if (msgId == UbxEsfMessages::UBX_ESF_INS)
            {
                LOG_DATA("UBX:  ESF-INS, Size {}, not implemented yet!", packet.getRawDataLength());
            }
            else if (msgId == UbxEsfMessages::UBX_ESF_MEAS)
            {
                LOG_DATA("UBX:  ESF-MEAS, Size {}, not implemented yet!", packet.getRawDataLength());
            }
            else if (msgId == UbxEsfMessages::UBX_ESF_RAW)
            {
                obs->data = UbxEsfRaw();
                for (auto& reserved1 : std::get<UbxEsfRaw>(obs->data).reserved1)
                {
                    reserved1 = packet.extractUint8();
                }
                for (int i = 0; i < (obs->payloadLength - 4) / 8; i++)
                {
                    NAV::vendor::ublox::UbxEsfRaw::UbxEsfRawData ubxEsfRawData;
                    ubxEsfRawData.data = packet.extractUint32();
                    ubxEsfRawData.sTtag = packet.extractUint32();
                    std::get<UbxEsfRaw>(obs->data).data.push_back(ubxEsfRawData);
                }

                // TODO: - UBX_ESF_RAW: Calculate the insTime somehow from the sensor time tag (sTtag)

                LOG_DATA("UBX:  ESF-RAW, {} measurements", (obs->payloadLength - 4) / 8);
            }
            else if (msgId == UbxEsfMessages::UBX_ESF_STATUS)
            {
                LOG_DATA("UBX:  ESF-STATUS, Size {}, not implemented yet!", packet.getRawDataLength());
            }
            else
            {
                LOG_DATA("UBX:  ESF-{:x}, Size {}, not implemented yet!", obs->msgId, packet.getRawDataLength());
            }
        }
        // High Rate Navigation Results Messages: High rate time, position, speed, heading
        else if (obs->msgClass == UbxClass::UBX_CLASS_HNR)
        {
            [[maybe_unused]] auto msgId = static_cast<UbxHnrMessages>(obs->msgId);
            LOG_DATA("UBX:  HNR-{:x}, Size {}, not implemented yet!", obs->msgId, packet.getRawDataLength());
        }
        // Information Messages: Printf-Style Messages, with IDs such as Error, Warning, Notice
        else if (obs->msgClass == UbxClass::UBX_CLASS_INF)
        {
            [[maybe_unused]] auto msgId = static_cast<UbxInfMessages>(obs->msgId);
            LOG_DATA("UBX:  INF-{:x}, Size {}, not implemented yet!", obs->msgId, packet.getRawDataLength());
        }
        // Logging Messages: Log creation, deletion, info and retrieval
        else if (obs->msgClass == UbxClass::UBX_CLASS_LOG)
        {
            [[maybe_unused]] auto msgId = static_cast<UbxLogMessages>(obs->msgId);
            LOG_DATA("UBX:  LOG-{:x}, Size {}, not implemented yet!", obs->msgId, packet.getRawDataLength());
        }
        // Multiple GNSS Assistance Messages: Assistance data for various GNSS
        else if (obs->msgClass == UbxClass::UBX_CLASS_MGA)
        {
            [[maybe_unused]] auto msgId = static_cast<UbxMgaMessages>(obs->msgId);
            LOG_DATA("UBX:  MGA-{:x}, Size {}, not implemented yet!", obs->msgId, packet.getRawDataLength());
        }
        // Monitoring Messages: Communication Status, CPU Load, Stack Usage, Task Status
        else if (obs->msgClass == UbxClass::UBX_CLASS_MON)
        {
            [[maybe_unused]] auto msgId = static_cast<UbxMonMessages>(obs->msgId);
            LOG_DATA("UBX:  MON-{:x}, Size {}, not implemented yet!", obs->msgId, packet.getRawDataLength());
        }
        // Navigation Results Messages: Position, Speed, Time, Acceleration, Heading, DOP, SVs used
        else if (obs->msgClass == UbxClass::UBX_CLASS_NAV)
        {
            auto msgId = static_cast<UbxNavMessages>(obs->msgId);
            if (msgId == UbxNavMessages::UBX_NAV_ATT)
            {
                obs->data = UbxNavAtt();

                std::get<UbxNavAtt>(obs->data).iTOW = packet.extractUint32();
                std::get<UbxNavAtt>(obs->data).version = packet.extractUint8();
                for (auto& reserved1 : std::get<UbxNavAtt>(obs->data).reserved1)
                {
                    reserved1 = packet.extractUint8();
                }
                std::get<UbxNavAtt>(obs->data).roll = packet.extractInt32();
                std::get<UbxNavAtt>(obs->data).pitch = packet.extractInt32();
                std::get<UbxNavAtt>(obs->data).heading = packet.extractInt32();
                std::get<UbxNavAtt>(obs->data).accRoll = packet.extractUint32();
                std::get<UbxNavAtt>(obs->data).accPitch = packet.extractUint32();
                std::get<UbxNavAtt>(obs->data).accHeading = packet.extractUint32();

                // Calculate the insTime with the iTOW
                auto currentTime = util::time::GetCurrentInsTime();
                if (!currentTime.empty())
                {
                    auto gpst = currentTime.toGPSweekTow();
                    currentTime = InsTime(gpst.gpsCycle,
                                          gpst.gpsWeek,
                                          static_cast<long double>(std::get<UbxNavAtt>(obs->data).iTOW) / 1000.0L);
                    obs->insTime = currentTime;
                }

                LOG_DATA("UBX:  NAV-ATT, iTOW {}, roll {}, pitch {}, heading {}", std::get<UbxNavAtt>(obs->data).iTOW, std::get<UbxNavAtt>(obs->data).roll, std::get<UbxNavAtt>(obs->data).pitch, std::get<UbxNavAtt>(obs->data).heading);
            }
            else if (msgId == UbxNavMessages::UBX_NAV_POSECEF)
            {
                obs->data = UbxNavPosecef();

                std::get<UbxNavPosecef>(obs->data).iTOW = packet.extractUint32();
                std::get<UbxNavPosecef>(obs->data).ecefX = packet.extractInt32();
                std::get<UbxNavPosecef>(obs->data).ecefY = packet.extractInt32();
                std::get<UbxNavPosecef>(obs->data).ecefZ = packet.extractInt32();
                std::get<UbxNavPosecef>(obs->data).pAcc = packet.extractUint32();

                // Calculate the insTime with the iTOW
                auto currentTime = util::time::GetCurrentInsTime();
                if (!currentTime.empty())
                {
                    auto gpst = currentTime.toGPSweekTow();
                    currentTime = InsTime(gpst.gpsCycle,
                                          gpst.gpsWeek,
                                          static_cast<long double>(std::get<UbxNavPosecef>(obs->data).iTOW) / 1000.0L);
                    obs->insTime = currentTime;
                }

                LOG_DATA("UBX: NAV-POSECEF, iTOW {}, x {}, y {}, z {}", std::get<UbxNavPosecef>(obs->data).iTOW, std::get<UbxNavPosecef>(obs->data).ecefX, std::get<UbxNavPosecef>(obs->data).ecefY, std::get<UbxNavPosecef>(obs->data).ecefZ);
            }
            else if (msgId == UbxNavMessages::UBX_NAV_POSLLH)
            {
                obs->data = UbxNavPosllh();

                std::get<UbxNavPosllh>(obs->data).iTOW = packet.extractUint32();
                std::get<UbxNavPosllh>(obs->data).lon = packet.extractInt32();
                std::get<UbxNavPosllh>(obs->data).lat = packet.extractInt32();
                std::get<UbxNavPosllh>(obs->data).height = packet.extractInt32();
                std::get<UbxNavPosllh>(obs->data).hMSL = packet.extractInt32();
                std::get<UbxNavPosllh>(obs->data).hAcc = packet.extractUint32();
                std::get<UbxNavPosllh>(obs->data).vAcc = packet.extractUint32();

                // Calculate the insTime with the iTOW
                auto currentTime = util::time::GetCurrentInsTime();
                if (!currentTime.empty())
                {
                    auto gpst = currentTime.toGPSweekTow();
                    currentTime = InsTime(gpst.gpsCycle,
                                          gpst.gpsWeek,
                                          static_cast<long double>(std::get<UbxNavPosllh>(obs->data).iTOW) / 1000.0L);
                    obs->insTime = currentTime;
                }

                LOG_DATA("UBX:  NAV-POSLLH, iTOW {}, lon {}, lat {}, height {}", std::get<UbxNavPosllh>(obs->data).iTOW, std::get<UbxNavPosllh>(obs->data).lon, std::get<UbxNavPosllh>(obs->data).lat, std::get<UbxNavPosllh>(obs->data).height);
            }
            else if (msgId == UbxNavMessages::UBX_NAV_VELNED)
            {
                obs->data = UbxNavVelned();

                std::get<UbxNavVelned>(obs->data).iTOW = packet.extractUint32();
                std::get<UbxNavVelned>(obs->data).velN = packet.extractInt32();
                std::get<UbxNavVelned>(obs->data).velE = packet.extractInt32();
                std::get<UbxNavVelned>(obs->data).velD = packet.extractInt32();
                std::get<UbxNavVelned>(obs->data).speed = packet.extractUint32();
                std::get<UbxNavVelned>(obs->data).gSpeed = packet.extractUint32();
                std::get<UbxNavVelned>(obs->data).heading = packet.extractInt32();
                std::get<UbxNavVelned>(obs->data).sAcc = packet.extractUint32();
                std::get<UbxNavVelned>(obs->data).cAcc = packet.extractUint32();

                // Calculate the insTime with the iTOW
                auto currentTime = util::time::GetCurrentInsTime();
                if (!currentTime.empty())
                {
                    auto gpst = currentTime.toGPSweekTow();
                    currentTime = InsTime(gpst.gpsCycle,
                                          gpst.gpsWeek,
                                          static_cast<long double>(std::get<UbxNavVelned>(obs->data).iTOW) / 1000.0L);
                    obs->insTime = currentTime;
                }

                LOG_DATA("UBX:  NAV-VELNED, iTOW {}, gSpeed {} [cm/s], heading {} [deg*1e-5], velD {} [cm/s]", std::get<UbxNavVelned>(obs->data).iTOW, std::get<UbxNavVelned>(obs->data).gSpeed, std::get<UbxNavVelned>(obs->data).heading, std::get<UbxNavVelned>(obs->data).velD);
            }
            else
            {
                LOG_DATA("UBX:  NAV-{:x}, Size {}, not implemented yet!", obs->msgId, packet.getRawDataLength());
            }
        }
        // Receiver Manager Messages: Satellite Status, RTC Status
        else if (obs->msgClass == UbxClass::UBX_CLASS_RXM)
        {
            auto msgId = static_cast<UbxRxmMessages>(obs->msgId);
            if (msgId == UbxRxmMessages::UBX_RXM_RAWX)
            {
                obs->data = UbxRxmRawx();

                std::get<UbxRxmRawx>(obs->data).rcvTow = packet.extractDouble();
                std::get<UbxRxmRawx>(obs->data).week = packet.extractUint16();
                std::get<UbxRxmRawx>(obs->data).leapS = packet.extractInt8();
                std::get<UbxRxmRawx>(obs->data).numMeas = packet.extractUint8();
                std::get<UbxRxmRawx>(obs->data).recStat = packet.extractUint8();
                std::get<UbxRxmRawx>(obs->data).version = packet.extractUint8();
                for (auto& reserved1 : std::get<UbxRxmRawx>(obs->data).reserved1)
                {
                    reserved1 = packet.extractUint8();
                }
                for (size_t i = 0; i < std::get<UbxRxmRawx>(obs->data).numMeas; i++)
                {
                    NAV::vendor::ublox::UbxRxmRawx::UbxRxmRawxData ubxRxmRawxData;
                    ubxRxmRawxData.prMes = packet.extractDouble();
                    ubxRxmRawxData.cpMes = packet.extractDouble();
                    ubxRxmRawxData.doMes = packet.extractFloat();
                    ubxRxmRawxData.gnssId = packet.extractUint8();
                    ubxRxmRawxData.svId = packet.extractUint8();
                    ubxRxmRawxData.sigId = packet.extractUint8();
                    ubxRxmRawxData.freqId = packet.extractUint8();
                    ubxRxmRawxData.locktime = packet.extractUint16();
                    ubxRxmRawxData.cno = packet.extractUint8();
                    ubxRxmRawxData.prStdev = packet.extractUint8();
                    ubxRxmRawxData.cpStdev = packet.extractUint8();
                    ubxRxmRawxData.doStdev = packet.extractUint8();
                    ubxRxmRawxData.trkStat = packet.extractUint8();
                    ubxRxmRawxData.reserved2 = packet.extractUint8();
                    std::get<UbxRxmRawx>(obs->data).data.push_back(ubxRxmRawxData);
                }

                obs->insTime = InsTime(0,
                                       std::get<UbxRxmRawx>(obs->data).week,
                                       static_cast<long double>(std::get<UbxRxmRawx>(obs->data).rcvTow));

                std::string satInfo;
                for (const auto& sat : std::get<UbxRxmRawx>(obs->data).data)
                {
                    satInfo += "[" + std::to_string(sat.gnssId);
                    satInfo += ", " + std::to_string(sat.svId);
                    satInfo += ", " + std::to_string(sat.sigId) + "], ";
                }

                LOG_DATA("UBX:  RXM-RAWX, Size {}, rcvTow {}, numMeas {}, satInfo {}",
                         packet.getRawDataLength(), std::get<UbxRxmRawx>(obs->data).rcvTow, std::get<UbxRxmRawx>(obs->data).numMeas, satInfo);
            }
            else if (msgId == UbxRxmMessages::UBX_RXM_SFRBX)
            {
                obs->data = UbxRxmSfrbx();

                std::get<UbxRxmSfrbx>(obs->data).gnssId = packet.extractUint8();
                std::get<UbxRxmSfrbx>(obs->data).svId = packet.extractUint8();
                std::get<UbxRxmSfrbx>(obs->data).reserved1 = packet.extractUint8();
                std::get<UbxRxmSfrbx>(obs->data).freqId = packet.extractUint8();
                std::get<UbxRxmSfrbx>(obs->data).numWords = packet.extractUint8();
                std::get<UbxRxmSfrbx>(obs->data).chn = packet.extractUint8();
                std::get<UbxRxmSfrbx>(obs->data).version = packet.extractUint8();
                std::get<UbxRxmSfrbx>(obs->data).reserved2 = packet.extractUint8();

                for (size_t i = 0; i < std::get<UbxRxmSfrbx>(obs->data).numWords; i++)
                {
                    std::get<UbxRxmSfrbx>(obs->data).dwrd.emplace_back(packet.extractUint32());
                }

                LOG_DATA("UBX:  RXM-SFRBX, gnssId {}, svId {}, freqId {}, numWords {}, chn {}, version {}",
                         std::get<UbxRxmSfrbx>(obs->data).gnssId,
                         std::get<UbxRxmSfrbx>(obs->data).svId,
                         std::get<UbxRxmSfrbx>(obs->data).freqId,
                         std::get<UbxRxmSfrbx>(obs->data).numWords,
                         std::get<UbxRxmSfrbx>(obs->data).chn,
                         std::get<UbxRxmSfrbx>(obs->data).version);
            }
            else
            {
                LOG_DATA("UBX:  RXM-{:x}, Size {}, not implemented yet!", obs->msgId, packet.getRawDataLength());
            }
        }
        // Security Feature Messages
        else if (obs->msgClass == UbxClass::UBX_CLASS_SEC)
        {
            [[maybe_unused]] auto msgId = static_cast<UbxSecMessages>(obs->msgId);
            LOG_DATA("UBX:  SEC-{:x}, Size {}, not implemented yet!", obs->msgId, packet.getRawDataLength());
        }
        // Timing Messages: Time Pulse Output, Time Mark Results
        else if (obs->msgClass == UbxClass::UBX_CLASS_TIM)
        {
            [[maybe_unused]] auto msgId = static_cast<UbxTimMessages>(obs->msgId);
            LOG_DATA("UBX:  TIM-{:x}, Size {}, not implemented yet!", obs->msgId, packet.getRawDataLength());
        }
        // Firmware Update Messages: Memory/Flash erase/write, Reboot, Flash identification, etc.
        else if (obs->msgClass == UbxClass::UBX_CLASS_UPD)
        {
            [[maybe_unused]] auto msgId = static_cast<UbxUpdMessages>(obs->msgId);
            LOG_DATA("UBX:  UPD-{:x}, Size {}, not implemented yet!", obs->msgId, packet.getRawDataLength());
        }
        else
        {
            LOG_DATA("UBX:  {:x}-{:x}, Size {}, not implemented yet!", static_cast<int>(obs->msgClass), static_cast<int>(obs->msgId), packet.getRawDataLength());
        }
    }
    else if (packet.type() == uart::protocol::Packet::Type::TYPE_ASCII)
    {
        LOG_DATA("NMEA: {}", packet.datastr().substr(0, packet.datastr().size() - 2));
    }
}

std::pair<uint8_t, uint8_t> NAV::vendor::ublox::checksumUBX(const std::vector<uint8_t>& data)
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

uint8_t NAV::vendor::ublox::checksumNMEA(const std::vector<uint8_t>& data)
{
    uint8_t calcChecksum = 0;
    //  |->                          <-|            Checksum without first $ sign and till the *
    // $--ZDA,hhmmss.ss,xx,xx,xxxx,xx,xx*hh<CR><LF>
    for (size_t i = 1; i < data.size() - 5; i++)
    {
        calcChecksum ^= data.at(i);
    }
    return calcChecksum;
}