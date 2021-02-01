#include "UbloxUtilities.hpp"
#include "UbloxTypes.hpp"

#include "util/Eigen.hpp"
#include "util/InsTransformations.hpp"
#include "util/Logger.hpp"

#include "util/Time/TimeBase.hpp"

void NAV::sensors::ublox::decryptUbloxObs(std::shared_ptr<NAV::UbloxObs>& obs, bool peek)
{
    if (obs->raw.type() == uart::protocol::Packet::Type::TYPE_BINARY)
    {
        obs->msgClass = static_cast<UbxClass>(obs->raw.extractUint8());
        obs->msgId = obs->raw.extractUint8();
        obs->payloadLength = obs->raw.extractUint16();

        // Ack/Nak Messages: Acknowledge or Reject messages to UBX-CFG input messages
        if (obs->msgClass == UbxClass::UBX_CLASS_ACK)
        {
            auto msgId = static_cast<UbxAckMessages>(obs->msgId);
            if (msgId == UbxAckMessages::UBX_ACK_ACK)
            {
                obs->data = UbxAckAck();
                std::get<UbxAckAck>(obs->data).clsID = obs->raw.extractUint8();
                std::get<UbxAckAck>(obs->data).msgID = obs->raw.extractUint8();

                LOG_DATA("UBX:  ACK-ACK, clsID {}, msgID {}", std::get<UbxAckAck>(obs->data).clsID, std::get<UbxAckAck>(obs->data).msgID);
            }
            else if (msgId == UbxAckMessages::UBX_ACK_NAK)
            {
                obs->data = UbxAckNak();
                std::get<UbxAckNak>(obs->data).clsID = obs->raw.extractUint8();
                std::get<UbxAckNak>(obs->data).msgID = obs->raw.extractUint8();

                if (!peek)
                {
                    LOG_DATA("UBX:  ACK-NAK, Size {}", obs->raw.getRawDataLength());
                }
            }
            else
            {
                if (!peek)
                {
                    LOG_DATA("UBX:  ACK-{:x}, Size {}, not implemented yet!", msgId, obs->raw.getRawDataLength());
                }
            }
        }
        // AssistNow Aiding Messages: Ephemeris, Almanac, other A-GPS data input
        else if (obs->msgClass == UbxClass::UBX_CLASS_AID)
        {
            [[maybe_unused]] auto msgId = static_cast<UbxAidMessages>(obs->msgId);
            if (!peek)
            {
                LOG_DATA("UBX:  AID-{:x}, Size {}, not implemented yet!", msgId, obs->raw.getRawDataLength());
            }
        }
        // Configuration Input Messages: Configure the receiver
        else if (obs->msgClass == UbxClass::UBX_CLASS_CFG)
        {
            [[maybe_unused]] auto msgId = static_cast<UbxCfgMessages>(obs->msgId);
            if (!peek)
            {
                LOG_DATA("UBX:  CFG-{:x}, Size {}, not implemented yet!", msgId, obs->raw.getRawDataLength());
            }
        }
        // External Sensor Fusion Messages: External Sensor Measurements and Status Information
        else if (obs->msgClass == UbxClass::UBX_CLASS_ESF)
        {
            auto msgId = static_cast<UbxEsfMessages>(obs->msgId);
            if (msgId == UbxEsfMessages::UBX_ESF_INS)
            {
                if (!peek)
                {
                    LOG_DATA("UBX:  ESF-INS, Size {}, not implemented yet!", obs->raw.getRawDataLength());
                }
            }
            else if (msgId == UbxEsfMessages::UBX_ESF_MEAS)
            {
                if (!peek)
                {
                    LOG_DATA("UBX:  ESF-MEAS, Size {}, not implemented yet!", obs->raw.getRawDataLength());
                }
            }
            else if (msgId == UbxEsfMessages::UBX_ESF_RAW)
            {
                obs->data = UbxEsfRaw();
                for (auto& reserved1 : std::get<UbxEsfRaw>(obs->data).reserved1)
                {
                    reserved1 = obs->raw.extractUint8();
                }
                for (int i = 0; i < (obs->payloadLength - 4) / 8; i++)
                {
                    std::get<UbxEsfRaw>(obs->data).data.emplace_back(obs->raw.extractUint32(), obs->raw.extractUint32());
                }

                // TODO: - UBX_ESF_RAW: Calculate the insTime somehow from the sensor time tag (sTtag)

                if (!peek)
                {
                    LOG_DATA("UBX:  ESF-RAW, {} measurements", (obs->payloadLength - 4) / 8);
                }
            }
            else if (msgId == UbxEsfMessages::UBX_ESF_STATUS)
            {
                if (!peek)
                {
                    LOG_DATA("UBX:  ESF-STATUS, Size {}, not implemented yet!", obs->raw.getRawDataLength());
                }
            }
            else
            {
                if (!peek)
                {
                    LOG_DATA("UBX:  ESF-{:x}, Size {}, not implemented yet!", msgId, obs->raw.getRawDataLength());
                }
            }
        }
        // High Rate Navigation Results Messages: High rate time, position, speed, heading
        else if (obs->msgClass == UbxClass::UBX_CLASS_HNR)
        {
            [[maybe_unused]] auto msgId = static_cast<UbxHnrMessages>(obs->msgId);
            if (!peek)
            {
                LOG_DATA("UBX:  HNR-{:x}, Size {}, not implemented yet!", msgId, obs->raw.getRawDataLength());
            }
        }
        // Information Messages: Printf-Style Messages, with IDs such as Error, Warning, Notice
        else if (obs->msgClass == UbxClass::UBX_CLASS_INF)
        {
            [[maybe_unused]] auto msgId = static_cast<UbxInfMessages>(obs->msgId);
            if (!peek)
            {
                LOG_DATA("UBX:  INF-{:x}, Size {}, not implemented yet!", msgId, obs->raw.getRawDataLength());
            }
        }
        // Logging Messages: Log creation, deletion, info and retrieval
        else if (obs->msgClass == UbxClass::UBX_CLASS_LOG)
        {
            [[maybe_unused]] auto msgId = static_cast<UbxLogMessages>(obs->msgId);
            if (!peek)
            {
                LOG_DATA("UBX:  LOG-{:x}, Size {}, not implemented yet!", msgId, obs->raw.getRawDataLength());
            }
        }
        // Multiple GNSS Assistance Messages: Assistance data for various GNSS
        else if (obs->msgClass == UbxClass::UBX_CLASS_MGA)
        {
            [[maybe_unused]] auto msgId = static_cast<UbxMgaMessages>(obs->msgId);
            if (!peek)
            {
                LOG_DATA("UBX:  MGA-{:x}, Size {}, not implemented yet!", msgId, obs->raw.getRawDataLength());
            }
        }
        // Monitoring Messages: Communication Status, CPU Load, Stack Usage, Task Status
        else if (obs->msgClass == UbxClass::UBX_CLASS_MON)
        {
            [[maybe_unused]] auto msgId = static_cast<UbxMonMessages>(obs->msgId);
            if (!peek)
            {
                LOG_DATA("UBX:  MON-{:x}, Size {}, not implemented yet!", msgId, obs->raw.getRawDataLength());
            }
        }
        // Navigation Results Messages: Position, Speed, Time, Acceleration, Heading, DOP, SVs used
        else if (obs->msgClass == UbxClass::UBX_CLASS_NAV)
        {
            auto msgId = static_cast<UbxNavMessages>(obs->msgId);
            if (msgId == UbxNavMessages::UBX_NAV_ATT)
            {
                obs->data = UbxNavAtt();

                std::get<UbxNavAtt>(obs->data).iTOW = obs->raw.extractUint32();
                std::get<UbxNavAtt>(obs->data).version = obs->raw.extractUint8();
                for (auto& reserved1 : std::get<UbxNavAtt>(obs->data).reserved1)
                {
                    reserved1 = obs->raw.extractUint8();
                }
                std::get<UbxNavAtt>(obs->data).roll = obs->raw.extractInt32();
                std::get<UbxNavAtt>(obs->data).pitch = obs->raw.extractInt32();
                std::get<UbxNavAtt>(obs->data).heading = obs->raw.extractInt32();
                std::get<UbxNavAtt>(obs->data).accRoll = obs->raw.extractUint32();
                std::get<UbxNavAtt>(obs->data).accPitch = obs->raw.extractUint32();
                std::get<UbxNavAtt>(obs->data).accHeading = obs->raw.extractUint32();

                // Calculate the insTime with the iTOW
                auto currentTime = util::time::GetCurrentTime();
                if (!currentTime.empty())
                {
                    auto gpst = currentTime.toGPSweekTow();
                    currentTime = InsTime(gpst.gpsCycle,
                                          gpst.gpsWeek,
                                          static_cast<long double>(std::get<UbxNavAtt>(obs->data).iTOW) / 1000.0L);
                    obs->insTime = currentTime;
                }

                if (!peek)
                {
                    LOG_DATA("UBX:  NAV-ATT, iTOW {}, roll {}, pitch {}, heading {}", std::get<UbxNavAtt>(obs->data).iTOW, std::get<UbxNavAtt>(obs->data).roll, std::get<UbxNavAtt>(obs->data).pitch, std::get<UbxNavAtt>(obs->data).heading);
                }
            }
            else if (msgId == UbxNavMessages::UBX_NAV_POSECEF)
            {
                obs->data = UbxNavPosecef();

                std::get<UbxNavPosecef>(obs->data).iTOW = obs->raw.extractUint32();
                std::get<UbxNavPosecef>(obs->data).ecefX = obs->raw.extractInt32();
                std::get<UbxNavPosecef>(obs->data).ecefY = obs->raw.extractInt32();
                std::get<UbxNavPosecef>(obs->data).ecefZ = obs->raw.extractInt32();
                std::get<UbxNavPosecef>(obs->data).pAcc = obs->raw.extractUint32();

                // Calculate the insTime with the iTOW
                auto currentTime = util::time::GetCurrentTime();
                if (!currentTime.empty())
                {
                    auto gpst = currentTime.toGPSweekTow();
                    currentTime = InsTime(gpst.gpsCycle,
                                          gpst.gpsWeek,
                                          static_cast<long double>(std::get<UbxNavPosecef>(obs->data).iTOW) / 1000.0L);
                    obs->insTime = currentTime;
                }

                obs->position_ecef.emplace(std::get<UbxNavPosecef>(obs->data).ecefX * 1e-2,
                                           std::get<UbxNavPosecef>(obs->data).ecefY * 1e-2,
                                           std::get<UbxNavPosecef>(obs->data).ecefZ * 1e-2);

                if (!peek)
                {
                    LOG_DATA("UBX: NAV-POSECEF, iTOW {}, x {}, y {}, z {}", std::get<UbxNavPosecef>(obs->data).iTOW, std::get<UbxNavPosecef>(obs->data).ecefX, std::get<UbxNavPosecef>(obs->data).ecefY, std::get<UbxNavPosecef>(obs->data).ecefZ);
                }
            }
            else if (msgId == UbxNavMessages::UBX_NAV_POSLLH)
            {
                obs->data = UbxNavPosllh();

                std::get<UbxNavPosllh>(obs->data).iTOW = obs->raw.extractUint32();
                std::get<UbxNavPosllh>(obs->data).lon = obs->raw.extractInt32();
                std::get<UbxNavPosllh>(obs->data).lat = obs->raw.extractInt32();
                std::get<UbxNavPosllh>(obs->data).height = obs->raw.extractInt32();
                std::get<UbxNavPosllh>(obs->data).hMSL = obs->raw.extractInt32();
                std::get<UbxNavPosllh>(obs->data).hAcc = obs->raw.extractUint32();
                std::get<UbxNavPosllh>(obs->data).vAcc = obs->raw.extractUint32();

                // Calculate the insTime with the iTOW
                auto currentTime = util::time::GetCurrentTime();
                if (!currentTime.empty())
                {
                    auto gpst = currentTime.toGPSweekTow();
                    currentTime = InsTime(gpst.gpsCycle,
                                          gpst.gpsWeek,
                                          static_cast<long double>(std::get<UbxNavPosllh>(obs->data).iTOW) / 1000.0L);
                    obs->insTime = currentTime;
                }

                Eigen::Vector3d latLonAlt(trafo::deg2rad(std::get<UbxNavPosllh>(obs->data).lat * 1e-7),
                                          trafo::deg2rad(std::get<UbxNavPosllh>(obs->data).lon * 1e-7),
                                          std::get<UbxNavPosllh>(obs->data).height * 1e-3);

                obs->position_ecef.emplace(trafo::lla2ecef_WGS84(latLonAlt));

                if (!peek)
                {
                    LOG_DATA("UBX:  NAV-POSLLH, iTOW {}, lon {}, lat {}, height {}", std::get<UbxNavPosllh>(obs->data).iTOW, std::get<UbxNavPosllh>(obs->data).lon, std::get<UbxNavPosllh>(obs->data).lat, std::get<UbxNavPosllh>(obs->data).height);
                }
            }
            else if (msgId == UbxNavMessages::UBX_NAV_VELNED)
            {
                obs->data = UbxNavVelned();

                std::get<UbxNavVelned>(obs->data).iTOW = obs->raw.extractUint32();
                std::get<UbxNavVelned>(obs->data).velN = obs->raw.extractInt32();
                std::get<UbxNavVelned>(obs->data).velE = obs->raw.extractInt32();
                std::get<UbxNavVelned>(obs->data).velD = obs->raw.extractInt32();
                std::get<UbxNavVelned>(obs->data).speed = obs->raw.extractUint32();
                std::get<UbxNavVelned>(obs->data).gSpeed = obs->raw.extractUint32();
                std::get<UbxNavVelned>(obs->data).heading = obs->raw.extractInt32();
                std::get<UbxNavVelned>(obs->data).sAcc = obs->raw.extractUint32();
                std::get<UbxNavVelned>(obs->data).cAcc = obs->raw.extractUint32();

                // Calculate the insTime with the iTOW
                auto currentTime = util::time::GetCurrentTime();
                if (!currentTime.empty())
                {
                    auto gpst = currentTime.toGPSweekTow();
                    currentTime = InsTime(gpst.gpsCycle,
                                          gpst.gpsWeek,
                                          static_cast<long double>(std::get<UbxNavVelned>(obs->data).iTOW) / 1000.0L);
                    obs->insTime = currentTime;
                }

                if (!peek)
                {
                    LOG_DATA("UBX:  NAV-VELNED, iTOW {}, gSpeed {} [cm/s], heading {} [deg*1e-5], velD {} [cm/s]", std::get<UbxNavVelned>(obs->data).iTOW, std::get<UbxNavVelned>(obs->data).gSpeed, std::get<UbxNavVelned>(obs->data).heading, std::get<UbxNavVelned>(obs->data).velD);
                }
            }
            else
            {
                if (!peek)
                {
                    LOG_DATA("UBX:  NAV-{:x}, Size {}, not implemented yet!", msgId, obs->raw.getRawDataLength());
                }
            }
        }
        // Receiver Manager Messages: Satellite Status, RTC Status
        else if (obs->msgClass == UbxClass::UBX_CLASS_RXM)
        {
            auto msgId = static_cast<UbxRxmMessages>(obs->msgId);
            if (msgId == UbxRxmMessages::UBX_RXM_RAWX)
            {
                obs->data = UbxRxmRawx();

                std::get<UbxRxmRawx>(obs->data).rcvTow = obs->raw.extractDouble();
                std::get<UbxRxmRawx>(obs->data).week = obs->raw.extractUint16();
                std::get<UbxRxmRawx>(obs->data).leapS = obs->raw.extractInt8();
                std::get<UbxRxmRawx>(obs->data).numMeas = obs->raw.extractUint8();
                std::get<UbxRxmRawx>(obs->data).recStat = obs->raw.extractUint8();
                for (auto& reserved1 : std::get<UbxRxmRawx>(obs->data).reserved1)
                {
                    reserved1 = obs->raw.extractUint8();
                }
                for (size_t i = 0; i < std::get<UbxRxmRawx>(obs->data).numMeas; i++)
                {
                    std::get<UbxRxmRawx>(obs->data).data.emplace_back(obs->raw.extractDouble(), // prMes
                                                                      obs->raw.extractDouble(), // cpMes
                                                                      obs->raw.extractFloat(),  // doMes
                                                                      obs->raw.extractUint8(),  // gnssId
                                                                      obs->raw.extractUint8(),  // svId
                                                                      obs->raw.extractUint8(),  // reserved2
                                                                      obs->raw.extractUint8(),  // freqId
                                                                      obs->raw.extractUint16(), // locktime
                                                                      obs->raw.extractUint8(),  // cno
                                                                      obs->raw.extractUint8(),  // prStdev
                                                                      obs->raw.extractUint8(),  // cpStdev
                                                                      obs->raw.extractUint8(),  // doStdev
                                                                      obs->raw.extractUint8(),  // trkStat
                                                                      obs->raw.extractUint8()   // reserved3
                    );
                }

                obs->insTime = InsTime(0,
                                       std::get<UbxRxmRawx>(obs->data).week,
                                       static_cast<long double>(std::get<UbxRxmRawx>(obs->data).rcvTow));

                if (!peek)
                {
                    LOG_DATA("UBX:  RXM-RAWX, Size {}, rcvTow {}, numMeas {}",
                             obs->raw.getRawDataLength(), std::get<UbxRxmRawx>(obs->data).rcvTow, std::get<UbxRxmRawx>(obs->data).numMeas);
                }
            }
            else if (msgId == UbxRxmMessages::UBX_RXM_SFRBX)
            {
                obs->data = UbxRxmSfrbx();

                std::get<UbxRxmSfrbx>(obs->data).gnssId = obs->raw.extractUint8();
                std::get<UbxRxmSfrbx>(obs->data).svId = obs->raw.extractUint8();
                std::get<UbxRxmSfrbx>(obs->data).reserved1 = obs->raw.extractUint8();
                std::get<UbxRxmSfrbx>(obs->data).freqId = obs->raw.extractUint8();
                std::get<UbxRxmSfrbx>(obs->data).numWords = obs->raw.extractUint8();
                std::get<UbxRxmSfrbx>(obs->data).chn = obs->raw.extractUint8();
                std::get<UbxRxmSfrbx>(obs->data).version = obs->raw.extractUint8();
                std::get<UbxRxmSfrbx>(obs->data).reserved2 = obs->raw.extractUint8();

                for (size_t i = 0; i < std::get<UbxRxmSfrbx>(obs->data).numWords; i++)
                {
                    std::get<UbxRxmSfrbx>(obs->data).dwrd.emplace_back(obs->raw.extractUint32());
                }

                if (!peek)
                {
                    LOG_DATA("UBX:  RXM-SFRBX, gnssId {}, svId {}, freqId {}, numWords {}",
                             std::get<UbxRxmSfrbx>(obs->data).gnssId, std::get<UbxRxmSfrbx>(obs->data).svId, std::get<UbxRxmSfrbx>(obs->data).freqId, std::get<UbxRxmSfrbx>(obs->data).numWords);
                }
            }
            else
            {
                if (!peek)
                {
                    LOG_DATA("UBX:  RXM-{:x}, Size {}, not implemented yet!", msgId, obs->raw.getRawDataLength());
                }
            }
        }
        // Security Feature Messages
        else if (obs->msgClass == UbxClass::UBX_CLASS_SEC)
        {
            [[maybe_unused]] auto msgId = static_cast<UbxSecMessages>(obs->msgId);
            if (!peek)
            {
                LOG_DATA("UBX:  SEC-{:x}, Size {}, not implemented yet!", msgId, obs->raw.getRawDataLength());
            }
        }
        // Timing Messages: Time Pulse Output, Time Mark Results
        else if (obs->msgClass == UbxClass::UBX_CLASS_TIM)
        {
            [[maybe_unused]] auto msgId = static_cast<UbxTimMessages>(obs->msgId);
            if (!peek)
            {
                LOG_DATA("UBX:  TIM-{:x}, Size {}, not implemented yet!", msgId, obs->raw.getRawDataLength());
            }
        }
        // Firmware Update Messages: Memory/Flash erase/write, Reboot, Flash identification, etc.
        else if (obs->msgClass == UbxClass::UBX_CLASS_UPD)
        {
            [[maybe_unused]] auto msgId = static_cast<UbxUpdMessages>(obs->msgId);
            if (!peek)
            {
                LOG_DATA("UBX:  UPD-{:x}, Size {}, not implemented yet!", msgId, obs->raw.getRawDataLength());
            }
        }
        else
        {
            if (!peek)
            {
                LOG_DATA("UBX:  {:x}-{:x}, Size {}, not implemented yet!", obs->msgClass, obs->msgId, obs->raw.getRawDataLength());
            }
        }
    }
    else if (obs->raw.type() == uart::protocol::Packet::Type::TYPE_ASCII)
    {
        if (!peek)
        {
            LOG_DATA("NMEA: {}", obs->raw.datastr().substr(0, obs->raw.datastr().size() - 2));
        }
    }
}

std::pair<uint8_t, uint8_t> NAV::sensors::ublox::checksumUBX(const std::vector<uint8_t>& data)
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

uint8_t NAV::sensors::ublox::checksumNMEA(const std::vector<uint8_t>& data)
{
    uint8_t calcChecksum = 0;
    for (size_t i = 1; i < data.size() - 5; i++)
    {
        calcChecksum ^= data.at(i);
    }
    return calcChecksum;
}