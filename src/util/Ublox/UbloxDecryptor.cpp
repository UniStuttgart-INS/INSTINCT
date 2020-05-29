#include "UbloxDecryptor.hpp"

#include "util/Logger.hpp"

void NAV::ublox::decryptUbloxObs(std::shared_ptr<NAV::UbloxObs>& obs, NAV::InsTime& /*currentInsTime*/, bool peek)
{
    if (obs->raw.type() == ublox::UbloxPacket::Type::TYPE_BINARY)
    {
        obs->msgClass = static_cast<ublox::UbxClass>(obs->raw.extractUint8());
        obs->msgId = obs->raw.extractUint8();
        obs->payloadLength = obs->raw.extractUint16();

        // Ack/Nak Messages: Acknowledge or Reject messages to UBX-CFG input messages
        if (obs->msgClass == ublox::UbxClass::UBX_CLASS_ACK)
        {
            auto msgId = static_cast<ublox::UbxAckMessages>(obs->msgId);
            if (msgId == ublox::UbxAckMessages::UBX_ACK_ACK)
            {
                obs->data = ublox::UbxAckAck();
                std::get<ublox::UbxAckAck>(obs->data).clsID = obs->raw.extractUint8();
                std::get<ublox::UbxAckAck>(obs->data).msgID = obs->raw.extractUint8();

                // TODO: Calculate the insTime somehow

                LOG_DATA("UBX:  ACK-ACK, clsID {}, msgID {}", std::get<ublox::UbxAckAck>(obs->data).clsID, std::get<ublox::UbxAckAck>(obs->data).msgID);
            }
            else if (msgId == ublox::UbxAckMessages::UBX_ACK_NAK)
            {
                obs->data = ublox::UbxAckNak();
                std::get<ublox::UbxAckNak>(obs->data).clsID = obs->raw.extractUint8();
                std::get<ublox::UbxAckNak>(obs->data).msgID = obs->raw.extractUint8();

                // TODO: Calculate the insTime somehow

                if (!peek)
                {
                    LOG_DATA("UBX:  ACK-NAK, Size {}", obs->raw.getRawDataLength());
                }
            }
            else
            {
                if (!peek)
                {
                    LOG_WARN("UBX:  ACK-{:x}, Size {}, not implemented yet!", msgId, obs->raw.getRawDataLength());
                }
            }
        }
        // AssistNow Aiding Messages: Ephemeris, Almanac, other A-GPS data input
        else if (obs->msgClass == ublox::UbxClass::UBX_CLASS_AID)
        {
            [[maybe_unused]] auto msgId = static_cast<ublox::UbxAidMessages>(obs->msgId);
            if (!peek)
            {
                LOG_WARN("UBX:  AID-{:x}, Size {}, not implemented yet!", msgId, obs->raw.getRawDataLength());
            }
        }
        // Configuration Input Messages: Configure the receiver
        else if (obs->msgClass == ublox::UbxClass::UBX_CLASS_CFG)
        {
            [[maybe_unused]] auto msgId = static_cast<ublox::UbxCfgMessages>(obs->msgId);
            if (!peek)
            {
                LOG_WARN("UBX:  CFG-{:x}, Size {}, not implemented yet!", msgId, obs->raw.getRawDataLength());
            }
        }
        // External Sensor Fusion Messages: External Sensor Measurements and Status Information
        else if (obs->msgClass == ublox::UbxClass::UBX_CLASS_ESF)
        {
            auto msgId = static_cast<ublox::UbxEsfMessages>(obs->msgId);
            if (msgId == ublox::UbxEsfMessages::UBX_ESF_INS)
            {
                if (!peek)
                {
                    LOG_WARN("UBX:  ESF-INS, Size {}, not implemented yet!", obs->raw.getRawDataLength());
                }
            }
            else if (msgId == ublox::UbxEsfMessages::UBX_ESF_MEAS)
            {
                if (!peek)
                {
                    LOG_WARN("UBX:  ESF-MEAS, Size {}, not implemented yet!", obs->raw.getRawDataLength());
                }
            }
            else if (msgId == ublox::UbxEsfMessages::UBX_ESF_RAW)
            {
                obs->data = ublox::UbxEsfRaw();
                for (auto& reserved1 : std::get<ublox::UbxEsfRaw>(obs->data).reserved1)
                {
                    reserved1 = obs->raw.extractUint8();
                }
                for (size_t i = 0; i < (obs->payloadLength - 4) / 8; i++)
                {
                    std::get<ublox::UbxEsfRaw>(obs->data).data.emplace_back(obs->raw.extractUint32(), obs->raw.extractUint32());
                }

                // TODO: Calculate the insTime somehow from the sensor time tag (sTtag)

                if (!peek)
                {
                    LOG_DATA("UBX:  ESF-RAW, {} measurements", (obs->payloadLength - 4) / 8);
                }
            }
            else if (msgId == ublox::UbxEsfMessages::UBX_ESF_STATUS)
            {
                if (!peek)
                {
                    LOG_WARN("UBX:  ESF-STATUS, Size {}, not implemented yet!", obs->raw.getRawDataLength());
                }
            }
            else
            {
                if (!peek)
                {
                    LOG_WARN("UBX:  ESF-{:x}, Size {}, not implemented yet!", msgId, obs->raw.getRawDataLength());
                }
            }
        }
        // High Rate Navigation Results Messages: High rate time, position, speed, heading
        else if (obs->msgClass == ublox::UbxClass::UBX_CLASS_HNR)
        {
            [[maybe_unused]] auto msgId = static_cast<ublox::UbxHnrMessages>(obs->msgId);
            if (!peek)
            {
                LOG_WARN("UBX:  HNR-{:x}, Size {}, not implemented yet!", msgId, obs->raw.getRawDataLength());
            }
        }
        // Information Messages: Printf-Style Messages, with IDs such as Error, Warning, Notice
        else if (obs->msgClass == ublox::UbxClass::UBX_CLASS_INF)
        {
            [[maybe_unused]] auto msgId = static_cast<ublox::UbxInfMessages>(obs->msgId);
            if (!peek)
            {
                LOG_WARN("UBX:  INF-{:x}, Size {}, not implemented yet!", msgId, obs->raw.getRawDataLength());
            }
        }
        // Logging Messages: Log creation, deletion, info and retrieval
        else if (obs->msgClass == ublox::UbxClass::UBX_CLASS_LOG)
        {
            [[maybe_unused]] auto msgId = static_cast<ublox::UbxLogMessages>(obs->msgId);
            if (!peek)
            {
                LOG_WARN("UBX:  LOG-{:x}, Size {}, not implemented yet!", msgId, obs->raw.getRawDataLength());
            }
        }
        // Multiple GNSS Assistance Messages: Assistance data for various GNSS
        else if (obs->msgClass == ublox::UbxClass::UBX_CLASS_MGA)
        {
            [[maybe_unused]] auto msgId = static_cast<ublox::UbxMgaMessages>(obs->msgId);
            if (!peek)
            {
                LOG_WARN("UBX:  MGA-{:x}, Size {}, not implemented yet!", msgId, obs->raw.getRawDataLength());
            }
        }
        // Monitoring Messages: Communication Status, CPU Load, Stack Usage, Task Status
        else if (obs->msgClass == ublox::UbxClass::UBX_CLASS_MON)
        {
            [[maybe_unused]] auto msgId = static_cast<ublox::UbxMonMessages>(obs->msgId);
            if (!peek)
            {
                LOG_WARN("UBX:  MON-{:x}, Size {}, not implemented yet!", msgId, obs->raw.getRawDataLength());
            }
        }
        // Navigation Results Messages: Position, Speed, Time, Acceleration, Heading, DOP, SVs used
        else if (obs->msgClass == ublox::UbxClass::UBX_CLASS_NAV)
        {
            auto msgId = static_cast<ublox::UbxNavMessages>(obs->msgId);
            if (msgId == ublox::UbxNavMessages::UBX_NAV_ATT)
            {
                obs->data = ublox::UbxNavAtt();

                std::get<ublox::UbxNavAtt>(obs->data).iTOW = obs->raw.extractUint32();
                std::get<ublox::UbxNavAtt>(obs->data).version = obs->raw.extractUint8();
                for (auto& reserved1 : std::get<ublox::UbxNavAtt>(obs->data).reserved1)
                {
                    reserved1 = obs->raw.extractUint8();
                }
                std::get<ublox::UbxNavAtt>(obs->data).roll = obs->raw.extractInt32();
                std::get<ublox::UbxNavAtt>(obs->data).pitch = obs->raw.extractInt32();
                std::get<ublox::UbxNavAtt>(obs->data).heading = obs->raw.extractInt32();
                std::get<ublox::UbxNavAtt>(obs->data).accRoll = obs->raw.extractUint32();
                std::get<ublox::UbxNavAtt>(obs->data).accPitch = obs->raw.extractUint32();
                std::get<ublox::UbxNavAtt>(obs->data).accHeading = obs->raw.extractUint32();

                // TODO: Calculate the insTime with the iTOW and get the week from somewhere

                if (!peek)
                {
                    LOG_DATA("UBX:  NAV-ATT, iTOW {}, roll {}, pitch {}, heading {}", std::get<ublox::UbxNavAtt>(obs->data).iTOW, std::get<ublox::UbxNavAtt>(obs->data).roll, std::get<ublox::UbxNavAtt>(obs->data).pitch, std::get<ublox::UbxNavAtt>(obs->data).heading);
                }
            }
            else if (msgId == ublox::UbxNavMessages::UBX_NAV_POSLLH)
            {
                obs->data = ublox::UbxNavPosllh();

                std::get<ublox::UbxNavPosllh>(obs->data).iTOW = obs->raw.extractUint32();
                std::get<ublox::UbxNavPosllh>(obs->data).lon = obs->raw.extractInt32();
                std::get<ublox::UbxNavPosllh>(obs->data).lat = obs->raw.extractInt32();
                std::get<ublox::UbxNavPosllh>(obs->data).height = obs->raw.extractInt32();
                std::get<ublox::UbxNavPosllh>(obs->data).hMSL = obs->raw.extractInt32();
                std::get<ublox::UbxNavPosllh>(obs->data).hAcc = obs->raw.extractUint32();
                std::get<ublox::UbxNavPosllh>(obs->data).vAcc = obs->raw.extractUint32();

                // TODO: Calculate the insTime with the iTOW and get the week from somewhere

                if (!peek)
                {
                    LOG_DATA("UBX:  NAV-POSLLH, iTOW {}, lon {}, lat {}, height {}", std::get<ublox::UbxNavPosllh>(obs->data).iTOW, std::get<ublox::UbxNavPosllh>(obs->data).lon, std::get<ublox::UbxNavPosllh>(obs->data).lat, std::get<ublox::UbxNavPosllh>(obs->data).height);
                }
            }
            else if (msgId == ublox::UbxNavMessages::UBX_NAV_VELNED)
            {
                obs->data = ublox::UbxNavVelned();

                std::get<ublox::UbxNavVelned>(obs->data).iTOW = obs->raw.extractUint32();
                std::get<ublox::UbxNavVelned>(obs->data).velN = obs->raw.extractInt32();
                std::get<ublox::UbxNavVelned>(obs->data).velE = obs->raw.extractInt32();
                std::get<ublox::UbxNavVelned>(obs->data).velD = obs->raw.extractInt32();
                std::get<ublox::UbxNavVelned>(obs->data).speed = obs->raw.extractUint32();
                std::get<ublox::UbxNavVelned>(obs->data).gSpeed = obs->raw.extractUint32();
                std::get<ublox::UbxNavVelned>(obs->data).heading = obs->raw.extractInt32();
                std::get<ublox::UbxNavVelned>(obs->data).sAcc = obs->raw.extractUint32();
                std::get<ublox::UbxNavVelned>(obs->data).cAcc = obs->raw.extractUint32();

                // TODO: Calculate the insTime with the iTOW and get the week from somewhere

                if (!peek)
                {
                    LOG_DATA("UBX:  NAV-VELNED, iTOW {}, gSpeed {} [cm/s], heading {} [deg*1e-5], velD {} [cm/s]", std::get<ublox::UbxNavVelned>(obs->data).iTOW, std::get<ublox::UbxNavVelned>(obs->data).gSpeed, std::get<ublox::UbxNavVelned>(obs->data).heading, std::get<ublox::UbxNavVelned>(obs->data).velD);
                }
            }
            else
            {
                if (!peek)
                {
                    LOG_WARN("UBX:  NAV-{:x}, Size {}, not implemented yet!", msgId, obs->raw.getRawDataLength());
                }
            }
        }
        // Receiver Manager Messages: Satellite Status, RTC Status
        else if (obs->msgClass == ublox::UbxClass::UBX_CLASS_RXM)
        {
            auto msgId = static_cast<ublox::UbxRxmMessages>(obs->msgId);
            if (msgId == ublox::UbxRxmMessages::UBX_RXM_RAWX)
            {
                obs->data = ublox::UbxRxmRawx();

                std::get<ublox::UbxRxmRawx>(obs->data).rcvTow = obs->raw.extractDouble();
                std::get<ublox::UbxRxmRawx>(obs->data).week = obs->raw.extractUint16();
                std::get<ublox::UbxRxmRawx>(obs->data).leapS = obs->raw.extractInt8();
                std::get<ublox::UbxRxmRawx>(obs->data).numMeas = obs->raw.extractUint8();
                std::get<ublox::UbxRxmRawx>(obs->data).recStat = obs->raw.extractUint8();
                for (auto& reserved1 : std::get<ublox::UbxRxmRawx>(obs->data).reserved1)
                {
                    reserved1 = obs->raw.extractUint8();
                }
                for (size_t i = 0; i < std::get<ublox::UbxRxmRawx>(obs->data).numMeas; i++)
                {
                    std::get<ublox::UbxRxmRawx>(obs->data).data.emplace_back(obs->raw.extractDouble(), // prMes
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

                obs->insTime.emplace(std::get<ublox::UbxRxmRawx>(obs->data).week, static_cast<long double>(std::get<ublox::UbxRxmRawx>(obs->data).rcvTow), 0);

                if (!peek)
                {
                    LOG_DATA("UBX:  RXM-RAWX, Size {}, rcvTow {}, numMeas {}",
                             obs->raw.getRawDataLength(), std::get<ublox::UbxRxmRawx>(obs->data).rcvTow, std::get<ublox::UbxRxmRawx>(obs->data).numMeas);
                }
            }
            else if (msgId == ublox::UbxRxmMessages::UBX_RXM_SFRBX)
            {
                obs->data = ublox::UbxRxmSfrbx();

                std::get<ublox::UbxRxmSfrbx>(obs->data).gnssId = obs->raw.extractUint8();
                std::get<ublox::UbxRxmSfrbx>(obs->data).svId = obs->raw.extractUint8();
                std::get<ublox::UbxRxmSfrbx>(obs->data).reserved1 = obs->raw.extractUint8();
                std::get<ublox::UbxRxmSfrbx>(obs->data).freqId = obs->raw.extractUint8();
                std::get<ublox::UbxRxmSfrbx>(obs->data).numWords = obs->raw.extractUint8();
                std::get<ublox::UbxRxmSfrbx>(obs->data).chn = obs->raw.extractUint8();
                std::get<ublox::UbxRxmSfrbx>(obs->data).version = obs->raw.extractUint8();
                std::get<ublox::UbxRxmSfrbx>(obs->data).reserved2 = obs->raw.extractUint8();

                for (size_t i = 0; i < std::get<ublox::UbxRxmSfrbx>(obs->data).numWords; i++)
                {
                    std::get<ublox::UbxRxmSfrbx>(obs->data).dwrd.emplace_back(obs->raw.extractUint32());
                }

                // TODO: Calculate the insTime somehow

                if (!peek)
                {
                    LOG_DATA("UBX:  RXM-SFRBX, gnssId {}, svId {}, freqId {}, numWords {}",
                             std::get<ublox::UbxRxmSfrbx>(obs->data).gnssId, std::get<ublox::UbxRxmSfrbx>(obs->data).svId, std::get<ublox::UbxRxmSfrbx>(obs->data).freqId, std::get<ublox::UbxRxmSfrbx>(obs->data).numWords);
                }
            }
            else
            {
                if (!peek)
                {
                    LOG_WARN("UBX:  RXM-{:x}, Size {}, not implemented yet!", msgId, obs->raw.getRawDataLength());
                }
            }
        }
        // Security Feature Messages
        else if (obs->msgClass == ublox::UbxClass::UBX_CLASS_SEC)
        {
            [[maybe_unused]] auto msgId = static_cast<ublox::UbxSecMessages>(obs->msgId);
            if (!peek)
            {
                LOG_WARN("UBX:  SEC-{:x}, Size {}, not implemented yet!", msgId, obs->raw.getRawDataLength());
            }
        }
        // Timing Messages: Time Pulse Output, Time Mark Results
        else if (obs->msgClass == ublox::UbxClass::UBX_CLASS_TIM)
        {
            [[maybe_unused]] auto msgId = static_cast<ublox::UbxTimMessages>(obs->msgId);
            if (!peek)
            {
                LOG_WARN("UBX:  TIM-{:x}, Size {}, not implemented yet!", msgId, obs->raw.getRawDataLength());
            }
        }
        // Firmware Update Messages: Memory/Flash erase/write, Reboot, Flash identification, etc.
        else if (obs->msgClass == ublox::UbxClass::UBX_CLASS_UPD)
        {
            [[maybe_unused]] auto msgId = static_cast<ublox::UbxUpdMessages>(obs->msgId);
            if (!peek)
            {
                LOG_WARN("UBX:  UPD-{:x}, Size {}, not implemented yet!", msgId, obs->raw.getRawDataLength());
            }
        }
        else
        {
            if (!peek)
            {
                LOG_WARN("UBX:  {:x}-{:x}, Size {}, not implemented yet!", obs->msgClass, obs->msgId, obs->raw.getRawDataLength());
            }
        }
    }
    else if (obs->raw.type() == ublox::UbloxPacket::Type::TYPE_ASCII)
    {
        if (!peek)
        {
            LOG_DATA("NMEA: {}", obs->raw.datastr().substr(0, obs->raw.datastr().size() - 2));
        }
    }
}