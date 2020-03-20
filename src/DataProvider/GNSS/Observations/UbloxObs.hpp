/**
 * @file UbloxObs.hpp
 * @brief ublox Observation Class
 * @author T. Topp (thomas.topp@nav.uni-stuttgart.de)
 * @date 2020-03-19
 */

#pragma once

#include "GnssObs.hpp"

#include "ub/protocol/types.hpp"
#include "ub/protocol/packet.hpp"

namespace NAV
{
/// ublox Observation Class
class UbloxObs : public GnssObs
{
  public:
    ub::protocol::uart::UbxClass msgClass;
    uint8_t msgId;
    uint16_t payloadLength;

    ub::protocol::uart::Packet* p;
};

} // namespace NAV
