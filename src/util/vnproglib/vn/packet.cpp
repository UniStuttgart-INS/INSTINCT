// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "packet.h"

#ifndef HAS_VECTORNAV_LIBRARY

    #include <cstdio>
    #include <cstdlib>
    #include <cstring>
    #include <string>
    #include <queue>
    #include <list>

// NOLINTBEGIN

namespace vn
{
namespace protocol
{
namespace uart
{

std::string Packet::datastr()
{
    return {};
}

Packet::Type Packet::type()
{
    return TYPE_UNKNOWN;
}

bool Packet::isValid()
{
    return false;
}

bool Packet::isError()
{
    return false;
}

bool Packet::isResponse()
{
    return false;
}

bool Packet::isAsciiAsync()
{
    return false;
}

AsciiAsync Packet::determineAsciiAsyncType()
{
    return AsciiAsync::VNOFF;
}

bool Packet::isCompatible(CommonGroup /* commonGroup */, TimeGroup /* timeGroup */, ImuGroup /* imuGroup */,
                          GpsGroup /* gpsGroup */, AttitudeGroup /* attitudeGroup */, InsGroup /* insGroup */, GpsGroup /* gps2Group */)
{
    return false;
}

void Packet::ensureCanExtract(size_t /* numOfBytes */)
{
}

uint8_t Packet::extractUint8()
{
    return {};
}

int8_t Packet::extractInt8()
{
    return {};
}

uint16_t Packet::extractUint16()
{
    return {};
}

int8_t Packet::extractInt16()
{
    return {};
}

uint32_t Packet::extractUint32()
{
    return {};
}

uint64_t Packet::extractUint64()
{
    return {};
}

float Packet::extractFloat()
{
    return {};
}

double Packet::extractDouble()
{
    return {};
}

vn::math::vec3f Packet::extractVec3f()
{
    return {};
}

vn::math::vec3d Packet::extractVec3d()
{
    return {};
}

vn::math::vec4f Packet::extractVec4f()
{
    return {};
}

vn::math::mat3f Packet::extractMat3f()
{
    return {};
}

size_t Packet::getPacketLength()
{
    return 0;
}

size_t Packet::getCurExtractLoc()
{
    return 0;
}

} // namespace uart
} // namespace protocol
} // namespace vn

// NOLINTEND

#endif