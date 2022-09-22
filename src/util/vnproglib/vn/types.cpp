// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "types.h"

#ifndef HAS_VECTORNAV_LIBRARY

// NOLINTBEGIN

namespace vn
{
namespace protocol
{
namespace uart
{

VpeStatus::VpeStatus()
{
}

VpeStatus::VpeStatus(uint16_t raw)
{
    attitudeQuality = 0x0003 & raw;
    gyroSaturation = (0x0004 & raw) != 0;
    gyroSaturationRecovery = (0x0008 & raw) != 0;
    magDisturbance = (0x0030 & raw) >> 4;
    magSaturation = (0x0040 & raw) != 0;
    accDisturbance = (0x0180 & raw) >> 7;
    accSaturation = (0x0200 & raw) != 0;
    knownMagDisturbance = (0x0800 & raw) != 0;
    knownAccelDisturbance = (0x1000 & raw) != 0;
}

CommonGroup& operator|=(CommonGroup& lhs, const CommonGroup& rhs)
{
    lhs = lhs | rhs;
    return lhs;
}

TimeGroup& operator|=(TimeGroup& lhs, const TimeGroup& rhs)
{
    lhs = lhs | rhs;
    return lhs;
}

ImuGroup& operator|=(ImuGroup& lhs, const ImuGroup& rhs)
{
    lhs = lhs | rhs;
    return lhs;
}

GpsGroup& operator|=(GpsGroup& lhs, const GpsGroup& rhs)
{
    lhs = lhs | rhs;
    return lhs;
}

AttitudeGroup& operator|=(AttitudeGroup& lhs, const AttitudeGroup& rhs)
{
    lhs = lhs | rhs;
    return lhs;
}

InsGroup& operator|=(InsGroup& lhs, const InsGroup& rhs)
{
    lhs = lhs | rhs;
    return lhs;
}

CommonGroup operator|(CommonGroup lhs, CommonGroup rhs)
{
    return CommonGroup(int(lhs) | int(rhs));
}

TimeGroup operator|(TimeGroup lhs, TimeGroup rhs)
{
    return TimeGroup(int(lhs) | int(rhs));
}

ImuGroup operator|(ImuGroup lhs, ImuGroup rhs)
{
    return ImuGroup(int(lhs) | int(rhs));
}

GpsGroup operator|(GpsGroup lhs, GpsGroup rhs)
{
    return GpsGroup(int(lhs) | int(rhs));
}

AttitudeGroup operator|(AttitudeGroup lhs, AttitudeGroup rhs)
{
    return AttitudeGroup(int(lhs) | int(rhs));
}

InsGroup operator|(InsGroup lhs, InsGroup rhs)
{
    return InsGroup(int(lhs) | int(rhs));
}

CommonGroup& operator&=(CommonGroup& lhs, const CommonGroup& rhs)
{
    lhs = lhs & rhs;
    return lhs;
}

TimeGroup& operator&=(TimeGroup& lhs, const TimeGroup& rhs)
{
    lhs = lhs & rhs;
    return lhs;
}

ImuGroup& operator&=(ImuGroup& lhs, const ImuGroup& rhs)
{
    lhs = lhs & rhs;
    return lhs;
}

GpsGroup& operator&=(GpsGroup& lhs, const GpsGroup& rhs)
{
    lhs = lhs & rhs;
    return lhs;
}

AttitudeGroup& operator&=(AttitudeGroup& lhs, const AttitudeGroup& rhs)
{
    lhs = lhs & rhs;
    return lhs;
}

InsGroup& operator&=(InsGroup& lhs, const InsGroup& rhs)
{
    lhs = lhs & rhs;
    return lhs;
}

CommonGroup operator&(CommonGroup lhs, CommonGroup rhs)
{
    return CommonGroup(int(lhs) & int(rhs));
}

TimeGroup operator&(TimeGroup lhs, TimeGroup rhs)
{
    return TimeGroup(int(lhs) & int(rhs));
}

ImuGroup operator&(ImuGroup lhs, ImuGroup rhs)
{
    return ImuGroup(int(lhs) & int(rhs));
}

GpsGroup operator&(GpsGroup lhs, GpsGroup rhs)
{
    return GpsGroup(int(lhs) & int(rhs));
}

AttitudeGroup operator&(AttitudeGroup lhs, AttitudeGroup rhs)
{
    return AttitudeGroup(int(lhs) & int(rhs));
}

InsGroup operator&(InsGroup lhs, InsGroup rhs)
{
    return InsGroup(int(lhs) & int(rhs));
}

CommonGroup operator~(CommonGroup rhs)
{
    return CommonGroup(~int(rhs));
}

TimeGroup operator~(TimeGroup rhs)
{
    return TimeGroup(~int(rhs));
}

ImuGroup operator~(ImuGroup rhs)
{
    return ImuGroup(~int(rhs));
}

GpsGroup operator~(GpsGroup rhs)
{
    return GpsGroup(~int(rhs));
}

AttitudeGroup operator~(AttitudeGroup rhs)
{
    return AttitudeGroup(~int(rhs));
}

InsGroup operator~(InsGroup rhs)
{
    return InsGroup(~int(rhs));
}

} // namespace uart
} // namespace protocol
} // namespace vn

// NOLINTEND

#endif