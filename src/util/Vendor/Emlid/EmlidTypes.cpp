// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "EmlidTypes.hpp"

NAV::vendor::emlid::ErbMessageID NAV::vendor::emlid::getMsgIdFromString(const std::string& idName)
{
    if (idName == "VER")
    {
        return ErbMessageID::ERB_MessageId_VER;
    }
    if (idName == "POS")
    {
        return ErbMessageID::ERB_MessageId_POS;
    }
    if (idName == "STAT")
    {
        return ErbMessageID::ERB_MessageId_STAT;
    }
    if (idName == "DPOS")
    {
        return ErbMessageID::ERB_MessageId_DPOS;
    }
    if (idName == "VEL")
    {
        return ErbMessageID::ERB_MessageId_VEL;
    }
    if (idName == "SVI")
    {
        return ErbMessageID::ERB_MessageId_SVI;
    }
    if (idName == "RTK")
    {
        return ErbMessageID::ERB_MessageId_RTK;
    }

    return ErbMessageID::ERB_MessageId_NONE;
}
