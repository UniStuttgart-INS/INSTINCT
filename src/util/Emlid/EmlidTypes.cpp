#include "EmlidTypes.hpp"

uint8_t NAV::Emlid::getMsgIdFromString(const std::string_view& idName)
{
    if (idName == "VER")
    {
        return ErbMessageID::ERB_Message_VER;
    }
    if (idName == "POS")
    {
        return ErbMessageID::ERB_Message_POS;
    }
    if (idName == "STAT")
    {
        return ErbMessageID::ERB_Message_STAT;
    }
    if (idName == "DPOS")
    {
        return ErbMessageID::ERB_Message_DPOS;
    }
    if (idName == "VEL")
    {
        return ErbMessageID::ERB_Message_VEL;
    }
    if (idName == "SVI")
    {
        return ErbMessageID::ERB_Message_SVI;
    }
    if (idName == "RTK")
    {
        return ErbMessageID::ERB_Message_RTK;
    }

    return static_cast<ErbMessageID>(NULL);
}
