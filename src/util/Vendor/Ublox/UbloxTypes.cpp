// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "UbloxTypes.hpp"

NAV::vendor::ublox::UbxClass NAV::vendor::ublox::getMsgClassFromString(const std::string& className)
{
    if (className == "NAV") { return UbxClass::UBX_CLASS_NAV; }
    if (className == "RXM") { return UbxClass::UBX_CLASS_RXM; }
    if (className == "INF") { return UbxClass::UBX_CLASS_INF; }
    if (className == "ACK") { return UbxClass::UBX_CLASS_ACK; }
    if (className == "CFG") { return UbxClass::UBX_CLASS_CFG; }
    if (className == "UPD") { return UbxClass::UBX_CLASS_UPD; }
    if (className == "MON") { return UbxClass::UBX_CLASS_MON; }
    if (className == "TIM") { return UbxClass::UBX_CLASS_TIM; }
    if (className == "ESF") { return UbxClass::UBX_CLASS_ESF; }
    if (className == "MGA") { return UbxClass::UBX_CLASS_MGA; }
    if (className == "LOG") { return UbxClass::UBX_CLASS_LOG; }
    if (className == "SEC") { return UbxClass::UBX_CLASS_SEC; }
    if (className == "HNR") { return UbxClass::UBX_CLASS_HNR; }

    return static_cast<UbxClass>(NULL);
}

uint8_t NAV::vendor::ublox::getMsgIdFromString(NAV::vendor::ublox::UbxClass msgClass, const std::string& idName)
{
    if (msgClass == UbxClass::UBX_CLASS_ACK)
    {
        if (idName == "ACK") { return UbxAckMessages::UBX_ACK_ACK; }
        if (idName == "NAK") { return UbxAckMessages::UBX_ACK_NAK; }
    }
    else if (msgClass == UbxClass::UBX_CLASS_CFG)
    {
        if (idName == "ANT") { return UbxCfgMessages::UBX_CFG_ANT; }
        if (idName == "BATCH") { return UbxCfgMessages::UBX_CFG_BATCH; }
        if (idName == "CFG") { return UbxCfgMessages::UBX_CFG_CFG; }
        if (idName == "DAT") { return UbxCfgMessages::UBX_CFG_DAT; }
        if (idName == "DGNSS") { return UbxCfgMessages::UBX_CFG_DGNSS; }
        if (idName == "DOSC") { return UbxCfgMessages::UBX_CFG_DOSC; }
        if (idName == "ESRC") { return UbxCfgMessages::UBX_CFG_ESRC; }
        if (idName == "GEOFENCE") { return UbxCfgMessages::UBX_CFG_GEOFENCE; }
        if (idName == "GNSS") { return UbxCfgMessages::UBX_CFG_GNSS; }
        if (idName == "HNR") { return UbxCfgMessages::UBX_CFG_HNR; }
        if (idName == "INF") { return UbxCfgMessages::UBX_CFG_INF; }
        if (idName == "ITFM") { return UbxCfgMessages::UBX_CFG_ITFM; }
        if (idName == "LOGFILTER") { return UbxCfgMessages::UBX_CFG_LOGFILTER; }
        if (idName == "MSG") { return UbxCfgMessages::UBX_CFG_MSG; }
        if (idName == "NAV5") { return UbxCfgMessages::UBX_CFG_NAV5; }
        if (idName == "NAVX5") { return UbxCfgMessages::UBX_CFG_NAVX5; }
        if (idName == "NMEA") { return UbxCfgMessages::UBX_CFG_NMEA; }
        if (idName == "ODO") { return UbxCfgMessages::UBX_CFG_ODO; }
        if (idName == "PM2") { return UbxCfgMessages::UBX_CFG_PM2; }
        if (idName == "PMS") { return UbxCfgMessages::UBX_CFG_PMS; }
        if (idName == "PRT") { return UbxCfgMessages::UBX_CFG_PRT; }
        if (idName == "PWR") { return UbxCfgMessages::UBX_CFG_PWR; }
        if (idName == "RATE") { return UbxCfgMessages::UBX_CFG_RATE; }
        if (idName == "RINV") { return UbxCfgMessages::UBX_CFG_RINV; }
        if (idName == "RST") { return UbxCfgMessages::UBX_CFG_RST; }
        if (idName == "RXM") { return UbxCfgMessages::UBX_CFG_RXM; }
        if (idName == "SBAS") { return UbxCfgMessages::UBX_CFG_SBAS; }
        if (idName == "SLAS") { return UbxCfgMessages::UBX_CFG_SLAS; }
        if (idName == "SMGR") { return UbxCfgMessages::UBX_CFG_SMGR; }
        if (idName == "TMODE2") { return UbxCfgMessages::UBX_CFG_TMODE2; }
        if (idName == "TMODE3") { return UbxCfgMessages::UBX_CFG_TMODE3; }
        if (idName == "TP5") { return UbxCfgMessages::UBX_CFG_TP5; }
        if (idName == "TXSLOT") { return UbxCfgMessages::UBX_CFG_TXSLOT; }
        if (idName == "USB") { return UbxCfgMessages::UBX_CFG_USB; }
    }
    else if (msgClass == UbxClass::UBX_CLASS_ESF)
    {
        if (idName == "INS") { return UbxEsfMessages::UBX_ESF_INS; }
        if (idName == "MEAS") { return UbxEsfMessages::UBX_ESF_MEAS; }
        if (idName == "RAW") { return UbxEsfMessages::UBX_ESF_RAW; }
        if (idName == "STATUS") { return UbxEsfMessages::UBX_ESF_STATUS; }
    }
    else if (msgClass == UbxClass::UBX_CLASS_HNR)
    {
        if (idName == "INS") { return UbxHnrMessages::UBX_HNR_INS; }
        if (idName == "PVT") { return UbxHnrMessages::UBX_HNR_PVT; }
    }
    else if (msgClass == UbxClass::UBX_CLASS_INF)
    {
        if (idName == "DEBUG") { return UbxInfMessages::UBX_INF_DEBUG; }
        if (idName == "ERROR") { return UbxInfMessages::UBX_INF_ERROR; }
        if (idName == "NOTICE") { return UbxInfMessages::UBX_INF_NOTICE; }
        if (idName == "TEST") { return UbxInfMessages::UBX_INF_TEST; }
        if (idName == "WARNING") { return UbxInfMessages::UBX_INF_WARNING; }
    }
    else if (msgClass == UbxClass::UBX_CLASS_LOG)
    {
        if (idName == "BATCH") { return UbxLogMessages::UBX_LOG_BATCH; }
        if (idName == "CREATE") { return UbxLogMessages::UBX_LOG_CREATE; }
        if (idName == "ERASE") { return UbxLogMessages::UBX_LOG_ERASE; }
        if (idName == "FINDTIME") { return UbxLogMessages::UBX_LOG_FINDTIME; }
        if (idName == "INFO") { return UbxLogMessages::UBX_LOG_INFO; }
        if (idName == "RETRIEVE") { return UbxLogMessages::UBX_LOG_RETRIEVE; }
        if (idName == "RETRIEVEBATCH") { return UbxLogMessages::UBX_LOG_RETRIEVEBATCH; }
        if (idName == "RETRIEVEPOS") { return UbxLogMessages::UBX_LOG_RETRIEVEPOS; }
        if (idName == "RETRIEVEPOSEXTRA") { return UbxLogMessages::UBX_LOG_RETRIEVEPOSEXTRA; }
        if (idName == "RETRIEVESTRING") { return UbxLogMessages::UBX_LOG_RETRIEVESTRING; }
        if (idName == "STRING") { return UbxLogMessages::UBX_LOG_STRING; }
    }
    else if (msgClass == UbxClass::UBX_CLASS_MGA)
    {
        if (idName == "ACK_DATA0") { return UbxMgaMessages::UBX_MGA_ACK_DATA0; }
        if (idName == "ANO") { return UbxMgaMessages::UBX_MGA_ANO; }
        if (idName == "BDS_ALM") { return UbxMgaMessages::UBX_MGA_BDS_ALM; }
        if (idName == "BDS_EPH") { return UbxMgaMessages::UBX_MGA_BDS_EPH; }
        if (idName == "BDS_HEALTH") { return UbxMgaMessages::UBX_MGA_BDS_HEALTH; }
        if (idName == "BDS_IONO") { return UbxMgaMessages::UBX_MGA_BDS_IONO; }
        if (idName == "BDS_UTC") { return UbxMgaMessages::UBX_MGA_BDS_UTC; }
        if (idName == "DBD") { return UbxMgaMessages::UBX_MGA_DBD; }
        if (idName == "FLASH_ACK") { return UbxMgaMessages::UBX_MGA_FLASH_ACK; }
        if (idName == "FLASH_DATA") { return UbxMgaMessages::UBX_MGA_FLASH_DATA; }
        if (idName == "FLASH_STOP") { return UbxMgaMessages::UBX_MGA_FLASH_STOP; }
        if (idName == "GAL_ALM") { return UbxMgaMessages::UBX_MGA_GAL_ALM; }
        if (idName == "GAL_EPH") { return UbxMgaMessages::UBX_MGA_GAL_EPH; }
        if (idName == "GAL_TIMEOFFSET") { return UbxMgaMessages::UBX_MGA_GAL_TIMEOFFSET; }
        if (idName == "GAL_UTC") { return UbxMgaMessages::UBX_MGA_GAL_UTC; }
        if (idName == "GLO_ALM") { return UbxMgaMessages::UBX_MGA_GLO_ALM; }
        if (idName == "GLO_EPH") { return UbxMgaMessages::UBX_MGA_GLO_EPH; }
        if (idName == "GLO_TIMEOFFSET") { return UbxMgaMessages::UBX_MGA_GLO_TIMEOFFSET; }
        if (idName == "GPS_ALM") { return UbxMgaMessages::UBX_MGA_GPS_ALM; }
        if (idName == "GPS_EPH") { return UbxMgaMessages::UBX_MGA_GPS_EPH; }
        if (idName == "GPS_HEALTH") { return UbxMgaMessages::UBX_MGA_GPS_HEALTH; }
        if (idName == "GPS_IONO") { return UbxMgaMessages::UBX_MGA_GPS_IONO; }
        if (idName == "GPS_UTC") { return UbxMgaMessages::UBX_MGA_GPS_UTC; }
        if (idName == "INI_CLKD") { return UbxMgaMessages::UBX_MGA_INI_CLKD; }
        if (idName == "INI_EOP") { return UbxMgaMessages::UBX_MGA_INI_EOP; }
        if (idName == "INI_FREQ") { return UbxMgaMessages::UBX_MGA_INI_FREQ; }
        if (idName == "INI_POS_LLH") { return UbxMgaMessages::UBX_MGA_INI_POS_LLH; }
        if (idName == "INI_POS_XYZ") { return UbxMgaMessages::UBX_MGA_INI_POS_XYZ; }
        if (idName == "INI_TIME_GNSS") { return UbxMgaMessages::UBX_MGA_INI_TIME_GNSS; }
        if (idName == "INI_TIME_UTC") { return UbxMgaMessages::UBX_MGA_INI_TIME_UTC; }
        if (idName == "QZSS_ALM") { return UbxMgaMessages::UBX_MGA_QZSS_ALM; }
        if (idName == "QZSS_EPH") { return UbxMgaMessages::UBX_MGA_QZSS_EPH; }
        if (idName == "QZSS_HEALTH") { return UbxMgaMessages::UBX_MGA_QZSS_HEALTH; }
    }
    else if (msgClass == UbxClass::UBX_CLASS_MON)
    {
        if (idName == "BATCH") { return UbxMonMessages::UBX_MON_BATCH; }
        if (idName == "GNSS") { return UbxMonMessages::UBX_MON_GNSS; }
        if (idName == "HW2") { return UbxMonMessages::UBX_MON_HW2; }
        if (idName == "HW") { return UbxMonMessages::UBX_MON_HW; }
        if (idName == "IO") { return UbxMonMessages::UBX_MON_IO; }
        if (idName == "MSGPP") { return UbxMonMessages::UBX_MON_MSGPP; }
        if (idName == "PATCH") { return UbxMonMessages::UBX_MON_PATCH; }
        if (idName == "RXBUFF") { return UbxMonMessages::UBX_MON_RXBUFF; }
        if (idName == "RXR") { return UbxMonMessages::UBX_MON_RXR; }
        if (idName == "SMGR") { return UbxMonMessages::UBX_MON_SMGR; }
        if (idName == "TXBUFF") { return UbxMonMessages::UBX_MON_TXBUFF; }
        if (idName == "VER") { return UbxMonMessages::UBX_MON_VER; }
    }
    else if (msgClass == UbxClass::UBX_CLASS_NAV)
    {
        if (idName == "AOPSTATUS") { return UbxNavMessages::UBX_NAV_AOPSTATUS; }
        if (idName == "ATT") { return UbxNavMessages::UBX_NAV_ATT; }
        if (idName == "CLOCK") { return UbxNavMessages::UBX_NAV_CLOCK; }
        if (idName == "DGPS") { return UbxNavMessages::UBX_NAV_DGPS; }
        if (idName == "DOP") { return UbxNavMessages::UBX_NAV_DOP; }
        if (idName == "EOE") { return UbxNavMessages::UBX_NAV_EOE; }
        if (idName == "GEOFENCE") { return UbxNavMessages::UBX_NAV_GEOFENCE; }
        if (idName == "HPPOSECEF") { return UbxNavMessages::UBX_NAV_HPPOSECEF; }
        if (idName == "HPPOSLLH") { return UbxNavMessages::UBX_NAV_HPPOSLLH; }
        if (idName == "ODO") { return UbxNavMessages::UBX_NAV_ODO; }
        if (idName == "ORB") { return UbxNavMessages::UBX_NAV_ORB; }
        if (idName == "POSECEF") { return UbxNavMessages::UBX_NAV_POSECEF; }
        if (idName == "POSLLH") { return UbxNavMessages::UBX_NAV_POSLLH; }
        if (idName == "PVT") { return UbxNavMessages::UBX_NAV_PVT; }
        if (idName == "RELPOSNED") { return UbxNavMessages::UBX_NAV_RELPOSNED; }
        if (idName == "RESETODO") { return UbxNavMessages::UBX_NAV_RESETODO; }
        if (idName == "SAT") { return UbxNavMessages::UBX_NAV_SAT; }
        if (idName == "SBAS") { return UbxNavMessages::UBX_NAV_SBAS; }
        if (idName == "SLAS") { return UbxNavMessages::UBX_NAV_SLAS; }
        if (idName == "SOL") { return UbxNavMessages::UBX_NAV_SOL; }
        if (idName == "STATUS") { return UbxNavMessages::UBX_NAV_STATUS; }
        if (idName == "SVIN") { return UbxNavMessages::UBX_NAV_SVIN; }
        if (idName == "SVINFO") { return UbxNavMessages::UBX_NAV_SVINFO; }
        if (idName == "TIMEBDS") { return UbxNavMessages::UBX_NAV_TIMEBDS; }
        if (idName == "TIMEGAL") { return UbxNavMessages::UBX_NAV_TIMEGAL; }
        if (idName == "TIMEGLO") { return UbxNavMessages::UBX_NAV_TIMEGLO; }
        if (idName == "TIMEGPS") { return UbxNavMessages::UBX_NAV_TIMEGPS; }
        if (idName == "TIMELS") { return UbxNavMessages::UBX_NAV_TIMELS; }
        if (idName == "TIMEUTC") { return UbxNavMessages::UBX_NAV_TIMEUTC; }
        if (idName == "VELECEF") { return UbxNavMessages::UBX_NAV_VELECEF; }
        if (idName == "VELNED") { return UbxNavMessages::UBX_NAV_VELNED; }
    }
    else if (msgClass == UbxClass::UBX_CLASS_RXM)
    {
        if (idName == "IMES") { return UbxRxmMessages::UBX_RXM_IMES; }
        if (idName == "MEASX") { return UbxRxmMessages::UBX_RXM_MEASX; }
        if (idName == "PMREQ") { return UbxRxmMessages::UBX_RXM_PMREQ; }
        if (idName == "RAWX") { return UbxRxmMessages::UBX_RXM_RAWX; }
        if (idName == "RLM") { return UbxRxmMessages::UBX_RXM_RLM; }
        if (idName == "RTCM") { return UbxRxmMessages::UBX_RXM_RTCM; }
        if (idName == "SFRBX") { return UbxRxmMessages::UBX_RXM_SFRBX; }
        if (idName == "SVSI") { return UbxRxmMessages::UBX_RXM_SVSI; }
    }
    else if (msgClass == UbxClass::UBX_CLASS_SEC)
    {
        if (idName == "UNIQID") { return UbxSecMessages::UBX_SEC_UNIQID; }
    }
    else if (msgClass == UbxClass::UBX_CLASS_TIM)
    {
        if (idName == "FCHG") { return UbxTimMessages::UBX_TIM_FCHG; }
        if (idName == "HOC") { return UbxTimMessages::UBX_TIM_HOC; }
        if (idName == "SMEAS") { return UbxTimMessages::UBX_TIM_SMEAS; }
        if (idName == "SVIN") { return UbxTimMessages::UBX_TIM_SVIN; }
        if (idName == "TM2") { return UbxTimMessages::UBX_TIM_TM2; }
        if (idName == "TOS") { return UbxTimMessages::UBX_TIM_TOS; }
        if (idName == "TP") { return UbxTimMessages::UBX_TIM_TP; }
        if (idName == "VCOCAL") { return UbxTimMessages::UBX_TIM_VCOCAL; }
        if (idName == "VRFY") { return UbxTimMessages::UBX_TIM_VRFY; }
    }
    else if (msgClass == UbxClass::UBX_CLASS_UPD)
    {
        if (idName == "SOS") { return UbxUpdMessages::UBX_UPD_SOS; }
    }

    return UINT8_MAX;
}

std::string NAV::vendor::ublox::getStringFromMsgClass(UbxClass msgClass)
{
    switch (msgClass)
    {
    case NAV::vendor::ublox::UBX_CLASS_NONE:
        return "NONE";
    case NAV::vendor::ublox::UBX_CLASS_NAV:
        return "NAV";
    case NAV::vendor::ublox::UBX_CLASS_RXM:
        return "RXM";
    case NAV::vendor::ublox::UBX_CLASS_INF:
        return "INF";
    case NAV::vendor::ublox::UBX_CLASS_ACK:
        return "ACK";
    case NAV::vendor::ublox::UBX_CLASS_CFG:
        return "CFG";
    case NAV::vendor::ublox::UBX_CLASS_UPD:
        return "UPD";
    case NAV::vendor::ublox::UBX_CLASS_MON:
        return "MON";
    case NAV::vendor::ublox::UBX_CLASS_TIM:
        return "TIM";
    case NAV::vendor::ublox::UBX_CLASS_ESF:
        return "ESF";
    case NAV::vendor::ublox::UBX_CLASS_MGA:
        return "MGA";
    case NAV::vendor::ublox::UBX_CLASS_LOG:
        return "LOG";
    case NAV::vendor::ublox::UBX_CLASS_SEC:
        return "SEC";
    case NAV::vendor::ublox::UBX_CLASS_HNR:
        return "HNR";
    default:
        break;
    }
    return "UNDEFINED";
}
std::string NAV::vendor::ublox::getStringFromMsgId(UbxClass msgClass, uint8_t msgId)
{
    if (msgClass == UbxClass::UBX_CLASS_ACK)
    {
        if (msgId == UbxAckMessages::UBX_ACK_ACK) { return "ACK"; }
        if (msgId == UbxAckMessages::UBX_ACK_NAK) { return "NAK"; }
    }
    else if (msgClass == UbxClass::UBX_CLASS_CFG)
    {
        if (msgId == UbxCfgMessages::UBX_CFG_ANT) { return "ANT"; }
        if (msgId == UbxCfgMessages::UBX_CFG_BATCH) { return "BATCH"; }
        if (msgId == UbxCfgMessages::UBX_CFG_CFG) { return "CFG"; }
        if (msgId == UbxCfgMessages::UBX_CFG_DAT) { return "DAT"; }
        if (msgId == UbxCfgMessages::UBX_CFG_DGNSS) { return "DGNSS"; }
        if (msgId == UbxCfgMessages::UBX_CFG_DOSC) { return "DOSC"; }
        if (msgId == UbxCfgMessages::UBX_CFG_ESRC) { return "ESRC"; }
        if (msgId == UbxCfgMessages::UBX_CFG_GEOFENCE) { return "GEOFENCE"; }
        if (msgId == UbxCfgMessages::UBX_CFG_GNSS) { return "GNSS"; }
        if (msgId == UbxCfgMessages::UBX_CFG_HNR) { return "HNR"; }
        if (msgId == UbxCfgMessages::UBX_CFG_INF) { return "INF"; }
        if (msgId == UbxCfgMessages::UBX_CFG_ITFM) { return "ITFM"; }
        if (msgId == UbxCfgMessages::UBX_CFG_LOGFILTER) { return "LOGFILTER"; }
        if (msgId == UbxCfgMessages::UBX_CFG_MSG) { return "MSG"; }
        if (msgId == UbxCfgMessages::UBX_CFG_NAV5) { return "NAV5"; }
        if (msgId == UbxCfgMessages::UBX_CFG_NAVX5) { return "NAVX5"; }
        if (msgId == UbxCfgMessages::UBX_CFG_NMEA) { return "NMEA"; }
        if (msgId == UbxCfgMessages::UBX_CFG_ODO) { return "ODO"; }
        if (msgId == UbxCfgMessages::UBX_CFG_PM2) { return "PM2"; }
        if (msgId == UbxCfgMessages::UBX_CFG_PMS) { return "PMS"; }
        if (msgId == UbxCfgMessages::UBX_CFG_PRT) { return "PRT"; }
        if (msgId == UbxCfgMessages::UBX_CFG_PWR) { return "PWR"; }
        if (msgId == UbxCfgMessages::UBX_CFG_RATE) { return "RATE"; }
        if (msgId == UbxCfgMessages::UBX_CFG_RINV) { return "RINV"; }
        if (msgId == UbxCfgMessages::UBX_CFG_RST) { return "RST"; }
        if (msgId == UbxCfgMessages::UBX_CFG_RXM) { return "RXM"; }
        if (msgId == UbxCfgMessages::UBX_CFG_SBAS) { return "SBAS"; }
        if (msgId == UbxCfgMessages::UBX_CFG_SLAS) { return "SLAS"; }
        if (msgId == UbxCfgMessages::UBX_CFG_SMGR) { return "SMGR"; }
        if (msgId == UbxCfgMessages::UBX_CFG_TMODE2) { return "TMODE2"; }
        if (msgId == UbxCfgMessages::UBX_CFG_TMODE3) { return "TMODE3"; }
        if (msgId == UbxCfgMessages::UBX_CFG_TP5) { return "TP5"; }
        if (msgId == UbxCfgMessages::UBX_CFG_TXSLOT) { return "TXSLOT"; }
        if (msgId == UbxCfgMessages::UBX_CFG_USB) { return "USB"; }
    }
    else if (msgClass == UbxClass::UBX_CLASS_ESF)
    {
        if (msgId == UbxEsfMessages::UBX_ESF_INS) { return "INS"; }
        if (msgId == UbxEsfMessages::UBX_ESF_MEAS) { return "MEAS"; }
        if (msgId == UbxEsfMessages::UBX_ESF_RAW) { return "RAW"; }
        if (msgId == UbxEsfMessages::UBX_ESF_STATUS) { return "STATUS"; }
    }
    else if (msgClass == UbxClass::UBX_CLASS_HNR)
    {
        if (msgId == UbxHnrMessages::UBX_HNR_INS) { return "INS"; }
        if (msgId == UbxHnrMessages::UBX_HNR_PVT) { return "PVT"; }
    }
    else if (msgClass == UbxClass::UBX_CLASS_INF)
    {
        if (msgId == UbxInfMessages::UBX_INF_DEBUG) { return "DEBUG"; }
        if (msgId == UbxInfMessages::UBX_INF_ERROR) { return "ERROR"; }
        if (msgId == UbxInfMessages::UBX_INF_NOTICE) { return "NOTICE"; }
        if (msgId == UbxInfMessages::UBX_INF_TEST) { return "TEST"; }
        if (msgId == UbxInfMessages::UBX_INF_WARNING) { return "WARNING"; }
    }
    else if (msgClass == UbxClass::UBX_CLASS_LOG)
    {
        if (msgId == UbxLogMessages::UBX_LOG_BATCH) { return "BATCH"; }
        if (msgId == UbxLogMessages::UBX_LOG_CREATE) { return "CREATE"; }
        if (msgId == UbxLogMessages::UBX_LOG_ERASE) { return "ERASE"; }
        if (msgId == UbxLogMessages::UBX_LOG_FINDTIME) { return "FINDTIME"; }
        if (msgId == UbxLogMessages::UBX_LOG_INFO) { return "INFO"; }
        if (msgId == UbxLogMessages::UBX_LOG_RETRIEVE) { return "RETRIEVE"; }
        if (msgId == UbxLogMessages::UBX_LOG_RETRIEVEBATCH) { return "RETRIEVEBATCH"; }
        if (msgId == UbxLogMessages::UBX_LOG_RETRIEVEPOS) { return "RETRIEVEPOS"; }
        if (msgId == UbxLogMessages::UBX_LOG_RETRIEVEPOSEXTRA) { return "RETRIEVEPOSEXTRA"; }
        if (msgId == UbxLogMessages::UBX_LOG_RETRIEVESTRING) { return "RETRIEVESTRING"; }
        if (msgId == UbxLogMessages::UBX_LOG_STRING) { return "STRING"; }
    }
    else if (msgClass == UbxClass::UBX_CLASS_MGA)
    {
        if (msgId == UbxMgaMessages::UBX_MGA_ACK_DATA0) { return "ACK_DATA0"; }
        if (msgId == UbxMgaMessages::UBX_MGA_ANO) { return "ANO"; }
        if (msgId == UbxMgaMessages::UBX_MGA_BDS_ALM) { return "BDS_ALM"; }
        if (msgId == UbxMgaMessages::UBX_MGA_BDS_EPH) { return "BDS_EPH"; }
        if (msgId == UbxMgaMessages::UBX_MGA_BDS_HEALTH) { return "BDS_HEALTH"; }
        if (msgId == UbxMgaMessages::UBX_MGA_BDS_IONO) { return "BDS_IONO"; }
        if (msgId == UbxMgaMessages::UBX_MGA_BDS_UTC) { return "BDS_UTC"; }
        if (msgId == UbxMgaMessages::UBX_MGA_DBD) { return "DBD"; }
        if (msgId == UbxMgaMessages::UBX_MGA_FLASH_ACK) { return "FLASH_ACK"; }
        if (msgId == UbxMgaMessages::UBX_MGA_FLASH_DATA) { return "FLASH_DATA"; }
        if (msgId == UbxMgaMessages::UBX_MGA_FLASH_STOP) { return "FLASH_STOP"; }
        if (msgId == UbxMgaMessages::UBX_MGA_GAL_ALM) { return "GAL_ALM"; }
        if (msgId == UbxMgaMessages::UBX_MGA_GAL_EPH) { return "GAL_EPH"; }
        if (msgId == UbxMgaMessages::UBX_MGA_GAL_TIMEOFFSET) { return "GAL_TIMEOFFSET"; }
        if (msgId == UbxMgaMessages::UBX_MGA_GAL_UTC) { return "GAL_UTC"; }
        if (msgId == UbxMgaMessages::UBX_MGA_GLO_ALM) { return "GLO_ALM"; }
        if (msgId == UbxMgaMessages::UBX_MGA_GLO_EPH) { return "GLO_EPH"; }
        if (msgId == UbxMgaMessages::UBX_MGA_GLO_TIMEOFFSET) { return "GLO_TIMEOFFSET"; }
        if (msgId == UbxMgaMessages::UBX_MGA_GPS_ALM) { return "GPS_ALM"; }
        if (msgId == UbxMgaMessages::UBX_MGA_GPS_EPH) { return "GPS_EPH"; }
        if (msgId == UbxMgaMessages::UBX_MGA_GPS_HEALTH) { return "GPS_HEALTH"; }
        if (msgId == UbxMgaMessages::UBX_MGA_GPS_IONO) { return "GPS_IONO"; }
        if (msgId == UbxMgaMessages::UBX_MGA_GPS_UTC) { return "GPS_UTC"; }
        if (msgId == UbxMgaMessages::UBX_MGA_INI_CLKD) { return "INI_CLKD"; }
        if (msgId == UbxMgaMessages::UBX_MGA_INI_EOP) { return "INI_EOP"; }
        if (msgId == UbxMgaMessages::UBX_MGA_INI_FREQ) { return "INI_FREQ"; }
        if (msgId == UbxMgaMessages::UBX_MGA_INI_POS_LLH) { return "INI_POS_LLH"; }
        if (msgId == UbxMgaMessages::UBX_MGA_INI_POS_XYZ) { return "INI_POS_XYZ"; }
        if (msgId == UbxMgaMessages::UBX_MGA_INI_TIME_GNSS) { return "INI_TIME_GNSS"; }
        if (msgId == UbxMgaMessages::UBX_MGA_INI_TIME_UTC) { return "INI_TIME_UTC"; }
        if (msgId == UbxMgaMessages::UBX_MGA_QZSS_ALM) { return "QZSS_ALM"; }
        if (msgId == UbxMgaMessages::UBX_MGA_QZSS_EPH) { return "QZSS_EPH"; }
        if (msgId == UbxMgaMessages::UBX_MGA_QZSS_HEALTH) { return "QZSS_HEALTH"; }
    }
    else if (msgClass == UbxClass::UBX_CLASS_MON)
    {
        if (msgId == UbxMonMessages::UBX_MON_BATCH) { return "BATCH"; }
        if (msgId == UbxMonMessages::UBX_MON_GNSS) { return "GNSS"; }
        if (msgId == UbxMonMessages::UBX_MON_HW2) { return "HW2"; }
        if (msgId == UbxMonMessages::UBX_MON_HW) { return "HW"; }
        if (msgId == UbxMonMessages::UBX_MON_IO) { return "IO"; }
        if (msgId == UbxMonMessages::UBX_MON_MSGPP) { return "MSGPP"; }
        if (msgId == UbxMonMessages::UBX_MON_PATCH) { return "PATCH"; }
        if (msgId == UbxMonMessages::UBX_MON_RXBUFF) { return "RXBUFF"; }
        if (msgId == UbxMonMessages::UBX_MON_RXR) { return "RXR"; }
        if (msgId == UbxMonMessages::UBX_MON_SMGR) { return "SMGR"; }
        if (msgId == UbxMonMessages::UBX_MON_TXBUFF) { return "TXBUFF"; }
        if (msgId == UbxMonMessages::UBX_MON_VER) { return "VER"; }
    }
    else if (msgClass == UbxClass::UBX_CLASS_NAV)
    {
        if (msgId == UbxNavMessages::UBX_NAV_AOPSTATUS) { return "AOPSTATUS"; }
        if (msgId == UbxNavMessages::UBX_NAV_ATT) { return "ATT"; }
        if (msgId == UbxNavMessages::UBX_NAV_CLOCK) { return "CLOCK"; }
        if (msgId == UbxNavMessages::UBX_NAV_DGPS) { return "DGPS"; }
        if (msgId == UbxNavMessages::UBX_NAV_DOP) { return "DOP"; }
        if (msgId == UbxNavMessages::UBX_NAV_EOE) { return "EOE"; }
        if (msgId == UbxNavMessages::UBX_NAV_GEOFENCE) { return "GEOFENCE"; }
        if (msgId == UbxNavMessages::UBX_NAV_HPPOSECEF) { return "HPPOSECEF"; }
        if (msgId == UbxNavMessages::UBX_NAV_HPPOSLLH) { return "HPPOSLLH"; }
        if (msgId == UbxNavMessages::UBX_NAV_ODO) { return "ODO"; }
        if (msgId == UbxNavMessages::UBX_NAV_ORB) { return "ORB"; }
        if (msgId == UbxNavMessages::UBX_NAV_POSECEF) { return "POSECEF"; }
        if (msgId == UbxNavMessages::UBX_NAV_POSLLH) { return "POSLLH"; }
        if (msgId == UbxNavMessages::UBX_NAV_PVT) { return "PVT"; }
        if (msgId == UbxNavMessages::UBX_NAV_RELPOSNED) { return "RELPOSNED"; }
        if (msgId == UbxNavMessages::UBX_NAV_RESETODO) { return "RESETODO"; }
        if (msgId == UbxNavMessages::UBX_NAV_SAT) { return "SAT"; }
        if (msgId == UbxNavMessages::UBX_NAV_SBAS) { return "SBAS"; }
        if (msgId == UbxNavMessages::UBX_NAV_SLAS) { return "SLAS"; }
        if (msgId == UbxNavMessages::UBX_NAV_SOL) { return "SOL"; }
        if (msgId == UbxNavMessages::UBX_NAV_STATUS) { return "STATUS"; }
        if (msgId == UbxNavMessages::UBX_NAV_SVIN) { return "SVIN"; }
        if (msgId == UbxNavMessages::UBX_NAV_SVINFO) { return "SVINFO"; }
        if (msgId == UbxNavMessages::UBX_NAV_TIMEBDS) { return "TIMEBDS"; }
        if (msgId == UbxNavMessages::UBX_NAV_TIMEGAL) { return "TIMEGAL"; }
        if (msgId == UbxNavMessages::UBX_NAV_TIMEGLO) { return "TIMEGLO"; }
        if (msgId == UbxNavMessages::UBX_NAV_TIMEGPS) { return "TIMEGPS"; }
        if (msgId == UbxNavMessages::UBX_NAV_TIMELS) { return "TIMELS"; }
        if (msgId == UbxNavMessages::UBX_NAV_TIMEUTC) { return "TIMEUTC"; }
        if (msgId == UbxNavMessages::UBX_NAV_VELECEF) { return "VELECEF"; }
        if (msgId == UbxNavMessages::UBX_NAV_VELNED) { return "VELNED"; }
    }
    else if (msgClass == UbxClass::UBX_CLASS_RXM)
    {
        if (msgId == UbxRxmMessages::UBX_RXM_IMES) { return "IMES"; }
        if (msgId == UbxRxmMessages::UBX_RXM_MEASX) { return "MEASX"; }
        if (msgId == UbxRxmMessages::UBX_RXM_PMREQ) { return "PMREQ"; }
        if (msgId == UbxRxmMessages::UBX_RXM_RAWX) { return "RAWX"; }
        if (msgId == UbxRxmMessages::UBX_RXM_RLM) { return "RLM"; }
        if (msgId == UbxRxmMessages::UBX_RXM_RTCM) { return "RTCM"; }
        if (msgId == UbxRxmMessages::UBX_RXM_SFRBX) { return "SFRBX"; }
        if (msgId == UbxRxmMessages::UBX_RXM_SVSI) { return "SVSI"; }
    }
    else if (msgClass == UbxClass::UBX_CLASS_SEC)
    {
        if (msgId == UbxSecMessages::UBX_SEC_UNIQID) { return "UNIQID"; }
    }
    else if (msgClass == UbxClass::UBX_CLASS_TIM)
    {
        if (msgId == UbxTimMessages::UBX_TIM_FCHG) { return "FCHG"; }
        if (msgId == UbxTimMessages::UBX_TIM_HOC) { return "HOC"; }
        if (msgId == UbxTimMessages::UBX_TIM_SMEAS) { return "SMEAS"; }
        if (msgId == UbxTimMessages::UBX_TIM_SVIN) { return "SVIN"; }
        if (msgId == UbxTimMessages::UBX_TIM_TM2) { return "TM2"; }
        if (msgId == UbxTimMessages::UBX_TIM_TOS) { return "TOS"; }
        if (msgId == UbxTimMessages::UBX_TIM_TP) { return "TP"; }
        if (msgId == UbxTimMessages::UBX_TIM_VCOCAL) { return "VCOCAL"; }
        if (msgId == UbxTimMessages::UBX_TIM_VRFY) { return "VRFY"; }
    }
    else if (msgClass == UbxClass::UBX_CLASS_UPD)
    {
        if (msgId == UbxUpdMessages::UBX_UPD_SOS) { return "SOS"; }
    }

    return "UNDEFINED";
}

uint8_t NAV::vendor::ublox::getMsgIdFromString(const std::string& className, const std::string& idName)
{
    return getMsgIdFromString(getMsgClassFromString(className), idName);
}

NAV::SatelliteSystem NAV::vendor::ublox::getSatSys(uint8_t gnssId)
{
    switch (gnssId)
    {
    case 0:
        return GPS;
    case 1:
        return SBAS;
    case 2:
        return GAL;
    case 3:
        return BDS;
    case 5:
        return QZSS;
    case 6:
        return GLO;
    case 7:
        return IRNSS;
    default:
        return SatSys_None;
    }
}

NAV::Code NAV::vendor::ublox::getCode(uint8_t gnssId, uint8_t sigId)
{
    if (gnssId == 0) // GPS
    {
        if (sigId == 0) { return Code::G1C; } // GPS L1C/A
        if (sigId == 3) { return Code::G2L; } // GPS L2 CL
        if (sigId == 4) { return Code::G2M; } // GPS L2 CM
        if (sigId == 6) { return Code::G5I; } // GPS L5 I
        if (sigId == 7) { return Code::G5X; } // GPS L5 Q
    }
    else if (gnssId == 1) // SBAS
    {
        if (sigId == 0) { return Code::S1C; } // SBAS L1C/A
    }
    else if (gnssId == 2) // Galileo
    {
        if (sigId == 0) { return Code::E1X; } // Galileo E1 C
        if (sigId == 1) { return Code::E1B; } // Galileo E1 B
        if (sigId == 3) { return Code::E5I; } // Galileo E5 aI
        if (sigId == 4) { return Code::E5X; } // Galileo E5 aQ
        if (sigId == 5) { return Code::E7I; } // Galileo E5 bI
        if (sigId == 6) { return Code::E7X; } // Galileo E5 bQ
    }
    else if (gnssId == 3) // Beidou
    {
        if (sigId == 0) { return Code::B2I; } // BeiDou B1I D1
        if (sigId == 1) { return Code::B2X; } // BeiDou B1I D2
        if (sigId == 2) { return Code::B7I; } // BeiDou B2I D1
        if (sigId == 3) { return Code::B7X; } // BeiDou B2I D2
        if (sigId == 5) { return Code::B1P; } // BeiDou B1 Cp (pilot)
        if (sigId == 6) { return Code::B1D; } // BeiDou B1 Cd (data)
        if (sigId == 7) { return Code::B5P; } // BeiDou B2 ap (pilot)
        if (sigId == 8) { return Code::B5D; } // BeiDou B2 ad (data)
    }
    else if (gnssId == 5) // QZSS
    {
        if (sigId == 0) { return Code::J1C; } // QZSS L1C/A
        if (sigId == 1) { return Code::J1S; } // QZSS L1S
        if (sigId == 4) { return Code::J2S; } // QZSS L2 CM
        if (sigId == 5) { return Code::J2L; } // QZSS L2 CL
        if (sigId == 8) { return Code::J5D; } // QZSS L5 I
        if (sigId == 9) { return Code::J5P; } // QZSS L5 Q
    }
    else if (gnssId == 6) // GLONASS
    {
        if (sigId == 0) { return Code::R1C; } // GLONASS L1 OF
        if (sigId == 2) { return Code::R2C; } // GLONASS L2 OF
    }
    else if (gnssId == 7) // IRNSS
    {
        if (sigId == 0) { return Code::I5A; } // NavIC L5 A
    }
    return Code::None;
}

std::ostream& operator<<(std::ostream& os, const NAV::vendor::ublox::UbxClass& obj)
{
    return os << fmt::format("{}", obj);
}