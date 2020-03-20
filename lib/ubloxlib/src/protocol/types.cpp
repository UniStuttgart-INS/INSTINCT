#include "ub/protocol/types.hpp"

ub::protocol::uart::UbxClass ub::protocol::uart::getMsgClassFromString(std::string className)
{
    if (className == "NAV")
        return UbxClass::UBX_CLASS_NAV;
    else if (className == "RXM")
        return UbxClass::UBX_CLASS_RXM;
    else if (className == "INF")
        return UbxClass::UBX_CLASS_INF;
    else if (className == "ACK")
        return UbxClass::UBX_CLASS_ACK;
    else if (className == "CFG")
        return UbxClass::UBX_CLASS_CFG;
    else if (className == "UPD")
        return UbxClass::UBX_CLASS_UPD;
    else if (className == "MON")
        return UbxClass::UBX_CLASS_MON;
    else if (className == "AID")
        return UbxClass::UBX_CLASS_AID;
    else if (className == "TIM")
        return UbxClass::UBX_CLASS_TIM;
    else if (className == "ESF")
        return UbxClass::UBX_CLASS_ESF;
    else if (className == "MGA")
        return UbxClass::UBX_CLASS_MGA;
    else if (className == "LOG")
        return UbxClass::UBX_CLASS_LOG;
    else if (className == "SEC")
        return UbxClass::UBX_CLASS_SEC;
    else if (className == "HNR")
        return UbxClass::UBX_CLASS_HNR;
    else
        return static_cast<UbxClass>(NULL);
}

uint8_t ub::protocol::uart::getMsgIdFromString(ub::protocol::uart::UbxClass msgClass, std::string idName)
{
    if (msgClass == UbxClass::UBX_CLASS_ACK)
    {
        if (idName == "ACK")
            return UbxAckMessages::UBX_ACK_ACK;
        else if (idName == "NAK")
            return UbxAckMessages::UBX_ACK_NAK;
    }
    else if (msgClass == UbxClass::UBX_CLASS_AID)
    {
        if (idName == "ALM")
            return UbxAidMessages::UBX_AID_ALM;
        else if (idName == "AOP")
            return UbxAidMessages::UBX_AID_AOP;
        else if (idName == "EPH")
            return UbxAidMessages::UBX_AID_EPH;
        else if (idName == "HUI")
            return UbxAidMessages::UBX_AID_HUI;
        else if (idName == "INI")
            return UbxAidMessages::UBX_AID_INI;
    }
    else if (msgClass == UbxClass::UBX_CLASS_CFG)
    {
        if (idName == "ANT")
            return UbxCfgMessages::UBX_CFG_ANT;
        else if (idName == "BATCH")
            return UbxCfgMessages::UBX_CFG_BATCH;
        else if (idName == "CFG")
            return UbxCfgMessages::UBX_CFG_CFG;
        else if (idName == "DAT")
            return UbxCfgMessages::UBX_CFG_DAT;
        else if (idName == "DGNSS")
            return UbxCfgMessages::UBX_CFG_DGNSS;
        else if (idName == "DOSC")
            return UbxCfgMessages::UBX_CFG_DOSC;
        else if (idName == "ESRC")
            return UbxCfgMessages::UBX_CFG_ESRC;
        else if (idName == "GEOFENCE")
            return UbxCfgMessages::UBX_CFG_GEOFENCE;
        else if (idName == "GNSS")
            return UbxCfgMessages::UBX_CFG_GNSS;
        else if (idName == "HNR")
            return UbxCfgMessages::UBX_CFG_HNR;
        else if (idName == "INF")
            return UbxCfgMessages::UBX_CFG_INF;
        else if (idName == "ITFM")
            return UbxCfgMessages::UBX_CFG_ITFM;
        else if (idName == "LOGFILTER")
            return UbxCfgMessages::UBX_CFG_LOGFILTER;
        else if (idName == "MSG")
            return UbxCfgMessages::UBX_CFG_MSG;
        else if (idName == "NAV5")
            return UbxCfgMessages::UBX_CFG_NAV5;
        else if (idName == "NAVX5")
            return UbxCfgMessages::UBX_CFG_NAVX5;
        else if (idName == "NMEA")
            return UbxCfgMessages::UBX_CFG_NMEA;
        else if (idName == "ODO")
            return UbxCfgMessages::UBX_CFG_ODO;
        else if (idName == "PM2")
            return UbxCfgMessages::UBX_CFG_PM2;
        else if (idName == "PMS")
            return UbxCfgMessages::UBX_CFG_PMS;
        else if (idName == "PRT")
            return UbxCfgMessages::UBX_CFG_PRT;
        else if (idName == "PWR")
            return UbxCfgMessages::UBX_CFG_PWR;
        else if (idName == "RATE")
            return UbxCfgMessages::UBX_CFG_RATE;
        else if (idName == "RINV")
            return UbxCfgMessages::UBX_CFG_RINV;
        else if (idName == "RST")
            return UbxCfgMessages::UBX_CFG_RST;
        else if (idName == "RXM")
            return UbxCfgMessages::UBX_CFG_RXM;
        else if (idName == "SBAS")
            return UbxCfgMessages::UBX_CFG_SBAS;
        else if (idName == "SLAS")
            return UbxCfgMessages::UBX_CFG_SLAS;
        else if (idName == "SMGR")
            return UbxCfgMessages::UBX_CFG_SMGR;
        else if (idName == "TMODE2")
            return UbxCfgMessages::UBX_CFG_TMODE2;
        else if (idName == "TMODE3")
            return UbxCfgMessages::UBX_CFG_TMODE3;
        else if (idName == "TP5")
            return UbxCfgMessages::UBX_CFG_TP5;
        else if (idName == "TXSLOT")
            return UbxCfgMessages::UBX_CFG_TXSLOT;
        else if (idName == "USB")
            return UbxCfgMessages::UBX_CFG_USB;
    }
    else if (msgClass == UbxClass::UBX_CLASS_ESF)
    {
        if (idName == "INS")
            return UbxEsfMessages::UBX_ESF_INS;
        else if (idName == "MEAS")
            return UbxEsfMessages::UBX_ESF_MEAS;
        else if (idName == "RAW")
            return UbxEsfMessages::UBX_ESF_RAW;
        else if (idName == "STATUS")
            return UbxEsfMessages::UBX_ESF_STATUS;
    }
    else if (msgClass == UbxClass::UBX_CLASS_HNR)
    {
        if (idName == "INS")
            return UbxHnrMessages::UBX_HNR_INS;
        else if (idName == "PVT")
            return UbxHnrMessages::UBX_HNR_PVT;
    }
    else if (msgClass == UbxClass::UBX_CLASS_INF)
    {
        if (idName == "DEBUG")
            return UbxInfMessages::UBX_INF_DEBUG;
        else if (idName == "ERROR")
            return UbxInfMessages::UBX_INF_ERROR;
        else if (idName == "NOTICE")
            return UbxInfMessages::UBX_INF_NOTICE;
        else if (idName == "TEST")
            return UbxInfMessages::UBX_INF_TEST;
        else if (idName == "WARNING")
            return UbxInfMessages::UBX_INF_WARNING;
    }
    else if (msgClass == UbxClass::UBX_CLASS_LOG)
    {
        if (idName == "BATCH")
            return UbxLogMessages::UBX_LOG_BATCH;
        else if (idName == "CREATE")
            return UbxLogMessages::UBX_LOG_CREATE;
        else if (idName == "ERASE")
            return UbxLogMessages::UBX_LOG_ERASE;
        else if (idName == "FINDTIME")
            return UbxLogMessages::UBX_LOG_FINDTIME;
        else if (idName == "INFO")
            return UbxLogMessages::UBX_LOG_INFO;
        else if (idName == "RETRIEVE")
            return UbxLogMessages::UBX_LOG_RETRIEVE;
        else if (idName == "RETRIEVEBATCH")
            return UbxLogMessages::UBX_LOG_RETRIEVEBATCH;
        else if (idName == "RETRIEVEPOS")
            return UbxLogMessages::UBX_LOG_RETRIEVEPOS;
        else if (idName == "RETRIEVEPOSEXTRA")
            return UbxLogMessages::UBX_LOG_RETRIEVEPOSEXTRA;
        else if (idName == "RETRIEVESTRING")
            return UbxLogMessages::UBX_LOG_RETRIEVESTRING;
        else if (idName == "STRING")
            return UbxLogMessages::UBX_LOG_STRING;
    }
    else if (msgClass == UbxClass::UBX_CLASS_MGA)
    {
        if (idName == "ACK_DATA0")
            return UbxMgaMessages::UBX_MGA_ACK_DATA0;
        else if (idName == "ANO")
            return UbxMgaMessages::UBX_MGA_ANO;
        else if (idName == "BDS_ALM")
            return UbxMgaMessages::UBX_MGA_BDS_ALM;
        else if (idName == "BDS_EPH")
            return UbxMgaMessages::UBX_MGA_BDS_EPH;
        else if (idName == "BDS_HEALTH")
            return UbxMgaMessages::UBX_MGA_BDS_HEALTH;
        else if (idName == "BDS_IONO")
            return UbxMgaMessages::UBX_MGA_BDS_IONO;
        else if (idName == "BDS_UTC")
            return UbxMgaMessages::UBX_MGA_BDS_UTC;
        else if (idName == "DBD")
            return UbxMgaMessages::UBX_MGA_DBD;
        else if (idName == "FLASH_ACK")
            return UbxMgaMessages::UBX_MGA_FLASH_ACK;
        else if (idName == "FLASH_DATA")
            return UbxMgaMessages::UBX_MGA_FLASH_DATA;
        else if (idName == "FLASH_STOP")
            return UbxMgaMessages::UBX_MGA_FLASH_STOP;
        else if (idName == "GAL_ALM")
            return UbxMgaMessages::UBX_MGA_GAL_ALM;
        else if (idName == "GAL_EPH")
            return UbxMgaMessages::UBX_MGA_GAL_EPH;
        else if (idName == "GAL_TIMEOFFSET")
            return UbxMgaMessages::UBX_MGA_GAL_TIMEOFFSET;
        else if (idName == "GAL_UTC")
            return UbxMgaMessages::UBX_MGA_GAL_UTC;
        else if (idName == "GLO_ALM")
            return UbxMgaMessages::UBX_MGA_GLO_ALM;
        else if (idName == "GLO_EPH")
            return UbxMgaMessages::UBX_MGA_GLO_EPH;
        else if (idName == "GLO_TIMEOFFSET")
            return UbxMgaMessages::UBX_MGA_GLO_TIMEOFFSET;
        else if (idName == "GPS_ALM")
            return UbxMgaMessages::UBX_MGA_GPS_ALM;
        else if (idName == "GPS_EPH")
            return UbxMgaMessages::UBX_MGA_GPS_EPH;
        else if (idName == "GPS_HEALTH")
            return UbxMgaMessages::UBX_MGA_GPS_HEALTH;
        else if (idName == "GPS_IONO")
            return UbxMgaMessages::UBX_MGA_GPS_IONO;
        else if (idName == "GPS_UTC")
            return UbxMgaMessages::UBX_MGA_GPS_UTC;
        else if (idName == "INI_CLKD")
            return UbxMgaMessages::UBX_MGA_INI_CLKD;
        else if (idName == "INI_EOP")
            return UbxMgaMessages::UBX_MGA_INI_EOP;
        else if (idName == "INI_FREQ")
            return UbxMgaMessages::UBX_MGA_INI_FREQ;
        else if (idName == "INI_POS_LLH")
            return UbxMgaMessages::UBX_MGA_INI_POS_LLH;
        else if (idName == "INI_POS_XYZ")
            return UbxMgaMessages::UBX_MGA_INI_POS_XYZ;
        else if (idName == "INI_TIME_GNSS")
            return UbxMgaMessages::UBX_MGA_INI_TIME_GNSS;
        else if (idName == "INI_TIME_UTC")
            return UbxMgaMessages::UBX_MGA_INI_TIME_UTC;
        else if (idName == "QZSS_ALM")
            return UbxMgaMessages::UBX_MGA_QZSS_ALM;
        else if (idName == "QZSS_EPH")
            return UbxMgaMessages::UBX_MGA_QZSS_EPH;
        else if (idName == "QZSS_HEALTH")
            return UbxMgaMessages::UBX_MGA_QZSS_HEALTH;
    }
    else if (msgClass == UbxClass::UBX_CLASS_MON)
    {
        if (idName == "BATCH")
            return UbxMonMessages::UBX_MON_BATCH;
        else if (idName == "GNSS")
            return UbxMonMessages::UBX_MON_GNSS;
        else if (idName == "HW2")
            return UbxMonMessages::UBX_MON_HW2;
        else if (idName == "HW")
            return UbxMonMessages::UBX_MON_HW;
        else if (idName == "IO")
            return UbxMonMessages::UBX_MON_IO;
        else if (idName == "MSGPP")
            return UbxMonMessages::UBX_MON_MSGPP;
        else if (idName == "PATCH")
            return UbxMonMessages::UBX_MON_PATCH;
        else if (idName == "RXBUFF")
            return UbxMonMessages::UBX_MON_RXBUFF;
        else if (idName == "RXR")
            return UbxMonMessages::UBX_MON_RXR;
        else if (idName == "SMGR")
            return UbxMonMessages::UBX_MON_SMGR;
        else if (idName == "TXBUFF")
            return UbxMonMessages::UBX_MON_TXBUFF;
        else if (idName == "VER")
            return UbxMonMessages::UBX_MON_VER;
    }
    else if (msgClass == UbxClass::UBX_CLASS_NAV)
    {
        if (idName == "AOPSTATUS")
            return UbxNavMessages::UBX_NAV_AOPSTATUS;
        else if (idName == "ATT")
            return UbxNavMessages::UBX_NAV_ATT;
        else if (idName == "CLOCK")
            return UbxNavMessages::UBX_NAV_CLOCK;
        else if (idName == "DGPS")
            return UbxNavMessages::UBX_NAV_DGPS;
        else if (idName == "DOP")
            return UbxNavMessages::UBX_NAV_DOP;
        else if (idName == "EOE")
            return UbxNavMessages::UBX_NAV_EOE;
        else if (idName == "GEOFENCE")
            return UbxNavMessages::UBX_NAV_GEOFENCE;
        else if (idName == "HPPOSECEF")
            return UbxNavMessages::UBX_NAV_HPPOSECEF;
        else if (idName == "HPPOSLLH")
            return UbxNavMessages::UBX_NAV_HPPOSLLH;
        else if (idName == "ODO")
            return UbxNavMessages::UBX_NAV_ODO;
        else if (idName == "ORB")
            return UbxNavMessages::UBX_NAV_ORB;
        else if (idName == "POSECEF")
            return UbxNavMessages::UBX_NAV_POSECEF;
        else if (idName == "POSLLH")
            return UbxNavMessages::UBX_NAV_POSLLH;
        else if (idName == "PVT")
            return UbxNavMessages::UBX_NAV_PVT;
        else if (idName == "RELPOSNED")
            return UbxNavMessages::UBX_NAV_RELPOSNED;
        else if (idName == "RESETODO")
            return UbxNavMessages::UBX_NAV_RESETODO;
        else if (idName == "SAT")
            return UbxNavMessages::UBX_NAV_SAT;
        else if (idName == "SBAS")
            return UbxNavMessages::UBX_NAV_SBAS;
        else if (idName == "SLAS")
            return UbxNavMessages::UBX_NAV_SLAS;
        else if (idName == "SOL")
            return UbxNavMessages::UBX_NAV_SOL;
        else if (idName == "STATUS")
            return UbxNavMessages::UBX_NAV_STATUS;
        else if (idName == "SVIN")
            return UbxNavMessages::UBX_NAV_SVIN;
        else if (idName == "SVINFO")
            return UbxNavMessages::UBX_NAV_SVINFO;
        else if (idName == "TIMEBDS")
            return UbxNavMessages::UBX_NAV_TIMEBDS;
        else if (idName == "TIMEGAL")
            return UbxNavMessages::UBX_NAV_TIMEGAL;
        else if (idName == "TIMEGLO")
            return UbxNavMessages::UBX_NAV_TIMEGLO;
        else if (idName == "TIMEGPS")
            return UbxNavMessages::UBX_NAV_TIMEGPS;
        else if (idName == "TIMELS")
            return UbxNavMessages::UBX_NAV_TIMELS;
        else if (idName == "TIMEUTC")
            return UbxNavMessages::UBX_NAV_TIMEUTC;
        else if (idName == "VELECEF")
            return UbxNavMessages::UBX_NAV_VELECEF;
        else if (idName == "VELNED")
            return UbxNavMessages::UBX_NAV_VELNED;
    }
    else if (msgClass == UbxClass::UBX_CLASS_RXM)
    {
        if (idName == "IMES")
            return UbxRxmMessages::UBX_RXM_IMES;
        else if (idName == "MEASX")
            return UbxRxmMessages::UBX_RXM_MEASX;
        else if (idName == "PMREQ")
            return UbxRxmMessages::UBX_RXM_PMREQ;
        else if (idName == "RAWX")
            return UbxRxmMessages::UBX_RXM_RAWX;
        else if (idName == "RLM")
            return UbxRxmMessages::UBX_RXM_RLM;
        else if (idName == "RTCM")
            return UbxRxmMessages::UBX_RXM_RTCM;
        else if (idName == "SFRBX")
            return UbxRxmMessages::UBX_RXM_SFRBX;
        else if (idName == "SVSI")
            return UbxRxmMessages::UBX_RXM_SVSI;
    }
    else if (msgClass == UbxClass::UBX_CLASS_SEC)
    {
        if (idName == "UNIQID")
            return UbxSecMessages::UBX_SEC_UNIQID;
    }
    else if (msgClass == UbxClass::UBX_CLASS_TIM)
    {
        if (idName == "FCHG")
            return UbxTimMessages::UBX_TIM_FCHG;
        else if (idName == "HOC")
            return UbxTimMessages::UBX_TIM_HOC;
        else if (idName == "SMEAS")
            return UbxTimMessages::UBX_TIM_SMEAS;
        else if (idName == "SVIN")
            return UbxTimMessages::UBX_TIM_SVIN;
        else if (idName == "TM2")
            return UbxTimMessages::UBX_TIM_TM2;
        else if (idName == "TOS")
            return UbxTimMessages::UBX_TIM_TOS;
        else if (idName == "TP")
            return UbxTimMessages::UBX_TIM_TP;
        else if (idName == "VCOCAL")
            return UbxTimMessages::UBX_TIM_VCOCAL;
        else if (idName == "VRFY")
            return UbxTimMessages::UBX_TIM_VRFY;
    }
    else if (msgClass == UbxClass::UBX_CLASS_UPD)
    {
        if (idName == "SOS")
            return UbxUpdMessages::UBX_UPD_SOS;
    }

    return UINT8_MAX;
}

uint8_t ub::protocol::uart::getMsgIdFromString(std::string className, std::string idName)
{
    return getMsgIdFromString(getMsgClassFromString(className), idName);
}