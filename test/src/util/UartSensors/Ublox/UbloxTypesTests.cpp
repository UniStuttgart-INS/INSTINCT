#include <catch2/catch.hpp>

#include "util/UartSensors/Ublox/UbloxTypes.hpp"
namespace ub = NAV::sensors::ublox;

namespace NAV::TEST
{
TEST_CASE("[UbloxTypes] getMsgClassFromString", "[UbloxTypes]")
{
    REQUIRE(ub::getMsgClassFromString("NAV") == ub::UbxClass::UBX_CLASS_NAV);
    REQUIRE(ub::getMsgClassFromString("RXM") == ub::UbxClass::UBX_CLASS_RXM);
    REQUIRE(ub::getMsgClassFromString("INF") == ub::UbxClass::UBX_CLASS_INF);
    REQUIRE(ub::getMsgClassFromString("ACK") == ub::UbxClass::UBX_CLASS_ACK);
    REQUIRE(ub::getMsgClassFromString("CFG") == ub::UbxClass::UBX_CLASS_CFG);
    REQUIRE(ub::getMsgClassFromString("UPD") == ub::UbxClass::UBX_CLASS_UPD);
    REQUIRE(ub::getMsgClassFromString("MON") == ub::UbxClass::UBX_CLASS_MON);
    REQUIRE(ub::getMsgClassFromString("TIM") == ub::UbxClass::UBX_CLASS_TIM);
    REQUIRE(ub::getMsgClassFromString("ESF") == ub::UbxClass::UBX_CLASS_ESF);
    REQUIRE(ub::getMsgClassFromString("MGA") == ub::UbxClass::UBX_CLASS_MGA);
    REQUIRE(ub::getMsgClassFromString("LOG") == ub::UbxClass::UBX_CLASS_LOG);
    REQUIRE(ub::getMsgClassFromString("SEC") == ub::UbxClass::UBX_CLASS_SEC);
    REQUIRE(ub::getMsgClassFromString("HNR") == ub::UbxClass::UBX_CLASS_HNR);
    REQUIRE(ub::getMsgClassFromString("657") == ub::UbxClass::UBX_CLASS_NONE);
}

// NOLINTNEXTLINE(google-readability-function-size, hicpp-function-size,readability-function-size)
TEST_CASE("[UbloxTypes] getMsgIdFromString(UbxClass, std::string)", "[UbloxTypes]")
{
    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_ACK, "ACK") == static_cast<uint8_t>(ub::UbxAckMessages::UBX_ACK_ACK));
    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_ACK, "NAK") == static_cast<uint8_t>(ub::UbxAckMessages::UBX_ACK_NAK));

    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_CFG, "ANT") == static_cast<uint8_t>(ub::UbxCfgMessages::UBX_CFG_ANT));
    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_CFG, "BATCH") == static_cast<uint8_t>(ub::UbxCfgMessages::UBX_CFG_BATCH));
    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_CFG, "CFG") == static_cast<uint8_t>(ub::UbxCfgMessages::UBX_CFG_CFG));
    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_CFG, "DAT") == static_cast<uint8_t>(ub::UbxCfgMessages::UBX_CFG_DAT));
    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_CFG, "DGNSS") == static_cast<uint8_t>(ub::UbxCfgMessages::UBX_CFG_DGNSS));
    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_CFG, "DOSC") == static_cast<uint8_t>(ub::UbxCfgMessages::UBX_CFG_DOSC));
    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_CFG, "ESRC") == static_cast<uint8_t>(ub::UbxCfgMessages::UBX_CFG_ESRC));
    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_CFG, "GEOFENCE") == static_cast<uint8_t>(ub::UbxCfgMessages::UBX_CFG_GEOFENCE));
    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_CFG, "GNSS") == static_cast<uint8_t>(ub::UbxCfgMessages::UBX_CFG_GNSS));
    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_CFG, "HNR") == static_cast<uint8_t>(ub::UbxCfgMessages::UBX_CFG_HNR));
    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_CFG, "INF") == static_cast<uint8_t>(ub::UbxCfgMessages::UBX_CFG_INF));
    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_CFG, "ITFM") == static_cast<uint8_t>(ub::UbxCfgMessages::UBX_CFG_ITFM));
    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_CFG, "LOGFILTER") == static_cast<uint8_t>(ub::UbxCfgMessages::UBX_CFG_LOGFILTER));
    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_CFG, "MSG") == static_cast<uint8_t>(ub::UbxCfgMessages::UBX_CFG_MSG));
    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_CFG, "NAV5") == static_cast<uint8_t>(ub::UbxCfgMessages::UBX_CFG_NAV5));
    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_CFG, "NAVX5") == static_cast<uint8_t>(ub::UbxCfgMessages::UBX_CFG_NAVX5));
    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_CFG, "NMEA") == static_cast<uint8_t>(ub::UbxCfgMessages::UBX_CFG_NMEA));
    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_CFG, "ODO") == static_cast<uint8_t>(ub::UbxCfgMessages::UBX_CFG_ODO));
    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_CFG, "PM2") == static_cast<uint8_t>(ub::UbxCfgMessages::UBX_CFG_PM2));
    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_CFG, "PMS") == static_cast<uint8_t>(ub::UbxCfgMessages::UBX_CFG_PMS));
    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_CFG, "PRT") == static_cast<uint8_t>(ub::UbxCfgMessages::UBX_CFG_PRT));
    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_CFG, "PWR") == static_cast<uint8_t>(ub::UbxCfgMessages::UBX_CFG_PWR));
    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_CFG, "RATE") == static_cast<uint8_t>(ub::UbxCfgMessages::UBX_CFG_RATE));
    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_CFG, "RINV") == static_cast<uint8_t>(ub::UbxCfgMessages::UBX_CFG_RINV));
    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_CFG, "RST") == static_cast<uint8_t>(ub::UbxCfgMessages::UBX_CFG_RST));
    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_CFG, "RXM") == static_cast<uint8_t>(ub::UbxCfgMessages::UBX_CFG_RXM));
    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_CFG, "SBAS") == static_cast<uint8_t>(ub::UbxCfgMessages::UBX_CFG_SBAS));
    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_CFG, "SLAS") == static_cast<uint8_t>(ub::UbxCfgMessages::UBX_CFG_SLAS));
    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_CFG, "SMGR") == static_cast<uint8_t>(ub::UbxCfgMessages::UBX_CFG_SMGR));
    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_CFG, "TMODE2") == static_cast<uint8_t>(ub::UbxCfgMessages::UBX_CFG_TMODE2));
    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_CFG, "TMODE3") == static_cast<uint8_t>(ub::UbxCfgMessages::UBX_CFG_TMODE3));
    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_CFG, "TP5") == static_cast<uint8_t>(ub::UbxCfgMessages::UBX_CFG_TP5));
    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_CFG, "TXSLOT") == static_cast<uint8_t>(ub::UbxCfgMessages::UBX_CFG_TXSLOT));
    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_CFG, "USB") == static_cast<uint8_t>(ub::UbxCfgMessages::UBX_CFG_USB));

    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_ESF, "INS") == static_cast<uint8_t>(ub::UbxEsfMessages::UBX_ESF_INS));
    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_ESF, "MEAS") == static_cast<uint8_t>(ub::UbxEsfMessages::UBX_ESF_MEAS));
    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_ESF, "RAW") == static_cast<uint8_t>(ub::UbxEsfMessages::UBX_ESF_RAW));
    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_ESF, "STATUS") == static_cast<uint8_t>(ub::UbxEsfMessages::UBX_ESF_STATUS));

    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_HNR, "INS") == static_cast<uint8_t>(ub::UbxHnrMessages::UBX_HNR_INS));
    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_HNR, "PVT") == static_cast<uint8_t>(ub::UbxHnrMessages::UBX_HNR_PVT));

    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_INF, "DEBUG") == static_cast<uint8_t>(ub::UbxInfMessages::UBX_INF_DEBUG));
    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_INF, "ERROR") == static_cast<uint8_t>(ub::UbxInfMessages::UBX_INF_ERROR));
    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_INF, "NOTICE") == static_cast<uint8_t>(ub::UbxInfMessages::UBX_INF_NOTICE));
    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_INF, "TEST") == static_cast<uint8_t>(ub::UbxInfMessages::UBX_INF_TEST));
    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_INF, "WARNING") == static_cast<uint8_t>(ub::UbxInfMessages::UBX_INF_WARNING));

    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_LOG, "BATCH") == static_cast<uint8_t>(ub::UbxLogMessages::UBX_LOG_BATCH));
    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_LOG, "CREATE") == static_cast<uint8_t>(ub::UbxLogMessages::UBX_LOG_CREATE));
    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_LOG, "ERASE") == static_cast<uint8_t>(ub::UbxLogMessages::UBX_LOG_ERASE));
    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_LOG, "FINDTIME") == static_cast<uint8_t>(ub::UbxLogMessages::UBX_LOG_FINDTIME));
    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_LOG, "INFO") == static_cast<uint8_t>(ub::UbxLogMessages::UBX_LOG_INFO));
    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_LOG, "RETRIEVE") == static_cast<uint8_t>(ub::UbxLogMessages::UBX_LOG_RETRIEVE));
    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_LOG, "RETRIEVEBATCH") == static_cast<uint8_t>(ub::UbxLogMessages::UBX_LOG_RETRIEVEBATCH));
    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_LOG, "RETRIEVEPOS") == static_cast<uint8_t>(ub::UbxLogMessages::UBX_LOG_RETRIEVEPOS));
    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_LOG, "RETRIEVEPOSEXTRA") == static_cast<uint8_t>(ub::UbxLogMessages::UBX_LOG_RETRIEVEPOSEXTRA));
    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_LOG, "RETRIEVESTRING") == static_cast<uint8_t>(ub::UbxLogMessages::UBX_LOG_RETRIEVESTRING));
    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_LOG, "STRING") == static_cast<uint8_t>(ub::UbxLogMessages::UBX_LOG_STRING));

    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_MGA, "ACK_DATA0") == static_cast<uint8_t>(ub::UbxMgaMessages::UBX_MGA_ACK_DATA0));
    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_MGA, "ANO") == static_cast<uint8_t>(ub::UbxMgaMessages::UBX_MGA_ANO));
    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_MGA, "BDS_ALM") == static_cast<uint8_t>(ub::UbxMgaMessages::UBX_MGA_BDS_ALM));
    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_MGA, "BDS_EPH") == static_cast<uint8_t>(ub::UbxMgaMessages::UBX_MGA_BDS_EPH));
    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_MGA, "BDS_HEALTH") == static_cast<uint8_t>(ub::UbxMgaMessages::UBX_MGA_BDS_HEALTH));
    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_MGA, "BDS_IONO") == static_cast<uint8_t>(ub::UbxMgaMessages::UBX_MGA_BDS_IONO));
    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_MGA, "BDS_UTC") == static_cast<uint8_t>(ub::UbxMgaMessages::UBX_MGA_BDS_UTC));
    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_MGA, "DBD") == static_cast<uint8_t>(ub::UbxMgaMessages::UBX_MGA_DBD));
    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_MGA, "FLASH_ACK") == static_cast<uint8_t>(ub::UbxMgaMessages::UBX_MGA_FLASH_ACK));
    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_MGA, "FLASH_DATA") == static_cast<uint8_t>(ub::UbxMgaMessages::UBX_MGA_FLASH_DATA));
    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_MGA, "FLASH_STOP") == static_cast<uint8_t>(ub::UbxMgaMessages::UBX_MGA_FLASH_STOP));
    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_MGA, "GAL_ALM") == static_cast<uint8_t>(ub::UbxMgaMessages::UBX_MGA_GAL_ALM));
    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_MGA, "GAL_EPH") == static_cast<uint8_t>(ub::UbxMgaMessages::UBX_MGA_GAL_EPH));
    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_MGA, "GAL_TIMEOFFSET") == static_cast<uint8_t>(ub::UbxMgaMessages::UBX_MGA_GAL_TIMEOFFSET));
    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_MGA, "GAL_UTC") == static_cast<uint8_t>(ub::UbxMgaMessages::UBX_MGA_GAL_UTC));
    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_MGA, "GLO_ALM") == static_cast<uint8_t>(ub::UbxMgaMessages::UBX_MGA_GLO_ALM));
    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_MGA, "GLO_EPH") == static_cast<uint8_t>(ub::UbxMgaMessages::UBX_MGA_GLO_EPH));
    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_MGA, "GLO_TIMEOFFSET") == static_cast<uint8_t>(ub::UbxMgaMessages::UBX_MGA_GLO_TIMEOFFSET));
    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_MGA, "GPS_ALM") == static_cast<uint8_t>(ub::UbxMgaMessages::UBX_MGA_GPS_ALM));
    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_MGA, "GPS_EPH") == static_cast<uint8_t>(ub::UbxMgaMessages::UBX_MGA_GPS_EPH));
    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_MGA, "GPS_HEALTH") == static_cast<uint8_t>(ub::UbxMgaMessages::UBX_MGA_GPS_HEALTH));
    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_MGA, "GPS_IONO") == static_cast<uint8_t>(ub::UbxMgaMessages::UBX_MGA_GPS_IONO));
    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_MGA, "GPS_UTC") == static_cast<uint8_t>(ub::UbxMgaMessages::UBX_MGA_GPS_UTC));
    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_MGA, "INI_CLKD") == static_cast<uint8_t>(ub::UbxMgaMessages::UBX_MGA_INI_CLKD));
    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_MGA, "INI_EOP") == static_cast<uint8_t>(ub::UbxMgaMessages::UBX_MGA_INI_EOP));
    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_MGA, "INI_FREQ") == static_cast<uint8_t>(ub::UbxMgaMessages::UBX_MGA_INI_FREQ));
    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_MGA, "INI_POS_LLH") == static_cast<uint8_t>(ub::UbxMgaMessages::UBX_MGA_INI_POS_LLH));
    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_MGA, "INI_POS_XYZ") == static_cast<uint8_t>(ub::UbxMgaMessages::UBX_MGA_INI_POS_XYZ));
    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_MGA, "INI_TIME_GNSS") == static_cast<uint8_t>(ub::UbxMgaMessages::UBX_MGA_INI_TIME_GNSS));
    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_MGA, "INI_TIME_UTC") == static_cast<uint8_t>(ub::UbxMgaMessages::UBX_MGA_INI_TIME_UTC));
    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_MGA, "QZSS_ALM") == static_cast<uint8_t>(ub::UbxMgaMessages::UBX_MGA_QZSS_ALM));
    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_MGA, "QZSS_EPH") == static_cast<uint8_t>(ub::UbxMgaMessages::UBX_MGA_QZSS_EPH));
    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_MGA, "QZSS_HEALTH") == static_cast<uint8_t>(ub::UbxMgaMessages::UBX_MGA_QZSS_HEALTH));

    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_MON, "BATCH") == static_cast<uint8_t>(ub::UbxMonMessages::UBX_MON_BATCH));
    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_MON, "GNSS") == static_cast<uint8_t>(ub::UbxMonMessages::UBX_MON_GNSS));
    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_MON, "HW2") == static_cast<uint8_t>(ub::UbxMonMessages::UBX_MON_HW2));
    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_MON, "HW") == static_cast<uint8_t>(ub::UbxMonMessages::UBX_MON_HW));
    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_MON, "IO") == static_cast<uint8_t>(ub::UbxMonMessages::UBX_MON_IO));
    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_MON, "MSGPP") == static_cast<uint8_t>(ub::UbxMonMessages::UBX_MON_MSGPP));
    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_MON, "PATCH") == static_cast<uint8_t>(ub::UbxMonMessages::UBX_MON_PATCH));
    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_MON, "RXBUFF") == static_cast<uint8_t>(ub::UbxMonMessages::UBX_MON_RXBUFF));
    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_MON, "RXR") == static_cast<uint8_t>(ub::UbxMonMessages::UBX_MON_RXR));
    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_MON, "SMGR") == static_cast<uint8_t>(ub::UbxMonMessages::UBX_MON_SMGR));
    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_MON, "TXBUFF") == static_cast<uint8_t>(ub::UbxMonMessages::UBX_MON_TXBUFF));
    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_MON, "VER") == static_cast<uint8_t>(ub::UbxMonMessages::UBX_MON_VER));

    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_NAV, "AOPSTATUS") == static_cast<uint8_t>(ub::UbxNavMessages::UBX_NAV_AOPSTATUS));
    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_NAV, "ATT") == static_cast<uint8_t>(ub::UbxNavMessages::UBX_NAV_ATT));
    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_NAV, "CLOCK") == static_cast<uint8_t>(ub::UbxNavMessages::UBX_NAV_CLOCK));
    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_NAV, "DGPS") == static_cast<uint8_t>(ub::UbxNavMessages::UBX_NAV_DGPS));
    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_NAV, "DOP") == static_cast<uint8_t>(ub::UbxNavMessages::UBX_NAV_DOP));
    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_NAV, "EOE") == static_cast<uint8_t>(ub::UbxNavMessages::UBX_NAV_EOE));
    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_NAV, "GEOFENCE") == static_cast<uint8_t>(ub::UbxNavMessages::UBX_NAV_GEOFENCE));
    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_NAV, "HPPOSECEF") == static_cast<uint8_t>(ub::UbxNavMessages::UBX_NAV_HPPOSECEF));
    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_NAV, "HPPOSLLH") == static_cast<uint8_t>(ub::UbxNavMessages::UBX_NAV_HPPOSLLH));
    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_NAV, "ODO") == static_cast<uint8_t>(ub::UbxNavMessages::UBX_NAV_ODO));
    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_NAV, "ORB") == static_cast<uint8_t>(ub::UbxNavMessages::UBX_NAV_ORB));
    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_NAV, "POSECEF") == static_cast<uint8_t>(ub::UbxNavMessages::UBX_NAV_POSECEF));
    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_NAV, "POSLLH") == static_cast<uint8_t>(ub::UbxNavMessages::UBX_NAV_POSLLH));
    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_NAV, "PVT") == static_cast<uint8_t>(ub::UbxNavMessages::UBX_NAV_PVT));
    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_NAV, "RELPOSNED") == static_cast<uint8_t>(ub::UbxNavMessages::UBX_NAV_RELPOSNED));
    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_NAV, "RESETODO") == static_cast<uint8_t>(ub::UbxNavMessages::UBX_NAV_RESETODO));
    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_NAV, "SAT") == static_cast<uint8_t>(ub::UbxNavMessages::UBX_NAV_SAT));
    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_NAV, "SBAS") == static_cast<uint8_t>(ub::UbxNavMessages::UBX_NAV_SBAS));
    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_NAV, "SLAS") == static_cast<uint8_t>(ub::UbxNavMessages::UBX_NAV_SLAS));
    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_NAV, "SOL") == static_cast<uint8_t>(ub::UbxNavMessages::UBX_NAV_SOL));
    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_NAV, "STATUS") == static_cast<uint8_t>(ub::UbxNavMessages::UBX_NAV_STATUS));
    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_NAV, "SVIN") == static_cast<uint8_t>(ub::UbxNavMessages::UBX_NAV_SVIN));
    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_NAV, "SVINFO") == static_cast<uint8_t>(ub::UbxNavMessages::UBX_NAV_SVINFO));
    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_NAV, "TIMEBDS") == static_cast<uint8_t>(ub::UbxNavMessages::UBX_NAV_TIMEBDS));
    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_NAV, "TIMEGAL") == static_cast<uint8_t>(ub::UbxNavMessages::UBX_NAV_TIMEGAL));
    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_NAV, "TIMEGLO") == static_cast<uint8_t>(ub::UbxNavMessages::UBX_NAV_TIMEGLO));
    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_NAV, "TIMEGPS") == static_cast<uint8_t>(ub::UbxNavMessages::UBX_NAV_TIMEGPS));
    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_NAV, "TIMELS") == static_cast<uint8_t>(ub::UbxNavMessages::UBX_NAV_TIMELS));
    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_NAV, "TIMEUTC") == static_cast<uint8_t>(ub::UbxNavMessages::UBX_NAV_TIMEUTC));
    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_NAV, "VELECEF") == static_cast<uint8_t>(ub::UbxNavMessages::UBX_NAV_VELECEF));
    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_NAV, "VELNED") == static_cast<uint8_t>(ub::UbxNavMessages::UBX_NAV_VELNED));

    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_RXM, "IMES") == static_cast<uint8_t>(ub::UbxRxmMessages::UBX_RXM_IMES));
    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_RXM, "MEASX") == static_cast<uint8_t>(ub::UbxRxmMessages::UBX_RXM_MEASX));
    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_RXM, "PMREQ") == static_cast<uint8_t>(ub::UbxRxmMessages::UBX_RXM_PMREQ));
    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_RXM, "RAWX") == static_cast<uint8_t>(ub::UbxRxmMessages::UBX_RXM_RAWX));
    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_RXM, "RLM") == static_cast<uint8_t>(ub::UbxRxmMessages::UBX_RXM_RLM));
    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_RXM, "RTCM") == static_cast<uint8_t>(ub::UbxRxmMessages::UBX_RXM_RTCM));
    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_RXM, "SFRBX") == static_cast<uint8_t>(ub::UbxRxmMessages::UBX_RXM_SFRBX));
    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_RXM, "SVSI") == static_cast<uint8_t>(ub::UbxRxmMessages::UBX_RXM_SVSI));

    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_SEC, "UNIQID") == static_cast<uint8_t>(ub::UbxSecMessages::UBX_SEC_UNIQID));

    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_TIM, "FCHG") == static_cast<uint8_t>(ub::UbxTimMessages::UBX_TIM_FCHG));
    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_TIM, "HOC") == static_cast<uint8_t>(ub::UbxTimMessages::UBX_TIM_HOC));
    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_TIM, "SMEAS") == static_cast<uint8_t>(ub::UbxTimMessages::UBX_TIM_SMEAS));
    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_TIM, "SVIN") == static_cast<uint8_t>(ub::UbxTimMessages::UBX_TIM_SVIN));
    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_TIM, "TM2") == static_cast<uint8_t>(ub::UbxTimMessages::UBX_TIM_TM2));
    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_TIM, "TOS") == static_cast<uint8_t>(ub::UbxTimMessages::UBX_TIM_TOS));
    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_TIM, "TP") == static_cast<uint8_t>(ub::UbxTimMessages::UBX_TIM_TP));
    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_TIM, "VCOCAL") == static_cast<uint8_t>(ub::UbxTimMessages::UBX_TIM_VCOCAL));
    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_TIM, "VRFY") == static_cast<uint8_t>(ub::UbxTimMessages::UBX_TIM_VRFY));

    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_UPD, "SOS") == static_cast<uint8_t>(ub::UbxUpdMessages::UBX_UPD_SOS));

    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_UPD, "AAA") == UINT8_MAX);
    REQUIRE(ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_NONE, "") == UINT8_MAX);
}

TEST_CASE("[UbloxTypes] getMsgIdFromString(std::string, std::string)", "[UbloxTypes]")
{
    REQUIRE(ub::getMsgIdFromString("ACK", "NAK") == ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_ACK, "NAK"));
    REQUIRE(ub::getMsgIdFromString("", "") == ub::getMsgIdFromString(ub::UbxClass::UBX_CLASS_NONE, ""));
}

} // namespace NAV::TEST