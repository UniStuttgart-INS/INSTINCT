// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "vn/sensors.h"

#ifndef HAS_VECTORNAV_LIBRARY

    #include "vn/vntime.h"
    #include "vn/vector.h"
    #include "vn/matrix.h"
    #include "vn/util.h"

    #include <string>
    #include <queue>
    #include <cstring>
    #include <cstdio>

// NOLINTBEGIN

using namespace vn::math;
using namespace vn::xplat;
using namespace vn::protocol::uart;

namespace vn
{
namespace sensors
{

std::vector<uint32_t> VnSensor::supportedBaudrates()
{
    static constexpr uint32_t br[] = {
        9600,
        19200,
        38400,
        57600,
        115200,
        128000,
        230400,
        460800,
        921600
    };

    return std::vector<uint32_t>(br, br + sizeof(br) / sizeof(uint32_t));
}

uint32_t VnSensor::baudrate()
{
    return 115200;
}

std::string VnSensor::port()
{
    return "N/A";
}

bool VnSensor::isConnected()
{
    return false;
}

bool VnSensor::verifySensorConnectivity()
{
    return false;
}

void VnSensor::connect(const std::string& /* portName */, uint32_t /* baudrate */)
{
}

void VnSensor::disconnect()
{
}

void VnSensor::registerRawDataReceivedHandler(void* /* userData */, RawDataReceivedHandler /* handler */)
{
}

void VnSensor::unregisterRawDataReceivedHandler()
{
}

void VnSensor::registerPossiblePacketFoundHandler(void* /* userData */, PossiblePacketFoundHandler /* handler */)
{
}

void VnSensor::unregisterPossiblePacketFoundHandler()
{
}

void VnSensor::registerAsyncPacketReceivedHandler(void* /* userData */, AsyncPacketReceivedHandler /* handler */)
{
}

void VnSensor::unregisterAsyncPacketReceivedHandler()
{
}

void VnSensor::registerErrorPacketReceivedHandler(void* /* userData */, ErrorPacketReceivedHandler /* handler */)
{
}

void VnSensor::unregisterErrorPacketReceivedHandler()
{
}

void VnSensor::writeSettings(bool /* waitForReply */)
{
}

void VnSensor::tare(bool /* waitForReply */)
{
}

void VnSensor::setGyroBias(bool /* waitForReply */)
{
}

void VnSensor::magneticDisturbancePresent(bool /* disturbancePresent */, bool /* waitForReply */)
{
}

void VnSensor::accelerationDisturbancePresent(bool /* disturbancePresent */, bool /* waitForReply */)
{
}

void VnSensor::restoreFactorySettings(bool /* waitForReply */)
{
}

void VnSensor::reset(bool /* waitForReply */)
{
}

void VnSensor::changeBaudRate(uint32_t /* baudrate */)
{
}

VnSensor::Family VnSensor::determineDeviceFamily()
{
    return VnSensor_Family_Unknown;
}

VnSensor::Family VnSensor::determineDeviceFamily(std::string modelNumber)
{
    if (modelNumber.find("VN-100") == 0)
        return VnSensor_Family_Vn100;

    if (modelNumber.find("VN-200") == 0)
        return VnSensor_Family_Vn200;

    if (modelNumber.find("VN-300") == 0)
        return VnSensor_Family_Vn300;

    return VnSensor_Family_Unknown;
}

BinaryOutputRegister VnSensor::readBinaryOutput1()
{
    return {};
}

BinaryOutputRegister VnSensor::readBinaryOutput2()
{
    return {};
}

BinaryOutputRegister VnSensor::readBinaryOutput3()
{
    return {};
}

void VnSensor::writeBinaryOutput1(BinaryOutputRegister& /* fields */, bool /* waitForReply */)
{
}

void VnSensor::writeBinaryOutput2(BinaryOutputRegister& /* fields */, bool /* waitForReply */)
{
}

void VnSensor::writeBinaryOutput3(BinaryOutputRegister& /* fields */, bool /* waitForReply */)
{
}

uint32_t VnSensor::readSerialBaudRate(uint8_t /* port */)
{
    return 0;
}

void VnSensor::writeSerialBaudRate(const uint32_t& /* baudrate */, uint8_t /* port */, bool /* waitForReply */)
{
}

AsciiAsync VnSensor::readAsyncDataOutputType(uint8_t /* port */)
{
    return AsciiAsync::VNOFF;
}

void VnSensor::writeAsyncDataOutputType(AsciiAsync /* ador */, uint8_t /* port */, bool /* waitForReply */)
{
}

uint32_t VnSensor::readAsyncDataOutputFrequency(uint8_t /* port */)
{
    return 0;
}

void VnSensor::writeAsyncDataOutputFrequency(const uint32_t& /* adof */, uint8_t /* port */, bool /* waitForReply */)
{
}

InsBasicConfigurationRegisterVn200 VnSensor::readInsBasicConfigurationVn200()
{
    return {};
}

void VnSensor::writeInsBasicConfigurationVn200(InsBasicConfigurationRegisterVn200& /* fields */, bool /* waitForReply */)
{
}

void VnSensor::writeInsBasicConfigurationVn200(Scenario /* scenario */, const uint8_t& /* ahrsAiding */, bool /* waitForReply */)
{
}

InsBasicConfigurationRegisterVn300 VnSensor::readInsBasicConfigurationVn300()
{
    return {};
}

void VnSensor::writeInsBasicConfigurationVn300(InsBasicConfigurationRegisterVn300& /* fields */, bool /* waitForReply */)
{
}

void VnSensor::writeInsBasicConfigurationVn300(
    Scenario /* scenario */,
    const uint8_t& /* ahrsAiding */,
    const uint8_t& /* estBaseline */,
    bool /* waitForReply */)
{
}

std::string VnSensor::readUserTag()
{
    return {};
}

void VnSensor::writeUserTag(const std::string& /* tag */, bool /* waitForReply */)
{
}

std::string VnSensor::readModelNumber()
{
    return {};
}

uint32_t VnSensor::readHardwareRevision()
{
    return {};
}

uint32_t VnSensor::readSerialNumber()
{
    return {};
}

std::string VnSensor::readFirmwareVersion()
{
    return {};
}

uint32_t VnSensor::readSerialBaudRate()
{
    return 0;
}

void VnSensor::writeSerialBaudRate(const uint32_t& /* baudrate */, bool /* waitForReply */)
{
}

AsciiAsync VnSensor::readAsyncDataOutputType()
{
    return AsciiAsync::VNOFF;
}

void VnSensor::writeAsyncDataOutputType(AsciiAsync /* ador */, bool /* waitForReply */)
{
}

uint32_t VnSensor::readAsyncDataOutputFrequency()
{
    return {};
}

void VnSensor::writeAsyncDataOutputFrequency(const uint32_t& /* adof */, bool /* waitForReply */)
{
}

vec3f VnSensor::readYawPitchRoll()
{
    return {};
}

vec4f VnSensor::readAttitudeQuaternion()
{
    return {};
}

QuaternionMagneticAccelerationAndAngularRatesRegister VnSensor::readQuaternionMagneticAccelerationAndAngularRates()
{
    return {};
}

vec3f VnSensor::readMagneticMeasurements()
{
    return {};
}

vec3f VnSensor::readAccelerationMeasurements()
{
    return {};
}

vec3f VnSensor::readAngularRateMeasurements()
{
    return {};
}

MagneticAccelerationAndAngularRatesRegister VnSensor::readMagneticAccelerationAndAngularRates()
{
    return {};
}

MagneticAndGravityReferenceVectorsRegister VnSensor::readMagneticAndGravityReferenceVectors()
{
    return {};
}

void VnSensor::writeMagneticAndGravityReferenceVectors(MagneticAndGravityReferenceVectorsRegister& /* fields */, bool /* waitForReply */)
{
}

void VnSensor::writeMagneticAndGravityReferenceVectors(const vec3f& /* magRef */, const vec3f& /* accRef */, bool /* waitForReply */)
{
}

FilterMeasurementsVarianceParametersRegister VnSensor::readFilterMeasurementsVarianceParameters()
{
    return {};
}

void VnSensor::writeFilterMeasurementsVarianceParameters(FilterMeasurementsVarianceParametersRegister& /* fields */, bool /* waitForReply */)
{
}

void VnSensor::writeFilterMeasurementsVarianceParameters(const float& /* angularWalkVariance */, const vec3f& /* angularRateVariance */, const vec3f& /* magneticVariance */, const vec3f& /* accelerationVariance */, bool /* waitForReply */)
{
}

MagnetometerCompensationRegister VnSensor::readMagnetometerCompensation()
{
    return {};
}

void VnSensor::writeMagnetometerCompensation(MagnetometerCompensationRegister& /* fields */, bool /* waitForReply */)
{
}

void VnSensor::writeMagnetometerCompensation(const mat3f& /* c */, const vec3f& /* b */, bool /* waitForReply */)
{
}

FilterActiveTuningParametersRegister VnSensor::readFilterActiveTuningParameters()
{
    return {};
}

void VnSensor::writeFilterActiveTuningParameters(FilterActiveTuningParametersRegister& /* fields */, bool /* waitForReply */)
{
}

void VnSensor::writeFilterActiveTuningParameters(
    const float& /* magneticDisturbanceGain */,
    const float& /* accelerationDisturbanceGain */,
    const float& /* magneticDisturbanceMemory */,
    const float& /* accelerationDisturbanceMemory */,
    bool /* waitForReply */)
{
}

AccelerationCompensationRegister VnSensor::readAccelerationCompensation()
{
    return {};
}

void VnSensor::writeAccelerationCompensation(AccelerationCompensationRegister& /* fields */, bool /* waitForReply */)
{
}

void VnSensor::writeAccelerationCompensation(
    const mat3f& /* c */,
    const vec3f& /* b */,
    bool /* waitForReply */)
{
}

mat3f VnSensor::readReferenceFrameRotation()
{
    return {};
}

void VnSensor::writeReferenceFrameRotation(const mat3f& /* c */, bool /* waitForReply */)
{
}

YawPitchRollMagneticAccelerationAndAngularRatesRegister VnSensor::readYawPitchRollMagneticAccelerationAndAngularRates()
{
    return {};
}

CommunicationProtocolControlRegister VnSensor::readCommunicationProtocolControl()
{
    return {};
}

void VnSensor::writeCommunicationProtocolControl(CommunicationProtocolControlRegister& /* fields */, bool /* waitForReply */)
{
}

void VnSensor::writeCommunicationProtocolControl(
    CountMode /* serialCount */,
    StatusMode /* serialStatus */,
    CountMode /* spiCount */,
    StatusMode /* spiStatus */,
    ChecksumMode /* serialChecksum */,
    ChecksumMode /* spiChecksum */,
    ErrorMode /* errorMode */,
    bool /* waitForReply */)
{
}

SynchronizationControlRegister VnSensor::readSynchronizationControl()
{
    return {};
}

void VnSensor::writeSynchronizationControl(SynchronizationControlRegister& /* fields */, bool /* waitForReply */)
{
}

void VnSensor::writeSynchronizationControl(
    SyncInMode /* syncInMode */,
    SyncInEdge /* syncInEdge */,
    const uint16_t& /* syncInSkipFactor */,
    SyncOutMode /* syncOutMode */,
    SyncOutPolarity /* syncOutPolarity */,
    const uint16_t& /* syncOutSkipFactor */,
    const uint32_t& /* syncOutPulseWidth */,
    bool /* waitForReply */)
{
}

SynchronizationStatusRegister VnSensor::readSynchronizationStatus()
{
    return {};
}

void VnSensor::writeSynchronizationStatus(SynchronizationStatusRegister& /* fields */, bool /* waitForReply */)
{
}

void VnSensor::writeSynchronizationStatus(
    const uint32_t& /* syncInCount */,
    const uint32_t& /* syncInTime */,
    const uint32_t& /* syncOutCount */,
    bool /* waitForReply */)
{
}

FilterBasicControlRegister VnSensor::readFilterBasicControl()
{
    return {};
}

void VnSensor::writeFilterBasicControl(FilterBasicControlRegister& /* fields */, bool /* waitForReply */)
{
}

void VnSensor::writeFilterBasicControl(
    MagneticMode /* magMode */,
    ExternalSensorMode /* extMagMode */,
    ExternalSensorMode /* extAccMode */,
    ExternalSensorMode /* extGyroMode */,
    const vec3f& /* gyroLimit */,
    bool /* waitForReply */)
{
}

HeaveConfigurationRegister VnSensor::readHeaveConfiguration()
{
    return {};
}

void VnSensor::writeHeaveConfiguration(HeaveConfigurationRegister& /* fields */, bool /* waitForReply */)
{
}

void VnSensor::writeHeaveConfiguration(
    const float& /* initialWavePeriod */,
    const float& /* initialWaveAmplitude */,
    const float& /* maxWavePeriod */,
    const float& /* minWaveAmplitude */,
    const float& /* delayedHeaveCutoffFreq */,
    const float& /* heaveCutoffFreq */,
    const float& /* heaveRateCutoffFreq */,
    bool /* waitForReply */)
{
}

VpeBasicControlRegister VnSensor::readVpeBasicControl()
{
    return {};
}

void VnSensor::writeVpeBasicControl(VpeBasicControlRegister& /* fields */, bool /* waitForReply */)
{
}

void VnSensor::writeVpeBasicControl(
    VpeEnable /* enable */,
    HeadingMode /* headingMode */,
    VpeMode /* filteringMode */,
    VpeMode /* tuningMode */,
    bool /* waitForReply */)
{
}

VpeMagnetometerBasicTuningRegister VnSensor::readVpeMagnetometerBasicTuning()
{
    return {};
}

void VnSensor::writeVpeMagnetometerBasicTuning(VpeMagnetometerBasicTuningRegister& /* fields */, bool /* waitForReply */)
{
}

void VnSensor::writeVpeMagnetometerBasicTuning(
    const vec3f& /* baseTuning */,
    const vec3f& /* adaptiveTuning */,
    const vec3f& /* adaptiveFiltering */,
    bool /* waitForReply */)
{
}

VpeMagnetometerAdvancedTuningRegister VnSensor::readVpeMagnetometerAdvancedTuning()
{
    return {};
}

void VnSensor::writeVpeMagnetometerAdvancedTuning(VpeMagnetometerAdvancedTuningRegister& /* fields */, bool /* waitForReply */)
{
}

void VnSensor::writeVpeMagnetometerAdvancedTuning(
    const vec3f& /* minFiltering */,
    const vec3f& /* maxFiltering */,
    const float& /* maxAdaptRate */,
    const float& /* disturbanceWindow */,
    const float& /* maxTuning */,
    bool /* waitForReply */)
{
}

VpeAccelerometerBasicTuningRegister VnSensor::readVpeAccelerometerBasicTuning()
{
    return {};
}

void VnSensor::writeVpeAccelerometerBasicTuning(VpeAccelerometerBasicTuningRegister& /* fields */, bool /* waitForReply */)
{
}

void VnSensor::writeVpeAccelerometerBasicTuning(
    const vec3f& /* baseTuning */,
    const vec3f& /* adaptiveTuning */,
    const vec3f& /* adaptiveFiltering */,
    bool /* waitForReply */)
{
}

VpeAccelerometerAdvancedTuningRegister VnSensor::readVpeAccelerometerAdvancedTuning()
{
    return {};
}

void VnSensor::writeVpeAccelerometerAdvancedTuning(VpeAccelerometerAdvancedTuningRegister& /* fields */, bool /* waitForReply */)
{
}

void VnSensor::writeVpeAccelerometerAdvancedTuning(
    const vec3f& /* minFiltering */,
    const vec3f& /* maxFiltering */,
    const float& /* maxAdaptRate */,
    const float& /* disturbanceWindow */,
    const float& /* maxTuning */,
    bool /* waitForReply */)
{
}

VpeGyroBasicTuningRegister VnSensor::readVpeGyroBasicTuning()
{
    return {};
}

void VnSensor::writeVpeGyroBasicTuning(VpeGyroBasicTuningRegister& /* fields */, bool /* waitForReply */)
{
}

void VnSensor::writeVpeGyroBasicTuning(
    const vec3f& /* angularWalkVariance */,
    const vec3f& /* baseTuning */,
    const vec3f& /* adaptiveTuning */,
    bool /* waitForReply */)
{
}

vec3f VnSensor::readFilterStartupGyroBias()
{
    return {};
}

void VnSensor::writeFilterStartupGyroBias(const vec3f& /* bias */, bool /* waitForReply */)
{
}

MagnetometerCalibrationControlRegister VnSensor::readMagnetometerCalibrationControl()
{
    return {};
}

void VnSensor::writeMagnetometerCalibrationControl(MagnetometerCalibrationControlRegister& /* fields */, bool /* waitForReply */)
{
}

void VnSensor::writeMagnetometerCalibrationControl(
    HsiMode /* hsiMode */,
    HsiOutput /* hsiOutput */,
    const uint8_t& /* convergeRate */,
    bool /* waitForReply */)
{
}

CalculatedMagnetometerCalibrationRegister VnSensor::readCalculatedMagnetometerCalibration()
{
    return {};
}

float VnSensor::readIndoorHeadingModeControl()
{
    return {};
}

void VnSensor::writeIndoorHeadingModeControl(const float& /* maxRateError */, bool /* waitForReply */)
{
}

vec3f VnSensor::readVelocityCompensationMeasurement()
{
    return {};
}

void VnSensor::writeVelocityCompensationMeasurement(const vec3f& /* velocity */, bool /* waitForReply */)
{
}

VelocityCompensationControlRegister VnSensor::readVelocityCompensationControl()
{
    return {};
}

void VnSensor::writeVelocityCompensationControl(VelocityCompensationControlRegister& /* fields */, bool /* waitForReply */)
{
}

void VnSensor::writeVelocityCompensationControl(
    VelocityCompensationMode /* mode */,
    const float& /* velocityTuning */,
    const float& /* rateTuning */,
    bool /* waitForReply */)
{
}

VelocityCompensationStatusRegister VnSensor::readVelocityCompensationStatus()
{
    return {};
}

ImuMeasurementsRegister VnSensor::readImuMeasurements()
{
    return {};
}

GpsConfigurationRegister VnSensor::readGpsConfiguration()
{
    return {};
}

void VnSensor::writeGpsConfiguration(GpsConfigurationRegister& /* fields */, bool /* waitForReply */)
{
}

void VnSensor::writeGpsConfiguration(GpsMode /* mode */, PpsSource /* ppsSource */, bool /* waitForReply */)
{
}

void VnSensor::writeGpsConfiguration(
    GpsMode /* mode */,
    PpsSource /* ppsSource */,
    GpsRate /* rate */,
    AntPower /* antPow */,
    bool /* waitForReply */)
{
}

vec3f VnSensor::readGpsAntennaOffset()
{
    return {};
}

void VnSensor::writeGpsAntennaOffset(const vec3f& /* position */, bool /* waitForReply */)
{
}

GpsSolutionLlaRegister VnSensor::readGpsSolutionLla()
{
    return {};
}

GpsSolutionEcefRegister VnSensor::readGpsSolutionEcef()
{
    return {};
}

InsSolutionLlaRegister VnSensor::readInsSolutionLla()
{
    return {};
}

InsSolutionEcefRegister VnSensor::readInsSolutionEcef()
{
    return {};
}

InsAdvancedConfigurationRegister VnSensor::readInsAdvancedConfiguration()
{
    return {};
}

void VnSensor::writeInsAdvancedConfiguration(InsAdvancedConfigurationRegister& /* fields */, bool /* waitForReply */)
{
}

void VnSensor::writeInsAdvancedConfiguration(
    const uint8_t& /* useMag */,
    const uint8_t& /* usePres */,
    const uint8_t& /* posAtt */,
    const uint8_t& /* velAtt */,
    const uint8_t& /* velBias */,
    FoamInit /* useFoam */,
    const uint8_t& /* gpsCovType */,
    const uint8_t& /* velCount */,
    const float& /* velInit */,
    const float& /* moveOrigin */,
    const float& /* gpsTimeout */,
    const float& /* deltaLimitPos */,
    const float& /* deltaLimitVel */,
    const float& /* minPosUncertainty */,
    const float& /* minVelUncertainty */,
    bool /* waitForReply */)
{
}

InsStateLlaRegister VnSensor::readInsStateLla()
{
    return {};
}

InsStateEcefRegister VnSensor::readInsStateEcef()
{
    return {};
}

StartupFilterBiasEstimateRegister VnSensor::readStartupFilterBiasEstimate()
{
    return {};
}

void VnSensor::writeStartupFilterBiasEstimate(StartupFilterBiasEstimateRegister& /* fields */, bool /* waitForReply */)
{
}

void VnSensor::writeStartupFilterBiasEstimate(
    const vec3f& /* gyroBias */,
    const vec3f& /* accelBias */,
    const float& /* pressureBias */,
    bool /* waitForReply */)
{
}

DeltaThetaAndDeltaVelocityRegister VnSensor::readDeltaThetaAndDeltaVelocity()
{
    return {};
}

DeltaThetaAndDeltaVelocityConfigurationRegister VnSensor::readDeltaThetaAndDeltaVelocityConfiguration()
{
    return {};
}

void VnSensor::writeDeltaThetaAndDeltaVelocityConfiguration(DeltaThetaAndDeltaVelocityConfigurationRegister& /* fields */, bool /* waitForReply */)
{
}

void VnSensor::writeDeltaThetaAndDeltaVelocityConfiguration(
    IntegrationFrame /* integrationFrame */,
    CompensationMode /* gyroCompensation */,
    AccCompensationMode /* accelCompensation */,
    bool /* waitForReply */)
{
}

void VnSensor::writeDeltaThetaAndDeltaVelocityConfiguration(
    IntegrationFrame /* integrationFrame */,
    CompensationMode /* gyroCompensation */,
    AccCompensationMode /* accelCompensation */,
    EarthRateCorrection /* earthRateCorrection */,
    bool /* waitForReply */)
{
}

ReferenceVectorConfigurationRegister VnSensor::readReferenceVectorConfiguration()
{
    return {};
}

void VnSensor::writeReferenceVectorConfiguration(ReferenceVectorConfigurationRegister& /* fields */, bool /* waitForReply */)
{
}

void VnSensor::writeReferenceVectorConfiguration(
    const uint8_t& /* useMagModel */,
    const uint8_t& /* useGravityModel */,
    const uint32_t& /* recalcThreshold */,
    const float& /* year */,
    const vec3d& /* position */,
    bool /* waitForReply */)
{
}

GyroCompensationRegister VnSensor::readGyroCompensation()
{
    return {};
}

void VnSensor::writeGyroCompensation(GyroCompensationRegister& /* fields */, bool /* waitForReply */)
{
}

void VnSensor::writeGyroCompensation(
    const mat3f& /* c */,
    const vec3f& /* b */,
    bool /* waitForReply */)
{
}

ImuFilteringConfigurationRegister VnSensor::readImuFilteringConfiguration()
{
    return {};
}

void VnSensor::writeImuFilteringConfiguration(ImuFilteringConfigurationRegister& /* fields */, bool /* waitForReply */)
{
}

void VnSensor::writeImuFilteringConfiguration(
    const uint16_t& /* magWindowSize */,
    const uint16_t& /* accelWindowSize */,
    const uint16_t& /* gyroWindowSize */,
    const uint16_t& /* tempWindowSize */,
    const uint16_t& /* presWindowSize */,
    FilterMode /* magFilterMode */,
    FilterMode /* accelFilterMode */,
    FilterMode /* gyroFilterMode */,
    FilterMode /* tempFilterMode */,
    FilterMode /* presFilterMode */,
    bool /* waitForReply */)
{
}

GpsCompassBaselineRegister VnSensor::readGpsCompassBaseline()
{
    return {};
}

void VnSensor::writeGpsCompassBaseline(GpsCompassBaselineRegister& /* fields */, bool /* waitForReply */)
{
}

void VnSensor::writeGpsCompassBaseline(
    const vec3f& /* position */,
    const vec3f& /* uncertainty */,
    bool /* waitForReply */)
{
}

GpsCompassEstimatedBaselineRegister VnSensor::readGpsCompassEstimatedBaseline()
{
    return {};
}

ImuRateConfigurationRegister VnSensor::readImuRateConfiguration()
{
    return {};
}

void VnSensor::writeImuRateConfiguration(ImuRateConfigurationRegister& /* fields */, bool /* waitForReply */)
{
}

void VnSensor::writeImuRateConfiguration(
    const uint16_t& /* imuRate */,
    const uint16_t& /* navDivisor */,
    const float& /* filterTargetRate */,
    const float& /* filterMinRate */,
    bool /* waitForReply */)
{
}

YawPitchRollTrueBodyAccelerationAndAngularRatesRegister VnSensor::readYawPitchRollTrueBodyAccelerationAndAngularRates()
{
    return {};
}

YawPitchRollTrueInertialAccelerationAndAngularRatesRegister VnSensor::readYawPitchRollTrueInertialAccelerationAndAngularRates()
{
    return {};
}

} // namespace sensors
} // namespace vn

// NOLINTEND

#endif