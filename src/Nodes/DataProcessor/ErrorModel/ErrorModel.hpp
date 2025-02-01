// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file ErrorModel.hpp
/// @brief Adds errors (biases and noise) to measurements
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2021-12-21

#pragma once

#include "internal/Node/Node.hpp"

#include "NodeData/GNSS/GnssObs.hpp"
#include "NodeData/IMU/ImuObs.hpp"
#include "NodeData/IMU/ImuObsWDelta.hpp"
#include "NodeData/State/PosVelAtt.hpp"
#include "Navigation/INS/Units.hpp"

#include "util/Random/RandomNumberGenerator.hpp"

#include "util/Eigen.hpp"
#include <random>
#include <map>

namespace NAV
{
/// Adds errors (biases and noise) to measurements
class ErrorModel : public Node
{
  public:
    /// @brief Default constructor
    ErrorModel();
    /// @brief Destructor
    ~ErrorModel() override;
    /// @brief Copy constructor
    ErrorModel(const ErrorModel&) = delete;
    /// @brief Move constructor
    ErrorModel(ErrorModel&&) = delete;
    /// @brief Copy assignment operator
    ErrorModel& operator=(const ErrorModel&) = delete;
    /// @brief Move assignment operator
    ErrorModel& operator=(ErrorModel&&) = delete;

    /// @brief String representation of the Class Type
    [[nodiscard]] static std::string typeStatic();

    /// @brief String representation of the Class Type
    [[nodiscard]] std::string type() const override;

    /// @brief String representation of the Class Category
    [[nodiscard]] static std::string category();

    /// @brief ImGui config window which is shown on double click
    /// @attention Don't forget to set _hasConfig to true in the constructor of the node
    void guiConfig() override;

    /// @brief Saves the node into a json object
    [[nodiscard]] json save() const override;

    /// @brief Restores the node from a json object
    /// @param[in] j Json object with the node state
    void restore(const json& j) override;

  private:
    constexpr static size_t OUTPUT_PORT_INDEX_FLOW = 0; ///< @brief Flow
    constexpr static size_t INPUT_PORT_INDEX_FLOW = 0;  ///< @brief Flow

    /// Input type
    enum class InputType : uint8_t
    {
        None,         ///< None
        ImuObs,       ///< ImuObs
        ImuObsWDelta, ///< ImuObsWDelta
        PosVelAtt,    ///< PosVelAtt
        GnssObs,      ///< GnssObs
    };

    /// Input type
    InputType _inputType = InputType::None;

    /// @brief Resets the node. It is guaranteed that the node is initialized when this is called.
    bool resetNode() override;

    /// @brief Called when a new link was established
    /// @param[in] startPin Pin where the link starts
    /// @param[in] endPin Pin where the link ends
    void afterCreateLink(OutputPin& startPin, InputPin& endPin) override;

    /// @brief Called when a link was deleted
    /// @param[in] startPin Pin where the link starts
    /// @param[in] endPin Pin where the link ends
    void afterDeleteLink(OutputPin& startPin, InputPin& endPin) override;

    /// @brief Callback when receiving data on a port
    /// @param[in] queue Queue with all the received data messages
    /// @param[in] pinIdx Index of the pin the data is received on
    void receiveObs(InputPin::NodeDataQueue& queue, size_t pinIdx);

    /// @brief Callback when receiving an ImuObs
    /// @param[in] imuObs Copied data to modify and send out again
    /// @param[in] accelerometerBias_p Accelerometer Bias in platform frame coordinates [m/s^2]
    /// @param[in] gyroscopeBias_p Gyroscope Bias in platform frame coordinates [rad/s]
    /// @param[in] accelerometerNoiseStd Accelerometer Noise standard deviation in platform frame coordinates [m/s^2]
    /// @param[in] gyroscopeNoiseStd Gyroscope Noise standard deviation in platform frame coordinates [rad/s]
    std::shared_ptr<ImuObs> receiveImuObs(const std::shared_ptr<ImuObs>& imuObs,
                                          const Eigen::Vector3d& accelerometerBias_p,
                                          const Eigen::Vector3d& gyroscopeBias_p,
                                          const Eigen::Vector3d& accelerometerNoiseStd,
                                          const Eigen::Vector3d& gyroscopeNoiseStd);

    /// @brief Callback when receiving an ImuObsWDelta
    /// @param[in] imuObs Copied data to modify and send out again
    /// @param[in] accelerometerBias_p Accelerometer Bias in platform frame coordinates [m/s^2]
    /// @param[in] gyroscopeBias_p Gyroscope Bias in platform frame coordinates [rad/s]
    /// @param[in] accelerometerNoiseStd Accelerometer Noise standard deviation in platform frame coordinates [m/s^2]
    /// @param[in] gyroscopeNoiseStd Gyroscope Noise standard deviation in platform frame coordinates [rad/s]
    [[nodiscard]] std::shared_ptr<ImuObsWDelta> receiveImuObsWDelta(const std::shared_ptr<ImuObsWDelta>& imuObs,
                                                                    const Eigen::Vector3d& accelerometerBias_p,
                                                                    const Eigen::Vector3d& gyroscopeBias_p,
                                                                    const Eigen::Vector3d& accelerometerNoiseStd,
                                                                    const Eigen::Vector3d& gyroscopeNoiseStd);

    /// @brief Callback when receiving an ImuObs
    /// @param[in] posVelAtt Copied data to modify and send out again
    [[nodiscard]] std::shared_ptr<PosVelAtt> receivePosVelAtt(const std::shared_ptr<PosVelAtt>& posVelAtt);

    /// @brief Callback when receiving an GnssObs
    /// @param[in] gnssObs Copied data to modify and send out again
    [[nodiscard]] std::shared_ptr<GnssObs> receiveGnssObs(const std::shared_ptr<GnssObs>& gnssObs);

    /// Last observation time
    InsTime _lastObservationTime;
    /// Time interval of the messages [s]
    double _dt = 0.0;

    // #########################################################################################################################################
    //                                                                 ImuObs
    // #########################################################################################################################################
    //
    /// 3D array which allow to accumulate RW noise for accelerometer
    Eigen::Vector3d _randomWalkAccelerometer = Eigen::Vector3d::Zero();

    /// 3D array which allow to accumulate RW noise for gyro
    Eigen::Vector3d _randomWalkGyroscope = Eigen::Vector3d::Zero();

    /// 3D array which allow to accumulate IRW veloctiy noise for accelerometer
    Eigen::Vector3d _integratedRandomWalkAccelerometer_velocity = Eigen::Vector3d::Zero();

    /// 3D array which allow to accumulate IRW for accelerometer
    Eigen::Vector3d _integratedRandomWalkAccelerometer = Eigen::Vector3d::Zero();

    /// 3D array which allow to accumulate IRW veloctiy noise for gyro
    Eigen::Vector3d _integratedRandomWalkGyroscope_velocity = Eigen::Vector3d::Zero();

    /// 3D array which allow to accumulate IRW for gyro
    Eigen::Vector3d _integratedRandomWalkGyroscope = Eigen::Vector3d::Zero();

    // --------------------------------------------------------------- Offset ------------------------------------------------------------------

    /// Selected unit for the accelerometer bias in the GUI
    Units::ImuAccelerometerUnits _imuAccelerometerBiasUnit = Units::ImuAccelerometerUnits::m_s2;
    /// Bias of the accelerometer in platform coordinates (Unit as selected)
    Eigen::Vector3d _imuAccelerometerBias_p = Eigen::Vector3d::Zero();

    /// Selected unit for the gyroscope bias in the GUI
    Units::ImuGyroscopeUnits _imuGyroscopeBiasUnit = Units::ImuGyroscopeUnits::rad_s;
    /// Bias of the gyroscope in platform coordinates (Unit as selected)
    Eigen::Vector3d _imuGyroscopeBias_p = Eigen::Vector3d::Zero();

    // ---------------------------------------------------------------- Noise ------------------------------------------------------------------

    /// Selected unit for the accelerometer noise in the GUI
    Units::ImuAccelerometerNoiseUnits _imuAccelerometerNoiseUnit = Units::ImuAccelerometerNoiseUnits::m_s2_sqrts;
    /// Noise of the accelerometer (Unit as selected)
    Eigen::Vector3d _imuAccelerometerNoise = Eigen::Vector3d::Zero();
    /// Random number generator for the accelerometer noise
    RandomNumberGenerator _imuAccelerometerRng;

    /// Selected unit for the gyroscope noise in the GUI
    Units::ImuGyroscopeNoiseUnits _imuGyroscopeNoiseUnit = Units::ImuGyroscopeNoiseUnits::rad_s_sqrts;
    /// Noise of the gyroscope (Unit as selected)
    Eigen::Vector3d _imuGyroscopeNoise = Eigen::Vector3d::Zero();
    /// Random number generator for the gyroscope noise
    RandomNumberGenerator _imuGyroscopeRng;

    /// Selected unit for the accelerometer RW noise in the GUI
    Units::ImuAccelerometerNoiseUnits _imuAccelerometerRWUnit = Units::ImuAccelerometerNoiseUnits::m_s2_sqrts;
    /// RW noise of the accelerometer (Unit as selected)
    Eigen::Vector3d _imuAccelerometerRW = Eigen::Vector3d::Zero();
    /// Random number generator for the accelerometer RW noise
    RandomNumberGenerator _imuAccelerometerRWRng;

    /// Selected unit for the accelerometer RW noise in the GUI
    Units::ImuGyroscopeNoiseUnits _imuGyroscopeRWUnit = Units::ImuGyroscopeNoiseUnits::rad_s_sqrts;
    /// RW noise of the accelerometer (Unit as selected)
    Eigen::Vector3d _imuGyroscopeRW = Eigen::Vector3d::Zero();
    /// Random number generator for the accelerometer RW noise
    RandomNumberGenerator _imuGyroscopeRWRng;

    /// Selected unit for the accelerometer IRW noise in the GUI
    Units::ImuAccelerometerIRWUnits _imuAccelerometerIRWUnit = Units::ImuAccelerometerIRWUnits::m_s3_sqrts;
    /// IRW noise of the accelerometer (Unit as selected)
    Eigen::Vector3d _imuAccelerometerIRW = Eigen::Vector3d::Zero();
    /// Random number generator for the accelerometer IRW noise
    RandomNumberGenerator _imuAccelerometerIRWRng;

    /// Selected unit for the accelerometer IRW noise in the GUI
    Units::ImuGyroscopeIRWUnits _imuGyroscopeIRWUnit = Units::ImuGyroscopeIRWUnits::rad_s2_sqrts;
    /// RW noise of the accelerometer (Unit as selected)
    Eigen::Vector3d _imuGyroscopeIRW = Eigen::Vector3d::Zero();
    /// Random number generator for the accelerometer RW noise
    RandomNumberGenerator _imuGyroscopeIRWRng;

    // #########################################################################################################################################
    //                                                                PosVelAtt
    // #########################################################################################################################################

    // --------------------------------------------------------------- Offset ------------------------------------------------------------------

    /// Possible units to specify a position bias with
    enum class PositionBiasUnits : uint8_t
    {
        meter,     ///< NED [m m m]
        rad_rad_m, ///< LatLonAlt [rad, rad, m]
        deg_deg_m, ///< LatLonAlt [deg, deg, m]
    };
    /// Selected unit for the position bias in the GUI
    PositionBiasUnits _positionBiasUnit = PositionBiasUnits::meter;
    /// Bias of the position (Unit as selected)
    Eigen::Vector3d _positionBias = Eigen::Vector3d::Zero();

    /// Possible units to specify an velocity bias with
    enum class VelocityBiasUnits : uint8_t
    {
        m_s, ///< [m/s]
    };
    /// Selected unit for the velocity bias in the GUI
    VelocityBiasUnits _velocityBiasUnit = VelocityBiasUnits::m_s;
    /// Bias of the velocity (Unit as selected)
    Eigen::Vector3d _velocityBias = Eigen::Vector3d::Zero();

    /// Possible units to specify a attitude bias with
    enum class AttitudeBiasUnits : uint8_t
    {
        rad, ///< [rad]
        deg, ///< [deg]
    };
    /// Selected unit for the attitude bias in the GUI
    AttitudeBiasUnits _attitudeBiasUnit = AttitudeBiasUnits::deg;
    /// Bias of the attitude (Unit as selected)
    Eigen::Vector3d _attitudeBias = Eigen::Vector3d::Zero();

    // ---------------------------------------------------------------- Noise ------------------------------------------------------------------

    /// Possible units to specify a position noise with
    enum class PositionNoiseUnits : uint8_t
    {
        meter,        ///< NED [m m m] (Standard deviation)
        rad_rad_m,    ///< LatLonAlt [rad, rad, m] (Standard deviation)
        deg_deg_m,    ///< LatLonAlt [deg, deg, m] (Standard deviation)
        meter2,       ///< NED [m^2 m^2 m^2] (Variance)
        rad2_rad2_m2, ///< LatLonAlt [rad^2, rad^2, m^2] (Variance)
        deg2_deg2_m2, ///< LatLonAlt [deg^2, deg^2, m^2] (Variance)
    };

    /// Selected unit for the position noise in the GUI
    PositionNoiseUnits _positionNoiseUnit = PositionNoiseUnits::meter;
    /// Noise of the position (Unit as selected)
    Eigen::Vector3d _positionNoise = Eigen::Vector3d::Zero();
    /// Random number generator for the position noise
    RandomNumberGenerator _positionRng;
    /// Possible units to specify an velocity noise with
    enum class VelocityNoiseUnits : uint8_t
    {
        m_s,   ///< [m/s] (Standard deviation)
        m2_s2, ///< [m^2/s^2] (Variance)
    };
    /// Selected unit for the velocity noise in the GUI
    VelocityNoiseUnits _velocityNoiseUnit = VelocityNoiseUnits::m_s;
    /// Noise of the velocity (Unit as selected)
    Eigen::Vector3d _velocityNoise = Eigen::Vector3d::Zero();
    /// Random number generator for the velocity noise
    RandomNumberGenerator _velocityRng;

    /// Possible units to specify a attitude noise with
    enum class AttitudeNoiseUnits : uint8_t
    {
        rad,  ///< [rad] (Standard deviation)
        deg,  ///< [deg] (Standard deviation)
        rad2, ///< [rad^2] (Variance)
        deg2, ///< [deg^2] (Variance)
    };
    /// Selected unit for the attitude noise in the GUI
    AttitudeNoiseUnits _attitudeNoiseUnit = AttitudeNoiseUnits::deg;
    /// Noise of the attitude (Unit as selected)
    Eigen::Vector3d _attitudeNoise = Eigen::Vector3d::Zero();
    /// Random number generator for the attitude noise
    RandomNumberGenerator _attitudeRng;

    // #########################################################################################################################################
    //                                                                GnssObs
    // #########################################################################################################################################

    // ---------------------------------------------------------------- Noise ------------------------------------------------------------------

    /// Possible units to specify a pseudorange noise with
    enum class PseudorangeNoiseUnits : uint8_t
    {
        meter, ///< [m] (Standard deviation)
    };
    /// Selected unit for the pseudorange noise in the GUI
    PseudorangeNoiseUnits _gui_pseudorangeNoiseUnit = PseudorangeNoiseUnits::meter;
    /// Noise of the pseudorange (Unit as selected)
    double _gui_pseudorangeNoise{ 0.3 };
    /// Random number generator for the pseudorange noise
    RandomNumberGenerator _pseudorangeRng;

    /// Possible units to specify a carrier-phase noise with
    enum class CarrierPhaseNoiseUnits : uint8_t
    {
        meter, ///< [m] (Standard deviation)
    };
    /// Selected unit for the carrier-phase noise in the GUI
    CarrierPhaseNoiseUnits _gui_carrierPhaseNoiseUnit = CarrierPhaseNoiseUnits::meter;
    /// Noise of the carrier-phase (Unit as selected)
    double _gui_carrierPhaseNoise{ 0.003 };
    /// Random number generator for the carrier-phase noise
    RandomNumberGenerator _carrierPhaseRng;

    /// Possible units to specify a range-rate noise with
    enum class DopplerNoiseUnits : uint8_t
    {
        m_s, ///< [m/s] (Standard deviation)
    };
    /// Selected unit for the range-rate noise in the GUI
    DopplerNoiseUnits _gui_dopplerNoiseUnit = DopplerNoiseUnits::m_s;
    /// Noise of the range-rate (Unit as selected)
    double _gui_dopplerNoise{ 0.05 };
    /// Random number generator for the range-rate noise
    RandomNumberGenerator _dopplerRng;

    // -------------------------------------------------------------- Ambiguity ----------------------------------------------------------------

    /// Ambiguity limits
    std::array<int, 2> _gui_ambiguityLimits = { { -20, 20 } };
    /// Random number generator for the ambiguity
    RandomNumberGenerator _ambiguityRng;
    /// Ambiguity map
    std::map<SatSigId, std::vector<std::pair<InsTime, int>>> _ambiguities;

    // ------------------------------------------------------------- Cycle-slip ----------------------------------------------------------------

    /// Possible units to specify the cycle-slip rate with
    enum class CycleSlipFrequencyUnits : uint8_t
    {
        per_day,    ///< [1/d]
        per_hour,   ///< [1/h]
        per_minute, ///< [1/m]
    };
    /// Selected unit for the cycle-slip frequency in the GUI
    CycleSlipFrequencyUnits _gui_cycleSlipFrequencyUnit = CycleSlipFrequencyUnits::per_hour;
    /// The cycle-slip frequency (Unit as selected)
    double _gui_cycleSlipFrequency{ 0.0 };
    /// Ambiguity limits cycle-slip
    int _gui_cycleSlipRange = 20;
    /// The time frame which is considered for a cycle slip
    InsTime _cycleSlipWindowStartTime;

    /// Possible units to specify the cycle-slip detection probability with
    enum class CycleSlipDetectionProbabilityUnits : uint8_t
    {
        percent, ///< [%]
    };
    /// Selected unit for the cycle-slip detection probability in the GUI
    CycleSlipDetectionProbabilityUnits _gui_cycleSlipDetectionProbabilityUnit = CycleSlipDetectionProbabilityUnits::percent;
    /// The chance to detect a cycle slip and set the Loss-of-Lock indicator
    double _gui_cycleSlipDetectionProbability{ 100.0 };

    /// Random number generator for the cycle-slip
    RandomNumberGenerator _cycleSlipRng;

    /// Cycle-slip information
    struct CycleSlipInfo
    {
        InsTime time;      ///< Time of the cycle-slip
        SatSigId satSigId; ///< Satellite Signal identifier
        bool LLI;          ///< Whether the LLI was set
    };
    /// List of produced cycle-slips
    std::vector<CycleSlipInfo> _cycleSlips;

    /// Frequencies used for calculation (GUI filter)
    Frequency _filterFreq = G01;
    /// Codes used for calculation (GUI filter)
    Code _filterCode = Code_Default;
};

} // namespace NAV
