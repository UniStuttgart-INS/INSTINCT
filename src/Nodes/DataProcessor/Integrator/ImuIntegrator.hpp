// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file ImuIntegrator.hpp
/// @brief Integrates ImuObs Data
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @author M. Maier (marcel.maier@ins.uni-stuttgart.de)
/// @date 2020-05-18

#pragma once

#include "internal/Node/Node.hpp"

#include "NodeData/IMU/ImuObs.hpp"
#include "NodeData/State/PosVelAtt.hpp"
#include "NodeData/State/LcKfInsGnssErrors.hpp"

#include "Navigation/Gravity/Gravity.hpp"
#include "Navigation/Math/NumericalIntegration.hpp"

namespace NAV
{
/// @brief Numerically integrates Imu data
class ImuIntegrator : public Node
{
  public:
    /// @brief Default constructor
    ImuIntegrator();
    /// @brief Destructor
    ~ImuIntegrator() override;
    /// @brief Copy constructor
    ImuIntegrator(const ImuIntegrator&) = delete;
    /// @brief Move constructor
    ImuIntegrator(ImuIntegrator&&) = delete;
    /// @brief Copy assignment operator
    ImuIntegrator& operator=(const ImuIntegrator&) = delete;
    /// @brief Move assignment operator
    ImuIntegrator& operator=(ImuIntegrator&&) = delete;

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
    constexpr static size_t OUTPUT_PORT_INDEX_INERTIAL_NAV_SOL = 0; ///< @brief Flow (InertialNavSol)
    constexpr static size_t INPUT_PORT_INDEX_IMU_OBS = 0;           ///< @brief Flow (ImuObs)
    constexpr static size_t INPUT_PORT_INDEX_POS_VEL_ATT_INIT = 1;  ///< @brief Flow (PosVelAtt)
    constexpr static size_t INPUT_PORT_INDEX_PVA_ERROR = 2;         ///< @brief Flow (LcKfInsGnssErrors)
    constexpr static size_t INPUT_PORT_INDEX_SYNC = 3;              ///< @brief Flow (NodeData)

    /// @brief Initialize the node
    bool initialize() override;

    /// @brief Deinitialize the node
    void deinitialize() override;

    /// @brief Adds/Deletes Input Pins depending on the variable _showCorrectionsInputPin
    void updateNumberOfInputPins();

    /// @brief Receive Function for the PosVelAtt initial values
    /// @param[in] queue Queue with all the received data messages
    /// @param[in] pinIdx Index of the pin the data is received on
    void recvPosVelAttInit(InputPin::NodeDataQueue& queue, size_t pinIdx);

    /// @brief Receive Function for the ImuObs at the time tₖ
    /// @param[in] queue Queue with all the received data messages
    /// @param[in] pinIdx Index of the pin the data is received on
    void recvImuObs(InputPin::NodeDataQueue& queue, size_t pinIdx);

    /// @brief Receive function for LcKfInsGnssErrors
    /// @param[in] queue Queue with all the received data messages
    /// @param[in] pinIdx Index of the pin the data is received on
    void recvLcKfInsGnssErrors(InputPin::NodeDataQueue& queue, size_t pinIdx);

    /// @brief Receive function for sync message
    /// @param[in] queue Queue with all the received data messages
    /// @param[in] pinIdx Index of the pin the data is received on
    void recvSync(InputPin::NodeDataQueue& queue, size_t pinIdx);

    /// @brief Integrates the Imu Observation data
    void integrateObservation();

    /// @brief Integrates the Imu Observation data in ECEF frame
    std::shared_ptr<const PosVelAtt> integrateObservationECEF();

    /// @brief Integrates the Imu Observation data in NED frame
    std::shared_ptr<const PosVelAtt> integrateObservationNED();

    // #########################################################################################################################################

    /// IMU Observation list
    /// Length depends on the integration algorithm. Newest observation first (tₖ, tₖ₋₁, tₖ₋₂, ...)
    std::deque<std::shared_ptr<const ImuObs>> _imuObservations;

    /// @brief Maximum amount of imu observations to keep
    size_t _maxSizeImuObservations = 0;

    /// Position, Velocity and Attitude states.
    /// Length depends on the integration algorithm. Newest state first (tₖ, tₖ₋₁, tₖ₋₂, ...)
    std::deque<std::shared_ptr<const PosVelAtt>> _posVelAttStates;

    /// @brief Maximum amount of states to keep
    size_t _maxSizeStates = 0;

    /// Time at initialization (needed to set time tag when TimeSinceStartup is used)
    InsTime _time__init;
    /// TimeSinceStartup at initialization (needed to set time tag when TimeSinceStartup is used)
    uint64_t _timeSinceStartup__init = 0;

    // #########################################################################################################################################

    /// @brief Available Integration Frames
    enum class IntegrationFrame : int
    {
        ECEF, ///< Earth-Centered Earth-Fixed frame
        NED,  ///< Local North-East-Down frame
    };
    /// Frame to integrate the observations in
    IntegrationFrame _integrationFrame = IntegrationFrame::NED;

    /// @brief Integration algorithm used for the update
    IntegrationAlgorithm _integrationAlgorithm = IntegrationAlgorithm::Heun;

    // #########################################################################################################################################

    /// Flag, whether the integrator should take the time from the IMU clock instead of the insTime
    bool _prefereTimeSinceStartupOverInsTime = false;

    /// Flag to let the integration algorithm use uncompensated acceleration and angular rates instead of compensated
    bool _prefereUncompensatedData = false;

    // #########################################################################################################################################

    /// @brief Gravity model selected in the GUI
    GravitationModel _gravitationModel = GravitationModel::EGM96;

    /// Apply the coriolis acceleration compensation to the measured accelerations
    bool _coriolisAccelerationCompensationEnabled = true;

    /// Apply the centrifugal acceleration compensation to the measured accelerations
    bool _centrifgalAccelerationCompensationEnabled = true;

    /// Apply the Earth rotation rate compensation to the measured angular rates
    bool _angularRateEarthRotationCompensationEnabled = true;

    /// Apply the transport rate compensation to the measured angular rates
    bool _angularRateTransportRateCompensationEnabled = true;

    // #########################################################################################################################################

    /// GUI flag, whether to show the input pin for PVA Corrections
    bool _showCorrectionsInputPin = false;

    /// Accumulated IMU biases
    std::shared_ptr<const LcKfInsGnssErrors> _lckfErrors = nullptr;
};

} // namespace NAV
