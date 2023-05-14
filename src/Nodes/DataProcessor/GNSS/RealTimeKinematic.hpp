// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file RealTimeKinematic.hpp
/// @brief Real-Time Kinematic (RTK) carrier-phase DGNSS
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2023-04-13

#pragma once

#include "util/Eigen.hpp"
#include <vector>

#include "internal/Node/Node.hpp"
#include "Navigation/GNSS/Core/Frequency.hpp"
#include "Navigation/GNSS/Core/Code.hpp"
#include "Navigation/GNSS/Core/SatelliteIdentifier.hpp"
#include "Navigation/GNSS/Core/ReceiverClock.hpp"
#include "Navigation/Atmosphere/Ionosphere/Ionosphere.hpp"
#include "Navigation/Atmosphere/Troposphere/Troposphere.hpp"
#include "Navigation/Transformations/Units.hpp"
#include "Navigation/Math/KalmanFilter.hpp"

#include "NodeData/GNSS/GnssObs.hpp"

namespace NAV
{
/// @brief Numerically integrates Imu data
class RealTimeKinematic : public Node
{
  public:
    /// @brief Default constructor
    RealTimeKinematic();
    /// @brief Destructor
    ~RealTimeKinematic() override;
    /// @brief Copy constructor
    RealTimeKinematic(const RealTimeKinematic&) = delete;
    /// @brief Move constructor
    RealTimeKinematic(RealTimeKinematic&&) = delete;
    /// @brief Copy assignment operator
    RealTimeKinematic& operator=(const RealTimeKinematic&) = delete;
    /// @brief Move assignment operator
    RealTimeKinematic& operator=(RealTimeKinematic&&) = delete;

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
    constexpr static size_t INPUT_PORT_INDEX_BASE_POS = 0;       ///< @brief Pos
    constexpr static size_t INPUT_PORT_INDEX_BASE_GNSS_OBS = 1;  ///< @brief GnssObs
    constexpr static size_t INPUT_PORT_INDEX_ROVER_GNSS_OBS = 2; ///< @brief GnssObs
    constexpr static size_t INPUT_PORT_INDEX_GNSS_NAV_INFO = 3;  ///< @brief GnssNavInfo

    constexpr static size_t OUTPUT_PORT_INDEX_RTKSOL = 0; ///< @brief Flow (RtkSol)

    // --------------------------------------------------------------- Gui -----------------------------------------------------------------

    /// @brief Initialize the node
    bool initialize() override;

    /// @brief Deinitialize the node
    void deinitialize() override;

    /// Index of the Pin currently being dragged
    int _dragAndDropPinIndex = -1;
    /// Amount of NavInfo input pins
    size_t _nNavInfoPins = 1;
    /// @brief Adds/Deletes Input Pins depending on the variable _nNavInfoPins
    void updateNumberOfInputPins();

    /// Frequencies used for calculation (GUI filter)
    Frequency _filterFreq = G01;
    /// Codes used for calculation (GUI filter)
    Code _filterCode = Code::G1C | Code::G2C | Code_G5I_G5Q_G5X
                       | Code_E1B_E1C_E1X | Code_E5I_E5Q_E5X | Code_E6B_E6C_E6X | Code_E7I_E7Q_E7X | Code_E8I_E8Q_E8X
                       | Code::R1C | Code::R2C | Code_R3I_R3Q_R3X | Code_R4A_R4B_R4X | Code_R6A_R6B_R6X
                       | Code_B1D_B1P_B1X | Code_B2I_B2Q_B2X | Code_B5D_B5P_B5X | Code_B6I_B6Q_B6X | Code_B7I_B7Q_B7X | Code_B8D_B8P_B8X
                       | Code::J1C | Code_J2S_J2L_J2X | Code_J5I_J5Q_J5X | Code_J6S_J6L_J6X
                       | Code::I5A | Code::I9A
                       | Code::S1C | Code_S5I_S5Q_S5X;
    /// List of satellites to exclude
    std::vector<SatId> _excludedSatellites;
    /// Elevation cut-off angle for satellites in [rad]
    double _elevationMask = static_cast<double>(15.0_deg);

    /// Ionosphere Model used for the calculation
    IonosphereModel _ionosphereModel = IonosphereModel::Klobuchar;

    /// Troposphere Models used for the calculation
    TroposphereModelSelection _troposphereModels;

    // #####################################################################################

    /// Possible Units for the Standard deviation of the acceleration due to user motion
    enum class StdevAccelUnits
    {
        m_sqrts3, ///< [ m / √(s^3) ]
    };
    /// Gui selection for the Unit of the input stdev_accel parameter for the StDev due to acceleration due to user motion
    StdevAccelUnits _stdevAccelUnits = StdevAccelUnits::m_sqrts3;

    /// @brief 𝜎_a Standard deviation of the acceleration due to user motion in horizontal and vertical component
    /// @note See Groves (2013) eq. (9.156)
    std::array<double, 2> _stdev_accel = { { 3.0, 1.5 } } /* [ m / √(s^3) ] */;

    /// Possible Units for the Standard deviation of the receiver clock phase drift
    enum class StdevClockPhaseUnits
    {
        m_sqrtHz, ///< [m / √(Hz)]
    };
    /// Gui selection for the Unit of the input stdev_cphi parameter
    StdevClockPhaseUnits _stdevClockPhaseUnits = StdevClockPhaseUnits::m_sqrtHz;

    /// @brief 𝜎_c𝛟 Standard deviation of the receiver clock phase drift
    /// @note See Groves (2013) eq. (9.153)
    double _stdev_cphi = 0.0 /* [m / √(Hz)] */;

    /// Possible Units for the Standard deviation of the receiver clock frequency drift
    enum class StdevClockFreqUnits
    {
        m_s2_sqrtHz, ///< [m / s^2 / √(Hz)]
    };
    /// Gui selection for the Unit of the input stdev_cf parameter
    StdevClockFreqUnits _stdevClockFreqUnits = StdevClockFreqUnits::m_s2_sqrtHz;

    /// @brief 𝜎_cf Standard deviation of the receiver clock frequency drift
    /// @note See Brown (2012) table 9.2
    double _stdev_cf = 5.0 /* [m / s^2 / √(Hz)] */;

    // ------------------------------------------------------------ Algorithm --------------------------------------------------------------

    /// Base position in ECEF frame [m]
    Eigen::Vector3d _e_basePosition = Eigen::Vector3d::Zero();

    /// Estimated position in ECEF frame [m]
    Eigen::Vector3d _e_position = Eigen::Vector3d::Zero();
    /// Estimated velocity in ECEF frame [m/s]
    Eigen::Vector3d _e_velocity = Eigen::Vector3d::Zero();
    /// Estimated receiver clock parameters
    ReceiverClock _recvClk;

    /// Kalman Filter representation
    KalmanFilter _kalmanFilter{ 8, 8 };

    /// Latest GNSS Observation from the base station
    std::shared_ptr<const GnssObs> _gnssObsBase = nullptr;
    /// Latest GNSS Observation from the rover
    std::shared_ptr<const GnssObs> _gnssObsRover = nullptr;

    /// @brief Receive Function for the Base Position
    /// @param[in] queue Queue with all the received data messages
    /// @param[in] pinIdx Index of the pin the data is received on
    void recvBasePos(InputPin::NodeDataQueue& queue, size_t pinIdx);

    /// @brief Receive Function for the Base GNSS Observations
    /// @param[in] queue Queue with all the received data messages
    /// @param[in] pinIdx Index of the pin the data is received on
    void recvBaseGnssObs(InputPin::NodeDataQueue& queue, size_t pinIdx);

    /// @brief Receive Function for the RoverGNSS Observations
    /// @param[in] queue Queue with all the received data messages
    /// @param[in] pinIdx Index of the pin the data is received on
    void recvRoverGnssObs(InputPin::NodeDataQueue& queue, size_t pinIdx);

    /// @brief Calculates the RTK solution
    void calcRealTimeKinematicSolution();

    /// @brief Calculates a SPP solution as fallback in case no base data is available
    void calcFallbackSppSolution();
};

} // namespace NAV