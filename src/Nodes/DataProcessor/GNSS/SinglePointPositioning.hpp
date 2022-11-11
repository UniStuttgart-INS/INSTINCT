// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file SinglePointPositioning.hpp
/// @brief Single Point Positioning (SPP) / Code Phase Positioning
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2022-04-29

#pragma once

#include <Eigen/Core>
#include <vector>

#include "internal/Node/Node.hpp"
#include "Navigation/GNSS/Core/Frequency.hpp"
#include "Navigation/GNSS/Core/Code.hpp"
#include "Navigation/GNSS/Core/SatelliteIdentifier.hpp"
#include "Navigation/Atmosphere/Ionosphere/Ionosphere.hpp"
#include "Navigation/Atmosphere/Troposphere/Troposphere.hpp"
#include "Navigation/Transformations/Units.hpp"

namespace NAV
{
/// @brief Numerically integrates Imu data
class SinglePointPositioning : public Node
{
  public:
    /// @brief Default constructor
    SinglePointPositioning();
    /// @brief Destructor
    ~SinglePointPositioning() override;
    /// @brief Copy constructor
    SinglePointPositioning(const SinglePointPositioning&) = delete;
    /// @brief Move constructor
    SinglePointPositioning(SinglePointPositioning&&) = delete;
    /// @brief Copy assignment operator
    SinglePointPositioning& operator=(const SinglePointPositioning&) = delete;
    /// @brief Move assignment operator
    SinglePointPositioning& operator=(SinglePointPositioning&&) = delete;

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
    constexpr static size_t INPUT_PORT_INDEX_POSVEL_INIT = 0;   ///< @brief PosVel
    constexpr static size_t INPUT_PORT_INDEX_GNSS_OBS = 1;      ///< @brief GnssObs
    constexpr static size_t INPUT_PORT_INDEX_GNSS_NAV_INFO = 2; ///< @brief GnssNavInfo

    constexpr static size_t OUTPUT_PORT_INDEX_SPPSOL = 0; ///< @brief Flow (PosVel)

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

    /// Troposphere Model used for the calculation
    TroposphereModel _troposphereModel = TroposphereModel::Saastamoinen;
    /// Mapping function for the zenith hydrostatic delay
    MappingFunction _zhdMappingFunction = MappingFunction::Cosecant;
    /// Mapping function for the zenith wet delay
    MappingFunction _zwdMappingFunction = MappingFunction::Cosecant;

    /// Use the weighted least squares algorithm
    bool _useWeightedLeastSquares = true;

    // ------------------------------------------------------------ Algorithm --------------------------------------------------------------

    /// Estimated position in ECEF frame [m]
    Eigen::Vector3d _e_position = Eigen::Vector3d::Zero();
    /// Estimated receiver clock bias [s]
    double _clkBias{ 0.0 };
    /// Estimated velocity in ECEF frame [m/s]
    Eigen::Vector3d _e_velocity = Eigen::Vector3d::Zero();
    /// Estimated receiver clock drift [s/s]
    double _clkDrift{ 0.0 };

    /// @brief Receive Function for the Gnss Observations
    /// @param[in] queue Queue with all the received data messages
    /// @param[in] pinIdx Index of the pin the data is received on
    void recvGnssObs(InputPin::NodeDataQueue& queue, size_t pinIdx);

    /// @brief Receive Function for the Initial PosVel Observation
    /// @param[in] queue Queue with all the received data messages
    /// @param[in] pinIdx Index of the pin the data is received on
    void recvPosVelInit(InputPin::NodeDataQueue& queue, size_t pinIdx);
};

} // namespace NAV