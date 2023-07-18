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
#include "Navigation/GNSS/Satellite/internal/SatNavData.hpp"
#include "Navigation/Atmosphere/Ionosphere/Ionosphere.hpp"
#include "Navigation/Atmosphere/Troposphere/Troposphere.hpp"
#include "Navigation/Transformations/Units.hpp"
#include "Navigation/Math/KalmanFilter.hpp"
#include "Navigation/Math/KeyedKalmanFilter.hpp"

#include "NodeData/GNSS/RtkSolution.hpp"
#include "NodeData/GNSS/GnssObs.hpp"
#include "NodeData/GNSS/GnssNavInfo.hpp"

namespace NAV
{
namespace RealTimeKinematicKF
{
namespace States
{

/// @brief State Keys of the Kalman filter
enum KFStates
{
    PosX,           ///< Position ECEF_X
    PosY,           ///< Position ECEF_Y
    PosZ,           ///< Position ECEF_Z
    VelX,           ///< Velocity ECEF_X
    VelY,           ///< Velocity ECEF_Y
    VelZ,           ///< Velocity ECEF_Z
    KFStates_COUNT, ///< Count
};
/// @brief Single differenced (rover - base) ambiguity N_br^1 (one for each satellite signal)
struct AmbiguitySD
{
    /// @brief Equal comparison operator
    /// @param rhs Right-hand side
    constexpr bool operator==(const AmbiguitySD& rhs) const { return satSigId == rhs.satSigId; }
    /// @brief Satellite Signal Id
    SatSigId satSigId;
};

/// Alias for the state key type
using StateKeyTypes = std::variant<KFStates, AmbiguitySD>;
/// @brief Vector with all position and velocity state keys
inline static const std::vector<StateKeyTypes> PosVel = { KFStates::PosX, KFStates::PosY, KFStates::PosZ,
                                                          KFStates::VelX, KFStates::VelY, KFStates::VelZ };
/// @brief All position keys
inline static const std::vector<KFStates> Pos = { KFStates::PosX, KFStates::PosY, KFStates::PosZ };
/// @brief All velocity keys
inline static const std::vector<KFStates> Vel = { KFStates::VelX, KFStates::VelY, KFStates::VelZ };

} // namespace States

namespace Meas
{

/// @brief Double differenced pseudorange measurement psr_br^1s (one for each satellite signal, referenced to the pivot satellite)
struct PsrDD
{
    /// @brief Equal comparison operator
    /// @param rhs Right-hand side
    constexpr bool operator==(const PsrDD& rhs) const { return satSigId == rhs.satSigId; }
    /// @brief Satellite Signal Id
    SatSigId satSigId;
};
/// @brief Double differenced carrier-phase measurement phi_br^1s (one for each satellite signal, referenced to the pivot satellite)
struct CarrierDD
{
    /// @brief Equal comparison operator
    /// @param rhs Right-hand side
    constexpr bool operator==(const CarrierDD& rhs) const { return satSigId == rhs.satSigId; }
    /// @brief Satellite Signal Id
    SatSigId satSigId;
};

/// Alias for the measurement key type
using MeasKeyTypes = std::variant<CarrierDD, PsrDD>;

} // namespace Meas
} // namespace RealTimeKinematicKF

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
    Code _filterCode = Code_ALL;
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
    StdevAccelUnits _gui_stdevAccelUnits = StdevAccelUnits::m_sqrts3;

    /// @brief GUI selection for the Standard deviation of the acceleration 𝜎_a due to user motion in horizontal and vertical component
    /// @note See Groves (2013) eq. (9.156)
    std::array<double, 2> _gui_stdevAccel = { { 3.0, 1.5 } } /* [ m / √(s^3) ] */;

    /// 𝜎²_a Variance of the acceleration due to user motion in horizontal and vertical component in [m / √(s^3)]
    std::array<double, 2> _varAccel{};

    /// Recalculates the variance of the acceleration with the GUI setting
    void recalcVarAccel();

    // ------------------------------------------------------------ Algorithm --------------------------------------------------------------

    /// Measured and calculated data per satellite
    struct SatData
    {
        /// @brief Constructor
        /// @param[in] satId Satellite Identifier
        /// @param[in] navData Satellite Navigation data
        /// @param[in] e_roverSatPos Satellite position in e frame seen from rover
        /// @param[in] lla_roverSatPos Satellite position in lla frame seen from rover
        /// @param[in] e_roverSatVel Satellite velocity in e frame seen from rover
        /// @param[in] e_baseSatPos Satellite position in e frame seen from base
        /// @param[in] lla_baseSatPos Satellite position in lla frame seen from base
        /// @param[in] e_baseSatVel Satellite velocity in e frame seen from base
        /// @param[in] e_roverPos Rover receiver position in e frame
        /// @param[in] e_basePos Base receiver position in e frame
        SatData(const SatId& satId, std::shared_ptr<SatNavData> navData,
                const Eigen::Vector3d& e_roverSatPos, const Eigen::Vector3d& lla_roverSatPos, const Eigen::Vector3d& e_roverSatVel,
                const Eigen::Vector3d& e_baseSatPos, const Eigen::Vector3d& lla_baseSatPos, const Eigen::Vector3d& e_baseSatVel,
                const Eigen::Vector3d& e_roverPos, const Eigen::Vector3d& e_basePos)
            : satId(satId), navData(std::move(navData)), rover(e_roverSatPos, lla_roverSatPos, e_roverSatVel, e_roverPos), base(e_baseSatPos, lla_baseSatPos, e_baseSatVel, e_basePos) {}

        SatId satId;                                   ///< Satellite Identifier
        std::shared_ptr<SatNavData> navData = nullptr; ///< Satellite Navigation data

        /// Receiver specific data
        struct ReceiverSpecificData
        {
            /// @brief Constructor
            /// @param[in] e_satPos Satellite position in e frame
            /// @param[in] lla_satPos Satellite position in lla frame
            /// @param[in] e_satVel Satellite velocity in e frame
            /// @param[in] e_recPos Receiver position in e frame
            ReceiverSpecificData(const Eigen::Vector3d& e_satPos,
                                 const Eigen::Vector3d& lla_satPos,
                                 Eigen::Vector3d e_satVel,
                                 const Eigen::Vector3d& e_recPos);

            Eigen::Vector3d e_satPos;   ///< Satellite position in ECEF frame coordinates [m]
            Eigen::Vector3d lla_satPos; ///< Satellite position in LLA frame coordinates [rad, rad, m]
            Eigen::Vector3d e_satVel;   ///< Satellite velocity in ECEF frame coordinates [m/s]

            Eigen::Vector3d e_lineOfSightUnitVector; ///< Line-of-sight unit vector in ECEF frame coordinates
            double satElevation = 0.0;               ///< Satellite Elevation [rad]
            double satAzimuth = 0.0;                 ///< Satellite Azimuth [rad]
        };

        ReceiverSpecificData rover; ///< Satellite data concerning the rover
        ReceiverSpecificData base;  ///< Satellite data concerning the base

        /// Data calculated for each observation
        struct Signal
        {
            /// @brief Constructor
            /// @param[in] gnssObsRover GNSS observation of the rover
            /// @param[in] obsIdxRover GNSS observation data index of the rover
            /// @param[in] gnssObsBase GNSS observation of the base
            /// @param[in] obsIdxBase GNSS observation data index of the base
            Signal(const std::shared_ptr<const GnssObs>& gnssObsRover, size_t obsIdxRover,
                   const std::shared_ptr<const GnssObs>& gnssObsBase, size_t obsIdxBase)
                : obsRover({ gnssObsRover, obsIdxRover }), obsBase({ gnssObsBase, obsIdxBase }) {}

            double singleDiffMeasPseudorange_br_s = 0.0; ///< Single difference of the pseudorange measurement
            double singleDiffMeasCarrier_br_s = 0.0;     ///< Single difference of the carrier-phase measurement

            double singleDiffEstPseudorange_br_s = 0.0; ///< Single difference estimate of the pseudorange
            double singleDiffEstCarrier_br_s = 0.0;     ///< Single difference estimate of the carrier-phase

            double doubleDiffMeasPseudorange_br_1s = 0.0; ///< Double difference of the pseudorange measurement
            double doubleDiffMeasCarrier_br_1s = 0.0;     ///< Double difference of the carrier-phase measurement

            double doubleDiffEstPseudorange_br_1s = 0.0; ///< Double difference estimate of the pseudorange
            double doubleDiffEstCarrier_br_1s = 0.0;     ///< Double difference estimate of the carrier-phase

            /// Receiver specific data
            struct Observation
            {
                std::shared_ptr<const GnssObs> gnssObs = nullptr; ///< GNSS observation
                size_t obsIdx = 0;                                ///< Gnss observation data index

                double psrEst = 0.0; ///< Estimated Pseudorange [m]

                /// @brief Returns the observation data
                [[nodiscard]] const GnssObs::ObservationData& obs() const
                {
                    return gnssObs->data.at(obsIdx);
                }
            };
            Observation obsRover; ///< Rover observation
            Observation obsBase;  ///< Base observation
        };

        /// @brief Measurements and calculations concerning each signal received by this satellite
        std::unordered_map<Frequency, Signal> signals;
    };

    /// Base position in ECEF frame [m]
    Eigen::Vector3d _e_basePosition = Eigen::Vector3d::Zero();
    /// Base position in LLA frame [rad, rad, m]
    Eigen::Vector3d _lla_basePosition = Eigen::Vector3d::Zero();

    /// Estimated position in ECEF frame [m]
    Eigen::Vector3d _e_roverPosition = Eigen::Vector3d::Zero();
    /// Estimated position in LLA frame [rad, rad, m]
    Eigen::Vector3d _lla_roverPosition = Eigen::Vector3d::Zero();
    /// Estimated velocity in ECEF frame [m/s]
    Eigen::Vector3d _e_roverVelocity = Eigen::Vector3d::Zero();

    /// @brief Pivot Satellite information
    struct PivotSatellite
    {
        bool reevaluate = false; ///< Flag whether to reevaluate the pivot satellite in the current epoch
        SatData satData;         ///< Satellite Data
    };

    /// Pivot satellites for each constellation
    std::map<Frequency, PivotSatellite> _pivotSatellites;

    /// Latest GNSS Observation from the base station
    std::shared_ptr<const GnssObs> _gnssObsBase = nullptr;
    /// Latest GNSS Observation from the rover
    std::shared_ptr<const GnssObs> _gnssObsRover = nullptr;

    /// Kalman Filter representation
    KeyedKalmanFilterD<RealTimeKinematicKF::States::StateKeyTypes, RealTimeKinematicKF::Meas::MeasKeyTypes> _kalmanFilter{ RealTimeKinematicKF::States::PosVel, {} };

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
    std::shared_ptr<RtkSolution> calcFallbackSppSolution();

    /// @brief Returns a list of observations used for calculation of RTK (only satellites filtered by GUI filter & NAV data available & ...)
    /// @param[in] gnssNavInfos Collection of all connected navigation data providers
    std::vector<SatData> selectSatellitesForCalculation(const std::vector<const GnssNavInfo*>& gnssNavInfos);

    /// @brief Update the pivot satellites for each constellation
    /// @param satelliteData List of GNSS observation data used for the calculation of this epoch
    void updatePivotSatellites(const std::vector<SatData>& satelliteData);

    /// @brief Calculates the measured double differences for each satellite
    /// @param satelliteData List of GNSS observation data used for the calculation of this epoch
    /// @return The amount of double differences calculated
    size_t calculateMeasurementDoubleDifferences(std::vector<SatData>& satelliteData);

    /// @brief Calculates the estimated double differences for each satellite
    /// @param satelliteData List of GNSS observation data used for the calculation of this epoch
    /// @param ionosphericCorrections Ionospheric correction parameters collected from the Nav data
    void calculateEstimatedDoubleDifferences(std::vector<SatData>& satelliteData, const IonosphericCorrections& ionosphericCorrections);
};

} // namespace NAV

namespace std
{
/// @brief Hash function (needed for unordered_map)
template<>
struct hash<NAV::RealTimeKinematicKF::States::AmbiguitySD>
{
    /// @brief Hash function
    /// @param[in] ambSD Single differenced ambiguity
    size_t operator()(const NAV::RealTimeKinematicKF::States::AmbiguitySD& ambSD) const
    {
        return NAV::RealTimeKinematicKF::States::KFStates_COUNT + std::hash<NAV::SatSigId>()(ambSD.satSigId);
    }
};
/// @brief Hash function (needed for unordered_map)
template<>
struct hash<NAV::RealTimeKinematicKF::Meas::PsrDD>
{
    /// @brief Hash function
    /// @param[in] psrDD Double differenced pseudorange
    size_t operator()(const NAV::RealTimeKinematicKF::Meas::PsrDD& psrDD) const
    {
        return std::hash<NAV::SatSigId>()(psrDD.satSigId);
    }
};
/// @brief Hash function (needed for unordered_map)
template<>
struct hash<NAV::RealTimeKinematicKF::Meas::CarrierDD>
{
    /// @brief Hash function
    /// @param[in] cpDD Double differenced carrier-phase
    size_t operator()(const NAV::RealTimeKinematicKF::Meas::CarrierDD& cpDD) const
    {
        return std::hash<NAV::SatSigId>()(cpDD.satSigId) << 12;
    }
};
} // namespace std