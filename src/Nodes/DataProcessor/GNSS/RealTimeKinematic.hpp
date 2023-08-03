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
#include "Navigation/GNSS/Errors.hpp"
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
    bool operator==(const AmbiguitySD& rhs) const { return satSigId == rhs.satSigId; }
    /// @brief Satellite Signal Id
    SatSigId satSigId;
};

/// Alias for the state key type
using StateKeyTypes = std::variant<KFStates, AmbiguitySD>;
/// @brief Vector with all position and velocity state keys
inline static const std::vector<StateKeyTypes> PosVel = { KFStates::PosX, KFStates::PosY, KFStates::PosZ,
                                                          KFStates::VelX, KFStates::VelY, KFStates::VelZ };
/// @brief All position keys
inline static const std::vector<StateKeyTypes> Pos = { KFStates::PosX, KFStates::PosY, KFStates::PosZ };
/// @brief All velocity keys
inline static const std::vector<StateKeyTypes> Vel = { KFStates::VelX, KFStates::VelY, KFStates::VelZ };

} // namespace States

namespace Meas
{

/// @brief Double differenced pseudorange measurement psr_br^1s (one for each satellite signal, referenced to the pivot satellite)
struct PsrDD
{
    /// @brief Equal comparison operator
    /// @param rhs Right-hand side
    bool operator==(const PsrDD& rhs) const { return satSigId == rhs.satSigId; }
    /// @brief Satellite Signal Id
    SatSigId satSigId;
};
/// @brief Double differenced carrier-phase measurement phi_br^1s (one for each satellite signal, referenced to the pivot satellite)
struct CarrierDD
{
    /// @brief Equal comparison operator
    /// @param rhs Right-hand side
    bool operator==(const CarrierDD& rhs) const { return satSigId == rhs.satSigId; }
    /// @brief Satellite Signal Id
    SatSigId satSigId;
};
/// @brief Double differenced doppler measurement d_br^1s (one for each satellite signal, referenced to the pivot satellite)
struct DopplerDD
{
    /// @brief Equal comparison operator
    /// @param rhs Right-hand side
    bool operator==(const DopplerDD& rhs) const { return satSigId == rhs.satSigId; }
    /// @brief Satellite Signal Id
    SatSigId satSigId;
};

/// Alias for the measurement key type
using MeasKeyTypes = std::variant<PsrDD, CarrierDD, DopplerDD>;

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
    Code _filterCode = Code_Default;
    /// List of satellites to exclude
    std::vector<SatId> _excludedSatellites;
    /// Elevation cut-off angle for satellites in [rad]
    double _elevationMask = static_cast<double>(15.0_deg);

    /// Utilized observations (Order from GnssObs::ObservationType: Psr, Carrier, Doppler)
    std::array<bool, 3> _usedObservations = { true, true, true };

    /// Ionosphere Model used for the calculation
    IonosphereModel _ionosphereModel = IonosphereModel::Klobuchar;

    /// Troposphere Models used for the calculation
    TroposphereModelSelection _troposphereModels;

    /// GNSS measurement error model to use
    GnssMeasurementErrorModel _gnssMeasurementErrorModel;

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

    /// 𝜎²_a Variance of the acceleration due to user motion in horizontal and vertical component in [m^2 / s^3]
    std::array<double, 2> _varAccel{};

    /// Recalculates the variance of the acceleration with the GUI setting
    void recalcVarAccel();

    // ------------------------------------------------------------ Algorithm --------------------------------------------------------------

    /// @brief Receiver Types
    enum ReceiverType
    {
        Base,  ///< Base
        Rover, ///< Rover
    };

    /// @brief Receiver information
    struct Receiver
    {
        /// @brief Constructor
        /// @param type Receiver enum type
        explicit Receiver(ReceiverType type) : type(type) {}

        /// Receiver Type
        ReceiverType type;
        /// Position in ECEF frame [m]
        Eigen::Vector3d e_pos = Eigen::Vector3d::Zero();
        /// Position in LLA frame [rad, rad, m]
        Eigen::Vector3d lla_pos = Eigen::Vector3d::Zero();
        /// Velocity in ECEF frame [m/s]
        Eigen::Vector3d e_vel = Eigen::Vector3d::Zero();
        /// Latest GNSS observation
        std::shared_ptr<const GnssObs> gnssObs = nullptr;
    };

    /// @brief Receivers
    std::array<Receiver, 2> _receiver = { { Receiver(Base), Receiver(Rover) } };

    /// Measured and calculated data per satellite
    struct SatData
    {
        /// @brief Constructor
        /// @param[in] satId Satellite Identifier
        /// @param[in] navData Satellite Navigation data
        SatData(const SatId& satId, std::shared_ptr<SatNavData> navData)
            : satId(satId), navData(std::move(navData)) {}

        SatId satId;                                   ///< Satellite Identifier
        std::shared_ptr<SatNavData> navData = nullptr; ///< Satellite Navigation data

        /// Receiver specific data
        struct ReceiverSpecificData
        {
            /// @brief Constructor
            /// @param[in] gnssObs GNSS observation
            /// @param[in] e_satPos Satellite position in e frame
            /// @param[in] lla_satPos Satellite position in lla frame
            /// @param[in] e_satVel Satellite velocity in e frame
            /// @param[in] e_recPos Receiver position in e frame
            /// @param[in] e_recVel Receiver velocity in e frame
            ReceiverSpecificData(std::shared_ptr<const GnssObs> gnssObs,
                                 const Eigen::Vector3d& e_satPos,
                                 const Eigen::Vector3d& lla_satPos,
                                 const Eigen::Vector3d& e_satVel,
                                 const Eigen::Vector3d& e_recPos,
                                 const Eigen::Vector3d& e_recVel);

            std::shared_ptr<const GnssObs> gnssObs = nullptr; ///< GNSS observation

            Eigen::Vector3d e_satPos;   ///< Satellite position in ECEF frame coordinates [m]
            Eigen::Vector3d lla_satPos; ///< Satellite position in LLA frame coordinates [rad, rad, m]
            Eigen::Vector3d e_satVel;   ///< Satellite velocity in ECEF frame coordinates [m/s]

            Eigen::Vector3d e_pLOS;    ///< Position Line-of-sight unit vector in ECEF frame coordinates
            Eigen::Vector3d e_vLOS;    ///< Velocity Line-of-sight unit vector in ECEF frame coordinates
            double satElevation = 0.0; ///< Satellite Elevation [rad]
            double satAzimuth = 0.0;   ///< Satellite Azimuth [rad]
        };

        unordered_map<ReceiverType, ReceiverSpecificData> receiverData; ///< Satellite data concerning a receiver
    };

    /// Observations
    struct Observation
    {
        /// @brief Constructor
        /// @param gnssObs GNSS observation
        /// @param obsIdx GNSS observation index for this measurement
        Observation(std::shared_ptr<const GnssObs> gnssObs, size_t obsIdx)
            : gnssObs(std::move(gnssObs)), obsIdx(obsIdx) {}

        double estimate = 0.0;    ///< Estimate
        double measurement = 0.0; ///< Measurement
        double measVar = 0.0;     ///< Variance of the measurement

        /// @brief Returns the observation data
        [[nodiscard]] const GnssObs::ObservationData& obs() const
        {
            return gnssObs->data.at(obsIdx);
        }

      private:
        std::shared_ptr<const GnssObs> gnssObs = nullptr; ///< GNSS observation
        size_t obsIdx = 0;                                ///< Gnss observation data index
    };

    /// @brief Observation storage type
    using Observations = unordered_map<SatSigId,
                                       unordered_map<ReceiverType,
                                                     unordered_map<GnssObs::ObservationType,
                                                                   Observation>>>;

    /// Differences (single or double)
    struct Difference
    {
        double estimate = 0.0;    ///< Estimate
        double measurement = 0.0; ///< Measurement
        double measVar = 0.0;     ///< Variance of the measurement
    };

    /// @brief Difference storage type
    using Differences = unordered_map<SatSigId,
                                      unordered_map<GnssObs::ObservationType,
                                                    Difference>>;

    /// @brief Pivot Satellite information
    struct PivotSatellite
    {
        bool reevaluate = false; ///< Flag whether to reevaluate the pivot satellite in the current epoch
        SatSigId satSigId;       ///< Satellite Signal Id
    };

    /// Pivot satellites for each constellation
    unordered_map<Code, PivotSatellite> _pivotSatellites;

    /// Last update time
    InsTime _lastUpdate;

    /// Kalman Filter representation
    KeyedKalmanFilterD<RealTimeKinematicKF::States::StateKeyTypes, RealTimeKinematicKF::Meas::MeasKeyTypes> _kalmanFilter;

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

    /// @brief Returns a list of satellites and observations used for calculation of RTK (only satellites filtered by GUI filter & NAV data available & ...)
    /// @param[in] obsTypes Observation types to take into account
    /// @param[in] gnssNavInfos Collection of all connected navigation data providers
    /// @return 0: List of satellite data; 1: List of observations; 2: Total amount of observations
    std::tuple<std::vector<SatData>, Observations, size_t> selectSatObservationsForCalculation(const std::unordered_set<GnssObs::ObservationType>& obsTypes,
                                                                                               const std::vector<const GnssNavInfo*>& gnssNavInfos);

    /// @brief Update the pivot satellites for each constellation
    /// @param[in] satelliteData List of satellite data used for the calculation of this epoch
    /// @param[in] observations List of GNSS observation data used for the calculation of this epoch
    void updatePivotSatellites(const std::vector<SatData>& satelliteData, const Observations& observations);

    /// @brief Calculates the observation estimates
    /// @param[in] satelliteData List of satellite data used for the calculation of this epoch
    /// @param[in, out] observations List of GNSS observation data used for the calculation of this epoch
    /// @param ionosphericCorrections Ionospheric correction parameters collected from the Nav data
    void calcObservationEstimates(const std::vector<SatData>& satelliteData, Observations& observations, const IonosphericCorrections& ionosphericCorrections);

    /// @brief Calculates the single difference of the measurements and estimates
    /// @param[in] observations List of GNSS observation data used for the calculation of this epoch
    [[nodiscard]] Differences calcSingleDifferences(const Observations& observations) const;

    /// @brief Calculates the double difference of the measurements and estimates
    /// @param[in] singleDifferences List of single differences
    [[nodiscard]] Differences calcDoubleDifferences(const Differences& singleDifferences) const;

    /// @brief Converts the enum to a string
    /// @param[in] receiver Enum value to convert into text
    /// @return String representation of the enum
    static const char* to_string(ReceiverType receiver);
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
/// @brief Hash function (needed for unordered_map)
template<>
struct hash<NAV::RealTimeKinematicKF::Meas::DopplerDD>
{
    /// @brief Hash function
    /// @param[in] dDD Double differenced doppler
    size_t operator()(const NAV::RealTimeKinematicKF::Meas::DopplerDD& dDD) const
    {
        return std::hash<NAV::SatSigId>()(dDD.satSigId) << 24;
    }
};
} // namespace std

#ifndef DOXYGEN_IGNORE

/// @brief Formatter for SatelliteSystem
template<>
struct fmt::formatter<NAV::RealTimeKinematicKF::States::StateKeyTypes>
{
    /// @brief Parse function to make the struct formattable
    /// @param[in] ctx Parser context
    /// @return Beginning of the context
    template<typename ParseContext>
    constexpr auto parse(ParseContext& ctx)
    {
        return ctx.begin();
    }

    /// @brief Defines how to format SatelliteSystem structs
    /// @param[in] state Struct to format
    /// @param[in, out] ctx Format context
    /// @return Output iterator
    template<typename FormatContext>
    auto format(const NAV::RealTimeKinematicKF::States::StateKeyTypes& state, FormatContext& ctx)
    {
        using namespace NAV::RealTimeKinematicKF::States; // NOLINT(google-build-using-namespace)

        if (const auto* s = std::get_if<NAV::RealTimeKinematicKF::States::KFStates>(&state))
        {
            switch (*s)
            {
            case PosX:
                return fmt::format_to(ctx.out(), "PosX");
            case PosY:
                return fmt::format_to(ctx.out(), "PosY");
            case PosZ:
                return fmt::format_to(ctx.out(), "PosZ");
            case VelX:
                return fmt::format_to(ctx.out(), "VelX");
            case VelY:
                return fmt::format_to(ctx.out(), "VelY");
            case VelZ:
                return fmt::format_to(ctx.out(), "VelZ");
            case KFStates_COUNT:
                return fmt::format_to(ctx.out(), "KFStates_COUNT");
            }
        }
        if (const auto* amb = std::get_if<NAV::RealTimeKinematicKF::States::AmbiguitySD>(&state))
        {
            return fmt::format_to(ctx.out(), "Amb({})", amb->satSigId);
        }

        return fmt::format_to(ctx.out(), "ERROR");
    }
};

/// @brief Formatter for SatelliteSystem
template<>
struct fmt::formatter<NAV::RealTimeKinematicKF::Meas::MeasKeyTypes>
{
    /// @brief Parse function to make the struct formattable
    /// @param[in] ctx Parser context
    /// @return Beginning of the context
    template<typename ParseContext>
    constexpr auto parse(ParseContext& ctx)
    {
        return ctx.begin();
    }

    /// @brief Defines how to format SatelliteSystem structs
    /// @param[in] meas Struct to format
    /// @param[in, out] ctx Format context
    /// @return Output iterator
    template<typename FormatContext>
    auto format(const NAV::RealTimeKinematicKF::Meas::MeasKeyTypes& meas, FormatContext& ctx)
    {
        if (const auto* psrDD = std::get_if<NAV::RealTimeKinematicKF::Meas::PsrDD>(&meas))
        {
            return fmt::format_to(ctx.out(), "psrDD({})", psrDD->satSigId);
        }
        if (const auto* phiDD = std::get_if<NAV::RealTimeKinematicKF::Meas::CarrierDD>(&meas))
        {
            return fmt::format_to(ctx.out(), "phiDD({})", phiDD->satSigId);
        }
        if (const auto* dDD = std::get_if<NAV::RealTimeKinematicKF::Meas::DopplerDD>(&meas))
        {
            return fmt::format_to(ctx.out(), "dopDD({})", dDD->satSigId);
        }

        return fmt::format_to(ctx.out(), "ERROR");
    }
};

#endif