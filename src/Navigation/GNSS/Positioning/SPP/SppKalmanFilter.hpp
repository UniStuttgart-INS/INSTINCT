// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file SppKalmanFilter.hpp
/// @brief SPP with a Kalman Filter
/// @author P. Peitschat (HiWi)
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2023-07-19

#pragma once

#include <set>
#include <variant>

#include "Navigation/Time/InsTime.hpp"
#include "Navigation/Math/KalmanFilter.hpp"
#include "Navigation/Math/KeyedKalmanFilter.hpp"

#include "Navigation/GNSS/Core/SatelliteSystem.hpp"
#include "Navigation/GNSS/Core/ReceiverClock.hpp"
#include "Navigation/GNSS/Positioning/SPP/SppCommon.hpp"
#include "Navigation/GNSS/Positioning/SppAlgorithmTypes.hpp"

#include "util/Eigen.hpp"

namespace NAV::GNSS::Positioning::SPP
{
namespace KF
{
namespace States
{
/// @brief State Keys of the Kalman filter
enum KFStates
{
    PosX,           ///< Position ECEF_X [m]
    PosY,           ///< Position ECEF_Y [m]
    PosZ,           ///< Position ECEF_Z [m]
    VelX,           ///< Velocity ECEF_X [m/s]
    VelY,           ///< Velocity ECEF_Y [m/s]
    VelZ,           ///< Velocity ECEF_Z [m/s]
    RecvClkErr,     ///< Receiver clock error [m]
    RecvClkDrift,   ///< Receiver clock drift [m/s]
    KFStates_COUNT, ///< Count
};
/// @brief Inter-system clock errors (one for additional satellite system)
struct InterSysErr
{
    /// @brief Equal comparison operator
    /// @param rhs Right-hand side
    bool operator==(const InterSysErr& rhs) const { return satSys == rhs.satSys; }
    /// @brief Satellite system
    SatelliteSystem satSys;
};
/// @brief Inter-system clock drifts (one for additional satellite system)
struct InterSysDrift
{
    /// @brief Equal comparison operator
    /// @param rhs Right-hand side
    bool operator==(const InterSysDrift& rhs) const { return satSys == rhs.satSys; }
    /// @brief Satellite system
    SatelliteSystem satSys;
};

/// Alias for the state key type
using StateKeyTypes = std::variant<KFStates, InterSysErr, InterSysDrift>;
/// @brief All position keys
inline static const std::vector<StateKeyTypes> Pos = { KFStates::PosX, KFStates::PosY, KFStates::PosZ };
/// @brief All velocity keys
inline static const std::vector<StateKeyTypes> Vel = { KFStates::VelX, KFStates::VelY, KFStates::VelZ };
/// @brief Vector with all position and velocity state keys
inline static const std::vector<StateKeyTypes> PosVel = { KFStates::PosX, KFStates::PosY, KFStates::PosZ,
                                                          KFStates::VelX, KFStates::VelY, KFStates::VelZ };
/// @brief Vector with all position, velocity and receiver clock state keys
inline static const std::vector<StateKeyTypes> PosVelRecvClk = { KFStates::PosX, KFStates::PosY, KFStates::PosZ,
                                                                 KFStates::VelX, KFStates::VelY, KFStates::VelZ,
                                                                 KFStates::RecvClkErr, KFStates::RecvClkDrift };

/// @brief All Inter-system clock errors keys
static std::vector<KF::States::StateKeyTypes> InterSysErrs;
/// @brief All Inter-system clock drifts keys
/// @note Groves2013 does not estimate inter-system drifts, but we do for all models.
static std::vector<KF::States::StateKeyTypes> InterSysDrifts;

} // namespace States

namespace Meas
{

/// @brief Pseudorange measurement [m]
struct Psr
{
    /// @brief Equal comparison operator
    /// @param rhs Right-hand side
    bool operator==(const Psr& rhs) const { return satSigId == rhs.satSigId; }
    /// @brief Satellite Signal Id
    SatSigId satSigId;
};
/// @brief Range-rate (doppler) measurement [m/s]
struct Doppler
{
    /// @brief Equal comparison operator
    /// @param rhs Right-hand side
    bool operator==(const Doppler& rhs) const { return satSigId == rhs.satSigId; }
    /// @brief Satellite Signal Id
    SatSigId satSigId;
};

/// Alias for the measurement key type
using MeasKeyTypes = std::variant<Psr, Doppler>;

} // namespace Meas
} // namespace KF

/// @brief The Spp Kalman Filter related options
class SppKalmanFilter // NOLINT(clang-analyzer-optin.performance.Padding)
{
  public:
    /// Possible Units for the initial covariance for the position (standard deviation σ or Variance σ²)
    enum class InitCovariancePositionUnits
    {
        m2, ///< Variance ECEF [m², m², m²]
        m,  ///< Standard deviation ECEF [m, m, m]
    };
    /// Gui selection for the Unit of the initial covariance for the position
    InitCovariancePositionUnits gui_initCovariancePositionUnit = InitCovariancePositionUnits::m;

    /// @brief GUI selection of the initial covariance diagonal values for position (standard deviation σ or Variance σ²)
    Eigen::Vector3d gui_initCovariancePosition{ 10, 10, 10 };
    /// @brief Initial covariance diagonal values for position (Variance σ²)
    Eigen::Vector3d _initCovariancePosition{ 10, 10, 10 };

    // ###########################################################################################################

    /// Possible Units for the initial covariance for the velocity (standard deviation σ or Variance σ²)
    enum class InitCovarianceVelocityUnits
    {
        m2_s2, ///< Variance [m²/s²]
        m_s,   ///< Standard deviation [m/s]
    };
    /// Gui selection for the Unit of the initial covariance for the velocity
    InitCovarianceVelocityUnits gui_initCovarianceVelocityUnit = InitCovarianceVelocityUnits::m_s;

    /// @brief GUI selection of the initial covariance diagonal values for velocity (standard deviation σ or Variance σ²)
    Eigen::Vector3d gui_initCovarianceVelocity{ 3, 3, 3 };
    /// @brief Initial covariance diagonal values for velocity (Variance σ²)
    Eigen::Vector3d _initCovarianceVelocity{ 3, 3, 3 };

    // ###########################################################################################################

    /// Possible Units for the initial covariance for the receiver clock error (standard deviation σ or Variance σ²)
    enum class InitCovarianceRecvClkErrUnits
    {
        s2, ///< Variance [s²]
        s,  ///< Standard deviation [s]
    };
    /// Gui selection for the Unit of the initial covariance for the receiver clock error
    InitCovarianceRecvClkErrUnits gui_initCovarianceRecvClkErrUnit = InitCovarianceRecvClkErrUnits::s;

    /// @brief GUI selection of the initial covariance for the receiver clock error (standard deviation σ or Variance σ²)
    double gui_initCovarianceRecvClkErr = 1e-8;
    /// @brief Initial covariance for the receiver clock error (Variance σ²)
    double _initCovarianceRecvClkErr = 1e-8;

    // ###########################################################################################################

    /// Possible Units for the initial covariance for the receiver clock drift (standard deviation σ or Variance σ²)
    enum class InitCovarianceRecvClkDriftUnits
    {
        s2_s2, ///< Variance [s²/s²]
        s_s,   ///< Standard deviation [s/s]
    };
    /// Gui selection for the Unit of the initial covariance for the receiver clock drift
    InitCovarianceRecvClkDriftUnits gui_initCovarianceRecvClkDriftUnit = InitCovarianceRecvClkDriftUnits::s_s;

    /// @brief GUI selection of the initial covariance for the receiver clock drift (standard deviation σ or Variance σ²)
    double gui_initCovarianceRecvClkDrift = 1e-10;
    /// @brief Initial covariance for the receiver clock drift (Variance σ²)
    double _initCovarianceRecvClkDrift = 1e-10;

    // ###########################################################################################################

    /// Gui selection for the Unit of the initial covariance for the inter-system clock error
    InitCovarianceRecvClkErrUnits gui_initCovarianceInterSysErrUnit = InitCovarianceRecvClkErrUnits::s;

    /// @brief GUI selection of the initial covariance for the inter-constellation clock error
    double gui_initCovarianceInterSysErr = 1e-8;
    /// @brief Initial covariance for the inter-constellation clock error
    double _initCovarianceInterSysErr = 1e-8;

    // ###########################################################################################################

    /// Gui selection for the Unit of the initial covariance for the inter-system clock drift
    InitCovarianceRecvClkDriftUnits gui_initCovarianceInterSysDriftUnit = InitCovarianceRecvClkDriftUnits::s_s;

    /// @brief GUI selection of the initial covariance for the inter-constellation clock drift
    double gui_initCovarianceInterSysDrift = 1e-10;
    /// @brief Initial covariance for the inter-constellation clock drift
    double _initCovarianceInterSysDrift = 1e-10;

    // ###########################################################################################################

    /// GUI option for the Q calculation algorithm
    enum class QCalculationAlgorithm
    {
        VanLoan, ///< Van-Loan
        Taylor1, ///< Taylor
    };
    /// Q calculation algorithm
    QCalculationAlgorithm qCalculationAlgorithm = QCalculationAlgorithm::VanLoan;

    /// @brief Gui selection of the process noise
    float processNoiseStandardDeviation = 1;

    // ###########################################################################################################

    /// Possible Units for the Standard deviation of the acceleration due to user motion
    enum class CovarianceAccelUnits
    {
        m_sqrts3, ///< [ m / √(s^3) ]
        m2_s3,    ///< [ m^2 / s^3 ]
    };
    /// Gui selection for the Unit of the input stdev_accel parameter for the StDev due to acceleration due to user motion
    CovarianceAccelUnits gui_covarianceAccelUnit = CovarianceAccelUnits::m_sqrts3;

    /// @brief GUI selection for the Standard deviation of the acceleration 𝜎_a due to user motion in horizontal and vertical component
    /// @note See Groves (2013) eq. (9.156)
    Eigen::Vector2d gui_covarianceAccel = { 3.0, 1.5 } /* [ m / √(s^3) ] */;
    /// @brief Covariance of the acceleration 𝜎_a due to user motion in horizontal and vertical component
    Eigen::Vector2d _covarianceAccel = { 3.0, 1.5 } /* [ m^2 / s^3 ] */;

    // ###########################################################################################################

    /// Possible Units for the Standard deviation of the clock phase drift
    enum class CovarianceClkPhaseDriftUnits
    {
        m_sqrts3, ///< [ m / √(s^3) ]
        m2_s3,    ///< [ m^2 / s^3 ]
    };
    /// Gui selection for the Unit of the input stdevClkPhaseDrift parameter
    CovarianceClkPhaseDriftUnits gui_covarianceClkPhaseDriftUnit = CovarianceClkPhaseDriftUnits::m_sqrts3;

    /// @brief GUI selection for the Standard deviation of the clock phase drift
    double gui_covarianceClkPhaseDrift = 0.2 /* [ m / √(s^3) ] */;
    /// @brief Covariance of the clock phase drift
    double _covarianceClkPhaseDrift = 0.2 /* [ m^2 / s^3 ] */;

    // ###########################################################################################################

    /// Possible Units for the Standard deviation of the clock frequency drift
    enum class CovarianceClkFrequencyDriftUnits
    {
        m_sqrts, ///< [ m / √(s) ]
        m2_s,    ///< [ m^2 / s ]
    };
    /// Gui selection for the Unit of the input stdevClkFrequencyDrift parameter
    CovarianceClkFrequencyDriftUnits gui_covarianceClkFrequencyDriftUnit = CovarianceClkFrequencyDriftUnits::m_sqrts;

    /// @brief GUI selection for the Standard deviation of the clock frequency drift
    double gui_covarianceClkFrequencyDrift = 0.1 /* [ m / √(s) ] */;
    /// @brief Covariance of the clock frequency drift
    double _covarianceClkFrequencyDrift = 0.1 /* [ m^2 / s ] */;

    // ###########################################################################################################

    /// Gui selection for the Unit of the input stdevClkPhaseDrift parameter
    CovarianceClkPhaseDriftUnits gui_covarianceInterSysClkPhaseDriftUnit = CovarianceClkPhaseDriftUnits::m_sqrts3;

    /// @brief GUI selection for the Standard deviation of the inter-system clock phase drift
    double gui_covarianceInterSysClkPhaseDrift = std::sqrt(2) * 0.2 /* [ m / √(s^3) ] */;
    /// @brief Covariance of the inter-system clock phase drift
    double _covarianceInterSysClkPhaseDrift = std::sqrt(2) * 0.2 /* [ m^2 / s^3 ] */;

    // ###########################################################################################################

    /// Gui selection for the Unit of the input stdevClkFrequencyDrift parameter
    CovarianceClkFrequencyDriftUnits gui_covarianceInterSysClkFrequencyDriftUnit = CovarianceClkFrequencyDriftUnits::m_sqrts;

    /// @brief GUI selection for the Standard deviation of the inter-system clock frequency drift
    double gui_covarianceInterSysClkFrequencyDrift = std::sqrt(2) * 0.1 /* [ m / √(s) ] */;
    /// @brief Covariance of the inter-system clock frequency drift
    double _covarianceInterSysClkFrequencyDrift = std::sqrt(2) * 0.1 /* [ m^2 / s ] */;

    // ###########################################################################################################

    /// @brief Creates the GUI for SPP if a Kalman Filter is selected
    void gui();

    /// @brief Initializes and resets the filter
    void initialize();

    /// @brief Checks wether the Kalman filter is initialized
    bool isInitialized() const;

    /// @brief Sets the vector that contains all satellite systems that the user has selected in the GUI besides Reference time satellite system
    /// @param[in] allSatSys selected satellite systems besides Reference time satellite system
    void setAllSatSysExceptRef(const std::vector<SatelliteSystem>& allSatSys);

    /// @ brief Gets the vector that contains all satellite systems that the user has selected in the GUI besides Reference time satellite system
    std::vector<SatelliteSystem> getAllSatSysExceptRef() const;

    /// @ brief Gets satellite system used as time reference
    SatelliteSystem getRefTimeSatSys() const;

    /// @brief Gets the state keys for inter-system clock estimation
    /// @param[in] insTime Epoch time
    void addInterSysStateKeys(const InsTime& insTime);

    /// @brief Initialize Kalman Filter from LSE solution
    /// @param[in] sppSolLSE SPP Least squares solution
    void initializeKalmanFilter(const std::shared_ptr<const SppSolution>& sppSolLSE);

    /// @brief Performs Single Point Positioning algorithm based on Kalman Filter
    /// @param[in] insTime Epoch time
    /// @param[in, out] calcData Data calculated for each satellite, that is available and selected
    /// @param[in] ionosphericCorrections Collection of all connected Ionospheric Corrections
    /// @param[in] ionosphereModel Ionosphere Model used for the calculation
    /// @param[in] troposphereModels Troposphere Models used for the calculation
    /// @param[in] gnssMeasurementErrorModel GNSS measurement error model to use
    /// @param[in] elevationMask Elevation cut-off angle for satellites in [rad]
    /// @param[in] useDoppler Boolean which enables the use of doppler observations
    /// @return Shared pointer to the SPP solution
    std::shared_ptr<NAV::SppSolution> estimateSppSolution(const InsTime& insTime,
                                                          std::vector<CalcData>& calcData,
                                                          const IonosphericCorrections& ionosphericCorrections,
                                                          const IonosphereModel& ionosphereModel,
                                                          const TroposphereModelSelection& troposphereModels,
                                                          const GnssMeasurementErrorModel& gnssMeasurementErrorModel,
                                                          double elevationMask,
                                                          bool useDoppler);

  private:
    /// @brief Does the Kalman Filter prediction
    /// @param[in] insTime Epoch time
    void kalmanFilterPrediction(const InsTime& insTime);

    /// @brief Does the Kalman Filter update
    /// @param[in] keyedObservations Storage type for Kalman Filter (contains innovations, Design matrix entries, weight matrix entries)
    /// @param[in] sppSolReferenceTimeSatelliteSystem Reference time satellite system selected from SPP LSE functions
    /// @param[in] satelliteSystems Available/Observed satellite systems in this epoch except reference satellite systems
    /// @param[in] useDoppler Boolean which enables the use of doppler observations
    /// @param[in] insTime Epoch time
    void kalmanFilterUpdate(const KeyedObservations& keyedObservations,
                            SatelliteSystem sppSolReferenceTimeSatelliteSystem,
                            const std::vector<SatelliteSystem>& satelliteSystems,
                            bool useDoppler,
                            const InsTime& insTime);

    /// @brief Sets Process Noise Matrix
    /// @param[in] dt Time interval between the Kalman Filter estimates in [s]
    void processNoiseMatrixGroves(double dt);

    /// @brief Assigns Kalman Filter estimates to internal variables
    /// @param[in] otherSatelliteSystems List of satellite systems (all, or available at this epoch)
    void assignInternalSolution(const std::vector<SatelliteSystem>& otherSatelliteSystems);

    /// @brief Assigns Kalman Filter estimates to internal variables as well as sppSol and state
    /// @param[in, out] sppSol Single Point Positioning Solution
    /// @param[in] otherSatelliteSystems List of satellite systems (all, or available at this epoch)
    void assignSolution(std::shared_ptr<NAV::SppSolution>& sppSol,
                        const std::vector<SatelliteSystem>& otherSatelliteSystems);

    /// Estimated position in ECEF frame [m]
    Eigen::Vector3d _e_position = Eigen::Vector3d::Zero();
    /// Estimated velocity in ECEF frame [m/s]
    Eigen::Vector3d _e_velocity = Eigen::Vector3d::Zero();

    /// Estimated receiver clock parameters
    ReceiverClock _recvClk;

    /// Time of last KF estimation
    InsTime _lastUpdate;

    /// Kalman Filter representation
    KeyedKalmanFilterD<NAV::GNSS::Positioning::SPP::KF::States::StateKeyTypes, NAV::GNSS::Positioning::SPP::KF::Meas::MeasKeyTypes> _kalmanFilter;

    /// Satellite system used as time reference
    SatelliteSystem _refTimeSatSys = SatSys_None;

    /// All satellite systems that the user has selected in the GUI besides Reference time satellite system
    std::vector<SatelliteSystem> _allSatSysExceptRef;

    /// Boolean that determines, if Kalman Filter is initialized (from weighted LSE solution)
    bool _initialized = false;
};

/// @brief Converts the provided data into a json object
/// @param[out] j Json object which gets filled with the info
/// @param[in] data Data to convert into json
void to_json(json& j, const SppKalmanFilter& data);
/// @brief Converts the provided json object into the data object
/// @param[in] j Json object with the needed values
/// @param[out] data Object to fill from the json
void from_json(const json& j, SppKalmanFilter& data);

} // namespace NAV::GNSS::Positioning::SPP

namespace std
{
/// @brief Hash function (needed for unordered_map)
template<>
struct hash<NAV::GNSS::Positioning::SPP::KF::States::InterSysErr>
{
    /// @brief Hash function
    /// @param[in] interSysErr Inter-system clock errors
    size_t operator()(const NAV::GNSS::Positioning::SPP::KF::States::InterSysErr& interSysErr) const
    {
        return NAV::GNSS::Positioning::SPP::KF::States::KFStates_COUNT + std::hash<NAV::SatelliteSystem>()(interSysErr.satSys);
    }
};
/// @brief Hash function (needed for unordered_map)
template<>
struct hash<NAV::GNSS::Positioning::SPP::KF::States::InterSysDrift>
{
    /// @brief Hash function
    /// @param[in] interSysDrift Inter-system clock drifts
    size_t operator()(const NAV::GNSS::Positioning::SPP::KF::States::InterSysDrift& interSysDrift) const
    {
        return NAV::GNSS::Positioning::SPP::KF::States::KFStates_COUNT + std::hash<NAV::SatelliteSystem>()(interSysDrift.satSys);
    }
};
/// @brief Hash function (needed for unordered_map)
template<>
struct hash<NAV::GNSS::Positioning::SPP::KF::Meas::Psr>
{
    /// @brief Hash function
    /// @param[in] psr Pseudorange observation
    size_t operator()(const NAV::GNSS::Positioning::SPP::KF::Meas::Psr& psr) const
    {
        return std::hash<NAV::SatSigId>()(psr.satSigId);
    }
};
/// @brief Hash function (needed for unordered_map)
template<>
struct hash<NAV::GNSS::Positioning::SPP::KF::Meas::Doppler>
{
    /// @brief Hash function
    /// @param[in] doppler Doppler observation
    size_t operator()(const NAV::GNSS::Positioning::SPP::KF::Meas::Doppler& doppler) const
    {
        return std::hash<NAV::SatSigId>()(doppler.satSigId) << 12;
    }
};
} // namespace std

#ifndef DOXYGEN_IGNORE

/// @brief Formatter
template<>
struct fmt::formatter<NAV::GNSS::Positioning::SPP::KF::States::KFStates>
{
    /// @brief Parse function to make the struct formattable
    /// @param[in] ctx Parser context
    /// @return Beginning of the context
    template<typename ParseContext>
    constexpr auto parse(ParseContext& ctx)
    {
        return ctx.begin();
    }

    /// @brief Defines how to format structs
    /// @param[in] state Struct to format
    /// @param[in, out] ctx Format context
    /// @return Output iterator
    template<typename FormatContext>
    auto format(const NAV::GNSS::Positioning::SPP::KF::States::KFStates& state, FormatContext& ctx)
    {
        using namespace NAV::GNSS::Positioning::SPP::KF::States; // NOLINT(google-build-using-namespace)

        switch (state)
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
        case RecvClkErr:
            return fmt::format_to(ctx.out(), "RecvClkErr");
        case RecvClkDrift:
            return fmt::format_to(ctx.out(), "RecvClkDrift");
        case KFStates_COUNT:
            return fmt::format_to(ctx.out(), "KFStates_COUNT");
        }

        return fmt::format_to(ctx.out(), "ERROR");
    }
};

/// @brief Formatter
template<>
struct fmt::formatter<NAV::GNSS::Positioning::SPP::KF::States::InterSysErr>
{
    /// @brief Parse function to make the struct formattable
    /// @param[in] ctx Parser context
    /// @return Beginning of the context
    template<typename ParseContext>
    constexpr auto parse(ParseContext& ctx)
    {
        return ctx.begin();
    }

    /// @brief Defines how to format structs
    /// @param[in] interSysErr Struct to format
    /// @param[in, out] ctx Format context
    /// @return Output iterator
    template<typename FormatContext>
    auto format(const NAV::GNSS::Positioning::SPP::KF::States::InterSysErr& interSysErr, FormatContext& ctx)
    {
        return fmt::format_to(ctx.out(), "InterSysErr({})", interSysErr.satSys);
    }
};

/// @brief Formatter
template<>
struct fmt::formatter<NAV::GNSS::Positioning::SPP::KF::States::InterSysDrift>
{
    /// @brief Parse function to make the struct formattable
    /// @param[in] ctx Parser context
    /// @return Beginning of the context
    template<typename ParseContext>
    constexpr auto parse(ParseContext& ctx)
    {
        return ctx.begin();
    }

    /// @brief Defines how to format structs
    /// @param[in] interSysDrift Struct to format
    /// @param[in, out] ctx Format context
    /// @return Output iterator
    template<typename FormatContext>
    auto format(const NAV::GNSS::Positioning::SPP::KF::States::InterSysDrift& interSysDrift, FormatContext& ctx)
    {
        return fmt::format_to(ctx.out(), "InterSysDrift({})", interSysDrift.satSys);
    }
};

/// @brief Formatter
template<>
struct fmt::formatter<NAV::GNSS::Positioning::SPP::KF::Meas::Psr>
{
    /// @brief Parse function to make the struct formattable
    /// @param[in] ctx Parser context
    /// @return Beginning of the context
    template<typename ParseContext>
    constexpr auto parse(ParseContext& ctx)
    {
        return ctx.begin();
    }

    /// @brief Defines how to format structs
    /// @param[in] psr Struct to format
    /// @param[in, out] ctx Format context
    /// @return Output iterator
    template<typename FormatContext>
    auto format(const NAV::GNSS::Positioning::SPP::KF::Meas::Psr& psr, FormatContext& ctx)
    {
        return fmt::format_to(ctx.out(), "psr({})", psr.satSigId);
    }
};

/// @brief Formatter
template<>
struct fmt::formatter<NAV::GNSS::Positioning::SPP::KF::Meas::Doppler>
{
    /// @brief Parse function to make the struct formattable
    /// @param[in] ctx Parser context
    /// @return Beginning of the context
    template<typename ParseContext>
    constexpr auto parse(ParseContext& ctx)
    {
        return ctx.begin();
    }

    /// @brief Defines how to format structs
    /// @param[in] doppler Struct to format
    /// @param[in, out] ctx Format context
    /// @return Output iterator
    template<typename FormatContext>
    auto format(const NAV::GNSS::Positioning::SPP::KF::Meas::Doppler& doppler, FormatContext& ctx)
    {
        return fmt::format_to(ctx.out(), "dop({})", doppler.satSigId);
    }
};

/// @brief Formatter
template<>
struct fmt::formatter<NAV::GNSS::Positioning::SPP::KF::States::StateKeyTypes>
{
    /// @brief Parse function to make the struct formattable
    /// @param[in] ctx Parser context
    /// @return Beginning of the context
    template<typename ParseContext>
    constexpr auto parse(ParseContext& ctx)
    {
        return ctx.begin();
    }

    /// @brief Defines how to format structs
    /// @param[in] state Struct to format
    /// @param[in, out] ctx Format context
    /// @return Output iterator
    template<typename FormatContext>
    auto format(const NAV::GNSS::Positioning::SPP::KF::States::StateKeyTypes& state, FormatContext& ctx)
    {
        using namespace NAV::GNSS::Positioning::SPP::KF::States; // NOLINT(google-build-using-namespace)

        if (const auto* s = std::get_if<NAV::GNSS::Positioning::SPP::KF::States::KFStates>(&state))
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
            case RecvClkErr:
                return fmt::format_to(ctx.out(), "RecvClkErr");
            case RecvClkDrift:
                return fmt::format_to(ctx.out(), "RecvClkDrift");
            case KFStates_COUNT:
                return fmt::format_to(ctx.out(), "KFStates_COUNT");
            }
        }
        if (const auto* interSysErr = std::get_if<NAV::GNSS::Positioning::SPP::KF::States::InterSysErr>(&state))
        {
            return fmt::format_to(ctx.out(), "InterSysErr({})", interSysErr->satSys);
        }
        if (const auto* interSysDrift = std::get_if<NAV::GNSS::Positioning::SPP::KF::States::InterSysDrift>(&state))
        {
            return fmt::format_to(ctx.out(), "InterSysDrift({})", interSysDrift->satSys);
        }

        return fmt::format_to(ctx.out(), "ERROR");
    }
};

/// @brief Formatter
template<>
struct fmt::formatter<NAV::GNSS::Positioning::SPP::KF::Meas::MeasKeyTypes>
{
    /// @brief Parse function to make the struct formattable
    /// @param[in] ctx Parser context
    /// @return Beginning of the context
    template<typename ParseContext>
    constexpr auto parse(ParseContext& ctx)
    {
        return ctx.begin();
    }

    /// @brief Defines how to format structs
    /// @param[in] meas Struct to format
    /// @param[in, out] ctx Format context
    /// @return Output iterator
    template<typename FormatContext>
    auto format(const NAV::GNSS::Positioning::SPP::KF::Meas::MeasKeyTypes& meas, FormatContext& ctx)
    {
        if (const auto* psr = std::get_if<NAV::GNSS::Positioning::SPP::KF::Meas::Psr>(&meas))
        {
            return fmt::format_to(ctx.out(), "psr({})", psr->satSigId);
        }
        if (const auto* doppler = std::get_if<NAV::GNSS::Positioning::SPP::KF::Meas::Doppler>(&meas))
        {
            return fmt::format_to(ctx.out(), "doppler({})", doppler->satSigId);
        }

        return fmt::format_to(ctx.out(), "ERROR");
    }
};

#endif