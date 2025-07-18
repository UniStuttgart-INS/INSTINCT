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

#include <cstddef>
#include <cstdint>
#include <memory>
#include "internal/Node/Node.hpp"
#include "internal/gui/widgets/DynamicInputPins.hpp"

#include "Navigation/GNSS/Core/Frequency.hpp"
#include "Navigation/GNSS/Core/Code.hpp"
#include "Navigation/GNSS/Core/SatelliteIdentifier.hpp"
#include "Navigation/GNSS/Positioning/SPP/Algorithm.hpp"
#include "Navigation/GNSS/Positioning/ReceiverClock.hpp"
#include "Navigation/GNSS/Positioning/RTK/Keys.hpp"
#include "Navigation/GNSS/Positioning/Observation.hpp"
#include "Navigation/GNSS/Positioning/ObservationEstimator.hpp"
#include "Navigation/GNSS/Positioning/ObservationFilter.hpp"
#include "Navigation/GNSS/Positioning/Receiver.hpp"
#include "Navigation/GNSS/Ambiguity/AmbiguityResolution.hpp"
#include "Navigation/GNSS/Ambiguity/CycleSlipDetector.hpp"
#include "Navigation/GNSS/SNRMask.hpp"
#include "Navigation/GNSS/Satellite/internal/SatNavData.hpp"
#include "Navigation/Transformations/Units.hpp"
#include "Navigation/Math/KalmanFilter.hpp"
#include "Navigation/Math/KeyedKalmanFilter.hpp"

#include "NodeData/GNSS/SppSolution.hpp"
#include "NodeData/GNSS/RtkSolution.hpp"
#include "NodeData/GNSS/GnssObs.hpp"
#include "NodeData/GNSS/GnssNavInfo.hpp"

#include "util/Eigen.hpp"
#include "util/Container/Unordered_map.hpp"

namespace NAV
{

/// @brief Numerically integrates Imu data
class RealTimeKinematic : public Node
{
  public:
    /// @brief Receiver Types
    enum ReceiverType : uint8_t
    {
        Rover,              ///< Rover
        Base,               ///< Base
        ReceiverType_COUNT, ///< Amount of receiver types
    };

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

    /// Possible Units for the Standard deviation of the ambiguities
    enum class StdevAmbiguityUnits : uint8_t
    {
        Cycle, ///< [cycle]
    };

    /// @brief Initialize the node
    bool initialize() override;

    /// @brief Deinitialize the node
    void deinitialize() override;

    /// @brief Function to call to add a new pin
    /// @param[in, out] node Pointer to this node
    static void pinAddCallback(Node* node);
    /// @brief Function to call to delete a pin
    /// @param[in, out] node Pointer to this node
    /// @param[in] pinIdx Input pin index to delete
    static void pinDeleteCallback(Node* node, size_t pinIdx);

    /// Observation Filter
    ObservationFilter _obsFilter{ ReceiverType::ReceiverType_COUNT };

    /// Observation Estimator
    ObservationEstimator _obsEstimator{ ReceiverType::ReceiverType_COUNT };
    /// Maximum amount of outliers to remove per epoch
    size_t _maxRemoveOutlier = 0;
    /// Amount of epochs to remove an outlier after being found (1 = only current)
    size_t _outlierRemoveEpochs = 1;
    /// Minumum amount of satellites for doing a NIS check
    size_t _outlierMinSat = 0;
    /// Minumum amount of pseudorange observables to leave when doing a NIS check
    size_t _outlierMinPsrObsKeep = 0;

    /// Do not attempt to remove outlier if the position variance is above the threshold in the startup phase [m^2]
    double _outlierMaxPosVarStartup = 0.1;

    /// @brief Wether to output a SPP solution if no base observations are available
    bool _calcSPPIfNoBase = false;
    /// Maximum time between base and rover observations to calculate the RTK solution
    double _maxTimeBetweenBaseRoverForRTK = 5.0;

    /// Ambiguity resolution algorithms and parameters to use
    AmbiguityResolutionParameters _ambiguityResolutionParameters;

    /// Ambiguity resolution strategy
    AmbiguityResolutionStrategy _ambiguityResolutionStrategy = AmbiguityResolutionStrategy::FixAndHold;

    /// Minimum amount of satellites with carrier observations to try fixing the solution
    size_t _nMinSatForAmbFix = 4;
    /// Minimum amount of satellites with carrier observations for holding the ambiguities
    size_t _nMinSatForAmbHold = 5;
    /// Do not attempt to fix if the position variance is above the threshold
    double _maxPosVar = 0.2;

    /// Make an update with the fixed ambiguities when true.
    /// Otherwise apply via $$a = a_fix$$ and $$b = b_float - Q_ba * Q_aa^-1 (a_fix - a_float)$$
    bool _applyFixedAmbiguitiesWithUpdate = true;

    /// Measurement noise standard deviation used when fixing ambiguities in [cycles] (GUI value)
    double _gui_ambFixUpdateStdDev = 1e-2;
    /// Gui selection for the Unit of the input for the StDev of the ambiguities (while float solution)
    StdevAmbiguityUnits _gui_ambFixUpdateStdDevUnits = StdevAmbiguityUnits::Cycle;
    /// Measurement noise variance used when fixing ambiguities in [cycles^2] (Value used for calculation)
    double _ambFixUpdateVariance = _gui_ambFixUpdateStdDev * _gui_ambFixUpdateStdDev;

    bool _maxPosVarTriggered = false;               ///< Trigger state of the check
    bool _outlierMaxPosVarStartupTriggered = false; ///< Trigger state of the check
    bool _nMinSatForAmbFixTriggered = false;        ///< Trigger state of the check
    bool _nMinSatForAmbHoldTriggered = false;       ///< Trigger state of the check

    // #####################################################################################

    /// Possible Units for the Standard deviation of the acceleration due to user motion
    enum class StdevAccelUnits : uint8_t
    {
        m_sqrts3, ///< [ m / √(s^3) ]
    };
    /// Gui selection for the Unit of the input stdev_accel parameter for the StDev due to acceleration due to user motion
    StdevAccelUnits _gui_stdevAccelUnits = StdevAccelUnits::m_sqrts3;

    /// @brief GUI selection for the Standard deviation of the acceleration 𝜎_a due to user motion in horizontal and vertical component
    /// @note See Groves (2013) eq. (9.156)
    std::array<double, 2> _gui_stdevAccel = { { 3.0, 1.5 } } /* [ m / √(s^3) ] */;

    /// Gui selection for the Unit of the input for the StDev of the ambiguities (while float solution)
    StdevAmbiguityUnits _gui_stdevAmbiguityFloatUnits = StdevAmbiguityUnits::Cycle;
    /// Process noise (standard deviation) for the ambiguities (while float solution) (Selection in GUI)
    double _gui_ambiguityFloatProcessNoiseStDev = 1e1;
    /// Process noise (variance) for the ambiguities (while float solution) in [cycles^2] (Value used for calculation)
    double _ambiguityFloatProcessNoiseVariance = _gui_ambiguityFloatProcessNoiseStDev * _gui_ambiguityFloatProcessNoiseStDev;

    /// Gui selection for the Unit of the input for the StDev of the ambiguities (while fix solution)
    StdevAmbiguityUnits _gui_stdevAmbiguityFixUnits = StdevAmbiguityUnits::Cycle;
    /// Process noise (standard deviation) for the ambiguities (while fix solution) (Selection in GUI)
    double _gui_ambiguityFixProcessNoiseStDev = 1e-4;
    /// Process noise (standard deviation) for the ambiguities (while fix solution) in [cycles^2] (Value used for calculation)
    double _ambiguityFixProcessNoiseVariance = _gui_ambiguityFixProcessNoiseStDev * _gui_ambiguityFixProcessNoiseStDev;

    // ------------------------------------------------------------ Algorithm --------------------------------------------------------------

    using Receiver = NAV::Receiver<ReceiverType>; ///< Receiver

    /// @brief SPP algorithm
    SPP::Algorithm _sppAlgorithm;

    /// @brief Receivers
    std::array<Receiver, ReceiverType::ReceiverType_COUNT> _receiver = { { Receiver(Base, {}), Receiver(Rover, {}) } };

    /// Flag, whether the observation was received this epoch
    bool _baseObsReceivedThisEpoch = true;

    /// Solution type of last epoch
    RtkSolution::SolutionType _lastSolutionStatus = RtkSolution::SolutionType::RTK_Float;

    /// Differences (single or double)
    struct Difference
    {
        double estimate = 0.0;    ///< Estimate
        double measurement = 0.0; ///< Measurement
    };

    /// @brief Difference storage type
    using Differences = unordered_map<SatSigId,
                                      unordered_map<GnssObs::ObservationType,
                                                    Difference>>;

    /// @brief Pivot satellite information
    struct PivotSatellite
    {
        /// @brief Constructor
        /// @param[in] satSigId Satellite Signal identifier
        explicit PivotSatellite(const SatSigId& satSigId) : satSigId(satSigId) {}

        SatSigId satSigId; ///< Satellite Signal identifier
    };

    /// Pivot satellites for each constellation
    unordered_map<std::pair<Code, GnssObs::ObservationType>, PivotSatellite> _pivotSatellites;

    /// Last update time
    InsTime _lastUpdate;
    /// Data interval [s]
    double _dataInterval = 1;

    /// Kalman Filter representation
    KeyedKalmanFilterD<RTK::States::StateKeyType, RTK::Meas::MeasKeyTypes> _kalmanFilter;

    /// Cycle-slip detector
    std::array<CycleSlipDetector, ReceiverType_COUNT> _cycleSlipDetector;

    /// List of event texts (pivot changes, cycle-slips)
    std::vector<std::pair<InsTime, std::vector<std::string>>> _events;

    /// Regex for filtering events
    std::string _eventFilterRegex;

    /// Whether to output state change events
    bool _outputStateEvents = false;

    /// Ambiguities which are fixed and hold. Values in [cycles]
    std::unordered_map<SatSigId, double> _ambiguitiesHold;

    /// Percentage of fix solutions
    size_t nFixSolutions = 0;
    /// Percentage of float solutions
    size_t nFloatSolutions = 0;
    /// Percentage of float solutions
    size_t nSingleSolutions = 0;

    size_t _nPivotChange = 0;      ///< Amount of pivot changes performed
    size_t _nCycleSlipsLLI = 0;    ///< Cycle-slip detection count (LLI)
    size_t _nCycleSlipsSingle = 0; ///< Cycle-slip detection count (Single)
    size_t _nCycleSlipsDual = 0;   ///< Cycle-slip detection count (Dual)
    size_t _nMeasExcludedNIS = 0;  ///< Measurement excluded due to NIS test count

    /// @brief Dynamic input pins
    /// @attention This should always be the last variable in the header, because it accesses others through the function callbacks
    gui::widgets::DynamicInputPins _dynamicInputPins{ INPUT_PORT_INDEX_GNSS_NAV_INFO, this, pinAddCallback, pinDeleteCallback };

    /// @brief Adds the event to the GUI list
    /// @param[in, out] rtkSol RtkSolution to update
    /// @param[in] text Text to display
    void addEventToGui(const std::shared_ptr<RtkSolution>& rtkSol, const std::string& text);

    /// @brief Prints the observations for logging
    /// @param[in] observations List of GNSS observation data used for the calculation of this epoch
    void printObservations(const Observations& observations);

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

    /// @brief Assign the SPP solution to the RTK filter
    /// @param sppSol SPP solution
    void assignSolutionToFilter(const std::shared_ptr<NAV::SppSolution>& sppSol);

    /// @brief Calculates the RTK solution
    void calcRealTimeKinematicSolution();

    /// @brief Calculates a SPP solution as fallback in case no base data is available
    std::shared_ptr<RtkSolution> calcFallbackSppSolution();

    /// @brief Does the Kalman Filter prediction
    void kalmanFilterPrediction();

    /// @brief Checks for cycle-slips
    /// @param[in] observations List of GNSS observation data used for the calculation of this epoch
    /// @param[in, out] rtkSol RtkSolution to update
    void checkForCycleSlip(Observations& observations, const std::shared_ptr<RtkSolution>& rtkSol);

    /// @brief Removes observations which do not have a second one to do double differences
    /// @param[in] observations List of GNSS observation data used for the calculation of this epoch
    /// @param[in] filtered Filtered observations
    /// @param[in, out] rtkSol RtkSolution to update
    void removeSingleObservations(Observations& observations, ObservationFilter::Filtered* filtered, const std::shared_ptr<RtkSolution>& rtkSol);

    /// @brief Update the specified pivot satellite and also does a pivot change in the Kalman filter state if necessary
    /// @param code Code for the pivot
    /// @param obsType Observation type of the pivot
    /// @param observations List of GNSS observation data used for the calculation of this epoch
    /// @param rtkSol RtkSolution to update
    /// @param reason Reason for changing the pivot
    /// @return The final pivot change result after the update. Empty if no pivot was selected
    std::optional<RtkSolution::PivotChange> updatePivotSatellite(Code code, GnssObs::ObservationType obsType,
                                                                 Observations& observations, const std::shared_ptr<RtkSolution>& rtkSol,
                                                                 const RtkSolution::PivotChange::Reason& reason);

    /// Prints all pivot satellites
    void printPivotSatellites();

    /// @brief Update the pivot satellites for each constellation
    /// @param[in] observations List of GNSS observation data used for the calculation of this epoch
    /// @param[in, out] rtkSol RtkSolution to update
    void updatePivotSatellites(Observations& observations, const std::shared_ptr<RtkSolution>& rtkSol);

    /// @brief Adapts the Kalman Filter if a pivot satellite signal was changed
    /// @param[in] newPivotSatSigId Newly added pivot satellite signal
    /// @param[in] oldPivotSatSigId Old pivot satellite signal
    /// @param[in] oldPivotObservedInEpoch Indicates if the old pivot satellite is still observed, or is
    /// @param[in, out] rtkSol RtkSolution to update
    void updateKalmanFilterAmbiguitiesForPivotChange(const SatSigId& newPivotSatSigId, const SatSigId& oldPivotSatSigId, bool oldPivotObservedInEpoch, const std::shared_ptr<RtkSolution>& rtkSol);

    /// @brief Adds or remove Ambiguities to/from the Kalman Filter state depending on the received observations
    /// @param[in] observations List of GNSS observation data used for the calculation of this epoch
    /// @param[in, out] rtkSol RtkSolution to update
    void addOrRemoveKalmanFilterAmbiguities(const Observations& observations, const std::shared_ptr<RtkSolution>& rtkSol);

    /// @brief Calculates the single difference of the measurements and estimates
    /// @param[in] observations List of GNSS observation data used for the calculation of this epoch
    [[nodiscard]] Differences calcSingleDifferences(const Observations& observations) const;

    /// @brief Calculates the double difference of the measurements and estimates
    /// @param[in] observations List of GNSS observation data used for the calculation of this epoch
    /// @param[in] singleDifferences List of single differences
    [[nodiscard]] Differences calcDoubleDifferences(const Observations& observations, const Differences& singleDifferences) const;

    /// @brief Calculate the measurement noise matrices for each observation type
    /// @param observations List of GNSS observation data used for the calculation of this epoch
    [[nodiscard]] unordered_map<GnssObs::ObservationType, KeyedMatrixXd<RTK::Meas::SingleObs<ReceiverType>>>
        calcSingleObsMeasurementNoiseMatrices(const Observations& observations) const;

    /// @brief Calculates the Measurement vector 𝐳, Measurement sensitivity matrix 𝐇 and Measurement noise covariance matrix 𝐑 for the KF update
    /// @param[in] observations List of GNSS observation data used for the calculation of this epoch
    /// @param[in] doubleDifferences List of double differences
    /// @param[in] Rtilde Single observation Measurement Noise Covariance Matrix R for each observation type
    void calcKalmanUpdateMatrices(const Observations& observations, const Differences& doubleDifferences,
                                  const unordered_map<GnssObs::ObservationType, KeyedMatrixXd<RTK::Meas::SingleObs<ReceiverType>>>& Rtilde);

    /// @brief Outlier removal info
    struct OutlierInfo
    {
        /// @brief Defaults Constructor
        OutlierInfo() = default;
        /// @brief Constructor
        /// @param key Measurement key
        explicit OutlierInfo(const RTK::Meas::MeasKeyTypes& key) : key(key) {}

        std::optional<RtkSolution::PivotChange::Reason> pivot;             ///< Pivot change reason, if it is a pivot
        GnssObs::ObservationType obsType = GnssObs::ObservationType_COUNT; ///< Observation type
        SatSigId satSigId;                                                 ///< Satellit signal id
        RTK::Meas::MeasKeyTypes key = RTK::Meas::PsrDD{};                  ///< Measurement key of the outlier
    };

    /// @brief Performs the NIS check
    /// @param[in, out] observations List of GNSS observation data. Elements can be removed
    /// @param[in, out] rtkSol RtkSolution to update
    /// @return Info of the removed outliers
    std::vector<OutlierInfo> removeOutlier(Observations& observations, const std::shared_ptr<RtkSolution>& rtkSol);

    /// @brief Update Valid information
    struct UpdateStatus
    {
        bool valid = true;      ///< Flag if the solution is valid
        double dx = 0.0;        ///< Position change due to the update
        double threshold = 0.0; ///< Allowed position change
    };

    /// @brief Does the Kalman Filter update
    /// @param[in, out] rtkSol RtkSolution to update
    /// @return Update valid information
    UpdateStatus kalmanFilterUpdate(const std::shared_ptr<RtkSolution>& rtkSol);

    /// @brief Resolves the ambiguities in the Kalman filter and updates the state and covariance matrix
    /// @param[in] nCarrierMeasUniqueSatellite Amount of used
    /// @param[in, out] rtkSol RtkSolution to update
    /// @return True if the ambiguities could be fixed
    bool resolveAmbiguities(size_t nCarrierMeasUniqueSatellite, const std::shared_ptr<RtkSolution>& rtkSol);

    /// @brief Apply the fixed ambiguities to the kalman filter state
    /// @param[in] fixedAmb Fixed ambiguities
    /// @param[in] ambKeys Ambiguity state keys
    /// @param[in] ambMeasKeys Ambiguity measurement keys
    void applyFixedAmbiguities(const Eigen::VectorXd& fixedAmb, const std::vector<RTK::States::StateKeyType>& ambKeys, const std::vector<RTK::Meas::MeasKeyTypes>& ambMeasKeys);
};

/// @brief Converts the enum to a string
/// @param[in] receiver Enum value to convert into text
/// @return String representation of the enum
[[nodiscard]] const char* to_string(const RealTimeKinematic::ReceiverType& receiver);

} // namespace NAV

#ifndef DOXYGEN_IGNORE

/// @brief Formatter
template<>
struct fmt::formatter<NAV::RealTimeKinematic::ReceiverType> : fmt::formatter<const char*>
{
    /// @brief Defines how to format structs
    /// @param[in] type Struct to format
    /// @param[in, out] ctx Format context
    /// @return Output iterator
    template<typename FormatContext>
    auto format(const NAV::RealTimeKinematic::ReceiverType& type, FormatContext& ctx) const
    {
        return fmt::formatter<const char*>::format(to_string(type), ctx);
    }
};

#endif
