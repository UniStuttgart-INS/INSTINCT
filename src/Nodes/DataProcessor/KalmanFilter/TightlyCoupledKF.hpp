// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file TightlyCoupledKF.hpp
/// @brief Kalman Filter class for the tightly coupled INS/GNSS integration
/// @author M. Maier (marcel.maier@ins.uni-stuttgart.de)
/// @date 2023-01-18

#pragma once

#include "internal/Node/Node.hpp"
#include "Navigation/GNSS/Core/Frequency.hpp"
#include "Navigation/GNSS/Core/Code.hpp"
#include "Navigation/Time/InsTime.hpp"
#include "NodeData/State/InertialNavSol.hpp"
#include "NodeData/GNSS/GnssObs.hpp"
#include "Navigation/GNSS/Core/ReceiverClock.hpp"
#include "Navigation/Atmosphere/Ionosphere/Ionosphere.hpp"
#include "Navigation/Atmosphere/Troposphere/Troposphere.hpp"
#include "NodeData/State/TcKfInsGnssErrors.hpp"

#include "Navigation/Math/KalmanFilter.hpp"
#include "Navigation/Transformations/Units.hpp"

namespace NAV
{
/// @brief Tightly-coupled Kalman Filter for INS/GNSS integration
class TightlyCoupledKF : public Node
{
  public:
    /// @brief Default constructor
    TightlyCoupledKF();
    /// @brief Destructor
    ~TightlyCoupledKF() override;
    /// @brief Copy constructor
    TightlyCoupledKF(const TightlyCoupledKF&) = delete;
    /// @brief Move constructor
    TightlyCoupledKF(TightlyCoupledKF&&) = delete;
    /// @brief Copy assignment operator
    TightlyCoupledKF& operator=(const TightlyCoupledKF&) = delete;
    /// @brief Move assignment operator
    TightlyCoupledKF& operator=(TightlyCoupledKF&&) = delete;
    /// @brief String representation of the class type
    [[nodiscard]] static std::string typeStatic();
    /// @brief String representation of the class type
    [[nodiscard]] std::string type() const override;
    /// @brief String representation of the class category
    [[nodiscard]] static std::string category();

    /// @brief ImGui config window which is shown on double click
    /// @attention Don't forget to set _hasConfig to true in the constructor of the node
    void guiConfig() override;

    /// @brief Saves the node into a json object
    [[nodiscard]] json save() const override;

    /// @brief Restores the node from a json object
    /// @param j Json object with the node state
    void restore(const json& j) override;

  private:
    constexpr static size_t INPUT_PORT_INDEX_GNSS_OBS = 1;      ///< @brief GnssObs
    constexpr static size_t INPUT_PORT_INDEX_GNSS_NAV_INFO = 2; ///< @brief GnssNavInfo
    constexpr static size_t OUTPUT_PORT_INDEX_ERROR = 0;        ///< @brief Flow (TcKfInsGnssErrors)
    constexpr static size_t OUTPUT_PORT_INDEX_SYNC = 1;         ///< @brief Flow (ImuObs)
    constexpr static size_t OUTPUT_PORT_INDEX_x = 2;            ///< @brief x̂ State vector
    constexpr static size_t OUTPUT_PORT_INDEX_P = 3;            ///< @brief 𝐏 Error covariance matrix
    constexpr static size_t OUTPUT_PORT_INDEX_Phi = 4;          ///< @brief 𝚽 State transition matrix
    constexpr static size_t OUTPUT_PORT_INDEX_Q = 5;            ///< @brief 𝐐 System/Process noise covariance matrix
    constexpr static size_t OUTPUT_PORT_INDEX_z = 6;            ///< @brief 𝐳 Measurement vector
    constexpr static size_t OUTPUT_PORT_INDEX_H = 7;            ///< @brief 𝐇 Measurement sensitivity Matrix
    constexpr static size_t OUTPUT_PORT_INDEX_R = 8;            ///< @brief 𝐑 = 𝐸{𝐰ₘ𝐰ₘᵀ} Measurement noise covariance matrix
    constexpr static size_t OUTPUT_PORT_INDEX_K = 9;            ///< @brief 𝐊 Kalman gain matrix

    /// @brief Initialize the node
    bool initialize() override;

    /// @brief Deinitialize the node
    void deinitialize() override;

    /// @brief Receive Function for the inertial navigation solution
    /// @param[in] queue Queue with all the received data messages
    /// @param[in] pinIdx Index of the pin the data is received on
    void recvInertialNavigationSolution(InputPin::NodeDataQueue& queue, size_t pinIdx);

    /// @brief Receive Function for the Gnss observations
    /// @param[in] queue Queue with all the received data messages
    /// @param[in] pinIdx Index of the pin the data is received on
    void recvGnssObs(InputPin::NodeDataQueue& queue, size_t pinIdx);

    /// @brief Predicts the state from the InertialNavSol
    /// @param[in] inertialNavSol Inertial navigation solution triggering the prediction
    /// @param[in] tau_i Time since the last prediction in [s]
    void tightlyCoupledPrediction(const std::shared_ptr<const InertialNavSol>& inertialNavSol, double tau_i);

    /// @brief Updates the predicted state from the InertialNavSol with the GNSS observation
    /// @param[in] gnssObservation Gnss observation triggering the update
    void tightlyCoupledUpdate(const std::shared_ptr<const GnssObs>& gnssObservation);

    /// @brief Add the output pins for the Kalman matrices
    void addKalmanMatricesPins();

    /// @brief Removes the output pins for the Kalman matrices
    void removeKalmanMatricesPins();

    /// Index of the Pin currently being dragged
    int _dragAndDropPinIndex = -1;
    /// Number of NavInfo input pins
    size_t _nNavInfoPins = 1;
    /// @brief Adds/Deletes Input Pins depending on the variable _nNavInfoPins
    void updateNumberOfInputPins();

    /// Estimated receiver clock parameters
    ReceiverClock _recvClk;

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

    /// Latest observation from the Inertial Integrator (Position, Velocity, Attitude and IMU measurements)
    std::shared_ptr<const InertialNavSol> _latestInertialNavSol = nullptr;

    /// Time when the last prediction was triggered
    InsTime _lastPredictTime;

    /// Time when the last GNSS message came and a prediction was requested
    InsTime _lastPredictRequestedTime;

    /// Time of last epoch
    InsTime _lastEpochTime;

    /// Accumulated Accelerometer biases
    Eigen::Vector3d _accumulatedAccelBiases;
    /// Accumulated Gyroscope biases
    Eigen::Vector3d _accumulatedGyroBiases;

    /// Kalman Filter representation - States: 3xAtt, 3xVel, 3xPos, 3xAccelBias, 3xGyroBias, receiver clock offset, receiver clock drift - Measurements: psr, psrRate (from Doppler)
    KalmanFilter _kalmanFilter{ 17, 8 };

    // ###########################################################################################################
    //                                               GUI Settings
    // ###########################################################################################################

    /// @brief Available Frames
    enum class Frame : int
    {
        ECEF, ///< Earth-Centered Earth-Fixed frame
        NED,  ///< Local North-East-Down frame
    };
    /// Frame to calculate the Kalman filter in
    Frame _frame = Frame::NED;

    /// @brief Show output pins for the Kalman matrices
    bool _showKalmanFilterOutputPins = false;

    /// @brief Check the rank of the Kalman matrices every iteration (computational expensive)
    bool _checkKalmanMatricesRanks = true;

    // ###########################################################################################################
    //                                                Parameters
    // ###########################################################################################################

    /// Lever arm between INS and GNSS in [m, m, m]
    Eigen::Vector3d _b_leverArm_InsGnss{ 0.0, 0.0, 0.0 };

    // ###########################################################################################################

    /// Possible Units for the Standard deviation of the noise on the accelerometer specific-force measurements
    enum class StdevAccelNoiseUnits
    {
        mg_sqrtHz,   ///< [mg / √(Hz)]
        m_s2_sqrtHz, ///< [m / s^2 / √(Hz)]
    };
    /// Gui selection for the Unit of the input stdev_ra parameter
    StdevAccelNoiseUnits _stdevAccelNoiseUnits = StdevAccelNoiseUnits::mg_sqrtHz;

    /// @brief 𝜎_ra Standard deviation of the noise on the accelerometer specific-force measurements
    /// @note Value from VN-310 Datasheet but verify with values from Brown (2012) table 9.3 for 'High quality'
    Eigen::Vector3d _stdev_ra = 0.04 /* [mg/√(Hz)] */ * Eigen::Vector3d::Ones();

    // ###########################################################################################################

    /// Possible Units for the Standard deviation of the noise on the gyro angular-rate measurements
    enum class StdevGyroNoiseUnits
    {
        deg_hr_sqrtHz, ///< [deg / hr /√(Hz)]
        rad_s_sqrtHz,  ///< [rad / s /√(Hz)]
    };
    /// Gui selection for the Unit of the input stdev_rg parameter
    StdevGyroNoiseUnits _stdevGyroNoiseUnits = StdevGyroNoiseUnits::deg_hr_sqrtHz;

    /// @brief 𝜎_rg Standard deviation of the noise on the gyro angular-rate measurements
    /// @note Value from VN-310 Datasheet but verify with values from Brown (2012) table 9.3 for 'High quality'
    Eigen::Vector3d _stdev_rg = 5 /* [deg/hr/√(Hz)]^2 */ * Eigen::Vector3d::Ones();

    // ###########################################################################################################

    /// Possible Units for the Variance of the accelerometer dynamic bias
    enum class StdevAccelBiasUnits
    {
        microg, ///< [µg]
        m_s2,   ///< [m / s^2]
    };
    /// Gui selection for the Unit of the input variance_bad parameter
    StdevAccelBiasUnits _stdevAccelBiasUnits = StdevAccelBiasUnits::microg;

    /// @brief 𝜎²_bad Variance of the accelerometer dynamic bias
    /// @note Value from VN-310 Datasheet (In-Run Bias Stability (Allan Variance))
    Eigen::Vector3d _stdev_bad = 10 /* [µg] */ * Eigen::Vector3d::Ones();

    /// @brief Correlation length of the accelerometer dynamic bias in [s]
    Eigen::Vector3d _tau_bad = 0.1 * Eigen::Vector3d::Ones();

    // ###########################################################################################################

    /// Possible Units for the Variance of the accelerometer dynamic bias
    enum class StdevGyroBiasUnits
    {
        deg_h, ///< [°/h]
        rad_s, ///< [1/s]
    };
    /// Gui selection for the Unit of the input variance_bad parameter
    StdevGyroBiasUnits _stdevGyroBiasUnits = StdevGyroBiasUnits::deg_h;

    /// @brief 𝜎²_bgd Variance of the gyro dynamic bias
    /// @note Value from VN-310 Datasheet (In-Run Bias Stability (Allan Variance))
    Eigen::Vector3d _stdev_bgd = 1 /* [°/h] */ * Eigen::Vector3d::Ones();

    /// @brief Correlation length of the gyro dynamic bias in [s]
    Eigen::Vector3d _tau_bgd = 0.1 * Eigen::Vector3d::Ones();

    // ###########################################################################################################

    /// Possible Units for the Standard deviation of the receiver clock phase drift
    enum class StdevClockPhaseUnits
    {
        m_sqrtHz, ///< [m / √(Hz)]
    };
    /// Gui selection for the Unit of the input stdev_cp parameter
    StdevClockPhaseUnits _stdevClockPhaseUnits = StdevClockPhaseUnits::m_sqrtHz;

    /// @brief 𝜎_cf Standard deviation of the receiver clock phase drift
    /// @note See Groves (2013) eq. (9.153)
    double _stdev_cp = 0 /* [m / √(Hz)] */;

    // ###########################################################################################################

    /// Possible Units for the Standard deviation of the receiver clock frequency drift
    enum class StdevClockFreqUnits
    {
        m_s_sqrtHz, ///< [m / s / √(Hz)]
    };
    /// Gui selection for the Unit of the input stdev_cf parameter
    StdevClockFreqUnits _stdevClockFreqUnits = StdevClockFreqUnits::m_s_sqrtHz;

    /// @brief 𝜎_cf Standard deviation of the receiver clock frequency drift
    /// @note See Brown (2012) table 9.2
    double _stdev_cf = 5 /* [m / s / √(Hz)] */;

    // ###########################################################################################################

    /// Possible Units for the Standard deviation of the pseudorange measurement
    enum class GnssMeasurementUncertaintyPseudorangeUnit
    {
        meter2, ///< Variance [m²]
        meter,  ///< Standard deviation [m]
    };
    /// Gui selection for the Unit of the input gnssMeasurementUncertaintyPseudorangeUnit parameter
    GnssMeasurementUncertaintyPseudorangeUnit _gnssMeasurementUncertaintyPseudorangeUnit = GnssMeasurementUncertaintyPseudorangeUnit::meter;

    /// @brief GUI selection of the GNSS pseudorange measurement uncertainty (standard deviation σ or Variance σ²).
    double _gnssMeasurementUncertaintyPseudorange = 5 /* [m] */;

    // ###########################################################################################################

    /// Possible Units for the Standard deviation of the pseudorange-rate measurement
    enum class GnssMeasurementUncertaintyPseudorangeRateUnit
    {
        m2_s2, ///< Variance [m²/s²]
        m_s,   ///< Standard deviation [m/s]
    };
    /// Gui selection for the Unit of the input gnssMeasurementUncertaintyPseudorangeRateUnit parameter
    GnssMeasurementUncertaintyPseudorangeRateUnit _gnssMeasurementUncertaintyPseudorangeRateUnit = GnssMeasurementUncertaintyPseudorangeRateUnit::m_s;

    /// @brief GUI selection of the GNSS pseudorange-rate measurement uncertainty (standard deviation σ or Variance σ²).
    double _gnssMeasurementUncertaintyPseudorangeRate = 5 /* [m/s] */;

    // ###########################################################################################################

    /// @brief Available Random processes
    enum class RandomProcess
    {
        // WhiteNoise,     ///< White noise
        // RandomConstant, ///< Random constant

        RandomWalk,   ///< Random Walk
        GaussMarkov1, ///< Gauss-Markov 1st Order

        // GaussMarkov2,   ///< Gauss-Markov 2nd Order
        // GaussMarkov3,   ///< Gauss-Markov 3rd Order
    };

    /// @brief Random Process used to estimate the accelerometer biases
    RandomProcess _randomProcessAccel = RandomProcess::RandomWalk;
    /// @brief Random Process used to estimate the gyroscope biases
    RandomProcess _randomProcessGyro = RandomProcess::RandomWalk;

    // ###########################################################################################################

    /// Possible Units for the initial covariance for the receiver clock phase drift (standard deviation σ or Variance σ²)
    enum class InitCovarianceClockPhaseUnit
    {
        m2, ///< Variance [m²]
        s2, ///< Variance [s²]
        m,  ///< Standard deviation [m]
        s   ///< Standard deviation [s]
    };
    /// Gui selection for the Unit of the initial covariance of the receiver clock phase drift
    InitCovarianceClockPhaseUnit _initCovariancePhaseUnit = InitCovarianceClockPhaseUnit::m;

    /// GUI selection of the initial covariance of the receiver clock phase drift (standard deviation σ or Variance σ²)
    double _initCovariancePhase{ 5 };

    // ###########################################################################################################

    /// Possible Units for the initial covariance for the receiver clock frequency drift (standard deviation σ or Variance σ²)
    enum class InitCovarianceClockFreqUnit
    {
        m2_s2, ///< Variance [m²/s²]
        m_s,   ///< Standard deviation [m/s]
    };
    /// Gui selection for the Unit of the initial covariance of the receiver clock frequency drift
    InitCovarianceClockFreqUnit _initCovarianceFreqUnit = InitCovarianceClockFreqUnit::m_s;

    /// GUI selection of the initial covariance of the receiver clock frequency drift (standard deviation σ or Variance σ²)
    double _initCovarianceFreq{ 5 };

    // ###########################################################################################################

    /// Possible Units for the initial covariance for the position (standard deviation σ or Variance σ²)
    enum class InitCovariancePositionUnit
    {
        rad2_rad2_m2, ///< Variance LatLonAlt^2 [rad^2, rad^2, m^2]
        rad_rad_m,    ///< Standard deviation LatLonAlt [rad, rad, m]
        meter2,       ///< Variance NED [m^2, m^2, m^2]
        meter,        ///< Standard deviation NED [m, m, m]
    };
    /// Gui selection for the Unit of the initial covariance for the position
    InitCovariancePositionUnit _initCovariancePositionUnit = InitCovariancePositionUnit::meter;

    /// GUI selection of the initial covariance diagonal values for position (standard deviation σ or Variance σ²)
    Eigen::Vector3d _initCovariancePosition{ 100, 100, 100 };

    // ###########################################################################################################

    /// Possible Units for the initial covariance for the velocity (standard deviation σ or Variance σ²)
    enum class InitCovarianceVelocityUnit
    {
        m2_s2, ///< Variance [m^2/s^2]
        m_s,   ///< Standard deviation [m/s]
    };
    /// Gui selection for the Unit of the initial covariance for the velocity
    InitCovarianceVelocityUnit _initCovarianceVelocityUnit = InitCovarianceVelocityUnit::m_s;

    /// GUI selection of the initial covariance diagonal values for velocity (standard deviation σ or Variance σ²)
    Eigen::Vector3d _initCovarianceVelocity{ 10, 10, 10 };

    // ###########################################################################################################

    /// Possible Units for the initial covariance for the attitude angles (standard deviation σ or Variance σ²)
    enum class InitCovarianceAttitudeAnglesUnit
    {
        rad2, ///< Variance [rad^2]
        deg2, ///< Variance [deg^2]
        rad,  ///< Standard deviation [rad]
        deg,  ///< Standard deviation [deg]
    };
    /// Gui selection for the Unit of the initial covariance for the attitude angles
    InitCovarianceAttitudeAnglesUnit _initCovarianceAttitudeAnglesUnit = InitCovarianceAttitudeAnglesUnit::deg;

    /// GUI selection of the initial covariance diagonal values for attitude angles (standard deviation σ or Variance σ²)
    Eigen::Vector3d _initCovarianceAttitudeAngles{ 10, 10, 10 };

    // ###########################################################################################################

    /// Possible Units for the initial covariance for the accelerometer biases (standard deviation σ or Variance σ²)
    enum class InitCovarianceBiasAccelUnit
    {
        m2_s4, ///< Variance [m^2/s^4]
        m_s2,  ///< Standard deviation [m/s^2]
    };
    /// Gui selection for the Unit of the initial covariance for the accelerometer biases
    InitCovarianceBiasAccelUnit _initCovarianceBiasAccelUnit = InitCovarianceBiasAccelUnit::m_s2;

    /// GUI selection of the initial covariance diagonal values for accelerometer biases (standard deviation σ or Variance σ²)
    Eigen::Vector3d _initCovarianceBiasAccel{ 1, 1, 1 };

    // ###########################################################################################################

    /// Possible Units for the initial covariance for the gyroscope biases (standard deviation σ or Variance σ²)
    enum class InitCovarianceBiasGyroUnit
    {
        rad2_s2, ///< Variance [rad²/s²]
        deg2_s2, ///< Variance [deg²/s²]
        rad_s,   ///< Standard deviation [rad/s]
        deg_s,   ///< Standard deviation [deg/s]
    };
    /// Gui selection for the Unit of the initial covariance for the gyroscope biases
    InitCovarianceBiasGyroUnit _initCovarianceBiasGyroUnit = InitCovarianceBiasGyroUnit::deg_s;

    /// GUI selection of the initial covariance diagonal values for gyroscope biases (standard deviation σ or Variance σ²)
    Eigen::Vector3d _initCovarianceBiasGyro{ 0.5, 0.5, 0.5 };

    // ###########################################################################################################

    /// GUI option for the Phi calculation algorithm
    enum class PhiCalculationAlgorithm
    {
        Exponential, ///< Van-Loan
        Taylor,      ///< Taylor
    };
    /// GUI option for the Phi calculation algorithm
    PhiCalculationAlgorithm _phiCalculationAlgorithm = PhiCalculationAlgorithm::Taylor;

    /// GUI option for the order of the Taylor polynom to calculate the Phi matrix
    int _phiCalculationTaylorOrder = 2;

    /// GUI option for the Q calculation algorithm
    enum class QCalculationAlgorithm
    {
        VanLoan, ///< Van-Loan
        Taylor1, ///< Taylor
    };
    /// GUI option for the Q calculation algorithm
    QCalculationAlgorithm _qCalculationAlgorithm = QCalculationAlgorithm::Taylor1;

    // ###########################################################################################################

    /// Possible Units for the initial accelerometer biases
    enum class InitBiasAccelUnit
    {
        m_s2, ///< acceleration [m/s^2]
    };
    /// Gui selection for the unit of the initial accelerometer biases
    InitBiasAccelUnit _initBiasAccelUnit = InitBiasAccelUnit::m_s2;

    /// GUI selection of the initial accelerometer biases
    Eigen::Vector3d _initBiasAccel{ 0.34, -0.11, 2.43 };

    // ###########################################################################################################

    /// Possible Units for the initial gyroscope biases
    enum class InitBiasGyroUnit
    {
        rad_s, ///< angular rate [rad/s]
        deg_s, ///< angular rate [deg/s]
    };
    /// Gui selection for the unit of the initial gyroscope biases
    InitBiasGyroUnit _initBiasGyroUnit = InitBiasGyroUnit::deg_s;

    /// GUI selection of the initial gyroscope biases
    Eigen::Vector3d _initBiasGyro{ -0.062, 0.021, -0.03 };

    // ###########################################################################################################
    //                                                Prediction
    // ###########################################################################################################

    // ------------------------------------------- System matrix 𝐅 ----------------------------------------------

    /// @brief Calculates the system matrix 𝐅 for the local navigation frame
    /// @param[in] n_Quat_b Attitude of the body with respect to n-system
    /// @param[in] b_specForce_ib Specific force of the body with respect to inertial frame in [m / s^2], resolved in body coordinates
    /// @param[in] n_omega_in Angular rate of navigation system with respect to the inertial system [rad / s], resolved in navigation coordinates.
    /// @param[in] n_velocity Velocity in n-system in [m / s]
    /// @param[in] lla_position Position as Lat Lon Alt in [rad rad m]
    /// @param[in] R_N Meridian radius of curvature in [m]
    /// @param[in] R_E Prime vertical radius of curvature (East/West) [m]
    /// @param[in] g_0 Magnitude of the gravity vector in [m/s^2] (see \cite Groves2013 Groves, ch. 2.4.7, eq. 2.135, p. 70)
    /// @param[in] r_eS_e Geocentric radius. The distance of a point on the Earth's surface from the center of the Earth in [m]
    /// @param[in] tau_bad Correleation length for the accelerometer in [s]
    /// @param[in] tau_bgd Correleation length for the gyroscope in [s]
    /// @note See Groves (2013) chapter 14.2.4, equation (14.63) and chapter 9.4.2, equation (9.149)
    [[nodiscard]] Eigen::Matrix<double, 17, 17> n_systemMatrix_F(const Eigen::Quaterniond& n_Quat_b,
                                                                 const Eigen::Vector3d& b_specForce_ib,
                                                                 const Eigen::Vector3d& n_omega_in,
                                                                 const Eigen::Vector3d& n_velocity,
                                                                 const Eigen::Vector3d& lla_position,
                                                                 double R_N,
                                                                 double R_E,
                                                                 double g_0,
                                                                 double r_eS_e,
                                                                 const Eigen::Vector3d& tau_bad,
                                                                 const Eigen::Vector3d& tau_bgd) const;

    /// @brief Calculates the system matrix 𝐅 for the ECEF frame
    /// @param[in] e_Quat_b Attitude of the body with respect to e-system
    /// @param[in] b_specForce_ib Specific force of the body with respect to inertial frame in [m / s^2], resolved in body coordinates
    /// @param[in] e_position Position in ECEF coordinates in [m]
    /// @param[in] e_gravitation Gravitational acceleration in [m/s^2]
    /// @param[in] r_eS_e Geocentric radius. The distance of a point on the Earth's surface from the center of the Earth in [m]
    /// @param[in] e_omega_ie Angular velocity of Earth with respect to inertial system, represented in e-sys in [rad/s]
    /// @param[in] tau_bad Correleation length for the accelerometer in [s]
    /// @param[in] tau_bgd Correleation length for the gyroscope in [s]
    /// @note See Groves (2013) chapter 14.2.3, equation (14.48) and chapter 9.4.2, equation (9.148)
    [[nodiscard]] Eigen::Matrix<double, 17, 17> e_systemMatrix_F(const Eigen::Quaterniond& e_Quat_b,
                                                                 const Eigen::Vector3d& b_specForce_ib,
                                                                 const Eigen::Vector3d& e_position,
                                                                 const Eigen::Vector3d& e_gravitation,
                                                                 double r_eS_e,
                                                                 const Eigen::Vector3d& e_omega_ie,
                                                                 const Eigen::Vector3d& tau_bad,
                                                                 const Eigen::Vector3d& tau_bgd) const;

    // ----------------------------- Noise input matrix 𝐆 & Noise scale matrix 𝐖 -------------------------------
    // ----------------------------------- System noise covariance matrix 𝐐 -------------------------------------

    /// @brief Calculates the noise input matrix 𝐆
    /// @param[in] ien_Quat_b Quaternion from body frame to {i,e,n} frame
    /// @note See \cite Groves2013 Groves, ch. 14.2.6, eq. 14.79, p. 590 (INS part) and ch. 9.4.2, eq. 9.151, p. 416 (GNSS part)
    [[nodiscard]] static Eigen::Matrix<double, 17, 14> noiseInputMatrix_G(const Eigen::Quaterniond& ien_Quat_b);

    /// @brief Calculates the noise scale matrix 𝐖
    /// @param[in] sigma2_ra Variance of the noise on the accelerometer specific-force measurements
    /// @param[in] sigma2_rg Variance of the noise on the gyro angular-rate measurements
    /// @param[in] sigma2_bad Variance of the accelerometer dynamic bias
    /// @param[in] sigma2_bgd Variance of the gyro dynamic bias
    /// @param[in] tau_bad Correleation length for the accelerometer in [s]
    /// @param[in] tau_bgd Correleation length for the gyroscope in [s]
    /// @param[in] sigma2_cPhi Variance of the noise on the clock offset in [m]
    /// @param[in] sigma2_cf Variance of the noise on the clock frequency in [m/s]
    /// @note See \cite Groves2013 Groves, ch. 14.2.6, eq. 14.79, p. 590 (INS part) and ch. 9.4.2, eq. 9.151, p. 416 (GNSS part)
    [[nodiscard]] Eigen::Matrix<double, 14, 14> noiseScaleMatrix_W(const Eigen::Vector3d& sigma2_ra, const Eigen::Vector3d& sigma2_rg,
                                                                   const Eigen::Vector3d& sigma2_bad, const Eigen::Vector3d& sigma2_bgd,
                                                                   const Eigen::Vector3d& tau_bad, const Eigen::Vector3d& tau_bgd,
                                                                   const double& sigma2_cPhi, const double& sigma2_cf);

    /// @brief System noise covariance matrix 𝐐_{k-1}
    /// @param[in] sigma2_ra Variance of the noise on the accelerometer specific-force measurements
    /// @param[in] sigma2_rg Variance of the noise on the gyro angular-rate measurements
    /// @param[in] sigma2_bad Variance of the accelerometer dynamic bias
    /// @param[in] sigma2_bgd Variance of the gyro dynamic bias
    /// @param[in] tau_bad Correleation length for the accelerometer in [s]
    /// @param[in] tau_bgd Correleation length for the gyroscope in [s]
    /// @param[in] sigma2_cPhi Variance of the noise on the clock offset in [m]
    /// @param[in] sigma2_cf Variance of the noise on the clock frequency in [m/s]
    /// @param[in] n_F_21 Submatrix 𝐅_21 of the system matrix 𝐅
    /// @param[in] T_rn_p Conversion matrix between cartesian and curvilinear perturbations to the position
    /// @param[in] n_Dcm_b Direction Cosine Matrix from body to navigation coordinates
    /// @param[in] tau_s Time interval in [s]
    /// @return The 15x15 matrix of system noise covariances
    [[nodiscard]] static Eigen::Matrix<double, 17, 17> n_systemNoiseCovarianceMatrix_Q(const Eigen::Vector3d& sigma2_ra, const Eigen::Vector3d& sigma2_rg,
                                                                                       const Eigen::Vector3d& sigma2_bad, const Eigen::Vector3d& sigma2_bgd,
                                                                                       const Eigen::Vector3d& tau_bad, const Eigen::Vector3d& tau_bgd,
                                                                                       const double& sigma2_cPhi, const double& sigma2_cf,
                                                                                       const Eigen::Matrix3d& n_F_21, const Eigen::Matrix3d& T_rn_p,
                                                                                       const Eigen::Matrix3d& n_Dcm_b, const double& tau_s);

    /// @brief System noise covariance matrix 𝐐_{k-1}
    /// @param[in] sigma2_ra Variance of the noise on the accelerometer specific-force measurements
    /// @param[in] sigma2_rg Variance of the noise on the gyro angular-rate measurements
    /// @param[in] sigma2_bad Variance of the accelerometer dynamic bias
    /// @param[in] sigma2_bgd Variance of the gyro dynamic bias
    /// @param[in] tau_bad Correleation length for the accelerometer in [s]
    /// @param[in] tau_bgd Correleation length for the gyroscope in [s]
    /// @param[in] sigma2_cPhi Variance of the noise on the clock offset in [m]
    /// @param[in] sigma2_cf Variance of the noise on the clock frequency in [m/s]
    /// @param[in] e_F_21 Submatrix 𝐅_21 of the system matrix 𝐅
    /// @param[in] e_Dcm_b Direction Cosine Matrix from body to Earth coordinates
    /// @param[in] tau_s Time interval in [s]
    /// @return The 15x15 matrix of system noise covariances
    [[nodiscard]] static Eigen::Matrix<double, 17, 17> e_systemNoiseCovarianceMatrix_Q(const Eigen::Vector3d& sigma2_ra, const Eigen::Vector3d& sigma2_rg,
                                                                                       const Eigen::Vector3d& sigma2_bad, const Eigen::Vector3d& sigma2_bgd,
                                                                                       const Eigen::Vector3d& tau_bad, const Eigen::Vector3d& tau_bgd,
                                                                                       const double& sigma2_cPhi, const double& sigma2_cf,
                                                                                       const Eigen::Matrix3d& e_F_21,
                                                                                       const Eigen::Matrix3d& e_Dcm_b, const double& tau_s);

    // --------------------------------------- Error covariance matrix P -----------------------------------------

    /// @brief Initial error covariance matrix P_0
    /// @param[in] variance_angles Initial Covariance of the attitude angles in [rad²]
    /// @param[in] variance_vel Initial Covariance of the velocity in [m²/s²]
    /// @param[in] variance_pos Initial Covariance of the position in [rad² rad² m²] n-frame / [m²] i,e-frame
    /// @param[in] variance_accelBias Initial Covariance of the accelerometer biases in [m^2/s^4]
    /// @param[in] variance_gyroBias Initial Covariance of the gyroscope biases in [rad^2/s^2]
    /// @param[in] variance_clkPhase Initial Covariance of the receiver clock phase drift in [m²]
    /// @param[in] variance_clkFreq Initial Covariance of the receiver clock frequency-drift in [m²/s²]
    /// @return The 17x17 matrix of initial state variances
    [[nodiscard]] Eigen::Matrix<double, 17, 17> initialErrorCovarianceMatrix_P0(const Eigen::Vector3d& variance_angles,
                                                                                const Eigen::Vector3d& variance_vel,
                                                                                const Eigen::Vector3d& variance_pos,
                                                                                const Eigen::Vector3d& variance_accelBias,
                                                                                const Eigen::Vector3d& variance_gyroBias,
                                                                                const double& variance_clkPhase,
                                                                                const double& variance_clkFreq) const;

    // ###########################################################################################################
    //                                                  Update
    // ###########################################################################################################

    /// @brief Measurement matrix for GNSS observations at timestep k, represented in navigation coordinates
    /// @param[in] R_N Meridian radius of curvature in [m]
    /// @param[in] R_E Prime vertical radius of curvature (East/West) [m]
    /// @param[in] lla_position Position as Lat Lon Alt in [rad rad m]
    /// @param[in] n_lineOfSightUnitVectors Vector of line-of-sight unit vectors to each satellite in NED frame coordinates (Groves ch. 8.5.3, eq. 8.41, p. 341)
    /// @param[in] pseudoRangeRateObservations Pseudorange-Rate observations
    /// @return The 2*m x 17 measurement matrix 𝐇 (m: number of satellites)
    [[nodiscard]] static Eigen::MatrixXd n_measurementMatrix_H(const double& R_N,
                                                               const double& R_E,
                                                               const Eigen::Vector3d& lla_position,
                                                               const std::vector<Eigen::Vector3d>& n_lineOfSightUnitVectors,
                                                               std::vector<double>& pseudoRangeRateObservations);

    /// @brief Measurement noise covariance matrix 𝐑
    /// @param[in] sigma_rhoZ Standard deviation of the zenith pseudo-range error in [m]
    /// @param[in] sigma_rZ Standard deviation of the zenith pseudo-range-rate error in [m/s]
    /// @param[in] satElevation Elevation angles of all m satellites in [rad]
    /// @return The 2*m x 2*m measurement covariance matrix 𝐑 (m: number of satellites)
    [[nodiscard]] static Eigen::MatrixXd measurementNoiseCovariance_R(const double& sigma_rhoZ,
                                                                      const double& sigma_rZ,
                                                                      const std::vector<double>& satElevation);

    /// @brief Calculates the elements for the measurement noise covariance matrix 𝐑
    /// @param[in] satElevation Elevation angles of all m satellites in [rad]
    /// @param[in] sigma_Z Standard deviation of the zenith pseudo-range error in [m] or the zenith pseudo-range-rate error in [m/s]
    /// @param[in] sigma_C Standard deviation of the clock pseudo-range error in [m] or the clock pseudo-range-rate error in [m/s]
    /// @param[in] sigma_A Standard deviation of the antenna pseudo-range error in [m] or the antenna pseudo-range-rate error in [m/s]
    /// @param[in] CN0 Carrier-to-Noise density of all m satellites in [dBHz]
    /// @param[in] rangeAccel Range acceleration of all m satellites in [m / s^2]
    /// @return Variance of the pseudo-range error in [m²] or pseudo-range-rate error in [m²/s²]
    [[nodiscard]] static double sigma2(const double& satElevation,
                                       const double& sigma_Z,
                                       const double& sigma_C,
                                       const double& sigma_A,
                                       const double& CN0,
                                       const double& rangeAccel);

    /// @brief Measurement innovation vector 𝜹𝐳
    /// @param[in] pseudoRangeObservations Vector of Pseudorange observations from all available satellites in [m]
    /// @param[in] pseudoRangeEstimates  Vector of Pseudorange estimates from all available satellites in [m/s]
    /// @param[in] pseudoRangeRateObservations  Vector of Pseudorange-Rate observations from all available satellites in [m]
    /// @param[in] pseudoRangeRateEstimates  Vector of Pseudorange-Rate estimates from all available satellites in [m/s]
    /// @return The 2*m x1 measurement innovation vector 𝜹𝐳 (m: number of satellites)
    [[nodiscard]] static Eigen::MatrixXd measurementInnovation_dz(const std::vector<double>& pseudoRangeObservations,
                                                                  const std::vector<double>& pseudoRangeEstimates,
                                                                  const std::vector<double>& pseudoRangeRateObservations,
                                                                  const std::vector<double>& pseudoRangeRateEstimates);
};

} // namespace NAV