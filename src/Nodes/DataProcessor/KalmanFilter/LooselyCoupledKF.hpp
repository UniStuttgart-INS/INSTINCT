// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file LooselyCoupledKF.hpp
/// @brief Kalman Filter class for the loosely coupled INS/GNSS integration
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @author M. Maier (marcel.maier@ins.uni-stuttgart.de)
/// @date 2021-08-04

#pragma once

#include "internal/Node/Node.hpp"

#include "Navigation/Time/InsTime.hpp"
#include "Navigation/INS/InertialIntegrator.hpp"

#include "NodeData/IMU/ImuObs.hpp"
#include "NodeData/State/PosVelAtt.hpp"

#include "Navigation/Math/KeyedKalmanFilter.hpp"

namespace NAV
{
/// @brief Loosely-coupled Kalman Filter for INS/GNSS integration
class LooselyCoupledKF : public Node
{
  public:
    /// @brief Default constructor
    LooselyCoupledKF();
    /// @brief Destructor
    ~LooselyCoupledKF() override;
    /// @brief Copy constructor
    LooselyCoupledKF(const LooselyCoupledKF&) = delete;
    /// @brief Move constructor
    LooselyCoupledKF(LooselyCoupledKF&&) = delete;
    /// @brief Copy assignment operator
    LooselyCoupledKF& operator=(const LooselyCoupledKF&) = delete;
    /// @brief Move assignment operator
    LooselyCoupledKF& operator=(LooselyCoupledKF&&) = delete;
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
    /// @param[in] j Json object with the node state
    void restore(const json& j) override;

    /// @brief State Keys of the Kalman filter
    enum KFStates
    {
        Roll,     ///< Roll
        Pitch,    ///< Pitch
        Yaw,      ///< Yaw
        VelN,     ///< Velocity North
        VelE,     ///< Velocity East
        VelD,     ///< Velocity Down
        PosLat,   ///< Latitude
        PosLon,   ///< Longitude
        PosAlt,   ///< Altitude
        AccBiasX, ///< Accelerometer Bias X
        AccBiasY, ///< Accelerometer Bias Y
        AccBiasZ, ///< Accelerometer Bias Z
        GyrBiasX, ///< Gyroscope Bias X
        GyrBiasY, ///< Gyroscope Bias Y
        GyrBiasZ, ///< Gyroscope Bias Z

        Psi_eb_1 = Roll,  ///< Angle between Earth and Body frame around 1. axis
        Psi_eb_2 = Pitch, ///< Angle between Earth and Body frame around 2. axis
        Psi_eb_3 = Yaw,   ///< Angle between Earth and Body frame around 3. axis
        VelX = VelN,      ///< ECEF Velocity X
        VelY = VelE,      ///< ECEF Velocity Y
        VelZ = VelD,      ///< ECEF Velocity Z
        PosX = PosLat,    ///< ECEF Position X
        PosY = PosLon,    ///< ECEF Position Y
        PosZ = PosAlt,    ///< ECEF Position Z
    };

    /// @brief Measurement Keys of the Kalman filter
    enum KFMeas
    {
        dPosLat, ///< Latitude difference
        dPosLon, ///< Longitude difference
        dPosAlt, ///< Altitude difference
        dVelN,   ///< Velocity North difference
        dVelE,   ///< Velocity East difference
        dVelD,   ///< Velocity Down difference

        dPosX = dPosLat, ///< ECEF Position X difference
        dPosY = dPosLon, ///< ECEF Position Y difference
        dPosZ = dPosAlt, ///< ECEF Position Z difference
        dVelX = dVelN,   ///< ECEF Velocity X difference
        dVelY = dVelE,   ///< ECEF Velocity Y difference
        dVelZ = dVelD,   ///< ECEF Velocity Z difference
    };

  private:
    constexpr static size_t INPUT_PORT_INDEX_IMU = 0;              ///< @brief Flow (ImuObs)
    constexpr static size_t INPUT_PORT_INDEX_GNSS = 1;             ///< @brief Flow (PosVel)
    constexpr static size_t INPUT_PORT_INDEX_POS_VEL_ATT_INIT = 2; ///< @brief Flow (PosVelAtt)
    constexpr static size_t OUTPUT_PORT_INDEX_SOLUTION = 0;        ///< @brief Flow (InsGnssLCKFSolution)

    /// @brief Initialize the node
    bool initialize() override;

    /// @brief Deinitialize the node
    void deinitialize() override;

    /// @brief Invoke the callback with a PosVelAtt solution (without LCKF specific output)
    /// @param[in] posVelAtt PosVelAtt solution
    void invokeCallbackWithPosVelAtt(const PosVelAtt& posVelAtt);

    /// @brief Receive Function for the IMU observation
    /// @param[in] queue Queue with all the received data messages
    /// @param[in] pinIdx Index of the pin the data is received on
    void recvImuObservation(InputPin::NodeDataQueue& queue, size_t pinIdx);

    /// @brief Receive Function for the PosVel observation
    /// @param[in] queue Queue with all the received data messages
    /// @param[in] pinIdx Index of the pin the data is received on
    void recvPosVelObservation(InputPin::NodeDataQueue& queue, size_t pinIdx);

    /// @brief Receive Function for the PosVelAtt observation
    /// @param[in] queue Queue with all the received data messages
    /// @param[in] pinIdx Index of the pin the data is received on
    void recvPosVelAttInit(InputPin::NodeDataQueue& queue, size_t pinIdx);

    /// @brief Predicts the state from the InertialNavSol
    /// @param[in] inertialNavSol Inertial navigation solution triggering the prediction
    /// @param[in] tau_i Time since the last prediction in [s]
    /// @param[in] imuPos IMU platform frame position with respect to body frame
    void looselyCoupledPrediction(const std::shared_ptr<const PosVelAtt>& inertialNavSol, double tau_i, const ImuPos& imuPos);

    /// @brief Updates the predicted state from the InertialNavSol with the PosVel observation
    /// @param[in] posVelObs PosVel measurement triggering the update
    void looselyCoupledUpdate(const std::shared_ptr<const PosVel>& posVelObs);

    /// Add or remove the external PVA Init pin
    void updateExternalPvaInitPin();

    /// @brief Inertial Integrator
    InertialIntegrator _inertialIntegrator;
    /// Prefer the raw acceleration measurements over the deltaVel & deltaTheta values
    bool _preferAccelerationOverDeltaMeasurements = false;

    /// Last received IMU observation (to get ImuPos)
    std::shared_ptr<const ImuObs> _lastImuObs = nullptr;

    /// Roll, Pitch and Yaw angles in [deg] used for initialization if not taken from separate pin
    std::array<double, 3> _initalRollPitchYaw{};
    /// Whether to initialize the state over an external pin
    bool _initializeStateOverExternalPin{};
    /// Time from the external init
    InsTime _externalInitTime;

    /// Whether the accumulated biases have been initialized in the 'inertialIntegrator'
    bool _initialSensorBiasesApplied = false;

    /// @brief Vector with all state keys
    inline static const std::vector<KFStates> States = { KFStates::Roll, KFStates::Pitch, KFStates::Yaw,
                                                         KFStates::VelN, KFStates::VelE, KFStates::VelD,
                                                         KFStates::PosLat, KFStates::PosLon, KFStates::PosAlt,
                                                         KFStates::AccBiasX, KFStates::AccBiasY, KFStates::AccBiasZ,
                                                         KFStates::GyrBiasX, KFStates::GyrBiasY, KFStates::GyrBiasZ };
    /// @brief All position keys
    inline static const std::vector<KFStates> KFPos = { KFStates::PosLat, KFStates::PosLon, KFStates::PosAlt };
    /// @brief All velocity keys
    inline static const std::vector<KFStates> KFVel = { KFStates::VelN, KFStates::VelE, KFStates::VelD };
    /// @brief All attitude keys
    inline static const std::vector<KFStates> KFAtt = { KFStates::Roll, KFStates::Pitch, KFStates::Yaw };
    /// @brief All acceleration bias keys
    inline static const std::vector<KFStates> KFAccBias = { KFStates::AccBiasX, KFStates::AccBiasY, KFStates::AccBiasZ };
    /// @brief All gyroscope bias keys
    inline static const std::vector<KFStates> KFGyrBias = { KFStates::GyrBiasX, KFStates::GyrBiasY, KFStates::GyrBiasZ };

    /// @brief All position and velocity keys
    inline static const std::vector<KFStates> KFPosVel = { KFStates::PosLat, KFStates::PosLon, KFStates::PosAlt,
                                                           KFStates::VelN, KFStates::VelE, KFStates::VelD };

    /// @brief Vector with all measurement keys
    inline static const std::vector<KFMeas> Meas = { KFMeas::dPosLat, KFMeas::dPosLon, KFMeas::dPosAlt, KFMeas::dVelN, KFMeas::dVelE, KFMeas::dVelD };
    /// @brief All position difference keys
    inline static const std::vector<KFMeas> dPos = { KFMeas::dPosLat, KFMeas::dPosLon, KFMeas::dPosAlt };
    /// @brief All velocity difference keys
    inline static const std::vector<KFMeas> dVel = { KFMeas::dVelN, KFMeas::dVelE, KFMeas::dVelD };

    /// Kalman Filter representation
    KeyedKalmanFilterD<KFStates, KFMeas> _kalmanFilter{ States, Meas };

    // #########################################################################################################################################
    //                                                              GUI settings
    // #########################################################################################################################################

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

    /// Possible Units for the GNSS measurement uncertainty for the position (standard deviation σ or Variance σ²)
    enum class GnssMeasurementUncertaintyPositionUnit
    {
        rad2_rad2_m2, ///< Variance LatLonAlt^2 [rad^2, rad^2, m^2]
        rad_rad_m,    ///< Standard deviation LatLonAlt [rad, rad, m]
        meter2,       ///< Variance NED [m^2, m^2, m^2]
        meter,        ///< Standard deviation NED [m, m, m]
    };
    /// Gui selection for the Unit of the GNSS measurement uncertainty for the position
    GnssMeasurementUncertaintyPositionUnit _gnssMeasurementUncertaintyPositionUnit = GnssMeasurementUncertaintyPositionUnit::meter;

    /// @brief GUI selection of the GNSS position measurement uncertainty (standard deviation σ or Variance σ²).
    /// SPP accuracy approx. 3m in horizontal direction and 3 times worse in vertical direction
    Eigen::Vector3d _gnssMeasurementUncertaintyPosition{ 0.3, 0.3, 0.3 * 3 };

    /// Whether to override the position uncertainty or use the one included in the measurement
    bool _gnssMeasurementUncertaintyPositionOverride = false;

    // ###########################################################################################################

    /// Possible Units for the GNSS measurement uncertainty for the velocity (standard deviation σ or Variance σ²)
    enum class GnssMeasurementUncertaintyVelocityUnit
    {
        m2_s2, ///< Variance [m^2/s^2]
        m_s,   ///< Standard deviation [m/s]
    };
    /// Gui selection for the Unit of the GNSS measurement uncertainty for the velocity
    GnssMeasurementUncertaintyVelocityUnit _gnssMeasurementUncertaintyVelocityUnit = GnssMeasurementUncertaintyVelocityUnit::m_s;

    /// GUI selection of the GNSS NED velocity measurement uncertainty (standard deviation σ or Variance σ²)
    Eigen::Vector3d _gnssMeasurementUncertaintyVelocity{ 0.5, 0.5, 0.5 };

    /// Whether to override the velocity uncertainty or use the one included in the measurement
    bool _gnssMeasurementUncertaintyVelocityOverride = false;

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
    Eigen::Vector3d _initCovariancePosition{ 100.0, 100.0, 100.0 };

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
    Eigen::Vector3d _initCovarianceVelocity{ 10.0, 10.0, 10.0 };

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
    Eigen::Vector3d _initCovarianceAttitudeAngles{ 10.0, 10.0, 10.0 };

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
    Eigen::Vector3d _initCovarianceBiasAccel{ 1.0, 1.0, 1.0 };

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

    /// Possible Units for the initial accelerometer biases
    enum class InitBiasAccelUnit
    {
        m_s2, ///< acceleration [m/s^2]
    };
    /// Gui selection for the unit of the initial accelerometer biases
    InitBiasAccelUnit _initBiasAccelUnit = InitBiasAccelUnit::m_s2;

    /// GUI selection of the initial accelerometer biases
    Eigen::Vector3d _initBiasAccel{ 0.0, 0.0, 0.0 };

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
    Eigen::Vector3d _initBiasGyro{ 0.0, 0.0, 0.0 };

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
    //                                                Prediction
    // ###########################################################################################################

    // ###########################################################################################################
    //                                             System matrix 𝐅
    // ###########################################################################################################

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
    /// @param[in] tau_bad Correlation length for the accelerometer in [s]
    /// @param[in] tau_bgd Correlation length for the gyroscope in [s]
    /// @note See Groves (2013) chapter 14.2.4, equation (14.63)
    [[nodiscard]] KeyedMatrix<double, KFStates, KFStates, 15, 15> n_systemMatrix_F(const Eigen::Quaterniond& n_Quat_b,
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
    /// @param[in] tau_bad Correlation length for the accelerometer in [s]
    /// @param[in] tau_bgd Correlation length for the gyroscope in [s]
    /// @note See Groves (2013) chapter 14.2.3, equation (14.48)
    [[nodiscard]] KeyedMatrix<double, KFStates, KFStates, 15, 15> e_systemMatrix_F(const Eigen::Quaterniond& e_Quat_b,
                                                                                   const Eigen::Vector3d& b_specForce_ib,
                                                                                   const Eigen::Vector3d& e_position,
                                                                                   const Eigen::Vector3d& e_gravitation,
                                                                                   double r_eS_e,
                                                                                   const Eigen::Vector3d& e_omega_ie,
                                                                                   const Eigen::Vector3d& tau_bad,
                                                                                   const Eigen::Vector3d& tau_bgd) const;

    // ###########################################################################################################
    //                                    Noise input matrix 𝐆 & Noise scale matrix 𝐖
    //                                     System noise covariance matrix 𝐐
    // ###########################################################################################################

    /// @brief Calculates the noise input matrix 𝐆
    /// @param[in] ien_Quat_b Quaternion from body frame to {i,e,n} frame
    /// @note See \cite Groves2013 Groves, ch. 14.2.6, eq. 14.79, p. 590
    [[nodiscard]] static KeyedMatrix<double, KFStates, KFStates, 15, 15> noiseInputMatrix_G(const Eigen::Quaterniond& ien_Quat_b);

    /// @brief Calculates the noise scale matrix 𝐖
    /// @param[in] sigma_ra Standard deviation of the noise on the accelerometer specific-force measurements
    /// @param[in] sigma_rg Standard deviation of the noise on the gyro angular-rate measurements
    /// @param[in] sigma_bad Standard deviation of the accelerometer dynamic bias
    /// @param[in] sigma_bgd Standard deviation of the gyro dynamic bias
    /// @param[in] tau_bad Correlation length for the accelerometer in [s]
    /// @param[in] tau_bgd Correlation length for the gyroscope in [s]
    /// @note See \cite Groves2013 Groves, ch. 14.2.6, eq. 14.79, p. 590
    [[nodiscard]] Eigen::Matrix<double, 15, 15> noiseScaleMatrix_W(const Eigen::Vector3d& sigma_ra, const Eigen::Vector3d& sigma_rg,
                                                                   const Eigen::Vector3d& sigma_bad, const Eigen::Vector3d& sigma_bgd,
                                                                   const Eigen::Vector3d& tau_bad, const Eigen::Vector3d& tau_bgd);

    /// @brief System noise covariance matrix 𝐐_{k-1}
    /// @param[in] sigma2_ra Variance of the noise on the accelerometer specific-force measurements
    /// @param[in] sigma2_rg Variance of the noise on the gyro angular-rate measurements
    /// @param[in] sigma2_bad Variance of the accelerometer dynamic bias
    /// @param[in] sigma2_bgd Variance of the gyro dynamic bias
    /// @param[in] tau_bad Correlation length for the accelerometer in [s]
    /// @param[in] tau_bgd Correlation length for the gyroscope in [s]
    /// @param[in] n_F_21 Submatrix 𝐅_21 of the system matrix 𝐅
    /// @param[in] T_rn_p Conversion matrix between cartesian and curvilinear perturbations to the position
    /// @param[in] n_Dcm_b Direction Cosine Matrix from body to navigation coordinates
    /// @param[in] tau_s Time interval in [s]
    /// @return The 15x15 matrix of system noise covariances
    [[nodiscard]] static KeyedMatrix<double, KFStates, KFStates, 15, 15> n_systemNoiseCovarianceMatrix_Q(const Eigen::Vector3d& sigma2_ra, const Eigen::Vector3d& sigma2_rg,
                                                                                                         const Eigen::Vector3d& sigma2_bad, const Eigen::Vector3d& sigma2_bgd,
                                                                                                         const Eigen::Vector3d& tau_bad, const Eigen::Vector3d& tau_bgd,
                                                                                                         const Eigen::Matrix3d& n_F_21, const Eigen::Matrix3d& T_rn_p,
                                                                                                         const Eigen::Matrix3d& n_Dcm_b, const double& tau_s);

    /// @brief System noise covariance matrix 𝐐_{k-1}
    /// @param[in] sigma2_ra Variance of the noise on the accelerometer specific-force measurements
    /// @param[in] sigma2_rg Variance of the noise on the gyro angular-rate measurements
    /// @param[in] sigma2_bad Variance of the accelerometer dynamic bias
    /// @param[in] sigma2_bgd Variance of the gyro dynamic bias
    /// @param[in] tau_bad Correlation length for the accelerometer in [s]
    /// @param[in] tau_bgd Correlation length for the gyroscope in [s]
    /// @param[in] e_F_21 Submatrix 𝐅_21 of the system matrix 𝐅
    /// @param[in] e_Dcm_b Direction Cosine Matrix from body to Earth coordinates
    /// @param[in] tau_s Time interval in [s]
    /// @return The 15x15 matrix of system noise covariances
    [[nodiscard]] static KeyedMatrix<double, KFStates, KFStates, 15, 15> e_systemNoiseCovarianceMatrix_Q(const Eigen::Vector3d& sigma2_ra, const Eigen::Vector3d& sigma2_rg,
                                                                                                         const Eigen::Vector3d& sigma2_bad, const Eigen::Vector3d& sigma2_bgd,
                                                                                                         const Eigen::Vector3d& tau_bad, const Eigen::Vector3d& tau_bgd,
                                                                                                         const Eigen::Matrix3d& e_F_21,
                                                                                                         const Eigen::Matrix3d& e_Dcm_b, const double& tau_s);

    // ###########################################################################################################
    //                                         Error covariance matrix P
    // ###########################################################################################################

    /// @brief Initial error covariance matrix P_0
    /// @param[in] variance_angles Initial Covariance of the attitude angles in [rad²]
    /// @param[in] variance_vel Initial Covariance of the velocity in [m²/s²]
    /// @param[in] variance_pos Initial Covariance of the position in [rad² rad² m²] n-frame / [m²] i,e-frame
    /// @param[in] variance_accelBias Initial Covariance of the accelerometer biases in [m^2/s^4]
    /// @param[in] variance_gyroBias Initial Covariance of the gyroscope biases in [rad^2/s^2]
    /// @return The 15x15 matrix of initial state variances
    [[nodiscard]] KeyedMatrix<double, KFStates, KFStates, 15, 15> initialErrorCovarianceMatrix_P0(const Eigen::Vector3d& variance_angles,
                                                                                                  const Eigen::Vector3d& variance_vel,
                                                                                                  const Eigen::Vector3d& variance_pos,
                                                                                                  const Eigen::Vector3d& variance_accelBias,
                                                                                                  const Eigen::Vector3d& variance_gyroBias) const;

    // ###########################################################################################################
    //                                                Correction
    // ###########################################################################################################

    /// @brief Measurement matrix for GNSS measurements at timestep k, represented in navigation coordinates
    /// @param[in] T_rn_p Conversion matrix between cartesian and curvilinear perturbations to the position
    /// @param[in] n_Dcm_b Direction Cosine Matrix from body to navigation coordinates
    /// @param[in] b_omega_ib Angular rate of body with respect to inertial system in body-frame coordinates in [rad/s]
    /// @param[in] b_leverArm_InsGnss l_{ba}^b lever arm from the INS to the GNSS antenna in body-frame coordinates [m]
    /// @param[in] n_Omega_ie Skew-symmetric matrix of the Earth-rotation vector in local navigation frame axes
    /// @return The 6x15 measurement matrix 𝐇
    [[nodiscard]] static KeyedMatrix<double, KFMeas, KFStates, 6, 15> n_measurementMatrix_H(const Eigen::Matrix3d& T_rn_p,
                                                                                            const Eigen::Matrix3d& n_Dcm_b,
                                                                                            const Eigen::Vector3d& b_omega_ib,
                                                                                            const Eigen::Vector3d& b_leverArm_InsGnss,
                                                                                            const Eigen::Matrix3d& n_Omega_ie);

    /// @brief Measurement matrix for GNSS measurements at timestep k, represented in Earth frame coordinates
    /// @param[in] e_Dcm_b Direction Cosine Matrix from body to Earth coordinates
    /// @param[in] b_omega_ib Angular rate of body with respect to inertial system in body-frame coordinates in [rad/s]
    /// @param[in] b_leverArm_InsGnss l_{ba}^b lever arm from the INS to the GNSS antenna in body-frame coordinates [m]
    /// @param[in] e_Omega_ie Skew-symmetric matrix of the Earth-rotation vector in Earth frame axes
    /// @return The 6x15 measurement matrix 𝐇
    [[nodiscard]] static KeyedMatrix<double, KFMeas, KFStates, 6, 15> e_measurementMatrix_H(const Eigen::Matrix3d& e_Dcm_b,
                                                                                            const Eigen::Vector3d& b_omega_ib,
                                                                                            const Eigen::Vector3d& b_leverArm_InsGnss,
                                                                                            const Eigen::Matrix3d& e_Omega_ie);

    /// @brief Measurement noise covariance matrix 𝐑
    /// @param[in] gnssVarianceLatLonAlt Variances of the position LLA in [rad² rad² m²]
    /// @param[in] gnssVarianceVelocity Variances of the velocity in [m²/s²]
    /// @return The 6x6 measurement covariance matrix 𝐑
    [[nodiscard]] static KeyedMatrix<double, KFMeas, KFMeas, 6, 6> n_measurementNoiseCovariance_R(const Eigen::Vector3d& gnssVarianceLatLonAlt,
                                                                                                  const Eigen::Vector3d& gnssVarianceVelocity);

    /// @brief Measurement noise covariance matrix 𝐑
    /// @param[in] gnssVariancePosition Variances of the position in [m²]
    /// @param[in] gnssVarianceVelocity Variances of the velocity in [m²/s²]
    /// @return The 6x6 measurement covariance matrix 𝐑
    [[nodiscard]] static KeyedMatrix<double, KFMeas, KFMeas, 6, 6> e_measurementNoiseCovariance_R(const Eigen::Vector3d& gnssVariancePosition,
                                                                                                  const Eigen::Vector3d& gnssVarianceVelocity);

    /// @brief Measurement innovation vector 𝜹𝐳
    /// @param[in] lla_positionMeasurement Position measurement as Lat Lon Alt in [rad rad m]
    /// @param[in] lla_positionEstimate Position estimate as Lat Lon Alt in [rad rad m]
    /// @param[in] n_velocityMeasurement Velocity measurement in the n frame in [m/s]
    /// @param[in] n_velocityEstimate Velocity estimate in the n frame in [m/s]
    /// @param[in] T_rn_p Conversion matrix between cartesian and curvilinear perturbations to the position
    /// @param[in] n_Quat_b Rotation quaternion from body to navigation coordinates
    /// @param[in] b_leverArm_InsGnss l_{ba}^b lever arm from the INS to the GNSS antenna in body-frame coordinates [m]
    /// @param[in] b_omega_ib Angular rate of body with respect to inertial system in body-frame coordinates in [rad/s]
    /// @param[in] n_Omega_ie Skew-symmetric matrix of the Earth-rotation vector in local navigation frame axes
    /// @return The 6x1 measurement innovation vector 𝜹𝐳
    [[nodiscard]] static KeyedVector<double, KFMeas, 6> n_measurementInnovation_dz(const Eigen::Vector3d& lla_positionMeasurement, const Eigen::Vector3d& lla_positionEstimate,
                                                                                   const Eigen::Vector3d& n_velocityMeasurement, const Eigen::Vector3d& n_velocityEstimate,
                                                                                   const Eigen::Matrix3d& T_rn_p, const Eigen::Quaterniond& n_Quat_b, const Eigen::Vector3d& b_leverArm_InsGnss,
                                                                                   const Eigen::Vector3d& b_omega_ib, const Eigen::Matrix3d& n_Omega_ie);

    /// @brief Measurement innovation vector 𝜹𝐳
    /// @param[in] e_positionMeasurement Position measurement in ECEF coordinates in [m]
    /// @param[in] e_positionEstimate Position estimate in ECEF coordinates in [m]
    /// @param[in] e_velocityMeasurement Velocity measurement in the e frame in [m/s]
    /// @param[in] e_velocityEstimate Velocity estimate in the e frame in [m/s]
    /// @param[in] e_Quat_b Rotation quaternion from body to Earth coordinates
    /// @param[in] b_leverArm_InsGnss l_{ba}^b lever arm from the INS to the GNSS antenna in body-frame coordinates [m]
    /// @param[in] b_omega_ib Angular rate of body with respect to inertial system in body-frame coordinates in [rad/s]
    /// @param[in] e_Omega_ie Skew-symmetric matrix of the Earth-rotation vector in Earth frame axes
    /// @return The 6x1 measurement innovation vector 𝜹𝐳
    [[nodiscard]] static KeyedVector<double, KFMeas, 6> e_measurementInnovation_dz(const Eigen::Vector3d& e_positionMeasurement, const Eigen::Vector3d& e_positionEstimate,
                                                                                   const Eigen::Vector3d& e_velocityMeasurement, const Eigen::Vector3d& e_velocityEstimate,
                                                                                   const Eigen::Quaterniond& e_Quat_b, const Eigen::Vector3d& b_leverArm_InsGnss,
                                                                                   const Eigen::Vector3d& b_omega_ib, const Eigen::Matrix3d& e_Omega_ie);
};

} // namespace NAV

#ifndef DOXYGEN_IGNORE

template<>
struct fmt::formatter<NAV::LooselyCoupledKF::KFStates> : fmt::formatter<const char*>
{
    /// @brief Defines how to format structs
    /// @param[in] st Struct to format
    /// @param[in, out] ctx Format context
    /// @return Output iterator
    template<typename FormatContext>
    auto format(const NAV::LooselyCoupledKF::KFStates& st, FormatContext& ctx) const
    {
        switch (st)
        {
        case NAV::LooselyCoupledKF::KFStates::Roll:
            return fmt::formatter<const char*>::format("Roll/Psi_eb_1", ctx);
        case NAV::LooselyCoupledKF::KFStates::Pitch:
            return fmt::formatter<const char*>::format("Pitch/Psi_eb_2", ctx);
        case NAV::LooselyCoupledKF::KFStates::Yaw:
            return fmt::formatter<const char*>::format("Yaw/Psi_eb_3", ctx);
        case NAV::LooselyCoupledKF::KFStates::VelN:
            return fmt::formatter<const char*>::format("VelN/VelX", ctx);
        case NAV::LooselyCoupledKF::KFStates::VelE:
            return fmt::formatter<const char*>::format("VelE/VelY", ctx);
        case NAV::LooselyCoupledKF::KFStates::VelD:
            return fmt::formatter<const char*>::format("VelD/VelZ", ctx);
        case NAV::LooselyCoupledKF::KFStates::PosLat:
            return fmt::formatter<const char*>::format("PosLat/PosX", ctx);
        case NAV::LooselyCoupledKF::KFStates::PosLon:
            return fmt::formatter<const char*>::format("PosLon/PosY", ctx);
        case NAV::LooselyCoupledKF::KFStates::PosAlt:
            return fmt::formatter<const char*>::format("PosAlt/PosZ", ctx);
        case NAV::LooselyCoupledKF::KFStates::AccBiasX:
            return fmt::formatter<const char*>::format("AccBiasX", ctx);
        case NAV::LooselyCoupledKF::KFStates::AccBiasY:
            return fmt::formatter<const char*>::format("AccBiasY", ctx);
        case NAV::LooselyCoupledKF::KFStates::AccBiasZ:
            return fmt::formatter<const char*>::format("AccBiasZ", ctx);
        case NAV::LooselyCoupledKF::KFStates::GyrBiasX:
            return fmt::formatter<const char*>::format("GyrBiasX", ctx);
        case NAV::LooselyCoupledKF::KFStates::GyrBiasY:
            return fmt::formatter<const char*>::format("GyrBiasY", ctx);
        case NAV::LooselyCoupledKF::KFStates::GyrBiasZ:
            return fmt::formatter<const char*>::format("GyrBiasZ", ctx);
        }

        return fmt::formatter<const char*>::format("ERROR", ctx);
    }
};
template<>
struct fmt::formatter<NAV::LooselyCoupledKF::KFMeas> : fmt::formatter<const char*>
{
    /// @brief Defines how to format structs
    /// @param[in] st Struct to format
    /// @param[in, out] ctx Format context
    /// @return Output iterator
    template<typename FormatContext>
    auto format(const NAV::LooselyCoupledKF::KFMeas& st, FormatContext& ctx) const
    {
        switch (st)
        {
        case NAV::LooselyCoupledKF::KFMeas::dPosLat:
            return fmt::formatter<const char*>::format("dPosLat/dPosX", ctx);
        case NAV::LooselyCoupledKF::KFMeas::dPosLon:
            return fmt::formatter<const char*>::format("dPosLon/dPosY", ctx);
        case NAV::LooselyCoupledKF::KFMeas::dPosAlt:
            return fmt::formatter<const char*>::format("dPosAlt/dPosZ", ctx);
        case NAV::LooselyCoupledKF::KFMeas::dVelN:
            return fmt::formatter<const char*>::format("dVelN/dVelX", ctx);
        case NAV::LooselyCoupledKF::KFMeas::dVelE:
            return fmt::formatter<const char*>::format("dVelE/dVelY", ctx);
        case NAV::LooselyCoupledKF::KFMeas::dVelD:
            return fmt::formatter<const char*>::format("dVelD/dVelZ", ctx);
        }

        return fmt::formatter<const char*>::format("ERROR", ctx);
    }
};

#endif

/// @brief Stream insertion operator overload
/// @param[in, out] os Output stream object to stream the time into
/// @param[in] obj Object to print
/// @return Returns the output stream object in order to chain stream insertions
std::ostream& operator<<(std::ostream& os, const NAV::LooselyCoupledKF::KFStates& obj);

/// @brief Stream insertion operator overload
/// @param[in, out] os Output stream object to stream the time into
/// @param[in] obj Object to print
/// @return Returns the output stream object in order to chain stream insertions
std::ostream& operator<<(std::ostream& os, const NAV::LooselyCoupledKF::KFMeas& obj);