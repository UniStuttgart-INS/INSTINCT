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

#include <cstdint>
#include <optional>
#include "internal/Node/Node.hpp"
#include "internal/gui/widgets/DynamicInputPins.hpp"

#include "Nodes/DataProcessor/KalmanFilter/LckfKeys.hpp"
#include "Navigation/INS/Units.hpp"
#include "Navigation/Time/InsTime.hpp"
#include "Navigation/INS/InertialIntegrator.hpp"

#include "NodeData/IMU/ImuObs.hpp"
#include "NodeData/State/InsGnssLCKFSolution.hpp"
#include "NodeData/Baro/BaroHgt.hpp"

#include "Navigation/Math/KeyedKalmanFilter.hpp"
#include "util/Container/KeyedMatrix.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Geometry/Quaternion.h>

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

  private:
    constexpr static size_t INPUT_PORT_INDEX_IMU = 0;              ///< @brief Flow (ImuObs)
    constexpr static size_t INPUT_PORT_INDEX_POS_VEL_ATT_INIT = 1; ///< @brief Flow (PosVelAtt)
    constexpr static size_t OUTPUT_PORT_INDEX_SOLUTION = 0;        ///< @brief Flow (InsGnssLCKFSolution)

    /// @brief Initialize the node
    bool initialize() override;

    /// @brief Deinitialize the node
    void deinitialize() override;

    /// @brief Checks wether there is an input pin with the same time
    /// @param[in] insTime Time to check
    bool hasInputPinWithSameTime(const InsTime& insTime) const;

    /// @brief Invoke the callback with a PosVelAtt solution
    /// @param[in] posVelAtt insGnssLCKF solution
    /// @param[in] mmaeData LCKF data that is used for Multiple Model Adaptive Estimation (MMAE)
    void invokeCallbackWithPosVelAtt(const PosVelAtt& posVelAtt, std::optional<InsGnssLCKFSolution::MMAEdata> mmaeData = std::nullopt);

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

    /// @brief Receive Function for the BaroHgt observation
    /// @param[in] queue Queue with all the received data messages
    /// @param[in] pinIdx Index of the pin the data is received on
    void recvBaroHeight(InputPin::NodeDataQueue& queue, size_t pinIdx);

    /// @brief Predicts the state from the InertialNavSol
    /// @param[in] inertialNavSol Inertial navigation solution triggering the prediction
    /// @param[in] tau_i Time since the last prediction in [s]
    /// @param[in] imuPos IMU platform frame position with respect to body frame
    void looselyCoupledPrediction(const std::shared_ptr<const PosVelAtt>& inertialNavSol, double tau_i, const ImuPos& imuPos);

    /// @brief Updates the predicted state from the InertialNavSol with the PosVel observation
    /// @param[in] posVelObs PosVel measurement triggering the update
    void looselyCoupledUpdate(const std::shared_ptr<const PosVel>& posVelObs);

    /// @brief Updates the predicted state from the InertialNavSol with the Baro observation
    /// @param[in] baroHgtObs Barometric height measurement triggering the update
    void looselyCoupledUpdate(const std::shared_ptr<const BaroHgt>& baroHgtObs);

    /// @brief Sets the covariance matrix P of the LCKF (and does the necessary unit conversions)
    /// @param[in] lckfSolution LCKF solution from prediction or update
    /// @param[in] R_N Prime vertical radius of curvature (East/West) [m]
    /// @param[in] R_E Meridian radius of curvature in [m]
    void setSolutionPosVelAttAndCov(const std::shared_ptr<PosVelAtt>& lckfSolution, double R_N, double R_E);

    /// @brief Calculates the covariance matrix P of the LCKF with attitude quaternion instead of RPY (and does the necessary unit conversions)
    /// @param[in] R_N Prime vertical radius of curvature (East/West) [m]
    /// @param[in] R_E Meridian radius of curvature in [m]
    /// @return The 18x18 P matrix of the LCKF (with attitude quaternion instead of RPY)
    [[nodiscard]] KeyedMatrix<double, LckfKeys::KFStatesQuat, LckfKeys::KFStatesQuat, LckfKeys::KFQuat_COUNT, LckfKeys::KFQuat_COUNT> calcSolutionCovariance(double R_N, double R_E) const;

    /// @brief Get the Total State Vector object (RPY case)
    /// @return Total state vector in n-Sys
    [[nodiscard]] KeyedVector<double, LckfKeys::KFStates, LckfKeys::KFStates_COUNT> getTotalStateVectorRPY() const;

    /// @brief Get the Total State Vector object (Quaternion case)
    /// @return Total state vector in n-Sys
    [[nodiscard]] KeyedVector<double, LckfKeys::KFStatesQuat, LckfKeys::KFQuat_COUNT> getTotalStateVectorQuat() const;

    /// @brief Inertial Integrator
    InertialIntegrator _inertialIntegrator;
    /// Prefer the raw acceleration measurements over the deltaVel & deltaTheta values
    bool _preferAccelerationOverDeltaMeasurements = false;

    /// Last received IMU observation (to get ImuPos)
    std::shared_ptr<const ImuObs> _lastImuObs = nullptr;

    /// Accumulator for height bias [m]
    double _heightBiasTotal = 0.0;
    /// Accumulator for height scale [m/m]
    double _heightScaleTotal = 1.0;

    /// Accumulator for acceleration bias [m/s²]
    Eigen::Vector3d _accelBiasTotal = Eigen::Vector3d::Zero();
    /// Accumulator gyro bias [rad/s]
    Eigen::Vector3d _gyroBiasTotal = Eigen::Vector3d::Zero();

    /// Roll, Pitch and Yaw angles in [deg] used for initialization if not taken from separate pin
    std::array<double, 3> _initalRollPitchYaw{};
    /// Whether to initialize the state over an external pin
    bool _initializeStateOverExternalPin{};

    /// Whether to enable barometric height (including the corresponding pin)
    bool _enableBaroHgt{};

    /// Time from the external init
    InsTime _externalInitTime;

    /// Whether the accumulated biases have been initialized in the 'inertialIntegrator'
    bool _initialSensorBiasesApplied = false;

    /// MMAE data, used if PosVel and BaroHgtObs are available at the same timestamp
    std::optional<InsGnssLCKFSolution::MMAEdata> _mmaeData;

    /// GUI option to choose between Euler angles and quaternions for the averaging of the attitude in MMAE
    enum class MmaeAttitudeAveraging : uint8_t
    {
        Euler,      ///< Euler angles (RPY)
        Quaternion, ///< Attitude quaternion
    };
    /// GUI option for the setting of the MMAE's attitude averaging
    MmaeAttitudeAveraging _mmaeAttitudeAveraging = MmaeAttitudeAveraging::Euler;

    /// Kalman Filter representation
    KeyedKalmanFilterD<LckfKeys::KFStates, LckfKeys::KFMeas> _kalmanFilter{ LckfKeys::States, LckfKeys::Meas };

    // #########################################################################################################################################
    //                                                              GUI settings
    // #########################################################################################################################################

    /// @brief Check the rank of the Kalman matrices every iteration (computational expensive)
    bool _checkKalmanMatricesRanks = true;

    // ###########################################################################################################
    //                                                Parameters
    // ###########################################################################################################

    /// Gui selection for the Unit of the input stdev_ra parameter
    Units::ImuAccelerometerFilterNoiseUnits _stdevAccelNoiseUnits = Units::ImuAccelerometerFilterNoiseUnits::mg_sqrtHz;

    /// @brief 𝜎_ra Standard deviation of the noise on the accelerometer specific-force measurements
    /// @note Value from VN-310 Datasheet but verify with values from Brown (2012) table 9.3 for 'High quality'
    Eigen::Vector3d _stdev_ra = 0.04 /* [mg/√(Hz)] */ * Eigen::Vector3d::Ones();

    // ###########################################################################################################

    /// Gui selection for the Unit of the input stdev_rg parameter
    Units::ImuGyroscopeFilterNoiseUnits _stdevGyroNoiseUnits = Units::ImuGyroscopeFilterNoiseUnits::deg_hr_sqrtHz;

    /// @brief 𝜎_rg Standard deviation of the noise on the gyro angular-rate measurements
    /// @note Value from VN-310 Datasheet but verify with values from Brown (2012) table 9.3 for 'High quality'
    Eigen::Vector3d _stdev_rg = 5 /* [deg/hr/√(Hz)]^2 */ * Eigen::Vector3d::Ones();

    // ###########################################################################################################

    /// Gui selection for the Unit of the input variance_bad parameter
    Units::ImuAccelerometerFilterBiasUnits _stdevAccelBiasUnits = Units::ImuAccelerometerFilterBiasUnits::microg;

    /// @brief 𝜎²_bad Variance of the accelerometer dynamic bias
    /// @note Value from VN-310 Datasheet (In-Run Bias Stability (Allan Variance))
    Eigen::Vector3d _stdev_bad = 10 /* [µg] */ * Eigen::Vector3d::Ones();

    /// @brief Correlation length of the accelerometer dynamic bias in [s]
    Eigen::Vector3d _tau_bad = 0.1 * Eigen::Vector3d::Ones();

    // ###########################################################################################################

    /// Gui selection for the Unit of the input variance_bad parameter
    Units::ImuGyroscopeFilterBiasUnits _stdevGyroBiasUnits = Units::ImuGyroscopeFilterBiasUnits::deg_h;

    /// @brief 𝜎²_bgd Variance of the gyro dynamic bias
    /// @note Value from VN-310 Datasheet (In-Run Bias Stability (Allan Variance))
    Eigen::Vector3d _stdev_bgd = 1 /* [°/h] */ * Eigen::Vector3d::Ones();

    /// @brief Correlation length of the gyro dynamic bias in [s]
    Eigen::Vector3d _tau_bgd = 0.1 * Eigen::Vector3d::Ones();

    // ###########################################################################################################

    /// Possible Units for the Variance of the barometric height bias
    enum class StdevBaroHeightBiasUnits : uint8_t
    {
        m, ///< [m]
    };
    /// Gui selection for the Unit of the barometric height bias uncertainty
    StdevBaroHeightBiasUnits _stdevBaroHeightBiasUnits = StdevBaroHeightBiasUnits::m;

    /// Uncertainty of the barometric height bias
    double _stdevBaroHeightBias = 1e-6;

    // ###########################################################################################################

    /// Possible Units for the Variance of the barometric height scale
    enum class StdevBaroHeightScaleUnits : uint8_t
    {
        m_m, ///< [m/m]
    };
    /// Gui selection for the Unit of the barometric height scale uncertainty
    StdevBaroHeightScaleUnits _stdevBaroHeightScaleUnits = StdevBaroHeightScaleUnits::m_m;

    /// Uncertainty of the barometric height scale
    double _stdevBaroHeightScale = 1e-8;

    // ###########################################################################################################

    /// @brief Available Random processes
    enum class RandomProcess : uint8_t
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
    enum class GnssMeasurementUncertaintyPositionUnit : uint8_t
    {
        meter2, ///< Variance [m^2, m^2, m^2]
        meter,  ///< Standard deviation [m, m, m]
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
    enum class GnssMeasurementUncertaintyVelocityUnit : uint8_t
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

    /// Possible Units for the barometric height measurement uncertainty (standard deviation σ or Variance σ²)
    enum class BaroHeightMeasurementUncertaintyUnit : uint8_t
    {
        m2, ///< Variance [m²]
        m,  ///< Standard deviation [m]
    };
    /// Gui selection for the Unit of the barometric height measurement uncertainty
    BaroHeightMeasurementUncertaintyUnit _barometricHeightMeasurementUncertaintyUnit = BaroHeightMeasurementUncertaintyUnit::m;

    /// GUI selection of the barometric height measurement uncertainty (standard deviation σ or Variance σ²)
    double _barometricHeightMeasurementUncertainty = 1.0;

    /// Whether to override the barometric height uncertainty or use the one included in the measurement
    bool _baroHeightMeasurementUncertaintyOverride = false;

    // ###########################################################################################################

    /// Possible Units for the initial covariance for the position (standard deviation σ or Variance σ²)
    enum class InitCovariancePositionUnit : uint8_t
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
    enum class InitCovarianceVelocityUnit : uint8_t
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
    enum class InitCovarianceAttitudeAnglesUnit : uint8_t
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
    enum class InitCovarianceBiasAccelUnit : uint8_t
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
    enum class InitCovarianceBiasGyroUnit : uint8_t
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

    /// Possible Units for the initial covariance for the height bias (standard deviation σ or Variance σ²)
    enum class InitCovarianceBiasHeightUnit : uint8_t
    {
        m2, ///< Variance [m²]
        m,  ///< Standard deviation [m]
    };
    /// Gui selection for the Unit of the initial covariance for the height bias
    InitCovarianceBiasHeightUnit _initCovarianceBiasHeightUnit = InitCovarianceBiasHeightUnit::m2;

    /// GUI selection of the initial covariance diagonal values for the height bias (standard deviation σ or Variance σ²)
    double _initCovarianceBiasHeight{ 100.0 };

    // ###########################################################################################################

    /// Possible Units for the initial covariance for the height scale (standard deviation σ or Variance σ²)
    enum class InitCovarianceScaleHeight : uint8_t
    {
        m2_m2, ///< Variance [m²/m²]
        m_m,   ///< Standard devation [m/m]
    };
    /// Gui selection for the Unit of the initial covariance for the height scale
    InitCovarianceScaleHeight _initCovarianceScaleHeightUnit = InitCovarianceScaleHeight::m2_m2;

    /// GUI selection of the initial covariance diagonal values for the height scale (standard deviation σ or Variance σ²)
    double _initCovarianceScaleHeight{ 0.01 };

    // ###########################################################################################################

    /// Possible Units for the initial accelerometer biases
    enum class InitBiasAccelUnit : uint8_t
    {
        microg, ///< [µg]
        m_s2,   ///< acceleration [m/s^2]
    };
    /// Gui selection for the unit of the initial accelerometer biases
    InitBiasAccelUnit _initBiasAccelUnit = InitBiasAccelUnit::m_s2;

    /// GUI selection of the initial accelerometer biases
    Eigen::Vector3d _initBiasAccel{ 0.0, 0.0, 0.0 };

    // ###########################################################################################################

    /// Possible Units for the initial gyroscope biases
    enum class InitBiasGyroUnit : uint8_t
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
    enum class PhiCalculationAlgorithm : uint8_t
    {
        Exponential, ///< Van-Loan
        Taylor,      ///< Taylor
    };
    /// GUI option for the Phi calculation algorithm
    PhiCalculationAlgorithm _phiCalculationAlgorithm = PhiCalculationAlgorithm::Taylor;

    /// GUI option for the order of the Taylor polynom to calculate the Phi matrix
    int _phiCalculationTaylorOrder = 2;

    /// GUI option for the Q calculation algorithm
    enum class QCalculationAlgorithm : uint8_t
    {
        VanLoan, ///< Van-Loan
        Taylor1, ///< Taylor
    };
    /// GUI option for the Q calculation algorithm
    QCalculationAlgorithm _qCalculationAlgorithm = QCalculationAlgorithm::Taylor1;

    // ###########################################################################################################
    //                                             Dynamic Input Pins
    // ###########################################################################################################
    //
    /// @brief Function to call to add a new pin
    /// @param[in, out] node Pointer to this node
    static void pinAddCallback(Node* node);
    /// @brief Function to call to delete a pin
    /// @param[in, out] node Pointer to this node
    /// @param[in] pinIdx Input pin index to delete
    static void pinDeleteCallback(Node* node, size_t pinIdx);

    /// Update the input pins depending on the GUI
    void updateInputPins();

    /// @brief Dynamic input pins
    /// @attention This should always be the last variable in the header, because it accesses others through the function callbacks
    gui::widgets::DynamicInputPins _dynamicInputPins{ 1, this, pinAddCallback, pinDeleteCallback };

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
    [[nodiscard]] KeyedMatrix<double, LckfKeys::KFStates, LckfKeys::KFStates, 17, 17> n_systemMatrix_F(const Eigen::Quaterniond& n_Quat_b,
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
    [[nodiscard]] KeyedMatrix<double, LckfKeys::KFStates, LckfKeys::KFStates, 17, 17> e_systemMatrix_F(const Eigen::Quaterniond& e_Quat_b,
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
    [[nodiscard]] static KeyedMatrix<double, LckfKeys::KFStates, LckfKeys::KFStates, 17, 17> noiseInputMatrix_G(const Eigen::Quaterniond& ien_Quat_b);

    /// @brief Calculates the noise scale matrix 𝐖
    /// @param[in] sigma_ra Standard deviation of the noise on the accelerometer specific-force measurements
    /// @param[in] sigma_rg Standard deviation of the noise on the gyro angular-rate measurements
    /// @param[in] sigma_bad Standard deviation of the accelerometer dynamic bias
    /// @param[in] sigma_bgd Standard deviation of the gyro dynamic bias
    /// @param[in] tau_bad Correlation length for the accelerometer in [s]
    /// @param[in] tau_bgd Correlation length for the gyroscope in [s]
    /// @param[in] sigma_heightBias Standard deviation of the height bias
    /// @param[in] sigma_heightScale Standard deviation of the height scale
    /// @note See \cite Groves2013 Groves, ch. 14.2.6, eq. 14.79, p. 590
    [[nodiscard]] Eigen::Matrix<double, 17, 17> noiseScaleMatrix_W(const Eigen::Vector3d& sigma_ra, const Eigen::Vector3d& sigma_rg,
                                                                   const Eigen::Vector3d& sigma_bad, const Eigen::Vector3d& sigma_bgd,
                                                                   const Eigen::Vector3d& tau_bad, const Eigen::Vector3d& tau_bgd,
                                                                   const double& sigma_heightBias, const double& sigma_heightScale);

    /// @brief System noise covariance matrix 𝐐_{k-1}
    /// @param[in] sigma2_ra Variance of the noise on the accelerometer specific-force measurements
    /// @param[in] sigma2_rg Variance of the noise on the gyro angular-rate measurements
    /// @param[in] sigma2_bad Variance of the accelerometer dynamic bias
    /// @param[in] sigma2_bgd Variance of the gyro dynamic bias
    /// @param[in] sigma_heightBias Standard deviation of the height bias
    /// @param[in] sigma_heightScale Standard deviation of the height scale
    /// @param[in] tau_bad Correlation length for the accelerometer in [s]
    /// @param[in] tau_bgd Correlation length for the gyroscope in [s]
    /// @param[in] n_F_21 Submatrix 𝐅_21 of the system matrix 𝐅
    /// @param[in] T_rn_p Conversion matrix between cartesian and curvilinear perturbations to the position
    /// @param[in] n_Dcm_b Direction Cosine Matrix from body to navigation coordinates
    /// @param[in] tau_s Time interval in [s]
    /// @return The 17x17 matrix of system noise covariances
    [[nodiscard]] static KeyedMatrix<double, LckfKeys::KFStates, LckfKeys::KFStates, 17, 17> n_systemNoiseCovarianceMatrix_Q(const Eigen::Vector3d& sigma2_ra, const Eigen::Vector3d& sigma2_rg,
                                                                                                                             const Eigen::Vector3d& sigma2_bad, const Eigen::Vector3d& sigma2_bgd,
                                                                                                                             const double& sigma_heightBias, const double& sigma_heightScale,
                                                                                                                             const Eigen::Vector3d& tau_bad, const Eigen::Vector3d& tau_bgd,
                                                                                                                             const Eigen::Matrix3d& n_F_21, const Eigen::Matrix3d& T_rn_p,
                                                                                                                             const Eigen::Matrix3d& n_Dcm_b, const double& tau_s);

    /// @brief System noise covariance matrix 𝐐_{k-1}
    /// @param[in] sigma2_ra Variance of the noise on the accelerometer specific-force measurements
    /// @param[in] sigma2_rg Variance of the noise on the gyro angular-rate measurements
    /// @param[in] sigma2_bad Variance of the accelerometer dynamic bias
    /// @param[in] sigma2_bgd Variance of the gyro dynamic bias
    /// @param[in] sigma_heightBias Standard deviation of the height bias
    /// @param[in] sigma_heightScale Standard deviation of the height scale
    /// @param[in] tau_bad Correlation length for the accelerometer in [s]
    /// @param[in] tau_bgd Correlation length for the gyroscope in [s]
    /// @param[in] e_F_21 Submatrix 𝐅_21 of the system matrix 𝐅
    /// @param[in] e_Dcm_b Direction Cosine Matrix from body to Earth coordinates
    /// @param[in] tau_s Time interval in [s]
    /// @return The 17x17 matrix of system noise covariances
    [[nodiscard]] static KeyedMatrix<double, LckfKeys::KFStates, LckfKeys::KFStates, 17, 17> e_systemNoiseCovarianceMatrix_Q(const Eigen::Vector3d& sigma2_ra, const Eigen::Vector3d& sigma2_rg,
                                                                                                                             const Eigen::Vector3d& sigma2_bad, const Eigen::Vector3d& sigma2_bgd,
                                                                                                                             const double& sigma_heightBias, const double& sigma_heightScale,
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
    /// @param[in] variance_heightBias Initial Covariance of the height bias [m^2]
    /// @param[in] variance_heightScale Initial Covariance of the height scale in [m^2 / m^2]
    /// @return The 17x17 matrix of initial state variances
    [[nodiscard]] KeyedMatrix<double, LckfKeys::KFStates, LckfKeys::KFStates, 17, 17> initialErrorCovarianceMatrix_P0(const Eigen::Vector3d& variance_angles,
                                                                                                                      const Eigen::Vector3d& variance_vel,
                                                                                                                      const Eigen::Vector3d& variance_pos,
                                                                                                                      const Eigen::Vector3d& variance_accelBias,
                                                                                                                      const Eigen::Vector3d& variance_gyroBias,
                                                                                                                      const double& variance_heightBias,
                                                                                                                      const double& variance_heightScale) const;

    // ###########################################################################################################
    //                                                Correction
    // ###########################################################################################################

    /// @brief Measurement matrix for GNSS measurements at timestep k, represented in navigation coordinates
    /// @param[in] T_rn_p Conversion matrix between cartesian and curvilinear perturbations to the position
    /// @param[in] n_Dcm_b Direction Cosine Matrix from body to navigation coordinates
    /// @param[in] b_omega_ib Angular rate of body with respect to inertial system in body-frame coordinates in [rad/s]
    /// @param[in] b_leverArm_InsGnss l_{ba}^b lever arm from the INS to the GNSS antenna in body-frame coordinates [m]
    /// @param[in] n_Omega_ie Skew-symmetric matrix of the Earth-rotation vector in local navigation frame axes
    /// @return The 6x17 measurement matrix 𝐇
    [[nodiscard]] static KeyedMatrix<double, LckfKeys::KFMeas, LckfKeys::KFStates, 6, 17> n_measurementMatrix_H(const Eigen::Matrix3d& T_rn_p,
                                                                                                                const Eigen::Matrix3d& n_Dcm_b,
                                                                                                                const Eigen::Vector3d& b_omega_ib,
                                                                                                                const Eigen::Vector3d& b_leverArm_InsGnss,
                                                                                                                const Eigen::Matrix3d& n_Omega_ie);

    /// @brief Measurement matrix for baro height measurements at timestep k, represented in navigation coordinates
    /// @param[in] height predicted height
    /// @param[in] scale height scale
    /// @return The 6x17 measurement matrix 𝐇
    [[nodiscard]] static KeyedMatrix<double, LckfKeys::KFMeas, LckfKeys::KFStates, 1, 17> n_measurementMatrix_H(const double& height, const double& scale);

    /// @brief Measurement matrix for GNSS measurements at timestep k, represented in Earth frame coordinates
    /// @param[in] e_Dcm_b Direction Cosine Matrix from body to Earth coordinates
    /// @param[in] b_omega_ib Angular rate of body with respect to inertial system in body-frame coordinates in [rad/s]
    /// @param[in] b_leverArm_InsGnss l_{ba}^b lever arm from the INS to the GNSS antenna in body-frame coordinates [m]
    /// @param[in] e_Omega_ie Skew-symmetric matrix of the Earth-rotation vector in Earth frame axes
    /// @return The 6x17 measurement matrix 𝐇
    [[nodiscard]] static KeyedMatrix<double, LckfKeys::KFMeas, LckfKeys::KFStates, 6, 17> e_measurementMatrix_H(const Eigen::Matrix3d& e_Dcm_b,
                                                                                                                const Eigen::Vector3d& b_omega_ib,
                                                                                                                const Eigen::Vector3d& b_leverArm_InsGnss,
                                                                                                                const Eigen::Matrix3d& e_Omega_ie);

    /// @brief Measurement matrix for barometric height measurements at timestep k, represented in Earth frame coordinates
    /// @param[in] e_positionEstimate predicted position
    /// @param[in] height predicted height (assuming that the KF internally converts to LLA at every epoch)
    /// @param[in] scale height scale
    /// @return The 1x17 measurement matrix 𝐇
    [[nodiscard]] static KeyedMatrix<double, LckfKeys::KFMeas, LckfKeys::KFStates, 1, 17> e_measurementMatrix_H(const Eigen::Vector3d& e_positionEstimate,
                                                                                                                const double& height,
                                                                                                                const double& scale);

    /// @brief Measurement noise covariance matrix 𝐑
    /// @param[in] posVelObs Position and velocity observation
    /// @param[in] lla_position Position as Lat Lon Alt in [rad rad m]
    /// @param[in] R_N Meridian radius of curvature in [m]
    /// @param[in] R_E Prime vertical radius of curvature (East/West) [m]
    /// @return The 6x6 measurement covariance matrix 𝐑
    [[nodiscard]] KeyedMatrix<double, LckfKeys::KFMeas, LckfKeys::KFMeas, 6, 6> n_measurementNoiseCovariance_R(const std::shared_ptr<const PosVel>& posVelObs,
                                                                                                               const Eigen::Vector3d& lla_position,
                                                                                                               double R_N,
                                                                                                               double R_E) const;

    /// @brief Measurement noise covariance matrix 𝐑
    /// @param[in] baroVarianceHeight Variance of height in [m²]
    /// @return The 1x1 measurement covariance matrix 𝐑
    [[nodiscard]] static KeyedMatrix<double, LckfKeys::KFMeas, LckfKeys::KFMeas, 1, 1> n_measurementNoiseCovariance_R(const double& baroVarianceHeight);

    /// @brief Measurement noise covariance matrix 𝐑
    /// @param[in] posVelObs Position and velocity observation
    /// @return The 6x6 measurement covariance matrix 𝐑
    [[nodiscard]] KeyedMatrix<double, LckfKeys::KFMeas, LckfKeys::KFMeas, 6, 6> e_measurementNoiseCovariance_R(const std::shared_ptr<const PosVel>& posVelObs) const;

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
    [[nodiscard]] static KeyedVector<double, LckfKeys::KFMeas, 6> n_measurementInnovation_dz(const Eigen::Vector3d& lla_positionMeasurement, const Eigen::Vector3d& lla_positionEstimate,
                                                                                             const Eigen::Vector3d& n_velocityMeasurement, const Eigen::Vector3d& n_velocityEstimate,
                                                                                             const Eigen::Matrix3d& T_rn_p, const Eigen::Quaterniond& n_Quat_b, const Eigen::Vector3d& b_leverArm_InsGnss,
                                                                                             const Eigen::Vector3d& b_omega_ib, const Eigen::Matrix3d& n_Omega_ie);

    /// @brief Measurement innovation vector 𝜹𝐳
    /// @param[in] baroheight barometric height measurement in [m]
    /// @param[in] height predicted height in [m]
    /// @param[in] heightbias height bias in [m]
    /// @param[in] heightscale height scale [m/m]
    /// @return The 1x1 measurement innovation vector 𝜹𝐳
    [[nodiscard]] static KeyedVector<double, LckfKeys::KFMeas, 1> n_measurementInnovation_dz(const double& baroheight, const double& height,
                                                                                             const double& heightbias, const double& heightscale);

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
    [[nodiscard]] static KeyedVector<double, LckfKeys::KFMeas, 6> e_measurementInnovation_dz(const Eigen::Vector3d& e_positionMeasurement, const Eigen::Vector3d& e_positionEstimate,
                                                                                             const Eigen::Vector3d& e_velocityMeasurement, const Eigen::Vector3d& e_velocityEstimate,
                                                                                             const Eigen::Quaterniond& e_Quat_b, const Eigen::Vector3d& b_leverArm_InsGnss,
                                                                                             const Eigen::Vector3d& b_omega_ib, const Eigen::Matrix3d& e_Omega_ie);
};

} // namespace NAV