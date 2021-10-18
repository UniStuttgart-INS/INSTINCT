/// @file LooselyCoupledKF.hpp
/// @brief Kalman Filter class for the loosely coupled INS/GNSS integration
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @author M. Maier (marcel.maier@ins.uni-stuttgart.de)
/// @date 2021-08-04

#pragma once

#include "internal/Node/Node.hpp"
#include "NodeData/State/InertialNavSol.hpp"

#include "KalmanFilter.hpp"

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
    /// @attention Don't forget to set hasConfig to true in the constructor of the node
    void guiConfig() override;

    /// @brief Saves the node into a json object
    [[nodiscard]] json save() const override;

    /// @brief Restores the node from a json object
    /// @param[in] j Json object with the node state
    void restore(const json& j) override;

  private:
    constexpr static size_t OutputPortIndex_PVAError = 0;  ///< @brief Flow (PVAError)
    constexpr static size_t OutputPortIndex_ImuBiases = 1; ///< @brief Flow (ImuBiases)
    constexpr static size_t OutputPortIndex_x = 2;         ///< @brief x̂ State vector
    constexpr static size_t OutputPortIndex_P = 3;         ///< @brief 𝐏 Error covariance matrix
    constexpr static size_t OutputPortIndex_Phi = 4;       ///< @brief 𝚽 State transition matrix
    constexpr static size_t OutputPortIndex_Q = 5;         ///< @brief 𝐐 System/Process noise covariance matrix
    constexpr static size_t OutputPortIndex_z = 6;         ///< @brief 𝐳 Measurement vector
    constexpr static size_t OutputPortIndex_H = 7;         ///< @brief 𝐇 Measurement sensitivity Matrix
    constexpr static size_t OutputPortIndex_R = 8;         ///< @brief 𝐑 = 𝐸{𝐰ₘ𝐰ₘᵀ} Measurement noise covariance matrix
    constexpr static size_t OutputPortIndex_K = 9;         ///< @brief 𝐊 Kalman gain matrix
    constexpr static size_t OutputPortIndex_Kz = 10;       ///< @brief 𝐊*𝐳 Kalman gain matrix * 𝐳 Measurement vector

    /// 𝐊*𝐳 Kalman gain matrix * 𝐳 Measurement vector
    Eigen::MatrixXd kalmanFilter_Kz;

    /// @brief Initialize the node
    bool initialize() override;

    /// @brief Deinitialize the node
    void deinitialize() override;

    /// @brief Receive Function for the intertial navigation solution
    /// @param[in] nodeData State vector (PosVelAtt)
    /// @param[in] linkId Id of the link over which the data is received
    void recvInertialNavigationSolution(const std::shared_ptr<const NodeData>& nodeData, ax::NodeEditor::LinkId linkId);

    /// @brief Receive Function for the GNSS navigation solution
    /// @param[in] nodeData State vector (PosVel)
    /// @param[in] linkId Id of the link over which the data is received
    void recvGNSSNavigationSolution(const std::shared_ptr<const NodeData>& nodeData, ax::NodeEditor::LinkId linkId);

    /// @brief Predicts the state from the InertialNavSol
    void looselyCoupledPrediction(const std::shared_ptr<const InertialNavSol>& inertialNavSol);

    /// @brief Updates the predicted state from the InertialNavSol with the GNSS measurement
    void looselyCoupledUpdate(const std::shared_ptr<const PosVelAtt>& gnssMeasurement);

    /// Latest Position, Velocity, Attitude and Imu observation
    std::shared_ptr<const InertialNavSol> latestInertialNavSol = nullptr;

    /// Kalman Filter representation
    KalmanFilter kalmanFilter{ 15, 6 };

    // ###########################################################################################################
    //                                                Parameters
    // ###########################################################################################################

    /// Timestamp of the KF
    double tau_KF = 0.01;

    /// GUI Gauss-Markov constant for the accelerometer 𝛽 = 1 / 𝜏 (𝜏 correlation length) - Value from Jekeli (p. 183)
    Eigen::Vector3d beta_accel = 2.0 / tau_KF * Eigen::Vector3d::Ones();
    /// GUI Gauss-Markov constant for the gyroscope 𝛽 = 1 / 𝜏 (𝜏 correlation length) - Value from Jekeli (p. 183)
    Eigen::Vector3d beta_gyro = 2.0 / tau_KF * Eigen::Vector3d::Ones();

    /// Lever arm between INS and GNSS in [m, m, m]
    Eigen::Vector3d leverArm_InsGnss{ 0.0, 0.0, 0.0 };

    // ###########################################################################################################

    /// Possible Units for the Variance of the noise on the accelerometer specific-force measurements
    enum class VarianceAccelNoiseUnits
    {
        mg_sqrtHz, ///< [mg/√(Hz)]
    };
    /// Gui selection for the Unit of the input variance_ra parameter
    VarianceAccelNoiseUnits varianceAccelNoiseUnits = VarianceAccelNoiseUnits::mg_sqrtHz;

    /// @brief 𝜎²_ra Variance of the noise on the accelerometer specific-force measurements
    /// @note Value from VN-310 Datasheet but verify with values from Brown (2012) table 9.3 for 'High quality'
    double variance_ra = 0.04 /* [mg/√(Hz)] */;

    // ###########################################################################################################

    /// Possible Units for the Variance of the noise on the gyro angular-rate measurements
    enum class VarianceGyroNoiseUnits
    {
        deg_hr_sqrtHz, ///< [deg/hr/√(Hz)]
    };
    /// Gui selection for the Unit of the input variance_rg parameter
    VarianceGyroNoiseUnits varianceGyroNoiseUnits = VarianceGyroNoiseUnits::deg_hr_sqrtHz;

    /// @brief 𝜎²_rg Variance of the noise on the gyro angular-rate measurements [deg²/s]
    /// @note Value from VN-310 Datasheet but verify with values from Brown (2012) table 9.3 for 'High quality'
    double variance_rg = 5 /* [deg/hr/√(Hz)] */;

    // ###########################################################################################################

    /// Possible Units for the Variance of the accelerometer dynamic bias
    enum class VarianceAccelBiasUnits
    {
        microg, ///< [µg]
    };
    /// Gui selection for the Unit of the input variance_bad parameter
    VarianceAccelBiasUnits varianceAccelBiasUnits = VarianceAccelBiasUnits::microg;

    /// @brief 𝜎²_bad Variance of the accelerometer dynamic bias
    /// @note Value from VN-310 Datasheet (In-Run Bias Stability (Allan Variance))
    double variance_bad = 10 /* [µg] */;

    // ###########################################################################################################

    /// Possible Units for the Variance of the accelerometer dynamic bias
    enum class VarianceGyroBiasUnits
    {
        deg_h, ///< [°/h]
    };
    /// Gui selection for the Unit of the input variance_bad parameter
    VarianceGyroBiasUnits varianceGyroBiasUnits = VarianceGyroBiasUnits::deg_h;

    /// @brief 𝜎²_bgd Variance of the gyro dynamic bias
    /// @note Value from VN-310 Datasheet (In-Run Bias Stability (Allan Variance))
    double variance_bgd = 1 /* [°/h] */;

    // ###########################################################################################################

    /// @brief Available Random processes
    enum class RandomProcess
    {
        WhiteNoise,     ///< White noise
        RandomConstant, ///< Random constant
        RandomWalk,     ///< Random Walk
        GaussMarkov1,   ///< Gauss-Markov 1st Order
        GaussMarkov2,   ///< Gauss-Markov 2nd Order
        GaussMarkov3,   ///< Gauss-Markov 3rd Order
    };

    /// @brief Random Process used to estimate the accelerometer biases
    RandomProcess randomProcessAccel = RandomProcess::RandomWalk;
    /// @brief Random Process used to estimate the gyroscope biases
    RandomProcess randomProcessGyro = RandomProcess::RandomWalk;

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
    GnssMeasurementUncertaintyPositionUnit gnssMeasurementUncertaintyPositionUnit = GnssMeasurementUncertaintyPositionUnit::meter;

    /// @brief GUI selection of the GNSS position measurement uncertainty (standard deviation σ or Variance σ²).
    /// SPP accuracy approx. 3m in horizontal direction and 3 times worse in vertical direction
    Eigen::Vector3d gnssMeasurementUncertaintyPosition{ 0.3, 0.3, 0.3 * 3 };

    // ###########################################################################################################

    /// Possible Units for the GNSS measurement uncertainty for the velocity (standard deviation σ or Variance σ²)
    enum class GnssMeasurementUncertaintyVelocityUnit
    {
        m2_s2, ///< Variance [m^2/s^2]
        m_s,   ///< Standard deviation [m/s]
    };
    /// Gui selection for the Unit of the GNSS measurement uncertainty for the velocity
    GnssMeasurementUncertaintyVelocityUnit gnssMeasurementUncertaintyVelocityUnit = GnssMeasurementUncertaintyVelocityUnit::m_s;

    /// GUI selection of the GNSS NED velocity measurement uncertainty (standard deviation σ or Variance σ²)
    Eigen::Vector3d gnssMeasurementUncertaintyVelocity{ 0.5, 0.5, 0.5 };

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
    InitCovariancePositionUnit initCovariancePositionUnit = InitCovariancePositionUnit::meter;

    /// GUI selection of the initial covariance diagonal values for position (standard deviation σ or Variance σ²)
    Eigen::Vector3d initCovariancePosition{ 100, 100, 100 };

    // ###########################################################################################################

    /// Possible Units for the initial covariance for the velocity (standard deviation σ or Variance σ²)
    enum class InitCovarianceVelocityUnit
    {
        m2_s2, ///< Variance [m^2/s^2]
        m_s,   ///< Standard deviation [m/s]
    };
    /// Gui selection for the Unit of the initial covariance for the velocity
    InitCovarianceVelocityUnit initCovarianceVelocityUnit = InitCovarianceVelocityUnit::m_s;

    /// GUI selection of the initial covariance diagonal values for velocity (standard deviation σ or Variance σ²)
    Eigen::Vector3d initCovarianceVelocity{ 10, 10, 10 };

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
    InitCovarianceAttitudeAnglesUnit initCovarianceAttitudeAnglesUnit = InitCovarianceAttitudeAnglesUnit::deg;

    /// GUI selection of the initial covariance diagonal values for attitude angles (standard deviation σ or Variance σ²)
    Eigen::Vector3d initCovarianceAttitudeAngles{ 10, 10, 10 };

    // ###########################################################################################################

    /// Possible Units for the initial covariance for the accelerometer biases (standard deviation σ or Variance σ²)
    enum class InitCovarianceBiasAccelUnit
    {
        m2_s4, ///< Variance [m^2/s^4]
        m_s2,  ///< Standard deviation [m/s^2]
    };
    /// Gui selection for the Unit of the initial covariance for the accelerometer biases
    InitCovarianceBiasAccelUnit initCovarianceBiasAccelUnit = InitCovarianceBiasAccelUnit::m_s2;

    /// GUI selection of the initial covariance diagonal values for accelerometer biases (standard deviation σ or Variance σ²)
    Eigen::Vector3d initCovarianceBiasAccel{ 1, 1, 1 };

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
    InitCovarianceBiasGyroUnit initCovarianceBiasGyroUnit = InitCovarianceBiasGyroUnit::deg_s;

    /// GUI selection of the initial covariance diagonal values for gyroscope biases (standard deviation σ or Variance σ²)
    Eigen::Vector3d initCovarianceBiasGyro{ 0.5, 0.5, 0.5 };

    // ###########################################################################################################

    /// GUI option for the Phi calculation algorithm
    enum class PhiCalculation
    {
        Taylor1,
        VanLoan,
    };
    /// GUI option for the Phi calculation algorithm
    PhiCalculation phiCalculation = PhiCalculation::Taylor1;

    /// GUI option for the Phi calculation algorithm
    enum class QCalculation
    {
        Groves,
        VanLoan,
    };
    /// GUI option for the Phi calculation algorithm
    QCalculation qCalculation = QCalculation::Groves;

    // ###########################################################################################################
    //                                                Prediction
    // ###########################################################################################################

    // ###########################################################################################################
    //                                           Transition matrix 𝚽
    // ###########################################################################################################

    /// @brief Updates the state transition matrix 𝚽 limited to first order in 𝐅𝜏ₛ
    /// @param[in] F System Matrix
    /// @param[in] tau_s time interval in [s]
    /// @note See Groves (2013) chapter 14.2.4, equation (14.72)
    static Eigen::MatrixXd transitionMatrix(const Eigen::MatrixXd& F, double tau_s);

    // ###########################################################################################################
    //                                             System matrix 𝐅
    // ###########################################################################################################

    /// @brief Calculates the system matrix 𝐅
    /// @param[in] quaternion_np Attitude of the platform with respect to n-system
    /// @param[in] specForce_ib_p Specific force of the body with respect to inertial frame in [m / s^2], resolved in platform coord.
    /// @param[in] angularRate_in_n Angular rate of navigation system with respect to the inertial system [rad / s], resolved in navigation coordinates.
    /// @param[in] velocity_n Velocity in n-system in [m / s]
    /// @param[in] position_lla Position as Lat Lon Alt in [rad rad m]
    /// @param[in] beta_a Gauss-Markov constant for the accelerometer 𝛽 = 1 / 𝜏 (𝜏 correlation length)
    /// @param[in] beta_omega Gauss-Markov constant for the gyroscope 𝛽 = 1 / 𝜏 (𝜏 correlation length)
    /// @note See Groves (2013) chapter 14.2.4, equation (14.63)
    static Eigen::Matrix<double, 15, 15> systemMatrixF(const Eigen::Quaterniond& quaternion_np, const Eigen::Vector3d& specForce_ib_p, const Eigen::Vector3d& angularRate_in_n, const Eigen::Vector3d& velocity_n, const Eigen::Vector3d& position_lla, const Eigen::Vector3d& beta_a, const Eigen::Vector3d& beta_omega);

    /// @brief Submatrix 𝐅_𝜓'_𝜓 of the system matrix 𝐅
    /// @param[in] angularRate_in_n Angular rate vector of the n-system with respect to the i-system in [rad / s], resolved in the n-system
    /// @return The 3x3 matrix 𝐅_11
    /// @note See T. Hobiger (2021) Inertialnavigation V07 - equation (7.22)
    /// @note See Groves (2013) equation (14.64)
    static Eigen::Matrix3d systemMatrixF_11_n(const Eigen::Vector3d& angularRate_in_n);

    /// @brief Submatrix 𝐅_𝜓'_𝛿v of the system matrix 𝐅
    /// @param[in] latitude_b Geodetic latitude of the body in [rad]
    /// @param[in] height_b Geodetic height of the body in [m]
    /// @return The 3x3 matrix 𝐅_12
    /// @note See T. Hobiger (2021) Inertialnavigation V07 - equation (7.21)
    /// @note See Groves (2013) equation (14.65)
    static Eigen::Matrix3d systemMatrixF_12_n(double latitude_b, double height_b);

    /// @brief Submatrix 𝐅_𝜓'_𝛿r of the system matrix 𝐅
    /// @param[in] latitude_b Geodetic latitude of the body in [rad]
    /// @param[in] height_b Geodetic height of the body in [m]
    /// @param[in] v_eb_n Velocity of the body with respect to the e-system in [m / s], resolved in the n-system
    /// @return The 3x3 matrix 𝐅_13
    /// @note See T. Hobiger (2021) Inertialnavigation V07 - equation (7.21)
    /// @note See Groves (2013) equation (14.66)
    static Eigen::Matrix3d systemMatrixF_13_n(double latitude_b, double height_b, const Eigen::Vector3d& v_eb_n);

    /// @brief Submatrix 𝐅_𝛿v'_𝜓 of the system matrix 𝐅
    /// @param[in] quaternion_np Attitude of the platform with respect to n-system
    /// @param[in] specForce_ib_p Specific force of the body with respect to inertial frame in [m / s^2], resolved in platform coord.
    /// @return The 3x3 matrix 𝐅_21
    /// @note See T. Hobiger (2021) Inertialnavigation V08 - equation (8.4)
    /// @note See Groves (2013) equation (14.67)
    static Eigen::Matrix3d systemMatrixF_21_n(const Eigen::Quaterniond& quaternion_np, const Eigen::Vector3d& specForce_ib_p);

    /// @brief Submatrix 𝐅_𝛿v'_𝛿v of the system matrix 𝐅
    /// @param[in] v_eb_n Velocity of the body with respect to the e-system in [m / s], resolved in the n-system
    /// @param[in] latitude_b Geodetic latitude of the body in [rad]
    /// @param[in] height_b Geodetic height of the body in [m]
    /// @return The 3x3 matrix 𝐅_22
    /// @note See T. Hobiger (2021) Inertialnavigation V08 - equation (8.6, 8.15)
    /// @note See Groves (2013) equation (14.68)
    static Eigen::Matrix3d systemMatrixF_22_n(const Eigen::Vector3d& v_eb_n, double latitude_b, double height_b);

    /// @brief Submatrix 𝐅_𝛿v'_𝛿r of the system matrix 𝐅
    /// @param[in] v_eb_n Velocity of the body with respect to the e-system in [m / s], resolved in the n-system
    /// @param[in] latitude_b Geodetic latitude of the body in [rad]
    /// @param[in] height_b Geodetic height of the body in [m]
    /// @return The 3x3 matrix 𝐅_23
    /// @note See T. Hobiger (2021) Inertialnavigation V08 - equation (8.14, 8.16)
    /// @note See Groves (2013) equation (14.69)
    static Eigen::Matrix3d systemMatrixF_23_n(const Eigen::Vector3d& v_eb_n, double latitude_b, double height_b);

    /// @brief Submatrix 𝐅_𝛿r'_𝛿v of the system matrix 𝐅
    /// @param[in] latitude_b Geodetic latitude of the body in [rad]
    /// @param[in] height_b Geodetic height of the body in [m]
    /// @return The 3x3 matrix 𝐅_32
    /// @note See T. Hobiger (2021) Inertialnavigation V07 - equation (7.5)
    /// @note See Groves (2013) equation (14.70)
    static Eigen::Matrix3d systemMatrixF_32_n(double latitude_b, double height_b);

    /// @brief Submatrix 𝐅_𝛿r'_𝛿r of the system matrix 𝐅
    /// @param[in] v_eb_n Velocity of the body with respect to the e-system in [m / s], resolved in the n-system
    /// @param[in] latitude_b Geodetic latitude of the body in [rad]
    /// @param[in] height_b Geodetic height of the body in [m]
    /// @return The 3x3 matrix 𝐅_33
    /// @note See T. Hobiger (2021) Inertialnavigation V07 - equation (7.5)
    /// @note See Groves (2013) equation (14.71)
    static Eigen::Matrix3d systemMatrixF_33_n(const Eigen::Vector3d& v_eb_n, double latitude_b, double height_b);

    /// @brief Submatrix 𝐅_𝛿a'_𝛿a of the system matrix 𝐅
    /// @param[in] beta_a Gauss-Markov constant for the accelerometer 𝛽 = 1 / 𝜏 (𝜏 correlation length)
    /// @return The 3x3 matrix 𝐅_44
    /// @note See T. Hobiger (2021) Inertialnavigation V06 - equation (6.3)
    static Eigen::Matrix3d systemMatrixF_44_n(const Eigen::Vector3d& beta_a);

    /// @brief Submatrix 𝐅_𝛿ω'_𝛿ω of the system matrix 𝐅
    /// @param[in] beta_omega Gauss-Markov constant for the gyroscope 𝛽 = 1 / 𝜏 (𝜏 correlation length)
    /// @return The 3x3 matrix 𝐅_44
    /// @note See T. Hobiger (2021) Inertialnavigation V06 - equation (6.3)
    static Eigen::Matrix3d systemMatrixF_55_n(const Eigen::Vector3d& beta_omega);

    // ###########################################################################################################
    //                                           Noise input matrix 𝐆
    // ###########################################################################################################

    /// @brief Calculates the noise input matrix 𝐆
    /// @param[in] sigma2_ra Variance of the noise on the accelerometer specific-force measurements
    /// @param[in] sigma2_rg Variance of the noise on the gyro angular-rate measurements
    /// @param[in] beta_a Gauss-Markov constant for the accelerometer 𝛽 = 1 / 𝜏 (𝜏 correlation length)
    /// @param[in] beta_omega Gauss-Markov constant for the gyroscope 𝛽 = 1 / 𝜏 (𝜏 correlation length)
    /// @note See T. Hobiger (2021) Inertialnavigation V06 - equation (6.5)
    Eigen::Matrix<double, 15, 6> noiseInputMatrixG(const double& sigma2_ra, const double& sigma2_rg, const Eigen::Vector3d& beta_a, const Eigen::Vector3d& beta_omega);

    /// @brief Submatrix 𝐆_a of the noise input matrix 𝐆
    /// @param[in] sigma2_ra Variance of the noise on the accelerometer specific-force measurements
    /// @param[in] beta_a Gauss-Markov constant for the accelerometer 𝛽 = 1 / 𝜏 (𝜏 correlation length)
    /// @note See T. Hobiger (2021) Inertialnavigation V06 - equation (6.3)
    Eigen::Matrix3d noiseInputMatrixG_a(const double& sigma2_ra, const Eigen::Vector3d& beta_a);

    /// @brief Submatrix 𝐆_ω of the noise input matrix 𝐆
    /// @param[in] sigma2_rg Variance of the noise on the gyro angular-rate measurements
    /// @param[in] beta_omega Gauss-Markov constant for the gyroscope 𝛽 = 1 / 𝜏 (𝜏 correlation length)
    /// @note See T. Hobiger (2021) Inertialnavigation V06 - equation (6.3)
    Eigen::Matrix3d noiseInputMatrixG_omega(const double& sigma2_rg, const Eigen::Vector3d& beta_omega);

    // ###########################################################################################################
    //                                     System noise covariance matrix 𝐐
    // ###########################################################################################################

    /// @brief System noise covariance matrix 𝐐_{k-1}
    /// @param[in] sigma2_ra Variance of the noise on the accelerometer specific-force measurements
    /// @param[in] sigma2_rg Variance of the noise on the gyro angular-rate measurements
    /// @param[in] sigma2_bad Variance of the accelerometer dynamic bias
    /// @param[in] sigma2_bgd Variance of the gyro dynamic bias
    /// @param[in] F_21_n Submatrix 𝐅_21 of the system matrix 𝐅
    /// @param[in] T_rn_p Conversion matrix between cartesian and curvilinear perturbations to the position
    /// @param[in] DCM_np Direction Cosine Matrix from platform to navigation coordinates
    /// @param[in] tau_s Time interval in [s]
    /// @return The 15x15 matrix of system noise covariances
    static Eigen::Matrix<double, 15, 15> systemNoiseCovarianceMatrix(const double& sigma2_ra, const double& sigma2_rg, const double& sigma2_bad, const double& sigma2_bgd, const Eigen::Matrix3d& F_21_n, const Eigen::Matrix3d& T_rn_p, const Eigen::Matrix3d& DCM_np, const double& tau_s);

    /// @brief S_ra Power Spectral Density of the accelerometer random noise
    /// @param[in] sigma2_ra 𝜎²_ra standard deviation of the noise on the accelerometer specific-force measurements in [m/s^2]
    /// @param[in] tau_i 𝜏ᵢ interval between the input of successive accelerometer outputs to the inertial navigation equations in [s]
    /// @note See P. Groves (2013) - Principles of GNSS, Inertial, and Multisensor Integrated Navigation Systems (ch. 14.2.6)
    [[nodiscard]] static double psdGyroNoise(const double& sigma2_ra, const double& tau_i);

    /// @brief S_rg Power Spectral Density of the gyroscope random noise
    /// @param[in] sigma2_rg 𝜎²_rg standard deviation of the noise on the gyroscope angular-rate measurements in [rad/s]
    /// @param[in] tau_i 𝜏ᵢ interval between the input of successive gyroscope outputs to the inertial navigation equations in [s]
    /// @note See P. Groves (2013) - Principles of GNSS, Inertial, and Multisensor Integrated Navigation Systems (ch. 14.2.6)
    [[nodiscard]] static double psdAccelNoise(const double& sigma2_rg, const double& tau_i);

    /// @brief S_bad Power Spectral Density of the accelerometer bias variation
    /// @param[in] sigma2_bad 𝜎²_bad standard deviation of the accelerometer dynamic bias [m/s^2]
    /// @param[in] tau_i 𝜏ᵢ interval between the input of successive accelerometer outputs to the inertial navigation equations in [s]
    /// @note See P. Groves (2013) - Principles of GNSS, Inertial, and Multisensor Integrated Navigation Systems (ch. 14.2.6)
    [[nodiscard]] static double psdAccelBiasVariation(const double& sigma2_bad, const double& tau_i);

    /// @brief S_bgd Power Spectral Density of the gyroscope bias variation
    /// @param[in] sigma2_bgd 𝜎²_bgd standard deviation of the gyroscope dynamic bias [rad/s]
    /// @param[in] tau_i 𝜏ᵢ interval between the input of successive gyroscope outputs to the inertial navigation equations in [s]
    /// @note See P. Groves (2013) - Principles of GNSS, Inertial, and Multisensor Integrated Navigation Systems (ch. 14.2.6)
    [[nodiscard]] static double psdGyroBiasVariation(const double& sigma2_bgd, const double& tau_i);

    /// @brief Conversion matrix between cartesian and curvilinear perturbations to the position
    /// @param[in] position_lla Position as Lat Lon Alt in [rad rad m]
    /// @param[in] R_N Meridian radius of curvature in [m]
    /// @param[in] R_E Prime vertical radius of curvature (East/West) [m]
    /// @return A 3x3 matrix
    static Eigen::Matrix3d conversionMatrixCartesianCurvilinear(const Eigen::Vector3d& position_lla, const double& R_N, const double& R_E);

    /// @brief Submatrix 𝐐_11 of the system noise covariance matrix 𝐐
    /// @param[in] S_rg Power Spectral Density of the gyroscope random noise
    /// @param[in] S_bgd Power Spectral Density of the gyroscope bias variation
    /// @param[in] tau_s Time interval in [s]
    /// @return The 3x3 matrix 𝐐_11
    /// @note See Groves (2013) equation (14.81)
    static Eigen::Matrix3d systemNoiseCovariance_11(const double& S_rg, const double& S_bgd, const double& tau_s);

    /// @brief Submatrix 𝐐_21 of the system noise covariance matrix 𝐐
    /// @param[in] S_rg Power Spectral Density of the gyroscope random noise
    /// @param[in] S_bgd Power Spectral Density of the gyroscope bias variation
    /// @param[in] F_21_n Submatrix 𝐅_21 of the system matrix 𝐅
    /// @param[in] tau_s Time interval in [s]
    /// @return The 3x3 matrix 𝐐_21
    /// @note See Groves (2013) equation (14.81)
    static Eigen::Matrix3d systemNoiseCovariance_21(const double& S_rg, const double& S_bgd, const Eigen::Matrix3d& F_21_n, const double& tau_s);

    /// @brief Submatrix 𝐐_22 of the system noise covariance matrix 𝐐
    /// @param[in] S_ra Power Spectral Density of the accelerometer random noise
    /// @param[in] S_bad Power Spectral Density of the accelerometer bias variation
    /// @param[in] S_rg Power Spectral Density of the gyroscope random noise
    /// @param[in] S_bgd Power Spectral Density of the gyroscope bias variation
    /// @param[in] F_21_n Submatrix 𝐅_21 of the system matrix 𝐅
    /// @param[in] tau_s Time interval in [s]
    /// @return The 3x3 matrix 𝐐_22
    /// @note See Groves (2013) equation (14.81)
    static Eigen::Matrix3d systemNoiseCovariance_22(const double& S_ra, const double& S_bad, const double& S_rg, const double& S_bgd, const Eigen::Matrix3d& F_21_n, const double& tau_s);

    /// @brief Submatrix 𝐐_25 of the system noise covariance matrix 𝐐
    /// @param[in] S_bgd Power Spectral Density of the gyroscope bias variation
    /// @param[in] F_21_n Submatrix 𝐅_21 of the system matrix 𝐅
    /// @param[in] DCM_np Direction Cosine Matrix from platform to navigation coordinates
    /// @param[in] tau_s Time interval in [s]
    /// @return The 3x3 matrix 𝐐_25
    /// @note See Groves (2013) equation (14.80)
    static Eigen::Matrix3d systemNoiseCovariance_25(const double& S_bgd, const Eigen::Matrix3d& F_21_n, const Eigen::Matrix3d& DCM_np, const double& tau_s);

    /// @brief Submatrix 𝐐_31 of the system noise covariance matrix 𝐐
    /// @param[in] S_rg Power Spectral Density of the gyroscope random noise
    /// @param[in] S_bgd Power Spectral Density of the gyroscope bias variation
    /// @param[in] F_21_n Submatrix 𝐅_21 of the system matrix 𝐅
    /// @param[in] T_rn_p Conversion matrix between cartesian and curvilinear perturbations to the position
    /// @param[in] tau_s Time interval in [s]
    /// @return The 3x3 matrix 𝐐_31
    /// @note See Groves (2013) equation (14.81)
    static Eigen::Matrix3d systemNoiseCovariance_31(const double& S_rg, const double& S_bgd, const Eigen::Matrix3d& F_21_n, const Eigen::Matrix3d& T_rn_p, const double& tau_s);

    /// @brief Submatrix 𝐐_32 of the system noise covariance matrix 𝐐
    /// @param[in] S_ra Power Spectral Density of the accelerometer random noise
    /// @param[in] S_bad Power Spectral Density of the accelerometer bias variation
    /// @param[in] S_rg Power Spectral Density of the gyroscope random noise
    /// @param[in] S_bgd Power Spectral Density of the gyroscope bias variation
    /// @param[in] F_21_n Submatrix 𝐅_21 of the system matrix 𝐅
    /// @param[in] T_rn_p Conversion matrix between cartesian and curvilinear perturbations to the position
    /// @param[in] tau_s Time interval in [s]
    /// @return The 3x3 matrix 𝐐_32
    /// @note See Groves (2013) equation (14.81)
    static Eigen::Matrix3d systemNoiseCovariance_32(const double& S_ra, const double& S_bad, const double& S_rg, const double& S_bgd, const Eigen::Matrix3d& F_21_n, const Eigen::Matrix3d& T_rn_p, const double& tau_s);

    /// @brief Submatrix 𝐐_33 of the system noise covariance matrix 𝐐
    /// @param[in] S_ra Power Spectral Density of the accelerometer random noise
    /// @param[in] S_bad Power Spectral Density of the accelerometer bias variation
    /// @param[in] S_rg Power Spectral Density of the gyroscope random noise
    /// @param[in] S_bgd Power Spectral Density of the gyroscope bias variation
    /// @param[in] T_rn_p Conversion matrix between cartesian and curvilinear perturbations to the position
    /// @param[in] F_21_n Submatrix 𝐅_21 of the system matrix 𝐅
    /// @param[in] tau_s Time interval in [s]
    /// @return The 3x3 matrix 𝐐_33
    /// @note See Groves (2013) equation (14.81)
    static Eigen::Matrix3d systemNoiseCovariance_33(const double& S_ra, const double& S_bad, const double& S_rg, const double& S_bgd, const Eigen::Matrix3d& T_rn_p, const Eigen::Matrix3d& F_21_n, const double& tau_s);

    /// @brief Submatrix 𝐐_34 of the system noise covariance matrix 𝐐
    /// @param[in] S_bad Power Spectral Density of the accelerometer bias variation
    /// @param[in] T_rn_p Conversion matrix between cartesian and curvilinear perturbations to the position
    /// @param[in] DCM_np Direction Cosine Matrix from platform to navigation coordinates
    /// @param[in] tau_s Time interval in [s]
    /// @return The 3x3 matrix 𝐐_34
    /// @note See Groves (2013) equation (14.81)
    static Eigen::Matrix3d systemNoiseCovariance_34(const double& S_bad, const Eigen::Matrix3d& T_rn_p, const Eigen::Matrix3d& DCM_np, const double& tau_s);

    /// @brief Submatrix 𝐐_35 of the system noise covariance matrix 𝐐
    /// @param[in] S_bgd Power Spectral Density of the gyroscope bias variation
    /// @param[in] F_21_n Submatrix 𝐅_21 of the system matrix 𝐅
    /// @param[in] T_rn_p Conversion matrix between cartesian and curvilinear perturbations to the position
    /// @param[in] DCM_np Direction Cosine Matrix from platform to navigation coordinates
    /// @param[in] tau_s Time interval in [s]
    /// @return The 3x3 matrix 𝐐_35
    /// @note See Groves (2013) equation (14.81)
    static Eigen::Matrix3d systemNoiseCovariance_35(const double& S_bgd, const Eigen::Matrix3d& F_21_n, const Eigen::Matrix3d& T_rn_p, const Eigen::Matrix3d& DCM_np, const double& tau_s);

    /// @brief Submatrix 𝐐_42 of the system noise covariance matrix 𝐐
    /// @param[in] S_bad Power Spectral Density of the accelerometer bias variation
    /// @param[in] DCM_np Direction Cosine Matrix from platform to navigation coordinates
    /// @param[in] tau_s Time interval in [s]
    /// @return The 3x3 matrix 𝐐_42
    /// @note See Groves (2013) equation (14.80)
    static Eigen::Matrix3d systemNoiseCovariance_42(const double& S_bad, const Eigen::Matrix3d& DCM_np, const double& tau_s);

    /// @brief Submatrix 𝐐_44 of the system noise covariance matrix 𝐐
    /// @param[in] S_bad Power Spectral Density of the accelerometer bias variation
    /// @param[in] tau_s Time interval in [s]
    /// @return The 3x3 matrix 𝐐_44
    /// @note See Groves (2013) equation (14.80)
    static Eigen::Matrix3d systemNoiseCovariance_44(const double& S_bad, const double& tau_s);

    /// @brief Submatrix 𝐐_51 of the system noise covariance matrix 𝐐
    /// @param[in] S_bgd Power Spectral Density of the gyroscope bias variation
    /// @param[in] DCM_np Direction Cosine Matrix from platform to navigation coordinates
    /// @param[in] tau_s Time interval in [s]
    /// @return The 3x3 matrix 𝐐_51
    /// @note See Groves (2013) equation (14.80)
    static Eigen::Matrix3d systemNoiseCovariance_51(const double& S_bgd, const Eigen::Matrix3d& DCM_np, const double& tau_s);

    /// @brief Submatrix 𝐐_55 of the system noise covariance matrix 𝐐
    /// @param[in] S_bgd Power Spectral Density of the gyroscope bias variation
    /// @param[in] tau_s Time interval in [s]
    /// @return The 3x3 matrix 𝐐_55
    /// @note See Groves (2013) equation (14.80)
    static Eigen::Matrix3d systemNoiseCovariance_55(const double& S_bgd, const double& tau_s);

    // ###########################################################################################################
    //                                                Correction
    // ###########################################################################################################

    /// @brief Measurement matrix for GNSS measurements at timestep k, represented in navigation coordinates
    /// @param[in] T_rn_p Conversion matrix between cartesian and curvilinear perturbations to the position
    /// @param[in] DCM_np Direction Cosine Matrix from platform to navigation coordinates
    /// @param[in] angularRate_ib_p Angular rate of body with respect to inertial system in platform coordinates in [rad/s]
    /// @param[in] leverArm_InsGnss l_{ba}^p lever arm from the INS to the GNSS antenna [m]
    /// @param[in] Omega_ie_n Skew-symmetric matrix of the Earth-rotation vector in local navigation frame axes
    /// @return The 6x15 measurement matrix 𝐇
    static Eigen::Matrix<double, 6, 15> measurementMatrix(const Eigen::Matrix3d& T_rn_p, const Eigen::Matrix3d& DCM_np, const Eigen::Vector3d& angularRate_ib_p, const Eigen::Vector3d& leverArm_InsGnss, const Eigen::Matrix3d& Omega_ie_n);

    /// @brief Submatrix 𝐇_r1 of the measurement sensitivity matrix 𝐇
    /// @param[in] T_rn_p Conversion matrix between cartesian and curvilinear perturbations to the position
    /// @param[in] DCM_np Direction Cosine Matrix from platform to navigation coordinates
    /// @param[in] leverArm_InsGnss l_{ba}^p lever arm from the INS to the GNSS antenna [m]
    /// @return The 3x3 matrix 𝐇_r1
    static Eigen::Matrix3d measurementMatrix_r1_n(const Eigen::Matrix3d& T_rn_p, const Eigen::Matrix3d& DCM_np, const Eigen::Vector3d& leverArm_InsGnss);

    /// @brief Submatrix 𝐇_v1 of the measurement sensitivity matrix 𝐇
    /// @param[in] DCM_np Direction Cosine Matrix from platform to navigation coordinates
    /// @param[in] angularRate_ib_p Angular rate of body with respect to inertial system in platform coordinates in [rad/s]
    /// @param[in] leverArm_InsGnss l_{ba}^p lever arm from the INS to the GNSS antenna [m]
    /// @param[in] Omega_ie_n Skew-symmetric matrix of the Earth-rotation vector in local navigation frame axes
    /// @return The 3x3 matrix 𝐇_v1
    static Eigen::Matrix3d measurementMatrix_v1_n(const Eigen::Matrix3d& DCM_np, const Eigen::Vector3d& angularRate_ib_p, const Eigen::Vector3d& leverArm_InsGnss, const Eigen::Matrix3d& Omega_ie_n);

    /// @brief Submatrix 𝐇_v5 of the measurement sensitivity matrix 𝐇
    /// @param[in] DCM_np Direction Cosine Matrix from platform to navigation coordinates
    /// @param[in] leverArm_InsGnss l_{ba}^p lever arm from the INS to the GNSS antenna [m]
    /// @return The 3x3 matrix 𝐇_v5
    static Eigen::Matrix3d measurementMatrix_v5_n(const Eigen::Matrix3d& DCM_np, const Eigen::Vector3d& leverArm_InsGnss);

    /// @brief Measurement noise covariance matrix 𝐑
    /// @param[in] gnssVarianceLatLonAlt Variances of the position LLA in [rad² rad² m²]
    /// @param[in] gnssVarianceVelocity Variances of the velocity in [m² m² m²]
    /// @return The 6x6 measurement covariance matrix 𝐑
    static Eigen::Matrix<double, 6, 6> measurementNoiseCovariance(const Eigen::Vector3d& gnssVarianceLatLonAlt, const Eigen::Vector3d& gnssVarianceVelocity);

    /// @brief Measurement innovation vector 𝜹𝐳
    /// @param[in] positionMeasurement_lla Position measurement as Lat Lon Alt in [rad rad m]
    /// @param[in] positionEstimate_lla Position estimate as Lat Lon Alt in [rad rad m]
    /// @param[in] velocityMeasurement_n Velocity measurement in the n frame in [m/s]
    /// @param[in] velocityEstimate_n Velocity estimate in the n frame in [m/s]
    /// @param[in] T_rn_p Conversion matrix between cartesian and curvilinear perturbations to the position
    /// @param[in] q_np Rotation quaternion from platform to navigation coordinates
    /// @param[in] leverArm_InsGnss l_{ba}^p lever arm from the INS to the GNSS antenna [m]
    /// @param[in] angularRate_ib_p Angular rate of body with respect to inertial system in platform coordinates in [rad/s]
    /// @param[in] Omega_ie_n Skew-symmetric matrix of the Earth-rotation vector in local navigation frame axes
    /// @return The 6x1 measurement innovation vector 𝜹𝐳
    static Eigen::Matrix<double, 6, 1> measurementInnovation(const Eigen::Vector3d& positionMeasurement_lla, const Eigen::Vector3d& positionEstimate_lla,
                                                             const Eigen::Vector3d& velocityMeasurement_n, const Eigen::Vector3d& velocityEstimate_n,
                                                             const Eigen::Matrix3d& T_rn_p, const Eigen::Quaterniond& q_np, const Eigen::Vector3d& leverArm_InsGnss,
                                                             const Eigen::Vector3d& angularRate_ib_p, const Eigen::Matrix3d& Omega_ie_n);
};
} // namespace NAV