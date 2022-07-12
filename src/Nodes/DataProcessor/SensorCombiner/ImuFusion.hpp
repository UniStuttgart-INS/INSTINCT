/// @file ImuFusion.hpp
/// @brief Combines signals of sensors that provide the same signal-type to one signal
/// @author M. Maier (marcel.maier@ins.uni-stuttgart.de)
/// @date 2022-03-24

#pragma once

#include "Nodes/DataProvider/IMU/Imu.hpp"

#include "util/Container/ScrollingBuffer.hpp"

#include "NodeData/IMU/ImuObs.hpp"
#include "NodeData/State/LcKfInsGnssErrors.hpp"

#include "Navigation/Math/KalmanFilter.hpp"

#include <deque>

namespace NAV
{
/// @brief Combines signals of sensors that provide the same signal-type to one signal
class ImuFusion : public Imu
{
  public:
    /// @brief Default constructor
    ImuFusion();
    /// @brief Destructor
    ~ImuFusion() override;
    /// @brief Copy constructor
    ImuFusion(const ImuFusion&) = delete;
    /// @brief Move constructor
    ImuFusion(ImuFusion&&) = delete;
    /// @brief Copy assignment operator
    ImuFusion& operator=(const ImuFusion&) = delete;
    /// @brief Move assignment operator
    ImuFusion& operator=(ImuFusion&&) = delete;

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

    /// @brief Information needed to link imu data on a certain pin
    struct PinData
    {
        /// @brief Stores the imu data coming from a pin
        struct SensorData
        {
            /// @brief Default constructor (needed to make serialization with json working)
            SensorData() = default;

            /// @brief Constructor
            /// @param[in] displayName Display name of the contained data
            explicit SensorData(std::string displayName)
                : displayName(std::move(displayName)) {}

            /// Display name of the contained data
            std::string displayName;
            /// Flag if data was received, as the buffer contains std::nan("") otherwise
            bool hasData = false;

            /// When connecting a new link. All data is flagged for delete and only those who are also present in the new link are kept
            bool markedForDelete = false;
        };

        /// @brief Possible Pin types

        // enum class PinType : int
        // {
        //     Flow, ///< NodeData Trigger
        // };

        /// @brief Constructor
        PinData() = default;
        /// @brief Destructor
        ~PinData() = default;
        /// @brief Copy constructor
        /// @param[in] other The other element to copy
        PinData(const PinData& other) = default;

        /// @brief Move constructor
        /// @param[in] other The other element to move
        PinData(PinData&& other) = default;

        /// @brief Copy assignment operator
        /// @param[in] rhs The other element to copy
        PinData& operator=(const PinData& rhs)
        {
            if (&rhs != this)
            {
                size = rhs.size;
                dataIdentifier = rhs.dataIdentifier;
                sensorData = rhs.sensorData;
                stride = rhs.stride;
            }

            return *this;
        }
        /// @brief Move assignment operator
        /// @param[in] rhs The other element to move
        PinData& operator=(PinData&& rhs) noexcept
        {
            if (&rhs != this)
            {
                size = rhs.size;
                dataIdentifier = std::move(rhs.dataIdentifier);
                sensorData = std::move(rhs.sensorData);
                stride = rhs.stride;
            }

            return *this;
        }

        /// Size of all buffers of the sensorData elements
        int size = 2000;
        /// Data Identifier of the connected pin
        std::string dataIdentifier;
        /// List with all the data
        std::vector<SensorData> sensorData;
        /// Amount of points to skip for plotting
        int stride = 1;
    };

  private:
    constexpr static size_t OUTPUT_PORT_INDEX_COMBINED_SIGNAL = 0; ///< @brief Flow (ImuObs)
    constexpr static size_t OUTPUT_PORT_INDEX_BIASES = 1;          ///< @brief Flow (ImuBiases)

    /// @brief Initialize the node
    bool initialize() override;

    /// @brief Deinitialize the node
    void deinitialize() override;

    /// @brief Adds/Deletes Input Pins depending on the variable _nInputPins
    void updateNumberOfInputPins();

    /// @brief Initializes the Kalman Filter
    void initializeKalmanFilter();

    /// @brief Initializes the rotation matrices used for the mounting angles of the sensors
    void initializeMountingAngles();

    /// @brief Receive Function for the signal at the time tₖ
    /// @param[in] nodeData Signal to process
    /// @param[in] linkId Id of the link over which the data is received
    void recvSignal(const std::shared_ptr<const NodeData>& nodeData, ax::NodeEditor::LinkId linkId);

    /// @brief Calculates the state-transition-matrix 𝚽
    /// @param[in] dt Time difference between two successive measurements
    /// @return State-transition-matrix 𝚽
    [[nodiscard]] Eigen::MatrixXd initialStateTransitionMatrix_Phi(double& dt) const;

    /// @brief Calculates the state-transition-matrix 𝚽
    /// @param[in] Phi State transition matrix from previous iteration
    /// @param[in] dt Time difference between two successive measurements
    /// @return State-transition-matrix 𝚽
    [[nodiscard]] static Eigen::MatrixXd stateTransitionMatrix_Phi(Eigen::MatrixXd& Phi, double& dt);

    /// @brief Calculates the process noise matrix Q
    /// @param[in] Q Process noise matrix of the previous time step
    /// @param[in] dt Time difference between two successive measurements
    /// @return Process noise matrix Q
    [[nodiscard]] Eigen::MatrixXd processNoiseMatrix_Q(Eigen::MatrixXd& Q, double& dt) const;

    /// @brief Calculates the design matrix H
    /// @param[in] DCM_accel Rotation matrix of mounting angles of an accelerometer w.r.t. a common reference
    /// @param[in] DCM_gyro Rotation matrix of mounting angles of a gyroscope w.r.t. a common reference
    /// @param[in] pinIndex Index of pin to identify sensor
    /// @return Design matrix H
    [[nodiscard]] Eigen::MatrixXd designMatrix_H(Eigen::Matrix3d& DCM_accel, Eigen::Matrix3d& DCM_gyro, size_t pinIndex) const;

    /// @brief Calculates the adaptive measurement noise matrix R
    /// @param[in] alpha Forgetting factor (i.e. weight on previous estimates), 0 < alpha < 1
    /// @param[in] R Measurement noise covariance matrix at the previous epoch
    /// @param[in] e Vector of residuals
    /// @param[in] H Design matrix
    /// @param[in] P Error covariance matrix
    /// @return Measurement noise matrix R
    /// @note See https://arxiv.org/pdf/1702.00884.pdf
    [[nodiscard]] static Eigen::MatrixXd measurementNoiseMatrix_R_adaptive(double alpha,
                                                                           Eigen::MatrixXd& R,
                                                                           Eigen::VectorXd& e,
                                                                           Eigen::MatrixXd& H,
                                                                           Eigen::MatrixXd& P);

    /// @brief Calculates the initial measurement noise matrix R
    /// @param[in] R Measurement noise uncertainty matrix for sensor at 'pinIndex'
    /// @param[in] pinIndex Index of pin to identify sensor
    /// @return Initial measurement noise matrix R
    [[nodiscard]] Eigen::MatrixXd measurementNoiseMatrix_R(Eigen::MatrixXd& R, size_t& pinIndex) const;

    /// @brief Initial error covariance matrix P_0
    /// @param[in] varAngRate Initial variance (3D) of the Angular Rate state in [rad²/s²]
    /// @param[in] varAngAcc Initial variance (3D) of the Angular Acceleration state in [(rad^2)/(s^4)]
    /// @param[in] varAcc Initial variance (3D) of the Acceleration state in [(m^2)/(s^4)]
    /// @param[in] varJerk Initial variance (3D) of the Jerk state in [(m^2)/(s^6)]
    /// @return The (_numStates) x (_numStates) matrix of initial state variances
    [[nodiscard]] Eigen::MatrixXd initialErrorCovarianceMatrix_P0(Eigen::Vector3d& varAngRate,
                                                                  Eigen::Vector3d& varAngAcc,
                                                                  Eigen::Vector3d& varAcc,
                                                                  Eigen::Vector3d& varJerk) const;

    /// @brief Combines the signals
    /// @param[in] imuObs Imu observation
    void combineSignals(std::shared_ptr<const ImuObs>& imuObs);

    /// Number of input pins
    size_t _nInputPins = 2;

    /// @brief Flag to check whether the design matrix H has been initialized
    bool _designMatrixInitialized = false;

    /// @brief Number of estimated states (accel and gyro)
    uint8_t _numStatesEst = 12;

    /// @brief Number of states per pin (biases of accel and gyro)
    uint8_t _numStatesPerPin = 6;

    /// @brief Number of states overall
    uint8_t _numStates = 12;

    /// @brief Number of measurements overall
    uint8_t _numMeasurements = 6;

    /// Data storage for each pin
    std::vector<PinData> _pinData;

    /// @brief Rotations of all connected accelerometers - key: pinIndex, value: Rotation matrix of the accelerometer platform to body frame
    std::vector<Eigen::Matrix3d> _imuRotations_accel;

    /// @brief Rotations of all connected gyros - key: pinIndex, value: Rotation matrix of the gyro platform to body frame
    std::vector<Eigen::Matrix3d> _imuRotations_gyro;

    /// Kalman Filter representation
    KalmanFilter _kalmanFilter{ _numStates, _numMeasurements };

    /// @brief Highest IMU sample rate (for time step in KF prediction)
    double _imuFrequency{ 100 };

    /// @brief Saves the timestamp of the measurement before in [s]
    InsTime _latestTimestamp{};

    /// @brief Container for process noise of each state
    std::vector<Eigen::Vector3d> _biasCovariances;

    /// @brief Container for process noise of each state
    std::vector<Eigen::Vector3d> _processNoiseVariances;

    /// @brief Container for measurement noises of each sensor
    std::vector<Eigen::Vector3d> _measurementNoiseVariances;

    /// @brief Check the rank of the Kalman matrices every iteration (computationally expensive)
    bool _checkKalmanMatricesRanks = false;

    // #########################################################################################################################################
    //                                                        Error Covariance Matrix P
    // #########################################################################################################################################

    /// Possible Units for the initial covariance for the angular rate (standard deviation σ or Variance σ²)
    enum class InitCovarianceAngularRateUnit
    {
        rad2_s2, ///< Variance [rad²/s², rad²/s², rad²/s²]
        rad_s,   ///< Standard deviation [rad/s, rad/s, rad/s]
        deg2_s2, ///< Variance [deg²/s², deg²/s², deg²/s²]
        deg_s,   ///< Standard deviation [deg/s, deg/s, deg/s]
    };
    /// Gui selection for the Unit of the initial covariance for the angular rate
    InitCovarianceAngularRateUnit _initCovarianceAngularRateUnit = InitCovarianceAngularRateUnit::deg_s;

    /// GUI selection of the initial covariance diagonal values for angular rate (standard deviation σ or Variance σ²)
    Eigen::Vector3d _initCovarianceAngularRate{ 1, 1, 1 };

    // #########################################################################################################################################

    /// Possible Units for the initial covariance for the angular acceleration (standard deviation σ or Variance σ²)
    enum class InitCovarianceAngularAccUnit
    {
        rad2_s4, ///< Variance [(rad^2)/(s^4), (rad^2)/(s^4), (rad^2)/(s^4)]
        rad_s2,  ///< Standard deviation [rad/s², rad/s², rad/s²]
        deg2_s4, ///< Variance [(deg^2)/(s^4), (deg^2)/(s^4), (deg^2)/(s^4)]
        deg_s2,  ///< Standard deviation [deg/s², deg/s², deg/s²]
    };
    /// Gui selection for the Unit of the initial covariance for the angular acceleration
    InitCovarianceAngularAccUnit _initCovarianceAngularAccUnit = InitCovarianceAngularAccUnit::deg_s2;

    /// GUI selection of the initial covariance diagonal values for angular acceleration (standard deviation σ or Variance σ²)
    Eigen::Vector3d _initCovarianceAngularAcc{ 0.1, 0.1, 0.1 };

    // #########################################################################################################################################

    /// Possible Units for the initial covariance for the acceleration (standard deviation σ or Variance σ²)
    enum class InitCovarianceAccelerationUnit
    {
        m2_s4, ///< Variance [(m^2)/(s^4), (m^2)/(s^4), (m^2)/(s^4)]
        m_s2,  ///< Standard deviation [m/s², m/s², m/s²]
    };
    /// Gui selection for the Unit of the initial covariance for the acceleration
    InitCovarianceAccelerationUnit _initCovarianceAccelerationUnit = InitCovarianceAccelerationUnit::m_s2;

    /// GUI selection of the initial covariance diagonal values for acceleration (standard deviation σ or Variance σ²)
    Eigen::Vector3d _initCovarianceAcceleration{ 0.1, 0.1, 0.1 };

    // #########################################################################################################################################

    /// Possible Units for the initial covariance for the jerk (standard deviation σ or Variance σ²)
    enum class InitCovarianceJerkUnit
    {
        m2_s6, ///< Variance [(m^2)/(s^6), (m^2)/(s^6), (m^2)/(s^6)]
        m_s3,  ///< Standard deviation [m/s³, m/s³, m/s³]
    };
    /// Gui selection for the Unit of the initial covariance for the jerk
    InitCovarianceJerkUnit _initCovarianceJerkUnit = InitCovarianceJerkUnit::m_s3;

    /// GUI selection of the initial covariance diagonal values for jerk (standard deviation σ or Variance σ²)
    Eigen::Vector3d _initCovarianceJerk{ 0.1, 0.1, 0.1 };

    // #########################################################################################################################################

    /// Possible Units for the initial covariance for the angular rate biases (standard deviation σ or Variance σ²)
    enum class InitCovarianceBiasAngRateUnit
    {
        rad2_s2, ///< Variance [rad²/s², rad²/s², rad²/s²]
        rad_s,   ///< Standard deviation [rad/s, rad/s, rad/s]
        deg2_s2, ///< Variance [deg²/s², deg²/s², deg²/s²]
        deg_s,   ///< Standard deviation [deg/s, deg/s, deg/s]
    };
    /// Gui selection for the Unit of the initial covariance for the angular rate biases
    std::vector<InitCovarianceBiasAngRateUnit> _initCovarianceBiasAngRateUnit = { InitCovarianceBiasAngRateUnit::deg_s };

    /// GUI selection of the initial covariance diagonal values for angular rate biases (standard deviation σ or Variance σ²)
    std::vector<Eigen::Vector3d> _initCovarianceBiasAngRate{ { 1, 1, 1 } };

    // #########################################################################################################################################

    /// Possible Units for the initial covariance for the acceleration biases (standard deviation σ or Variance σ²)
    enum class InitCovarianceBiasAccUnit
    {
        m2_s4, ///< Variance [(m^2)/(s^4), (m^2)/(s^4), (m^2)/(s^4)]
        m_s2,  ///< Standard deviation [m/s², m/s², m/s²]
    };
    /// Gui selection for the Unit of the initial covariance for the acceleration biases
    std::vector<InitCovarianceBiasAccUnit> _initCovarianceBiasAccUnit = { InitCovarianceBiasAccUnit::m_s2 };

    /// GUI selection of the initial covariance diagonal values for acceleration biases (standard deviation σ or Variance σ²)
    std::vector<Eigen::Vector3d> _initCovarianceBiasAcc{ { 0.1, 0.1, 0.1 } };

    // #########################################################################################################################################
    //                                                         Process Noise Matrix Q
    // #########################################################################################################################################

    /// Possible Units for the variance for the process noise of the angular acceleration (standard deviation σ or Variance σ²)
    enum class VarAngularAccNoiseUnit
    {
        rad2_s4, ///< Variance [(rad^2)/(s^4), (rad^2)/(s^4), (rad^2)/(s^4)]
        rad_s2,  ///< Standard deviation [rad/s², rad/s², rad/s²]
        deg2_s4, ///< Variance [(deg^2)/(s^4), (deg^2)/(s^4), (deg^2)/(s^4)]
        deg_s2,  ///< Standard deviation [deg/s², deg/s², deg/s²]
    };
    /// Gui selection for the Unit of the angular acceleration process noise
    VarAngularAccNoiseUnit _varAngularAccNoiseUnit = VarAngularAccNoiseUnit::deg_s2;

    /// GUI selection of the angular acceleration process noise diagonal values
    Eigen::Vector3d _varAngularAccNoise{ 0.1, 0.1, 0.1 };

    // #########################################################################################################################################

    /// Possible Units for the variance for the process noise of the jerk (standard deviation σ or Variance σ²)
    enum class VarJerkNoiseUnit
    {
        m2_s6, ///< Variance [(m^2)/(s^6), (m^2)/(s^6), (m^2)/(s^6)]
        m_s3,  ///< Standard deviation [m/s³, m/s³, m/s³]
    };
    /// Gui selection for the Unit of the jerk process noise
    VarJerkNoiseUnit _varJerkNoiseUnit = VarJerkNoiseUnit::m_s3;

    /// GUI selection of the jerk process noise diagonal values
    Eigen::Vector3d _varJerkNoise{ 0.1, 0.1, 0.1 };

    // #########################################################################################################################################

    /// Possible Units for the variance for the process noise of the angular rate (standard deviation σ or Variance σ²)
    enum class VarBiasAngRateNoiseUnit
    {
        rad2_s2, ///< Variance [rad²/s², rad²/s², rad²/s²]
        rad_s,   ///< Standard deviation [rad/s, rad/s, rad/s]
        deg2_s2, ///< Variance [deg²/s², deg²/s², deg²/s²]
        deg_s,   ///< Standard deviation [deg/s, deg/s, deg/s]
    };
    /// Gui selection for the Unit of the process noise of the angular rate
    std::vector<VarBiasAngRateNoiseUnit> _varBiasAngRateNoiseUnit = { VarBiasAngRateNoiseUnit::deg_s };

    /// GUI selection of the process noise of the angular rate diagonal values (standard deviation σ or Variance σ²)
    std::vector<Eigen::Vector3d> _varBiasAngRateNoise = { { 1, 1, 1 } };

    // #########################################################################################################################################

    /// Possible Units for the variance for the process noise of the acceleration (standard deviation σ or Variance σ²)
    enum class VarBiasAccelerationNoiseUnit
    {
        m2_s4, ///< Variance [(m^2)/(s^4), (m^2)/(s^4), (m^2)/(s^4)]
        m_s2,  ///< Standard deviation [m/s², m/s², m/s²]
    };
    /// Gui selection for the Unit of the process noise of the acceleration
    std::vector<VarBiasAccelerationNoiseUnit> _varBiasAccelerationNoiseUnit = { VarBiasAccelerationNoiseUnit::m_s2 };

    /// GUI selection of the process noise of the acceleration diagonal values (standard deviation σ or Variance σ²)
    std::vector<Eigen::Vector3d> _varBiasAccelerationNoise{ { 0.1, 0.1, 0.1 } };

    // #########################################################################################################################################
    //                                                       Measurement Noise Matrix R
    // #########################################################################################################################################

    /// Possible Units for the initial covariance of the angular rate measurements (standard deviation σ or Variance σ²)
    enum class MeasurementUncertaintyAngularRateUnit
    {
        rad2_s2, ///< Variance [rad²/s², rad²/s², rad²/s²]
        rad_s,   ///< Standard deviation [rad/s, rad/s, rad/s]
        deg2_s2, ///< Variance [deg²/s², deg²/s², deg²/s²]
        deg_s,   ///< Standard deviation [deg/s, deg/s, deg/s]
    };
    /// Gui selection for the unit of the angular rate's measurement uncertainty
    std::vector<MeasurementUncertaintyAngularRateUnit> _measurementUncertaintyAngularRateUnit = { MeasurementUncertaintyAngularRateUnit::deg_s };

    /// Gui selection of the angular rate measurement uncertainty diagonal values
    std::vector<Eigen::Vector3d> _measurementUncertaintyAngularRate{ { 1, 1, 1 } };

    /// Possible Units for the initial covariance of the acceleration measurements (standard deviation σ or Variance σ²)
    enum class MeasurementUncertaintyAccelerationUnit
    {
        m2_s4, ///< Variance [(m^2)/(s^4), (m^2)/(s^4), (m^2)/(s^4)]
        m_s2,  ///< Standard deviation [m/s², m/s², m/s²]
    };
    /// Gui selection for the unit of the acceleration's measurement uncertainty
    std::vector<MeasurementUncertaintyAccelerationUnit> _measurementUncertaintyAccelerationUnit = { MeasurementUncertaintyAccelerationUnit::m_s2 };

    /// Gui selection of the angular acceleration measurement uncertainty diagonal values
    std::vector<Eigen::Vector3d> _measurementUncertaintyAcceleration{ { 0.1, 0.1, 0.1 } };
};

} // namespace NAV
