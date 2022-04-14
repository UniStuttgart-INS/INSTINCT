/// @file SensorCombiner.hpp
/// @brief Combines signals of sensors that provide the same signal-type to one signal
/// @author M. Maier (marcel.maier@ins.uni-stuttgart.de)
/// @date 2022-03-24

#pragma once

// #include "internal/Node/Node.hpp"

#include "Nodes/DataProvider/IMU/Imu.hpp"

#include "util/Container/ScrollingBuffer.hpp"

#include "NodeData/IMU/ImuObs.hpp"

#include "Navigation/Math/KalmanFilter.hpp"

#include <deque>

namespace NAV
{
/// @brief Combines signals of sensors that provide the same signal-type to one signal
class SensorCombiner : public Imu
{
  public:
    /// @brief Default constructor
    SensorCombiner();
    /// @brief Destructor
    ~SensorCombiner() override;
    /// @brief Copy constructor
    SensorCombiner(const SensorCombiner&) = delete;
    /// @brief Move constructor
    SensorCombiner(SensorCombiner&&) = delete;
    /// @brief Copy assignment operator
    SensorCombiner& operator=(const SensorCombiner&) = delete;
    /// @brief Move assignment operator
    SensorCombiner& operator=(SensorCombiner&&) = delete;

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
            /// @param[in] size Size of the buffer
            explicit SensorData(std::string displayName, size_t size)
                : displayName(std::move(displayName)), buffer(size) {}

            /// Display name of the contained data
            std::string displayName;
            /// Buffer for the data
            ScrollingBuffer<double> buffer;
            /// Flag if data was received, as the buffer contains std::nan("") otherwise
            bool hasData = false;

            /// When connecting a new link. All data is flagged for delete and only those who are also present in the new link are kept
            bool markedForDelete = false;
        };

        /// @brief Possible Pin types
        enum class PinType : int
        {
            Flow, ///< NodeData Trigger
        };

        /// @brief Constructor
        PinData() = default;
        /// @brief Destructor
        ~PinData() = default;
        /// @brief Copy constructor
        /// @param[in] other The other element to copy
        PinData(const PinData& other)
            : size(other.size),
              dataIdentifier(other.dataIdentifier),
              sensorData(other.sensorData),
              pinType(other.pinType),
              stride(other.stride) {}

        /// @brief Move constructor
        /// @param[in] other The other element to move
        PinData(PinData&& other) noexcept
            : size(other.size),
              dataIdentifier(std::move(other.dataIdentifier)),
              sensorData(std::move(other.sensorData)),
              pinType(other.pinType),
              stride(other.stride) {}

        /// @brief Copy assignment operator
        /// @param[in] rhs The other element to copy
        PinData& operator=(const PinData& rhs)
        {
            if (&rhs != this)
            {
                size = rhs.size;
                dataIdentifier = rhs.dataIdentifier;
                sensorData = rhs.sensorData;
                pinType = rhs.pinType;
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
                pinType = rhs.pinType;
                stride = rhs.stride;
            }

            return *this;
        }

        /// @brief Adds a sensorData Element to the list
        /// @param[in] dataIndex Index where to add the data to
        /// @param[in] displayName Display name of the contained data
        void addSensorDataItem(size_t dataIndex, const std::string& displayName)
        {
            if (sensorData.size() > dataIndex)
            {
                if (sensorData.at(dataIndex).displayName == displayName) // Item was restored already at this position
                {
                    sensorData.at(dataIndex).markedForDelete = false;
                    return;
                }

                // Some other item was restored at this position
                if (!sensorData.at(dataIndex).markedForDelete)
                {
                    LOG_WARN("Adding SensorData item '{}' at position {}, but at this position exists already the item '{}'. Reordering the items to match the data. Consider resaving the flow file.",
                             displayName, dataIndex, sensorData.at(dataIndex).displayName);
                }
                auto searchIter = std::find_if(sensorData.begin(),
                                               sensorData.end(),
                                               [displayName](const SensorData& sensorData) { return sensorData.displayName == displayName; });
                auto iter = sensorData.begin();
                std::advance(iter, dataIndex);
                if (searchIter == sensorData.end()) // Item does not exist yet. Developer added a new item to the list
                {
                    sensorData.insert(iter, SensorData{ displayName, static_cast<size_t>(size) });
                }
                else // Item exists already. Developer reordered the items in the list
                {
                    std::rotate(searchIter, searchIter + 1, iter);
                }
                iter->markedForDelete = false;
            }
            else if (std::find_if(sensorData.begin(),
                                  sensorData.end(),
                                  [displayName](const SensorData& sensorData) { return sensorData.displayName == displayName; })
                     != sensorData.end())
            {
                LOG_ERROR("Adding the SensorData item {} at position {}, but this sensor item was found at another position already",
                          displayName, dataIndex);
            }
            else // Item not there yet. Add to the end of the list
            {
                sensorData.emplace_back(displayName, static_cast<size_t>(size));
            }
        }

        /// Size of all buffers of the sensorData elements
        int size = 2000;
        /// Data Identifier of the connected pin
        std::string dataIdentifier;
        /// List with all the data
        std::vector<SensorData> sensorData;
        /// Pin Type
        PinType pinType = PinType::Flow;
        /// Amount of points to skip for plotting
        int stride = 1;
        /// Mutex to lock the buffer so that the GUI thread and the calculation threads don't cause a data race
        std::mutex mutex;
    };

  private:
    constexpr static size_t OUTPUT_PORT_INDEX_COMBINED_SIGNAL = 0; ///< @brief Flow (InertialNavSol)

    /// @brief Initialize the node
    bool initialize() override;

    /// @brief Deinitialize the node
    void deinitialize() override;

    /// @brief Adds/Deletes Input Pins depending on the variable _nInputPins
    void updateNumberOfInputPins();

    /// @brief Receive Function for the signal at the time tₖ
    /// @param[in] nodeData Signal to process
    /// @param[in] linkId Id of the link over which the data is received
    void recvSignal(const std::shared_ptr<const NodeData>& nodeData, ax::NodeEditor::LinkId linkId);

    /// @brief Calculates the state-transition-matrix 𝚽
    /// @return State-transition-matrix 𝚽
    [[nodiscard]] const Eigen::MatrixXd stateTransitionMatrix_Phi();

    /// @brief Calculates the process noise matrix Q
    /// @param[in] dt Time difference between two successive measurements
    /// @param[in] sigma_w Standard deviation of angular acceleration (omegaDot) in [rad/s^2]
    /// @param[in] sigma_f Standard deviation of specific force in [m/s^3]
    /// @param[in] sigma_biasw Standard deviation of the bias on the angular acceleration (omegaDot) in [rad/s^2]
    /// @param[in] sigma_biasf Standard deviation of the bias on the specific force in [m/s^3]
    /// @param[in] M Number of connected sensors
    /// @return Process noise matrix Q
    [[nodiscard]] static Eigen::MatrixXd processNoiseMatrix_Q(double dt,
                                                              double sigma_w,
                                                              double sigma_f,
                                                              double sigma_biasw,
                                                              double sigma_biasf,
                                                              uint8_t M);

    /// @brief Calculates the design matrix H
    /// @param[in] DCM Rotation matrix of mounting angles of a sensor w.r.t. a common reference
    /// @return Design matrix H
    [[nodiscard]] const Eigen::MatrixXd designMatrix_H(Eigen::Matrix<double, 3, 3>& DCM);

    /// @brief Calculates the adaptive measurement noise matrix R
    /// @param[in] alpha Forgetting factor (i.e. weight on previous estimates), 0 < alpha < 1
    /// @param[in] R Measurement noise covariance matrix at the previous epoch
    /// @param[in] e Vector of residuals
    /// @param[in] H Design matrix
    /// @param[in] P Error covariance matrix
    /// @return Measurement noise matrix R
    /// @note See https://arxiv.org/pdf/1702.00884.pdf
    [[nodiscard]] static Eigen::MatrixXd measurementNoiseMatrix_R(double alpha,
                                                                  Eigen::MatrixXd& R,
                                                                  Eigen::VectorXd& e,
                                                                  Eigen::Matrix<double, Eigen::Dynamic, 9>& H,
                                                                  Eigen::Matrix<double, 9, 9>& P);

    /// @brief Calculates the initial measurement noise matrix R
    /// @param[in] sigma_w Standard deviation of angular velocity
    /// @param[in] sigma_f Standard deviation of specific force
    /// @param[in] M Number of connected sensors
    /// @return Initial measurement noise matrix R
    [[nodiscard]] static Eigen::MatrixXd measurementNoiseMatrix_R_init(double sigma_w, double sigma_f, uint8_t M);

    /// @brief Initial error covariance matrix P_0
    /// @param[in] M Number of connected sensors
    /// @return The (_numStates) x (_numStates) matrix of initial state variances
    [[nodiscard]] static Eigen::MatrixXd initialErrorCovarianceMatrix_P0(uint8_t M); // TODO: Extend by GUI inputs of 2x3D variances per one sensor

    // Eigen::Matrix<double, Eigen::Dynamic, 1> residuals(double omega, double omegadot, double f, double w, double a, Eigen::Matrix<double, 3, Eigen::Dynamic> IMU_pos, Eigen::Matrix<double, 3, 3> DCM, uint8_t M, uint64_t ti);

    /// @brief Combines the signals
    void combineSignals();

    /// Number of input pins
    size_t _nInputPins = 1;

    /// Data storage for each pin
    std::vector<PinData> _pinData;

    /// IMU Observation list
    /// Length depends on the integration algorithm. Newest observation first (tₖ, tₖ₋₁, tₖ₋₂, ...)
    std::deque<std::shared_ptr<const ImuObs>> _imuObservations;

    /// @brief Maximum amount of imu observations to keep
    size_t _maxSizeImuObservations = 0;

    /// @brief Number of states overall
    uint8_t _numStates = 0;

    /// @brief Number of measurements
    uint8_t _numMeasurements = 0;

    /// Kalman Filter representation
    KalmanFilter _kalmanFilter{ 12, 6 };

    // #########################################################################################################################################
    //                                                        Error Covariance Matrix P
    // #########################################################################################################################################

    /// Possible Units for the initial covariance for the angular rate (standard deviation σ or Variance σ²)
    enum class InitCovarianceAngularRateUnit
    {
        rad2_s2, ///< Variance [(rad/s)^2, (rad/s)^2, (rad/s)^2]
        rad_s,   ///< Standard deviation [rad/s, rad/s, rad/s]
        deg2_s2, ///< Variance [(deg/s)^2, (deg/s)^2, (deg/s)^2]
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
        rad_s2,  ///< Standard deviation [rad/s^2, rad/s^2, rad/s^2]
        deg2_s4, ///< Variance [(deg^2)/(s^4), (deg^2)/(s^4), (deg^2)/(s^4)]
        deg_s2,  ///< Standard deviation [deg/s^2, deg/s^2, deg/s^2]
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
        m_s2,  ///< Standard deviation [m/s^2, m/s^2, m/s^2]
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
        m_s3,  ///< Standard deviation [m/s^3, m/s^3, m/s^3]
    };
    /// Gui selection for the Unit of the initial covariance for the jerk
    InitCovarianceJerkUnit _initCovarianceJerkUnit = InitCovarianceJerkUnit::m_s3;

    /// GUI selection of the initial covariance diagonal values for jerk (standard deviation σ or Variance σ²)
    Eigen::Vector3d _initCovarianceJerk{ 0.1, 0.1, 0.1 };

    // #########################################################################################################################################

    /// Possible Units for the initial covariance for the angular acceleration biases (standard deviation σ or Variance σ²)
    enum class InitCovarianceBiasAngAccUnit
    {
        rad2_s4, ///< Variance [(rad^2)/(s^4), (rad^2)/(s^4), (rad^2)/(s^4)]
        rad_s2,  ///< Standard deviation [rad/s^2, rad/s^2, rad/s^2]
        deg2_s4, ///< Variance [(deg^2)/(s^4), (deg^2)/(s^4), (deg^2)/(s^4)]
        deg_s2,  ///< Standard deviation [deg/s^2, deg/s^2, deg/s^2]
    };
    /// Gui selection for the Unit of the initial covariance for the angular acceleration biases
    InitCovarianceBiasAngAccUnit _initCovarianceBiasAngAccUnit = InitCovarianceBiasAngAccUnit::deg_s2;

    /// GUI selection of the initial covariance diagonal values for angular acceleration biases (standard deviation σ or Variance σ²)
    Eigen::Vector3d _initCovarianceBiasAngAcc{ 1, 1, 1 };

    // #########################################################################################################################################

    /// Possible Units for the initial covariance for the jerk biases (standard deviation σ or Variance σ²)
    enum class InitCovarianceBiasJerkUnit
    {
        m2_s6, ///< Variance [(m^2)/(s^6), (m^2)/(s^6), (m^2)/(s^6)]
        m_s3,  ///< Standard deviation [m/s^3, m/s^3, m/s^3]
    };
    /// Gui selection for the Unit of the initial covariance for the jerk biases
    InitCovarianceBiasJerkUnit _initCovarianceBiasJerkUnit = InitCovarianceBiasJerkUnit::m_s3;

    /// GUI selection of the initial covariance diagonal values for jerk biases (standard deviation σ or Variance σ²)
    Eigen::Vector3d _initCovarianceBiasJerk{ 0.1, 0.1, 0.1 };

    // #########################################################################################################################################
    //                                                         Process Noise Matrix Q
    // #########################################################################################################################################

    /// Possible Units for the initial covariance for the angular acceleration (standard deviation σ or Variance σ²)
    enum class StdevAngularAccUnit
    {
        rad_s2, ///< Standard deviation [rad/s^2, rad/s^2, rad/s^2]
        deg_s2, ///< Standard deviation [deg/s^2, deg/s^2, deg/s^2]
    };
    /// Gui selection for the Unit of the angular acceleration process noise
    StdevAngularAccUnit _stdevAngularAccUnit = StdevAngularAccUnit::deg_s2;

    /// GUI selection of the angular acceleration process noise diagonal values
    Eigen::Vector3d _stdevAngularAcc{ 0.1, 0.1, 0.1 };

    // #########################################################################################################################################

    /// Possible Units for the initial covariance for the jerk (standard deviation σ or Variance σ²)
    enum class StdevJerkUnit
    {
        m_s3, ///< Standard deviation [m/s^3, m/s^3, m/s^3]
    };
    /// Gui selection for the Unit of the jerk process noise
    StdevJerkUnit _stdevJerkUnit = StdevJerkUnit::m_s3;

    /// GUI selection of the jerk process noise diagonal values
    Eigen::Vector3d _stdevJerk{ 0.1, 0.1, 0.1 };

    // #########################################################################################################################################
    //                                                       Measurement Noise Matrix R
    // #########################################################################################################################################
};

} // namespace NAV
