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
    /// @param[in] dt Time difference between two successive measurements
    /// @param[in] M Number of connected sensors
    /// @return State-transition-matrix 𝚽
    Eigen::Matrix<double, 9, 9> stateTransitionMatrix_Phi(double dt, uint8_t M);

    /// @brief Calculates the process noise matrix Q
    /// @param[in] dt Time difference between two successive measurements
    /// @param[in] sigma_a Standard deviation of angular acceleration (omegaDot)
    /// @param[in] sigma_f Standard deviation of specific force
    /// @param[in] sigma_biasw Standard deviation of the bias on the angular acceleration (omegaDot)
    /// @param[in] sigma_biasf Standard deviation of the bias on the specific force
    /// @param[in] M Number of connected sensors
    /// @return Process noise matrix Q
    Eigen::Matrix<double, 9, 9> processNoiseMatrix_Q(double dt, double sigma_a, double sigma_f, double sigma_biasw, double sigma_biasf, uint8_t M);

    /// @brief Calculates the design matrix H
    /// @param[in] omega Angular velocity in [rad/s]
    /// @param[in] omegadot Angular acceleration in [rad/s^2]
    /// @param[in] R Measurement noise matrix
    /// @param[in] DCM Rotation matrix of mounting angles of a sensor w.r.t. a common reference
    /// @param[in] M Number of connected sensors
    /// @return Design matrix H
    Eigen::Matrix<double, 12, 9> designMatrix_H(double omega, double omegadot, Eigen::Matrix<double, 12, 12> R, Eigen::Matrix<double, 3, 3> DCM, uint8_t M); // FIXME: # of rows must equal # of measurements, in R, too

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
};

} // namespace NAV
