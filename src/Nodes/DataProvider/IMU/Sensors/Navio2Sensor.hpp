/// @file Navio2Sensor.hpp
/// @brief Vector Nav Sensors
/// @author T. Topp (thomas.topp@nav.uni-stuttgart.de)
/// @date 2020-07-13

#pragma once

#include "NodeData/IMU/ImuObs.hpp"
#include "../Imu.hpp"

#if !__APPLE__
    #include "navio/Common/InertialSensor.h"
#endif

#include "util/CallbackTimer.hpp"
#include <chrono>

namespace NAV
{
/// Navio2Sensor Sensor Class
class Navio2Sensor final : public Imu
{
  public:
    /// @brief Constructor
    /// @param[in] name Name of the Sensor
    /// @param[in] options Program options string map
    Navio2Sensor(const std::string& name, const std::map<std::string, std::string>& options);

    /// @brief Default constructor
    Navio2Sensor() = default;
    /// @brief Destructor
    ~Navio2Sensor() final;
    /// @brief Copy constructor
    Navio2Sensor(const Navio2Sensor&) = delete;
    /// @brief Move constructor
    Navio2Sensor(Navio2Sensor&&) = delete;
    /// @brief Copy assignment operator
    Navio2Sensor& operator=(const Navio2Sensor&) = delete;
    /// @brief Move assignment operator
    Navio2Sensor& operator=(Navio2Sensor&&) = delete;

    /// @brief Returns the String representation of the Class Type
    /// @return The class type
    [[nodiscard]] constexpr std::string_view type() const final
    {
        return std::string_view("Navio2Sensor");
    }

    /// @brief Returns the String representation of the Class Category
    /// @return The class category
    [[nodiscard]] constexpr std::string_view category() const final
    {
        return std::string_view("DataProvider");
    }

    /// @brief Returns Gui Configuration options for the class
    /// @return The gui configuration
    [[nodiscard]] std::vector<ConfigOptions> guiConfig() const final
    {
        std::vector<ConfigOptions> configs = { { CONFIG_LIST, "Imu", "Select the IMU", { "[MPU9250]", "LSM9DS1" } },
                                               { CONFIG_INT, "Frequency", "Data Output Frequency", { "0", "100", "200" } } };
        auto imuConfigs = Imu::guiConfig();
        configs.insert(configs.end(), imuConfigs.begin(), imuConfigs.end());
        return configs;
    }

    /// @brief Returns the context of the class
    /// @return The class context
    [[nodiscard]] constexpr NodeContext context() const final
    {
        return NodeContext::REAL_TIME;
    }

    /// @brief Returns the number of Ports
    /// @param[in] portType Specifies the port type
    /// @return The number of ports
    [[nodiscard]] constexpr uint8_t nPorts(PortType portType) const final
    {
        switch (portType)
        {
        case PortType::In:
            break;
        case PortType::Out:
            return 2U;
        }

        return 0U;
    }

    /// @brief Returns the data types provided by this class
    /// @param[in] portType Specifies the port type
    /// @param[in] portIndex Port index on which the data is sent
    /// @return The data type and subtitle
    [[nodiscard]] constexpr std::pair<std::string_view, std::string_view> dataType(PortType portType, uint8_t portIndex) const final
    {
        switch (portType)
        {
        case PortType::In:
            break;
        case PortType::Out:
            if (portIndex == 0)
            {
                return std::make_pair(ImuObs().type(), std::string_view(""));
            }
            if (portIndex == 1)
            {
                return std::make_pair(ImuPos().type(), std::string_view(""));
            }
        }

        return std::make_pair(std::string_view(""), std::string_view(""));
    }

    /// @brief Handles the data sent on the input port
    /// @param[in] portIndex The input port index
    /// @param[in, out] data The data send on the input port
    void handleInputData([[maybe_unused]] uint8_t portIndex, [[maybe_unused]] std::shared_ptr<NodeData> data) final {}

    /// @brief Requests the node to send out its data
    /// @param[in] portIndex The output port index
    /// @return The requested data or nullptr if no data available
    [[nodiscard]] std::shared_ptr<NodeData> requestOutputData(uint8_t portIndex) final
    {
        if (portIndex == 1)
        {
            return imuPos;
        }

        return nullptr;
    }

  private:
    /// Enumeration of IMUs on the Navio2
    enum ImuType
    {
        /// MPU9250
        MPU,
        /// LSM9DS1
        LSM
    };

#if !__APPLE__
    /// Sensor object
    std::unique_ptr<InertialSensor> sensor;
#endif

    /// The Imu type
    ImuType imuType = ImuType::MPU;

    /// OutputFrequency to calculate rateDivisor field.
    uint16_t outputFrequency = 1;

    /// Timer object to handle async data requests
    CallbackTimer timer;
    /// Start Time to calculate the TimeSinceStartup
    std::chrono::time_point<std::chrono::high_resolution_clock> startTime;

    /// Accelerometer X data, which are read into by the sensor
    float ax{};
    /// Accelerometer Y data, which are read into by the sensor
    float ay{};
    /// Accelerometer Z data, which are read into by the sensor
    float az{};
    /// Gyroscope X data, which are read into by the sensor
    float gx{};
    /// Gyroscope Y data, which are read into by the sensor
    float gy{};
    /// Gyroscope Z data, which are read into by the sensor
    float gz{};
    /// Magnetometer X data, which are read into by the sensor
    float mx{};
    /// Magnetometer Y data, which are read into by the sensor
    float my{};
    /// Magnetometer Z data, which are read into by the sensor
    float mz{};

    /// @brief Function which performs the async data reading
    /// @param[in, out] userData Pointer to the Navio2Sensor object
    static void readImuThread(void* userData);
};

} // namespace NAV