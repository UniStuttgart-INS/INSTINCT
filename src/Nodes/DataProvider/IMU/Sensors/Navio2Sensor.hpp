/// @file Navio2Sensor.hpp
/// @brief Vector Nav Sensors
/// @author T. Topp (thomas.topp@nav.uni-stuttgart.de)
/// @date 2020-07-13

#pragma once

#ifndef DISABLE_SENSORS

    #include "NodeData/IMU/ImuObs.hpp"
    #include "../Imu.hpp"

    #include "navio/Common/InertialSensor.h"

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
        return { { CONFIG_LIST, "Imu", "Select the IMU", { "[MPU9250]", "LSM9DS1" } },
                 { CONFIG_INT, "Frequency", "Data Output Frequency", { "0", "100", "200" } } };
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
            return 1U;
        }

        return 0U;
    }

    /// @brief Returns the data types provided by this class
    /// @param[in] portType Specifies the port type
    /// @param[in] portIndex Port index on which the data is sent
    /// @return The data type
    [[nodiscard]] constexpr std::string_view dataType(PortType portType, uint8_t portIndex) const final
    {
        switch (portType)
        {
        case PortType::In:
            break;
        case PortType::Out:
            if (portIndex == 0)
            {
                return ImuObs().type();
            }
        }

        return std::string_view("");
    }

    /// @brief Handles the data sent on the input port
    /// @param[in] portIndex The input port index
    /// @param[in, out] data The data send on the input port
    void handleInputData([[maybe_unused]] uint8_t portIndex, [[maybe_unused]] std::shared_ptr<NodeData> data) final {}

  private:
    /// Enumeration of IMUs on the Navio2
    enum ImuType
    {
        /// MPU9250
        MPU,
        /// LSM9DS1
        LSM
    };

    /// Sensor object
    std::unique_ptr<InertialSensor> sensor;

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

#endif