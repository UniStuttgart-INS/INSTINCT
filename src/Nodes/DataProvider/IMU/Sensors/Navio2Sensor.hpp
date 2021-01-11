/// @file Navio2Sensor.hpp
/// @brief Vector Nav Sensors
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2020-07-13

#pragma once

#include "Nodes/DataProvider/IMU/Imu.hpp"

#if !__APPLE__
    #include "navio/Common/InertialSensor.h"
#endif

#include "util/CallbackTimer.hpp"
#include <chrono>

namespace NAV
{
/// Navio2Sensor Sensor Class
class Navio2Sensor : public Imu
{
  public:
    /// @brief Default constructor
    Navio2Sensor();
    /// @brief Destructor
    ~Navio2Sensor() override;
    /// @brief Copy constructor
    Navio2Sensor(const Navio2Sensor&) = delete;
    /// @brief Move constructor
    Navio2Sensor(Navio2Sensor&&) = delete;
    /// @brief Copy assignment operator
    Navio2Sensor& operator=(const Navio2Sensor&) = delete;
    /// @brief Move assignment operator
    Navio2Sensor& operator=(Navio2Sensor&&) = delete;

    /// @brief String representation of the Class Type
    [[nodiscard]] static std::string typeStatic();

    /// @brief String representation of the Class Type
    [[nodiscard]] std::string type() const override;

    /// @brief String representation of the Class Category
    [[nodiscard]] static std::string category();

    /// @brief ImGui config window which is shown on double click
    /// @attention Don't forget to set hasConfig to true in the constructor of the node
    void guiConfig() override;

    /// @brief Saves the node into a json object
    [[nodiscard]] json save() const override;

    /// @brief Restores the node from a json object
    /// @param[in] j Json object with the node state
    void restore(const json& j) override;

    /// @brief Initialize the node
    bool initialize() override;

    /// @brief Deinitialize the node
    void deinitialize() override;

  private:
    constexpr static size_t OutputPortIndex_Navio2Sensor = 0; ///< @brief Delegate
    constexpr static size_t OutputPortIndex_ImuObs = 1;       ///< @brief Flow (ImuObs)

    /// Enumeration of IMUs on the Navio2
    enum ImuType : int
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
    int outputFrequency = 100;

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