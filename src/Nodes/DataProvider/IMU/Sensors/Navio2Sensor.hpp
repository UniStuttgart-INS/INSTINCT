// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file Navio2Sensor.hpp
/// @brief Navio2 Sensors
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2020-07-13

#pragma once

#include "Nodes/DataProvider/IMU/Imu.hpp"

#if !__APPLE__ && !defined(WIN32) && !defined(_WIN32) && !defined(__WIN32)
    #include "Navio/Common/InertialSensor.h"
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
    /// @attention Don't forget to set _hasConfig to true in the constructor of the node
    void guiConfig() override;

    /// @brief Saves the node into a json object
    [[nodiscard]] json save() const override;

    /// @brief Restores the node from a json object
    /// @param[in] j Json object with the node state
    void restore(const json& j) override;

    /// @brief Resets the node. It is guaranteed that the node is initialized when this is called.
    bool resetNode() override;

  private:
    constexpr static size_t OUTPUT_PORT_INDEX_IMU_OBS = 0; ///< @brief Flow (ImuObs)

    /// @brief Initialize the node
    bool initialize() override;

    /// @brief Deinitialize the node
    void deinitialize() override;

    /// Enumeration of IMUs on the Navio2
    enum ImuType : int
    {
        /// MPU9250
        MPU,
        /// LSM9DS1
        LSM
    };

#if !__APPLE__ && !defined(WIN32) && !defined(_WIN32) && !defined(__WIN32)
    /// Sensor object
    std::unique_ptr<InertialSensor> _sensor;
#endif

    /// The Imu type
    ImuType _imuType = ImuType::MPU;

    /// OutputFrequency to calculate rateDivisor field.
    int _outputFrequency = 100;

    /// Timer object to handle async data requests
    CallbackTimer _timer;
    /// Start Time to calculate the TimeSinceStartup
    std::chrono::time_point<std::chrono::steady_clock> _startTime;

    /// Accelerometer X data, which are read into by the sensor
    float _ax{};
    /// Accelerometer Y data, which are read into by the sensor
    float _ay{};
    /// Accelerometer Z data, which are read into by the sensor
    float _az{};
    /// Gyroscope X data, which are read into by the sensor
    float _gx{};
    /// Gyroscope Y data, which are read into by the sensor
    float _gy{};
    /// Gyroscope Z data, which are read into by the sensor
    float _gz{};
    /// Magnetometer X data, which are read into by the sensor
    float _mx{};
    /// Magnetometer Y data, which are read into by the sensor
    float _my{};
    /// Magnetometer Z data, which are read into by the sensor
    float _mz{};

    /// @brief Function which performs the async data reading
    /// @param[in, out] userData Pointer to the Navio2Sensor object
    static void readImuThread(void* userData);
};

} // namespace NAV