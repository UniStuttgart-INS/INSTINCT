/// @file KvhSensor.hpp
/// @brief KVH Sensors
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2020-06-30

#pragma once

#include "Nodes/DataProvider/IMU/Imu.hpp"
#include "Nodes/DataProvider/Protocol/UartSensor.hpp"
#include "util/UartSensors/KVH/KvhUartSensor.hpp"

namespace NAV
{
/// KVH Sensor Class
class KvhSensor : public Imu, public UartSensor
{
  public:
    /// @brief Default constructor
    KvhSensor();
    /// @brief Destructor
    ~KvhSensor() override;
    /// @brief Copy constructor
    KvhSensor(const KvhSensor&) = delete;
    /// @brief Move constructor
    KvhSensor(KvhSensor&&) = delete;
    /// @brief Copy assignment operator
    KvhSensor& operator=(const KvhSensor&) = delete;
    /// @brief Move assignment operator
    KvhSensor& operator=(KvhSensor&&) = delete;

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

  private:
    constexpr static size_t OutputPortIndex_KvhObs = 1; ///< @brief Flow (KvhObs)

    /// @brief Initialize the node
    bool initialize() override;

    /// @brief Deinitialize the node
    void deinitialize() override;

    /// @brief Callback handler for notifications of new asynchronous data packets received
    /// @param[in, out] userData Pointer to the data we supplied when we called registerAsyncPacketReceivedHandler
    /// @param[in] p Encapsulation of the data packet. At this state, it has already been validated and identified as an asynchronous data message
    /// @param[in] index Advanced usage item and can be safely ignored for now
    static void asciiOrBinaryAsyncMessageReceived(void* userData, uart::protocol::Packet& p, size_t index);

    /// Sensor Object
    sensors::kvh::KvhUartSensor sensor;

    /// Previous Sequence number to check for order errors
    uint8_t prevSequenceNumber = UINT8_MAX;
};

} // namespace NAV