/// @file VectorNavBinaryOutput.hpp
/// @brief Binary Outputs from VectorNav Sensors
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2021-07-01

#pragma once

#include <memory>

#include "NodeData/InsObs.hpp"
#include "NodeData/IMU/ImuPos.hpp"
#include "util/Eigen.hpp"

#include "util/Vendor/VectorNav/BinaryOutputs/TimeOutputs.hpp"
#include "util/Vendor/VectorNav/BinaryOutputs/ImuOutputs.hpp"
#include "util/Vendor/VectorNav/BinaryOutputs/GnssOutputs.hpp"
#include "util/Vendor/VectorNav/BinaryOutputs/AttitudeOutputs.hpp"
#include "util/Vendor/VectorNav/BinaryOutputs/InsOutputs.hpp"

namespace NAV
{
/// IMU Observation storage class
class VectorNavBinaryOutput : public InsObs
{
  public:
    /// @brief Constructor
    /// @param[in] imuPos Reference to the position and rotation info of the Imu
    explicit VectorNavBinaryOutput(const ImuPos& imuPos)
        : imuPos(imuPos) {}

    /// @brief Returns the type of the data class
    /// @return The data type
    [[nodiscard]] static std::string type()
    {
        return "VectorNavBinaryOutput";
    }

    /// @brief Returns the parent types of the data class
    /// @return The parent data types
    [[nodiscard]] static std::vector<std::string> parentTypes()
    {
        return { InsObs::type() };
    }

    /// @brief Binary Group 2 – Time Outputs
    std::shared_ptr<vendor::vectornav::TimeOutputs> timeOutputs = nullptr;

    /// @brief Binary Group 3 – IMU Outputs
    std::shared_ptr<vendor::vectornav::ImuOutputs> imuOutputs = nullptr;

    /// @brief Binary Group 4 – GNSS1 Outputs
    std::shared_ptr<vendor::vectornav::GnssOutputs> gnss1Outputs = nullptr;

    /// @brief Binary Group 5 – Attitude Outputs
    std::shared_ptr<vendor::vectornav::AttitudeOutputs> attitudeOutputs = nullptr;

    /// @brief Binary Group 6 – INS Outputs
    std::shared_ptr<vendor::vectornav::InsOutputs> insOutputs = nullptr;

    /// @brief Binary Group 7 – GNSS2 Outputs
    std::shared_ptr<vendor::vectornav::GnssOutputs> gnss2Outputs = nullptr;

    /// Position and rotation information for conversion from platform to body frame
    const ImuPos& imuPos;
};

} // namespace NAV
