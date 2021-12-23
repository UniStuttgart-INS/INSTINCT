/// @file ErrorModel.hpp
/// @brief Adds errors (biases and noise) to measurements
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2021-12-21

#pragma once

#include "internal/Node/Node.hpp"

#include "NodeData/IMU/ImuObs.hpp"

#include <Eigen/Core>

namespace NAV
{
/// Adds errors (biases and noise) to measurements
class ErrorModel : public Node
{
  public:
    /// @brief Default constructor
    ErrorModel();
    /// @brief Destructor
    ~ErrorModel() override;
    /// @brief Copy constructor
    ErrorModel(const ErrorModel&) = delete;
    /// @brief Move constructor
    ErrorModel(ErrorModel&&) = delete;
    /// @brief Copy assignment operator
    ErrorModel& operator=(const ErrorModel&) = delete;
    /// @brief Move assignment operator
    ErrorModel& operator=(ErrorModel&&) = delete;

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
    constexpr static size_t OutputPortIndex_ImuObs = 0; ///< @brief Flow (ImuObs)
    constexpr static size_t InputPortIndex_ImuObs = 0;  ///< @brief Flow (ImuObs)

    /// @brief Initialize the node
    bool initialize() override;

    /// @brief Callback when receiving data on a port
    /// @param[in] nodeData ImuObs to process
    /// @param[in] linkId Id of the link over which the data is received
    void receiveObs(const std::shared_ptr<const NodeData>& nodeData, ax::NodeEditor::LinkId linkId);

    // #########################################################################################################################################
    //                                                                 ImuObs
    // #########################################################################################################################################

    /// Possible units to specify an accelerometer bias with
    enum class ImuAccelerometerBiasUnits
    {
        m_s2, ///< [m/s^2]
    };

    /// Selected unit for the accelerometer bias in the GUI
    ImuAccelerometerBiasUnits imuAccelerometerBiasUnit = ImuAccelerometerBiasUnits::m_s2;

    /// Bias of the accelerometer in platform coordinates [m/s^2]
    Eigen::Vector3d imuAccelerometerBias_p = Eigen::Vector3d::Zero();

    /// Possible units to specify an gyroscope bias with
    enum class ImuGyroscopeBiasUnits
    {
        rad_s, ///< [rad/s]
        deg_s, ///< [deg/s]
    };

    /// Selected unit for the gyroscope bias in the GUI
    ImuGyroscopeBiasUnits imuGyroscopeBiasUnit = ImuGyroscopeBiasUnits::rad_s;

    /// Bias of the gyroscope in platform coordinates [rad/s]
    Eigen::Vector3d imuGyroscopeBias_p = Eigen::Vector3d::Zero();
};

} // namespace NAV
