/// @file ImuIntegrator.hpp
/// @brief Integrates ImuObs Data
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2020-05-18

#pragma once

#include "Nodes/Node.hpp"

#include "NodeData/IMU/ImuObs.hpp"
#include "NodeData/State/PosVelAtt.hpp"

namespace NAV
{
class ImuIntegrator : public Node
{
  public:
    /// @brief Default constructor
    ImuIntegrator();
    /// @brief Destructor
    ~ImuIntegrator() override;
    /// @brief Copy constructor
    ImuIntegrator(const ImuIntegrator&) = delete;
    /// @brief Move constructor
    ImuIntegrator(ImuIntegrator&&) = delete;
    /// @brief Copy assignment operator
    ImuIntegrator& operator=(const ImuIntegrator&) = delete;
    /// @brief Move assignment operator
    ImuIntegrator& operator=(ImuIntegrator&&) = delete;

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

    /// @brief Called when a new link is to be established
    /// @param[in] startPin Pin where the link starts
    /// @param[in] endPin Pin where the link ends
    /// @return True if link is allowed, false if link is rejected
    bool onCreateLink(Pin* startPin, Pin* endPin) override;

  private:
    constexpr static size_t InputPortIndex_ImuObs = 0;     ///< @brief Flow (ImuObs)
    constexpr static size_t InputPortIndex_Position = 1;   ///< @brief Matrix
    constexpr static size_t InputPortIndex_Velocity = 2;   ///< @brief Matrix
    constexpr static size_t InputPortIndex_Quaternion = 3; ///< @brief Matrix

    /// @brief Initialize the node
    bool initialize() override;

    /// @brief Deinitialize the node
    void deinitialize() override;

    /// @brief Get the current Position
    /// @param[out] position The Vector to return the result in
    /// @return True, if the position was successfully returned, otherwise false (Pin not connected, ...)
    bool getCurrentPosition(Eigen::Vector3d& position);

    /// @brief Set the current Position on the input pin
    /// @param[in] position The position to set
    void setCurrentPosition(const Eigen::Vector3d& position);

    /// @brief Get the current Velocity
    /// @param[out] velocity The Vector to return the result in
    /// @return True, if the velocity was successfully returned, otherwise false (Pin not connected, ...)
    bool getCurrentVelocity(Eigen::Vector3d& velocity);

    /// @brief Set the current Velocity on the input pin
    /// @param[in] velocity The velocity to set
    void setCurrentVelocity(const Eigen::Vector3d& velocity);

    /// @brief Get the current Attitude Quaternion
    /// @param[out] quaternion_nb The Quaternion to return the result in
    /// @return True, if the attitude was successfully returned, otherwise false (Pin not connected, ...)
    bool getCurrentQuaternion_nb(Eigen::Quaterniond& quaternion_nb);

    /// @brief Set the current Attitude Quaternion on the input pin
    /// @param[in] quaternion_nb The Attitude Quaternion to set
    void setCurrentQuaternion_nb(const Eigen::Quaterniond& quaternion_nb);

    /// @brief Integrates the Imu Observation data
    /// @param[in] nodeData ImuObs to process
    /// @param[in] linkId Id of the link over which the data is received
    void integrateObservation(const std::shared_ptr<NodeData>& nodeData, ax::NodeEditor::LinkId linkId);

    /// IMU Observation at the time tₖ₋₁
    std::shared_ptr<ImuObs> imuObs__t1 = nullptr;
    /// IMU Observation at the time tₖ₋₂
    std::shared_ptr<ImuObs> imuObs__t2 = nullptr;

    /// Position, Velocity and Attitude at the time tₖ₋₂
    std::shared_ptr<PosVelAtt> posVelAtt__t2 = nullptr;

    /// Position, Velocity and Attitude at initialization
    std::shared_ptr<PosVelAtt> posVelAtt__init = nullptr;

    enum IntegrationFrame : int
    {
        ECEF,
        NED
    };
    /// Frame to integrate the observations in
    IntegrationFrame integrationFrame = IntegrationFrame::ECEF;

    /// Gravity Model selection
    enum GravityModel : int
    {
        WGS84,
        WGS84_Skydel,
        Somigliana
    };
    GravityModel gravityModel = GravityModel::WGS84;

    /// g_n Gravity vector in [m/s^2], in navigation coordinates
    const Eigen::Vector3d gravity_n__t1;
};

} // namespace NAV
