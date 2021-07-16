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

  private:
    constexpr static size_t OutputPortIndex_PosVelAtt__t0 = 1; ///< @brief Flow (PosVelAtt)

    /// @brief Initialize the node
    bool initialize() override;

    /// @brief Deinitialize the node
    void deinitialize() override;

    /// @brief Receive Function for the ImuObs at the time tₖ
    /// @param[in] nodeData ImuObs to process
    /// @param[in] linkId Id of the link over which the data is received
    void recvImuObs__t0(const std::shared_ptr<NodeData>& nodeData, ax::NodeEditor::LinkId linkId);

    /// @brief Receive Function for the ImuObs at the time tₖ₋₁
    /// @param[in] nodeData ImuObs to process
    /// @param[in] linkId Id of the link over which the data is received
    void recvImuObs__t1(const std::shared_ptr<NodeData>& nodeData, ax::NodeEditor::LinkId linkId);

    /// @brief Receive Function for the ImuObs at the time tₖ₋₂
    /// @param[in] nodeData ImuObs to process
    /// @param[in] linkId Id of the link over which the data is received
    void recvImuObs__t2(const std::shared_ptr<NodeData>& nodeData, ax::NodeEditor::LinkId linkId);

    /// @brief Receive Function for the PosVelAtt at the time tₖ₋₁
    /// @param[in] nodeData PosVelAtt to process
    /// @param[in] linkId Id of the link over which the data is received
    void recvState__t1(const std::shared_ptr<NodeData>& nodeData, ax::NodeEditor::LinkId linkId);

    /// @brief Receive Function for the PosVelAtt at the time tₖ₋₂
    /// @param[in] nodeData PosVelAtt to process
    /// @param[in] linkId Id of the link over which the data is received
    void recvState__t2(const std::shared_ptr<NodeData>& nodeData, ax::NodeEditor::LinkId linkId);

    /// @brief Integrates the Imu Observation data
    void integrateObservation();

    /// IMU Observation at the time tₖ
    std::shared_ptr<ImuObs> imuObs__t0 = nullptr;
    /// IMU Observation at the time tₖ₋₁
    std::shared_ptr<ImuObs> imuObs__t1 = nullptr;
    /// IMU Observation at the time tₖ₋₂
    std::shared_ptr<ImuObs> imuObs__t2 = nullptr;

    /// Position, Velocity and Attitude at the time tₖ₋₁
    std::shared_ptr<PosVelAtt> posVelAtt__t1 = nullptr;
    /// Position, Velocity and Attitude at the time tₖ₋₂
    std::shared_ptr<PosVelAtt> posVelAtt__t2 = nullptr;

    /// Position, Velocity and Attitude at initialization (needed to transform the ECEF position into NED)
    std::shared_ptr<PosVelAtt> posVelAtt__init = nullptr;

    /// Time at initialization (needed to set time tag when TimeSinceStartup is used)
    InsTime time__init;
    /// TimeSinceStartup at initialization (needed to set time tag when TimeSinceStartup is used)
    uint64_t timeSinceStartup__init = 0;

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
        Somigliana,
        EGM96
    };
    GravityModel gravityModel = GravityModel::WGS84;

    /// Flag, whether the integrator should take the time from the IMU clock instead of the insTime
    bool prefereTimeSinceStartupOverInsTime = false;
};

} // namespace NAV
