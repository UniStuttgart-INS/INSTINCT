/// @file ImuIntegrator.hpp
/// @brief Integrates ImuObs Data
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2020-05-18

#pragma once

#include "Nodes/Node.hpp"

#include "NodeData/IMU/ImuObs.hpp"
#include "NodeData/State/StateData.hpp"

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

    /// @brief Initialize the node
    bool initialize() override;

    /// @brief Deinitialize the node
    void deinitialize() override;

  private:
    constexpr static size_t OutputPortIndex_ImuIntegrator = 0; ///< @brief Delegate
    constexpr static size_t OutputPortIndex_StateData = 1;     ///< @brief Flow (StateData)
    constexpr static size_t InputPortIndex_ImuObs = 0;         ///< @brief Flow (ImuObs)
    constexpr static size_t InputPortIndex_StateData = 1;      ///< @brief Object (StateData)

  private:
    /// @brief Integrates the Imu Observation data
    /// @param[in] nodeData ImuObs to process
    /// @param[in] linkId Id of the link over which the data is received
    void integrateObservation(std::shared_ptr<NodeData> nodeData, ax::NodeEditor::LinkId linkId);

    /// IMU Observation at the time tₖ₋₁
    std::shared_ptr<ImuObs> imuObs__t1 = nullptr;
    /// IMU Observation at the time tₖ₋₂
    std::shared_ptr<ImuObs> imuObs__t2 = nullptr;

    /// State Data at the time tₖ₋₂
    std::shared_ptr<StateData> stateData__t2 = nullptr;

    /// State Data at initialization
    std::shared_ptr<StateData> stateData__init = nullptr;

    enum IntegrationFrame : int
    {
        ECEF,
        NED
    };
    IntegrationFrame integrationFrame = IntegrationFrame::ECEF;
};

} // namespace NAV
