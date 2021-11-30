/// @file ImuIntegrator.hpp
/// @brief Integrates ImuObs Data
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @author M. Maier (marcel.maier@ins.uni-stuttgart.de)
/// @date 2020-05-18

#pragma once

#include "internal/Node/Node.hpp"

#include "NodeData/IMU/ImuObs.hpp"
#include "NodeData/State/PosVelAtt.hpp"
#include "NodeData/State/PVAError.hpp"
#include "NodeData/State/ImuBiases.hpp"

#include "Navigation/Gravity/Gravity.hpp"

#include <deque>

namespace NAV
{
/// @brief Numerically integrates Imu data
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
    constexpr static size_t OutputPortIndex_InertialNavSol__t0 = 0; ///< @brief Flow (InertialNavSol)

    /// @brief Initialize the node
    bool initialize() override;

    /// @brief Deinitialize the node
    void deinitialize() override;

    /// @brief Receive Function for the ImuObs at the time tₖ
    /// @param[in] nodeData ImuObs to process
    /// @param[in] linkId Id of the link over which the data is received
    void recvImuObs__t0(const std::shared_ptr<const NodeData>& nodeData, ax::NodeEditor::LinkId linkId);

    /// @brief Receive Function for the PosVelAtt at the time tₖ₋₁
    /// @param[in] nodeData PosVelAtt to process
    /// @param[in] linkId Id of the link over which the data is received
    void recvState__t1(const std::shared_ptr<const NodeData>& nodeData, ax::NodeEditor::LinkId linkId);

    /// @brief Receive function for PVAError
    /// @param[in] nodeData PVAError received
    /// @param[in] linkId Id of the link over which the data is received
    void recvPVAError(const std::shared_ptr<const NodeData>& nodeData, ax::NodeEditor::LinkId linkId);

    /// @brief Receive function for ImuBiases
    /// @param[in] nodeData Observation received
    /// @param[in] linkId Id of the link over which the data is received
    void recvImuBiases(const std::shared_ptr<const NodeData>& nodeData, ax::NodeEditor::LinkId linkId);

    /// @brief Integrates the Imu Observation data
    void integrateObservation();

    /// Accumulated IMU biases
    std::shared_ptr<const ImuBiases> imuBiases = nullptr;

    /// IMU Observation list
    /// Length depends on the integration algorithm. Newest observation first (tₖ, tₖ₋₁, tₖ₋₂, ...)
    std::deque<std::shared_ptr<const ImuObs>> imuObservations;

    /// @brief Maximum amount of imu observations to keep
    size_t maxSizeImuObservations = 0;

    /// Position, Velocity and Attitude states.
    /// Length depends on the integration algorithm. Newest state first (tₖ, tₖ₋₁, tₖ₋₂, ...)
    std::deque<std::shared_ptr<const PosVelAtt>> posVelAttStates;

    /// @brief Maximum amount of states to keep
    size_t maxSizeStates = 0;

    /// Time at initialization (needed to set time tag when TimeSinceStartup is used)
    InsTime time__init;
    /// TimeSinceStartup at initialization (needed to set time tag when TimeSinceStartup is used)
    uint64_t timeSinceStartup__init = 0;

    /// @brief Available Integration Frames
    enum class IntegrationFrame : int
    {
        ECEF, ///< Earth-Centered Earth-Fixed frame
        NED,  ///< Local North-East-Down frame
    };
    /// Frame to integrate the observations in
    IntegrationFrame integrationFrame = IntegrationFrame::NED;

    /// @brief Gravity model selected in the GUI
    GravityModel gravityModel = GravityModel::EGM96;

    /// Integration Algorithm selection
    enum class IntegrationAlgorithm : size_t
    {
        RectangularRule, ///< Rectangular rule
        Simpson,         ///< Simpson
        RungeKutta1,     ///< Runge-Kutta 1st order
        RungeKutta3,     ///< Runge-Kutta 3rd order
        COUNT,           ///< Amount of available integration algorithms
    };
    /// @brief Integration algorithm used for the attitude update
    IntegrationAlgorithm integrationAlgorithmAttitude = IntegrationAlgorithm::RungeKutta3;
    /// @brief Integration algorithm used for the velocity update
    IntegrationAlgorithm integrationAlgorithmVelocity = IntegrationAlgorithm::Simpson;
    /// @brief Integration algorithm used for the position update
    IntegrationAlgorithm integrationAlgorithmPosition = IntegrationAlgorithm::RectangularRule;

    /// @brief Converts the enum to a string
    /// @param[in] algorithm Enum value to convert into text
    /// @return String representation of the enum
    static const char* to_string(IntegrationAlgorithm algorithm);

    /// GUI flag, whether to show the input pin for PVA Corrections
    bool showCorrectionsInputPin = false;

    /// Pointer to the most recent PVA error
    std::shared_ptr<const PVAError> pvaError = nullptr;

    /// Runge Kutta uses intermediate observations but propagates only every other state. Because of this 2 separate state solutions can coexist.
    /// To avoid this only every seconds state can be output resulting in halving the output frequency. The accuracy of the results is not affected by this.
    bool calculateIntermediateValues = true;

    /// Flag to skip every second calculation
    bool skipIntermediateCalculation = false;

    /// Flag, whether the integrator should take the time from the IMU clock instead of the insTime
    bool prefereTimeSinceStartupOverInsTime = false;

    /// Flag to toggle centrifugal acceleration compensation of the acceleration at the current position
    bool centrifugalAccCompensation = true;

    /// Flag to toggle coriolis acceleration compensation of the acceleration at the current position
    bool coriolisCompensation = true;

    /// Flag to let the integration algorithm use uncompensated acceleration and angular rates instead of compensated
    bool prefereUncompensatedData = false;
};

} // namespace NAV
