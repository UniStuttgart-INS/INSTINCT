#include "ImuError.hpp"

#include "NodeData/IMU/ImuObs.hpp"

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"

#include <Eigen/Core>

NAV::ImuError::ImuError()
{
    name = typeStatic();

    LOG_TRACE("{}: called", name);
    hasConfig = true;
    guiConfigDefaultWindowSize = { 350, 123 };

    nm::CreateOutputPin(this, "ImuObs", Pin::Type::Flow, { NAV::ImuObs::type() });

    nm::CreateInputPin(this, "ImuObs", Pin::Type::Flow, { NAV::ImuObs::type() }, &ImuError::receiveObs);
}

NAV::ImuError::~ImuError()
{
    LOG_TRACE("{}: called", nameId());
}

std::string NAV::ImuError::typeStatic()
{
    return "ImuError";
}

std::string NAV::ImuError::type() const
{
    return typeStatic();
}

std::string NAV::ImuError::category()
{
    return "DataProcessor";
}

void NAV::ImuError::guiConfig()
{
}

[[nodiscard]] json NAV::ImuError::save() const
{
    LOG_TRACE("{}: called", nameId());

    json j;

    // j["outputType"] = outputType;

    return j;
}

void NAV::ImuError::restore(json const& j)
{
    LOG_TRACE("{}: called", nameId());

    if (j.contains("posVelSource"))
    {
        // j.at("posVelSource").get_to(posVelSource);
    }
}

bool NAV::ImuError::initialize()
{
    LOG_TRACE("{}: called", nameId());

    return true;
}

void NAV::ImuError::receiveObs(const std::shared_ptr<const NodeData>& nodeData, ax::NodeEditor::LinkId /*linkId*/)
{
    auto imuObs = std::make_shared<ImuObs>(*std::static_pointer_cast<const ImuObs>(nodeData));

    imuObs->accelUncompXYZ.value() += Eigen::Vector3d{ 1, 2, 3 };
    imuObs->gyroUncompXYZ.value() += Eigen::Vector3d{ 1, 2, 3 };

    invokeCallbacks(OutputPortIndex_ImuObs, imuObs);
}