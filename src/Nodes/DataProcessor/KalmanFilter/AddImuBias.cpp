#include "AddImuBias.hpp"

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"

#include "NodeData/IMU/ImuObs.hpp"

NAV::AddImuBias::AddImuBias()
{
    LOG_TRACE("{}: called", name);

    hasConfig = false;
    kind = Kind::Simple;

    nm::CreateInputPin(this, "ImuObs", Pin::Type::Flow, { ImuObs::type() }, &AddImuBias::recvImuObs);
    nm::CreateInputPin(this, "ImuBiases", Pin::Type::Flow, { ImuBiases::type() }, &AddImuBias::recvImuBiases);
    nm::CreateOutputPin(this, "ImuObs", Pin::Type::Flow, ImuObs::type());
}

NAV::AddImuBias::~AddImuBias()
{
    LOG_TRACE("{}: called", nameId());
}

std::string NAV::AddImuBias::typeStatic()
{
    return "AddImuBias";
}

std::string NAV::AddImuBias::type() const
{
    return typeStatic();
}

std::string NAV::AddImuBias::category()
{
    return "Data Processor";
}

void NAV::AddImuBias::recvImuObs(const std::shared_ptr<NodeData>& nodeData, ax::NodeEditor::LinkId /* linkId */)
{
    if (imuBiases)
    {
        auto imuObs = std::dynamic_pointer_cast<ImuObs>(nodeData);
        if (imuObs->accelUncompXYZ.has_value())
        {
            imuObs->accelUncompXYZ.value() += imuBiases->biasAccel;
        }
        if (imuObs->accelCompXYZ.has_value())
        {
            imuObs->accelCompXYZ.value() += imuBiases->biasAccel;
        }

        if (imuObs->gyroUncompXYZ.has_value())
        {
            imuObs->gyroUncompXYZ.value() += imuBiases->biasGyro;
        }
        if (imuObs->gyroCompXYZ.has_value())
        {
            imuObs->gyroCompXYZ.value() += imuBiases->biasGyro;
        }
    }

    invokeCallbacks(OutputPortIndex_ImuObs, nodeData);
}

void NAV::AddImuBias::recvImuBiases(const std::shared_ptr<NodeData>& nodeData, ax::NodeEditor::LinkId /* linkId */)
{
    imuBiases = std::dynamic_pointer_cast<ImuBiases>(nodeData);
}