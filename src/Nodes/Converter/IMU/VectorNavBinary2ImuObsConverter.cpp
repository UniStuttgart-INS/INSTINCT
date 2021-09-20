#include "VectorNavBinary2ImuObsConverter.hpp"

#include "util/Logger.hpp"

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"

#include "NodeData/IMU/VectorNavBinaryOutput.hpp"
#include "NodeData/IMU/ImuObsWDelta.hpp"

NAV::VectorNavBinary2ImuObsConverter::VectorNavBinary2ImuObsConverter()
{
    name = typeStatic();

    LOG_TRACE("{}: called", name);
    hasConfig = false;

    nm::CreateOutputPin(this, "ImuObs", Pin::Type::Flow, { NAV::ImuObsWDelta::type() });

    nm::CreateInputPin(this, "VectorNavBinaryOutput", Pin::Type::Flow, { NAV::VectorNavBinaryOutput::type() }, &VectorNavBinary2ImuObsConverter::convertObs);
}

NAV::VectorNavBinary2ImuObsConverter::~VectorNavBinary2ImuObsConverter()
{
    LOG_TRACE("{}: called", nameId());
}

std::string NAV::VectorNavBinary2ImuObsConverter::typeStatic()
{
    return "VectorNavBinary2ImuObsConverter";
}

std::string NAV::VectorNavBinary2ImuObsConverter::type() const
{
    return typeStatic();
}

std::string NAV::VectorNavBinary2ImuObsConverter::category()
{
    return "Converter";
}

void NAV::VectorNavBinary2ImuObsConverter::convertObs(const std::shared_ptr<const NodeData>& nodeData, ax::NodeEditor::LinkId /*linkId*/)
{
    auto vnObs = std::dynamic_pointer_cast<const VectorNavBinaryOutput>(nodeData);

    auto imuObs = std::make_shared<ImuObsWDelta>(vnObs->imuPos);

    imuObs->insTime = vnObs->insTime;

    if (vnObs->timeOutputs)
    {
        if (vnObs->timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_TIMESTARTUP)
        {
            imuObs->timeSinceStartup = vnObs->timeOutputs->timeStartup;
        }
    }
    if (vnObs->imuOutputs)
    {
        if (vnObs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_UNCOMPMAG)
        {
            imuObs->magUncompXYZ = vnObs->imuOutputs->uncompMag.cast<double>();
        }
        if (vnObs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_UNCOMPACCEL)
        {
            imuObs->accelUncompXYZ = vnObs->imuOutputs->uncompAccel.cast<double>();
        }
        if (vnObs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_UNCOMPGYRO)
        {
            imuObs->gyroUncompXYZ = vnObs->imuOutputs->uncompGyro.cast<double>();
        }
        if (vnObs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_MAG)
        {
            imuObs->magCompXYZ = vnObs->imuOutputs->mag.cast<double>();
        }
        if (vnObs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_ACCEL)
        {
            imuObs->accelCompXYZ = vnObs->imuOutputs->accel.cast<double>();
        }
        if (vnObs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_ANGULARRATE)
        {
            imuObs->gyroCompXYZ = vnObs->imuOutputs->angularRate.cast<double>();
        }
        if (vnObs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_TEMP)
        {
            imuObs->temperature = vnObs->imuOutputs->temp;
        }
        if (vnObs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_DELTATHETA)
        {
            imuObs->dtime = vnObs->imuOutputs->deltaTime;
            imuObs->dtheta = vnObs->imuOutputs->deltaTheta.cast<double>();
        }
        if (vnObs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_DELTAVEL)
        {
            imuObs->dvel = vnObs->imuOutputs->deltaV.cast<double>();
        }
    }

    if (imuObs->magUncompXYZ.has_value() || imuObs->accelUncompXYZ.has_value() || imuObs->gyroUncompXYZ.has_value()
        || imuObs->magCompXYZ.has_value() || imuObs->accelCompXYZ.has_value() || imuObs->gyroCompXYZ.has_value()
        || (!std::isnan(imuObs->dtime) && (imuObs->dtheta.has_value() || imuObs->dvel.has_value())))
    {
        invokeCallbacks(OutputPortIndex_ImuObsWDelta, imuObs);
    }
}
