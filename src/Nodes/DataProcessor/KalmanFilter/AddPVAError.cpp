#include "AddPVAError.hpp"

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"

#include "util/InsTransformations.hpp"
#include "NodeData/State/PosVelAtt.hpp"

NAV::AddPVAError::AddPVAError()
{
    LOG_TRACE("{}: called", name);

    hasConfig = false;
    kind = Kind::Simple;

    nm::CreateInputPin(this, "PosVelAtt", Pin::Type::Flow, { PosVelAtt::type() }, &AddPVAError::recvPosVelAtt);
    nm::CreateInputPin(this, "PVAError", Pin::Type::Flow, { PVAError::type() }, &AddPVAError::recvPVAError);
    nm::CreateOutputPin(this, "PosVelAtt", Pin::Type::Flow, PosVelAtt::type());
}

NAV::AddPVAError::~AddPVAError()
{
    LOG_TRACE("{}: called", nameId());
}

std::string NAV::AddPVAError::typeStatic()
{
    return "AddPVAError";
}

std::string NAV::AddPVAError::type() const
{
    return typeStatic();
}

std::string NAV::AddPVAError::category()
{
    return "Data Processor";
}

bool NAV::AddPVAError::initialize()
{
    LOG_TRACE("{}: called", nameId());

    pvaError.reset();

    return true;
}

void NAV::AddPVAError::deinitialize()
{
    LOG_TRACE("{}: called", nameId());
}

void NAV::AddPVAError::recvPosVelAtt(const std::shared_ptr<NodeData>& nodeData, ax::NodeEditor::LinkId /* linkId */)
{
    if (pvaError)
    {
        auto posVelAtt = std::dynamic_pointer_cast<PosVelAtt>(nodeData);

        posVelAtt->position_ecef() = trafo::lla2ecef_WGS84(posVelAtt->latLonAlt() + pvaError->positionError_lla());

        posVelAtt->velocity_n() += pvaError->velocityError_n();

        auto rollPitchYaw_corrected = posVelAtt->rollPitchYaw() + pvaError->attitudeError_n();
        posVelAtt->quaternion_nb() = trafo::quat_nb(rollPitchYaw_corrected(0), rollPitchYaw_corrected(1), rollPitchYaw_corrected(2));

        // quat_bn is used because the error is also substracted which in quaternions is the conjugated quaternion // TODO check if quaternions possible
        // auto q_nb_error = trafo::quat_nb(pvaError->attitudeError_n()(0), pvaError->attitudeError_n()(1), pvaError->attitudeError_n()(2));
        // posVelAtt->quaternion_nb() = posVelAtt->quaternion_nb() * q_nb_error;

        pvaError.reset();
    }

    invokeCallbacks(OutputPortIndex_PosVelAtt, nodeData);
}

void NAV::AddPVAError::recvPVAError(const std::shared_ptr<NodeData>& nodeData, ax::NodeEditor::LinkId /* linkId */)
{
    pvaError = std::dynamic_pointer_cast<PVAError>(nodeData);
}