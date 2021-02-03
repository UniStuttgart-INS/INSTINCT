#include "PosVelAttInitializer.hpp"

#include "util/Logger.hpp"

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"

NAV::PosVelAttInitializer::PosVelAttInitializer()
{
    name = typeStatic();

    LOG_TRACE("{}: called", name);

    color = ImColor(255, 128, 128);
    hasConfig = true;

    nm::CreateInputPin(this, "ImuObs", Pin::Type::Flow, { NAV::ImuObs::type() }, &PosVelAttInitializer::receiveImuObs);
    nm::CreateInputPin(this, "GnssObs", Pin::Type::Flow, { NAV::UbloxObs::type(), NAV::RtklibPosObs::type() }, &PosVelAttInitializer::receiveGnssObs);
    nm::CreateInputPin(this, "Position ECEF", Pin::Type::Matrix, { "Eigen::MatrixXd", "BlockMatrix" });
    nm::CreateInputPin(this, "Velocity NED", Pin::Type::Matrix, { "Eigen::MatrixXd", "BlockMatrix" });
    nm::CreateInputPin(this, "Quaternion nb", Pin::Type::Matrix, { "Eigen::MatrixXd", "BlockMatrix" });
}

NAV::PosVelAttInitializer::~PosVelAttInitializer()
{
    LOG_TRACE("{}: called", nameId());
}

std::string NAV::PosVelAttInitializer::typeStatic()
{
    return "PosVelAttInitializer";
}

std::string NAV::PosVelAttInitializer::type() const
{
    return typeStatic();
}

std::string NAV::PosVelAttInitializer::category()
{
    return "State";
}

void NAV::PosVelAttInitializer::guiConfig()
{
    if (ImGui::InputDouble("Initialization Duration", &initDuration, 0.0, 0.0, "%.3f s"))
    {
        flow::ApplyChanges();
    }
}

[[nodiscard]] json NAV::PosVelAttInitializer::save() const
{
    LOG_TRACE("{}: called", nameId());

    json j;

    j["initDuration"] = initDuration;

    return j;
}

void NAV::PosVelAttInitializer::restore(json const& j)
{
    LOG_TRACE("{}: called", nameId());

    if (j.contains("initDuration"))
    {
        j.at("initDuration").get_to(initDuration);
    }
}

bool NAV::PosVelAttInitializer::initialize()
{
    LOG_TRACE("{}: called", nameId());

    countAveragedPosition = 0.0;
    countAveragedVelocity = 0.0;
    countAveragedAttitude = 0.0;

    return true;
}

void NAV::PosVelAttInitializer::deinitialize()
{
    LOG_TRACE("{}: called", nameId());
}

void NAV::PosVelAttInitializer::receiveImuObs(const std::shared_ptr<NodeData>& nodeData, ax::NodeEditor::LinkId /*linkId*/)
{
    auto obs = std::static_pointer_cast<ImuObs>(nodeData);

    if (!obs->insTime.has_value())
    {
        LOG_ERROR("{}: Can only process data with an insTime", nameId());
        return;
    }

    if (startTime.empty())
    {
        startTime = obs->insTime.value();
    }
}

void NAV::PosVelAttInitializer::receiveGnssObs(const std::shared_ptr<NodeData>& nodeData, ax::NodeEditor::LinkId linkId)
{
    if (Link* link = nm::FindLink(linkId))
    {
        if (Pin* sourcePin = nm::FindPin(link->startPinId))
        {
            if (sourcePin->dataIdentifier.front() == RtklibPosObs::type())
            {
                receiveRtklibPosObs(std::static_pointer_cast<RtklibPosObs>(nodeData));
            }
            else if (sourcePin->dataIdentifier.front() == UbloxObs::type())
            {
                receiveUbloxObs(std::static_pointer_cast<UbloxObs>(nodeData));
            }
        }
    }
}

void NAV::PosVelAttInitializer::receiveUbloxObs(const std::shared_ptr<UbloxObs>& obs)
{
    if (!obs->insTime.has_value())
    {
        LOG_ERROR("{}: Can only process data with an insTime", nameId());
        return;
    }

    if (startTime.empty())
    {
        startTime = obs->insTime.value();
    }
}

void NAV::PosVelAttInitializer::receiveRtklibPosObs(const std::shared_ptr<RtklibPosObs>& obs)
{
    if (!obs->insTime.has_value())
    {
        LOG_ERROR("{}: Can only process data with an insTime", nameId());
        return;
    }

    if (startTime.empty())
    {
        startTime = obs->insTime.value();
    }
}

// LOG_INFO("{}: State initialized to Lat {:3.4f} [°], Lon {:3.4f} [°], Alt {:4.4f} [m]", name,
//                  trafo::rad2deg(initialState.latitude()),
//                  trafo::rad2deg(initialState.longitude()),
//                  initialState.altitude());
// LOG_INFO("{}: State initialized to v_N {:3.5f} [m/s], v_E {:3.5f}, v_D {:3.5f}", name,
//             initialState.velocity_n().x(),
//             initialState.velocity_n().x(),
//             initialState.velocity_n().z());
// LOG_INFO("{}: State initialized to Roll {:3.5f} [°], Pitch {:3.5f} [°], Yaw {:3.4f} [°]", name,
//             trafo::rad2deg(initialState.rollPitchYaw().x()),
//             trafo::rad2deg(initialState.rollPitchYaw().y()),
//             trafo::rad2deg(initialState.rollPitchYaw().z()));