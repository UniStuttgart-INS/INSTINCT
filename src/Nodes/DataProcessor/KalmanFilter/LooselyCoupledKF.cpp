#include "LooselyCoupledKF.hpp"

#include "util/Logger.hpp"
#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"

NAV::LooselyCoupledKF::LooselyCoupledKF()
{
    name = typeStatic();

    LOG_TRACE("{}: called", name);

    hasConfig = true;
    // guiConfigDefaultWindowSize = {};

    nm::CreateInputPin(this, "PosVelAtt (t0)", Pin::Type::Flow, { NAV::PosVelAtt::type() }, &LooselyCoupledKF::recvState__t0);

    nm::CreateOutputPin(this, "PosVelAtt", Pin::Type::Flow, NAV::PosVelAtt::type());
}

NAV::LooselyCoupledKF::~LooselyCoupledKF()
{
    LOG_TRACE("{}: called", nameId());
}

std::string NAV::LooselyCoupledKF::typeStatic()
{
    return "LooselyCoupledKF";
}

std::string NAV::LooselyCoupledKF::type() const
{
    return typeStatic();
}

std::string NAV::LooselyCoupledKF::category()
{
    return "Data Processor";
}

void NAV::LooselyCoupledKF::guiConfig()
{
    
}

[[nodiscard]] json NAV::LooselyCoupledKF::save() const
{
    LOG_TRACE("{}: called", nameId());

    // TODO: save, once there is something to save

    json j;

    j["LC KF somthing to save"] = "something to save";

    return j;
}

void NAV::LooselyCoupledKF::restore(json const& j)
{
    LOG_TRACE("{}: called", nameId());

    // TODO: restore, once there is something to restore

    LOG_DATA("restored j is {}", j);
}

bool NAV::LooselyCoupledKF::initialize()
{
    LOG_TRACE("{}: called", nameId());

    posVelAtt__t0 = nullptr;

    LOG_DEBUG("LooselyCoupledKF initialized");

    return true;
}

void NAV::LooselyCoupledKF::deinitialize()
{
    LOG_TRACE("{}: called", nameId());
}

void NAV::LooselyCoupledKF::recvState__t0(const std::shared_ptr<NodeData>& NodeData, ax::NodeEditor::LinkId /*linkId*/ ) // NOLINT(readability-convert-member-functions-to-static)
{
    // TODO: cast nodeData as a dynamic pointer to state observation
    LOG_DATA("NodeData received: {}",NodeData);
}

void NAV::LooselyCoupledKF::filterObservation()
{
    // TODO: Develop algorithms

    // TODO: Reset the data ports

    // Push out the new data
    invokeCallbacks(OutputPortIndex_PosVelAtt__t0, posVelAtt__t0);
}