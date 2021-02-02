#include "ARMA.hpp"

#include "util/Logger.hpp"

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"

NAV::ARMA::ARMA()
{
    name = typeStatic();

    LOG_TRACE("{}: called", name);

    color = ImColor(255, 128, 128);
    hasConfig = true;

    nm::CreateInputPin(this, "ImuObs", Pin::Type::Flow, { NAV::ImuObs::type() }, &ARMA::receiveImuObs);

    nm::CreateOutputPin(this, "ImuObs", Pin::Type::Flow, { NAV::ImuObs::type() });
}

NAV::ARMA::~ARMA()
{
    LOG_TRACE("{}: called", nameId());
}

std::string NAV::ARMA::typeStatic()
{
    return "ARMA";
}

std::string NAV::ARMA::type() const
{
    return typeStatic();
}

std::string NAV::ARMA::category()
{
    return "DataProcessor";
}

void NAV::ARMA::guiConfig()
{
    if (ImGui::Combo("combo 2 (one-liner)", &orderSelection, "aaaa\0bbbb\0cccc\0dddd\0eeee\0\0"))
    {
        flow::ApplyChanges();
    }
}

[[nodiscard]] json NAV::ARMA::save() const
{
    LOG_TRACE("{}: called", nameId());

    json j;

    // j["outputFrequency"] = outputFrequency;

    return j;
}

void NAV::ARMA::restore(json const& j)
{
    LOG_TRACE("{}: called", nameId());

    if (j.contains("outputFrequency"))
    {
        // j.at("outputFrequency").get_to(outputFrequency);
    }
}

bool NAV::ARMA::initialize()
{
    LOG_TRACE("{}: called", nameId());

    return true;
}

void NAV::ARMA::deinitialize()
{
    LOG_TRACE("{}: called", nameId());
}

void NAV::ARMA::receiveImuObs(const std::shared_ptr<NodeData>& nodeData, ax::NodeEditor::LinkId /*linkId*/)
{
    auto obs = std::static_pointer_cast<ImuObs>(nodeData);

    // buffer.push_back(obs);

    // for (auto& obs : buffer)
    // {
    // }
    // buffer.pop_back();

    LOG_TRACE("{}: called {}", nameId(), obs->insTime->GetStringOfDate());

    auto newImuObs = std::make_shared<ImuObs>(obs->imuPos);

    newImuObs->insTime = obs->insTime;
    newImuObs->accelUncompXYZ = obs->accelUncompXYZ;

    invokeCallbacks(OutputPortIndex_ImuObs, newImuObs);
}