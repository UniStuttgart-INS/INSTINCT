#include "TimeSynchronizer.hpp"

#include "util/Logger.hpp"

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"

NAV::TimeSynchronizer::TimeSynchronizer()
{
    name = typeStatic();

    LOG_TRACE("{}: called", name);

    color = ImColor(255, 128, 128);
    hasConfig = true;

    nm::CreateOutputPin(this, "", Pin::Type::Delegate, "TimeSynchronizer", this);

    nm::CreateOutputPin(this, "Obs", Pin::Type::Flow, NAV::ImuObs::type(), &TimeSynchronizer::pollData);

    nm::CreateInputPin(this, "Obs", Pin::Type::Flow, NAV::ImuObs::type(), &TimeSynchronizer::syncObs);
    nm::CreateInputPin(this, "Time", Pin::Type::Flow, NAV::InsObs::type(), &TimeSynchronizer::syncTime);
}

NAV::TimeSynchronizer::~TimeSynchronizer()
{
    LOG_TRACE("{}: called", nameId());
}

std::string NAV::TimeSynchronizer::typeStatic()
{
    return "TimeSynchronizer";
}

std::string NAV::TimeSynchronizer::type() const
{
    return typeStatic();
}

std::string NAV::TimeSynchronizer::category()
{
    return "Time";
}

void NAV::TimeSynchronizer::guiConfig()
{
    if (ImGui::Combo("Port Type", &selectedPortType, "VectorNavObs\0ImuObs\0KvhObs\0\0"))
    {
        if (selectedPortType == 0)
        {
            inputPins.at(InputPortIndex_ObsToSync).dataIdentifier = VectorNavObs::type();
        }
        else if (selectedPortType == 1)
        {
            inputPins.at(InputPortIndex_ObsToSync).dataIdentifier = ImuObs::type();
        }
        else if (selectedPortType == 2)
        {
            inputPins.at(InputPortIndex_ObsToSync).dataIdentifier = KvhObs::type();
        }
        outputPins.at(OutputPortIndex_ObsToSync).dataIdentifier = inputPins.at(InputPortIndex_ObsToSync).dataIdentifier;

        LOG_DEBUG("{}: Use Fixed Start Time changed to {}", nameId(), useFixedStartTime);
        flow::ApplyChanges();
    }
    if (ImGui::Checkbox("Use Fixed Start Time", &useFixedStartTime))
    {
        LOG_DEBUG("{}: Use Fixed Start Time changed to {}", nameId(), useFixedStartTime);
        flow::ApplyChanges();
        deinitialize();
    }

    if (useFixedStartTime)
    {
        if (ImGui::InputInt("Gps Cycle", &gpsCycle))
        {
            if (gpsCycle < 0)
            {
                gpsCycle = 0;
            }
            LOG_DEBUG("{}: Gps Cycle changed to {}", nameId(), gpsCycle);
            flow::ApplyChanges();
            deinitialize();
        }
        if (ImGui::InputInt("Gps Week", &gpsWeek))
        {
            if (gpsWeek < 0)
            {
                gpsWeek = 0;
            }
            LOG_DEBUG("{}: Gps Week changed to {}", nameId(), gpsWeek);
            flow::ApplyChanges();
            deinitialize();
        }

        if (ImGui::InputFloat("Gps ToW", &gpsToW, 1.0F, 3600.0F))
        {
            if (gpsToW < 0)
            {
                gpsToW = 0;
            }
            if (gpsToW > InsTimeUtil::SECONDS_PER_WEEK)
            {
                gpsToW = static_cast<float>(InsTimeUtil::SECONDS_PER_WEEK);
            }
            LOG_DEBUG("{}: Gps Week changed to {}", nameId(), gpsWeek);
            flow::ApplyChanges();
            deinitialize();
        }
    }
}

[[nodiscard]] json NAV::TimeSynchronizer::save() const
{
    LOG_TRACE("{}: called", nameId());

    json j;

    j["selectedPortType"] = selectedPortType;
    j["useFixedStartTime"] = useFixedStartTime;
    j["gpsCycle"] = gpsCycle;
    j["gpsWeek"] = gpsWeek;
    j["gpsToW"] = gpsToW;

    return j;
}

void NAV::TimeSynchronizer::restore(json const& j)
{
    LOG_TRACE("{}: called", nameId());

    if (j.contains("selectedPortType"))
    {
        j.at("selectedPortType").get_to(selectedPortType);
        if (selectedPortType == 0)
        {
            inputPins.at(InputPortIndex_ObsToSync).dataIdentifier = VectorNavObs::type();
        }
        else if (selectedPortType == 1)
        {
            inputPins.at(InputPortIndex_ObsToSync).dataIdentifier = ImuObs::type();
        }
        else if (selectedPortType == 2)
        {
            inputPins.at(InputPortIndex_ObsToSync).dataIdentifier = KvhObs::type();
        }
        outputPins.at(OutputPortIndex_ObsToSync).dataIdentifier = inputPins.at(InputPortIndex_ObsToSync).dataIdentifier;
    }
    if (j.contains("useFixedStartTime"))
    {
        j.at("useFixedStartTime").get_to(useFixedStartTime);
    }
    if (j.contains("gpsCycle"))
    {
        j.at("gpsCycle").get_to(gpsCycle);
    }
    if (j.contains("gpsWeek"))
    {
        j.at("gpsWeek").get_to(gpsWeek);
    }
    if (j.contains("gpsToW"))
    {
        j.at("gpsToW").get_to(gpsToW);
    }
}

bool NAV::TimeSynchronizer::initialize()
{
    deinitialize();

    LOG_TRACE("{}: called", nameId());

    if (!Node::initialize())
    {
        return false;
    }

    startupGpsTime.reset();
    startupImuTime.reset();
    prevSequenceNumber.reset();

    if (useFixedStartTime)
    {
        startupGpsTime = InsTime(gpsCycle, gpsWeek, gpsToW);
    }

    return isInitialized = true;
}

void NAV::TimeSynchronizer::deinitialize()
{
    LOG_TRACE("{}: called", nameId());

    Node::deinitialize();
}

void NAV::TimeSynchronizer::syncTime(const std::shared_ptr<NAV::NodeData>& nodeData, ax::NodeEditor::LinkId /*linkId*/)
{
    auto obs = std::static_pointer_cast<InsObs>(nodeData);

    if (obs->insTime.has_value() && !startupGpsTime.has_value())
    {
        LOG_INFO("Time Synchronizer ({}) found start time {}", name, obs->insTime.value().toGPSweekTow());
        startupGpsTime = obs->insTime;
    }
}

std::shared_ptr<NAV::NodeData> NAV::TimeSynchronizer::pollData(bool peek)
{
    auto connectedLinks = nm::FindConnectedLinksToPin(inputPins.at(InputPortIndex_ObsToSync).id);
    for (Link* link : connectedLinks)
    {
        if (Pin* pin = nm::FindPin(link->startPinId))
        {
            Node* node = pin->parentNode;
            auto* callback = std::get_if<std::shared_ptr<NAV::NodeData> (Node::*)(bool)>(&pin->data);
            if (node != nullptr && callback != nullptr && *callback != nullptr)
            {
                std::shared_ptr<NAV::NodeData> data = (node->**callback)(peek);
                if (outputPins.at(OutputPortIndex_ObsToSync).dataIdentifier == ImuObs::type()
                    || outputPins.at(OutputPortIndex_ObsToSync).dataIdentifier == VectorNavObs::type())
                {
                    if (syncImuObs(std::static_pointer_cast<ImuObs>(data)))
                    {
                        return data;
                    }
                }
                else if (outputPins.at(OutputPortIndex_ObsToSync).dataIdentifier == KvhObs::type())
                {
                    if (syncKvhObs(std::static_pointer_cast<KvhObs>(data)))
                    {
                        return data;
                    }
                }
            }
        }
    }

    return nullptr;
}

void NAV::TimeSynchronizer::syncObs(const std::shared_ptr<NAV::NodeData>& nodeData, ax::NodeEditor::LinkId /*linkId*/)
{
    if (inputPins.at(InputPortIndex_ObsToSync).dataIdentifier == ImuObs::type()
        || inputPins.at(InputPortIndex_ObsToSync).dataIdentifier == VectorNavObs::type())
    {
        if (syncImuObs(std::static_pointer_cast<ImuObs>(nodeData)))
        {
            invokeCallbacks(OutputPortIndex_ObsToSync, nodeData);
        }
    }
    else if (inputPins.at(InputPortIndex_ObsToSync).dataIdentifier == KvhObs::type())
    {
        if (syncKvhObs(std::static_pointer_cast<KvhObs>(nodeData)))
        {
            invokeCallbacks(OutputPortIndex_ObsToSync, nodeData);
        }
    }
}

bool NAV::TimeSynchronizer::syncImuObs(const std::shared_ptr<NAV::ImuObs>& obs)
{
    if (obs == nullptr)
    {
        return false;
    }

    if (startupGpsTime.has_value())
    {
        if (!startupImuTime.has_value())
        {
            startupImuTime = obs->timeSinceStartup.value();
        }

        obs->insTime = startupGpsTime.value()
                       + std::chrono::nanoseconds(obs->timeSinceStartup.value() - startupImuTime.value());

        return true;
    }

    return obs->insTime.has_value();
}

bool NAV::TimeSynchronizer::syncKvhObs(const std::shared_ptr<NAV::KvhObs>& obs)
{
    if (obs == nullptr)
    {
        return false;
    }

    if (startupGpsTime.has_value())
    {
        constexpr long double dataRate = 1000.0L;

        if (!prevSequenceNumber.has_value())
        {
            prevSequenceNumber = obs->sequenceNumber;
        }

        int sequenceNumberDiff = obs->sequenceNumber - prevSequenceNumber.value();
        if (sequenceNumberDiff < -100)
        {
            sequenceNumberDiff = obs->sequenceNumber + 128 - prevSequenceNumber.value();
        }
        prevSequenceNumber = obs->sequenceNumber;

        obs->insTime = startupGpsTime;
        obs->insTime = startupGpsTime.value()
                       + std::chrono::duration<long double>(static_cast<long double>(sequenceNumberDiff) / dataRate);

        startupGpsTime = obs->insTime;
    }

    return obs->insTime.has_value();
}
