#include "TimeSynchronizer.hpp"

#include "util/Logger.hpp"

#include "NodeData/IMU/VectorNavObs.hpp"
#include "NodeData/GNSS/UbloxObs.hpp"
#include "ub/protocol/types.hpp"

#include "NodeInterface.hpp"

NAV::TimeSynchronizer::TimeSynchronizer(std::string name, std::deque<std::string>& /*options*/)
    : Node(name)
{
    LOG_TRACE("called for {}", name);
}

NAV::TimeSynchronizer::~TimeSynchronizer() {}

NAV::NavStatus NAV::TimeSynchronizer::syncUbloxSensor(std::shared_ptr<NAV::NodeData> observation, std::shared_ptr<NAV::Node> userData)
{
    auto obs = std::static_pointer_cast<UbloxObs>(observation);
    auto obj = std::static_pointer_cast<TimeSynchronizer>(userData);

    LOG_TRACE("called for {}", obj->name);

    if (obs->msgClass == ub::protocol::uart::UbxClass::UBX_CLASS_RXM && obs->msgId == ub::protocol::uart::UbxRxmMessages::UBX_RXM_RAWX
        && obs->insTime.has_value())
    {
        if (!obj->startupGpsTime.has_value())
            obj->startupGpsTime = obs->insTime;
    }

    return obj->invokeCallbacks(NodeInterface::getCallbackPort("TimeSynchronizer", "UbloxObs"), obs);
}

NAV::NavStatus NAV::TimeSynchronizer::syncVectorNavSensor(std::shared_ptr<NAV::NodeData> observation, std::shared_ptr<NAV::Node> userData)
{
    auto obs = std::static_pointer_cast<VectorNavObs>(observation);
    auto obj = std::static_pointer_cast<TimeSynchronizer>(userData);

    LOG_TRACE("called for {}", obj->name);

    if (obj->startupGpsTime.has_value())
    {
        if (!obj->startupImuTime.has_value())
            obj->startupImuTime = obs->timeSinceStartup;
        else
        {
            obs->insTime = obj->startupGpsTime;
            obs->insTime.value().addDiffSec(static_cast<long double>(obs->timeSinceStartup.value() - obj->startupImuTime.value()) / 1000000000.0L);
        }
    }

    return obj->invokeCallbacks(NodeInterface::getCallbackPort("TimeSynchronizer", "VectorNavObs"), obs);
}