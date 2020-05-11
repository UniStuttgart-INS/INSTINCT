#include "ImuIntegrator.hpp"

#include "NodeInterface.hpp"

#include "util/Logger.hpp"

#include "NodeData/IMU/ImuObs.hpp"
#include "NodeData/IMU/IntegratedImuObs.hpp"

NAV::ImuIntegrator::ImuIntegrator(const std::string& name, std::deque<std::string>& options)
    : Integrator(name, options)
{
    LOG_TRACE("called for {}", name);
}

/// Default Destructor
NAV::ImuIntegrator::~ImuIntegrator()
{
    LOG_TRACE("called for {}", name);
}

NAV::NavStatus NAV::ImuIntegrator::integrateImuObs(std::shared_ptr<NAV::NodeData> observation, std::shared_ptr<NAV::Node> userData)
{
    auto obj = std::static_pointer_cast<ImuIntegrator>(userData);
    auto obs = std::static_pointer_cast<ImuObs>(observation);

    LOG_TRACE("called for {}", obj->name);

    auto integratedData = std::make_shared<IntegratedImuObs>();
    integratedData->insTime = obs->insTime;

    return obj->invokeCallbacks(NodeInterface::getCallbackPort("ImuIntegrator", "IntegratedImuObs"), integratedData);
}