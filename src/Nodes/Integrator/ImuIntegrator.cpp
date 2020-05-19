#include "ImuIntegrator.hpp"

NAV::ImuIntegrator::ImuIntegrator(const std::string& name, std::deque<std::string>& /* options */)
    : Node(name)
{
    // Process the provided options from the config file
    // if (!options.empty())
    // {
    //     std::stoi(options.at(0));
    //     options.pop_front();
    // }
}

void NAV::ImuIntegrator::integrateObservation(std::shared_ptr<NAV::ImuObs>& obs)
{
    invokeCallbacks(obs);
}