#include "ExampleNode.hpp"

NAV::ExampleNode::ExampleNode(const std::string& name, std::deque<std::string>& /* options */)
    : Node(name)
{
    // Process the provided options from the config file
    // if (!options.empty())
    // {
    //     std::stoi(options.at(0));
    //     options.pop_front();
    // }
}

void NAV::ExampleNode::processObservation(std::shared_ptr<NAV::InsObs>& obs)
{
    // Process the data here

    // Create data and invoke callbacks with it
    std::shared_ptr<NAV::InsObs> outputData;
    invokeCallbacks(outputData);
    // Or pass the original data
    invokeCallbacks(obs);
}