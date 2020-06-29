#include "ExampleNode.hpp"

NAV::ExampleNode::ExampleNode(const std::string& name, const std::map<std::string, std::string>& /* options */)
    : Node(name)
{
    // Process the provided options from the config file
    // if (options.count(""))
    // {
    //     std::stoi(options.at(""));
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