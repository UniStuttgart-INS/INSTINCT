#include "DataProcessor.hpp"
#include "util/Logger.hpp"

NAV::DataProcessor::DataProcessor(const std::string name)
    : name(name) {}

NAV::DataProcessor::~DataProcessor() {}

bool NAV::DataProcessor::isInitialized()
{
    LOG_TRACE("called for {} with value {}", name, initialized);
    return initialized;
}

std::string NAV::DataProcessor::getName()
{
    LOG_TRACE("called for {}", name);
    return name;
}