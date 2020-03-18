#include "DataProvider.hpp"
#include "util/Logger.hpp"

NAV::DataProvider::DataProvider(const std::string name)
    : name(name) {}

NAV::DataProvider::~DataProvider() {}

bool NAV::DataProvider::isInitialized()
{
    LOG_TRACE("called for {} with value {}", name, initialized);
    return initialized;
}

std::string NAV::DataProvider::getName()
{
    LOG_TRACE("called for {}", name);
    return name;
}