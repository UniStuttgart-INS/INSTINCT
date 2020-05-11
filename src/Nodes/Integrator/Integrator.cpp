#include "Integrator.hpp"

#include "util/Logger.hpp"

NAV::Integrator::Integrator(const std::string& name, std::deque<std::string>& options)
    : Node(name)
{
    LOG_TRACE("called for {}", name);
}

NAV::Integrator::~Integrator()
{
    LOG_TRACE("called");
}