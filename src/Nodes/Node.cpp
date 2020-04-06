#include "Node.hpp"
#include "util/Logger.hpp"

NAV::Node::Node(const std::string name)
    : name(name) {}

NAV::Node::~Node() {}

bool NAV::Node::isInitialized()
{
    LOG_TRACE("called for {} with value {}", name, initialized);
    return initialized;
}

std::string NAV::Node::getName()
{
    LOG_TRACE("called for {}", name);
    return name;
}