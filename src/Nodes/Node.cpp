#include "Node.hpp"
#include "util/Logger.hpp"

NAV::Node::Node(const std::string name)
    : name(name) {}

NAV::Node::~Node() {}

std::string NAV::Node::getName()
{
    LOG_TRACE("called for {}", name);
    return name;
}