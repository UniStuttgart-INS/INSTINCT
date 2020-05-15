#include "Node.hpp"

NAV::Node::Node(std::string name)
    : name(std::move(name)) {}

const std::string& NAV::Node::getName() const
{
    return name;
}