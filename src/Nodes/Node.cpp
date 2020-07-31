#include "Node.hpp"

NAV::Node::Node(std::string name)
    : name(std::move(name)) {}

const std::string& NAV::Node::getName() const
{
    return name;
}

std::shared_ptr<NAV::NodeData> NAV::Node::requestOutputData([[maybe_unused]] uint8_t portIndex)
{
    return nullptr;
}

std::shared_ptr<NAV::NodeData> NAV::Node::requestOutputDataPeek([[maybe_unused]] uint8_t portIndex)
{
    return nullptr;
}

void NAV::Node::initialize()
{
}

void NAV::Node::deinitialize()
{
}

void NAV::Node::resetNode()
{
}