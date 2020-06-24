#include "Node.hpp"

NAV::Node::Node(std::string name)
    : name(std::move(name)) {}

const std::string& NAV::Node::getName() const
{
    return name;
}

std::shared_ptr<NAV::NodeData> NAV::Node::requestOutputData(uint8_t /* portIndex */)
{
    return nullptr;
}

std::shared_ptr<NAV::NodeData> NAV::Node::requestOutputDataPeek(uint8_t /* portIndex */)
{
    return nullptr;
}

void NAV::Node::resetNode()
{
}