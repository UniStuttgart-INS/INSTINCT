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

bool NAV::Node::isFileReader()
{
    return false;
}

std::shared_ptr<NAV::NodeData> NAV::Node::pollData()
{
    return nullptr;
}

std::optional<uint64_t> NAV::Node::peekNextUpdateTime()
{
    return std::nullopt;
}