#include "GroupBox.hpp"

#include "util/Logger.hpp"

NAV::GroupBox::GroupBox()
{
    LOG_TRACE("called");

    name = typeStatic();
    kind = Node::Kind::GroupBox;
    color = ImColor(255, 255, 255);
    hasConfig = true;
    size = ImVec2(400, 300);
}

NAV::GroupBox::~GroupBox()
{
    LOG_TRACE("called");
}

std::string NAV::GroupBox::typeStatic()
{
    return "GroupBox";
}

std::string NAV::GroupBox::type() const
{
    return typeStatic();
}

std::string NAV::GroupBox::category()
{
    return "GroupBox";
}

[[nodiscard]] json NAV::GroupBox::save() const { return {}; }

void NAV::GroupBox::restore(json const& /* j */) {}