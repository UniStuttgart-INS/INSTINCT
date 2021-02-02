#include "Transformation.hpp"

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"

NAV::Transformation::Transformation()
{
    name = typeStatic();

    LOG_TRACE("{}: called", name);

    color = ImColor(255, 128, 128);
    hasConfig = true;
    kind = Kind::Simple;

    nm::CreateOutputPin(this, "", Pin::Type::Matrix, "Eigen::MatrixXd", &matrix);
    nm::CreateInputPin(this, "", Pin::Type::Matrix, { "Eigen::MatrixXd", "BlockMatrix" }, &Transformation::notifyFunction);
}

NAV::Transformation::~Transformation()
{
    LOG_TRACE("{}: called", nameId());
}

std::string NAV::Transformation::typeStatic()
{
    return "Transformation";
}

std::string NAV::Transformation::type() const
{
    return typeStatic();
}

std::string NAV::Transformation::category()
{
    return "Simple";
}

void NAV::Transformation::guiConfig()
{
}

[[nodiscard]] json NAV::Transformation::save() const
{
    LOG_TRACE("{}: called", nameId());

    json j;

    return j;
}

void NAV::Transformation::restore(json const& j)
{
    LOG_TRACE("{}: called", nameId());

    if (j.contains("nRows"))
    {
        // j.at("nRows").get_to(nRows);
    }
}

bool NAV::Transformation::initialize()
{
    LOG_TRACE("{}: called", nameId());

    return true;
}

void NAV::Transformation::deinitialize()
{
    LOG_TRACE("{}: called", nameId());
}

void NAV::Transformation::onNotifyValueChanged(ax::NodeEditor::LinkId linkId)
{
    if (Link* link = nm::FindLink(linkId))
    {
        LOG_TRACE("{}: called for {} ==> {}", nameId(), size_t(link->startPinId), size_t(link->endPinId));
    }
}

void NAV::Transformation::notifyFunction(ax::NodeEditor::LinkId linkId)
{
    if (Link* link = nm::FindLink(linkId))
    {
        LOG_TRACE("{}: called for {} ==> {}", nameId(), size_t(link->startPinId), size_t(link->endPinId));
    }
}