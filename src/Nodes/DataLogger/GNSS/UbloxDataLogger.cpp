#include "UbloxDataLogger.hpp"

#include "NodeData/GNSS/UbloxObs.hpp"

#include "util/Logger.hpp"

#include "gui/widgets/FileDialog.hpp"

#include <iomanip> // std::setprecision

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"

NAV::UbloxDataLogger::UbloxDataLogger()
{
    name = typeStatic();

    LOG_TRACE("{}: called", name);

    fileType = FileType::BINARY;

    color = ImColor(255, 128, 128);
    hasConfig = true;

    nm::CreateInputPin(this, "writeObservation", Pin::Type::Flow, NAV::UbloxObs::type(), &UbloxDataLogger::writeObservation);
}

NAV::UbloxDataLogger::~UbloxDataLogger()
{
    LOG_TRACE("{}: called", nameId());
}

std::string NAV::UbloxDataLogger::typeStatic()
{
    return "UbloxDataLogger";
}

std::string NAV::UbloxDataLogger::type() const
{
    return typeStatic();
}

std::string NAV::UbloxDataLogger::category()
{
    return "Data Logger";
}

void NAV::UbloxDataLogger::guiConfig()
{
    if (gui::widgets::FileDialogSave(path, "Save File", ".ubx", { ".ubx" }, size_t(id), nameId()))
    {
        flow::ApplyChanges();
        deinitialize();
    }
}

[[nodiscard]] json NAV::UbloxDataLogger::save() const
{
    LOG_TRACE("{}: called", nameId());

    json j;

    j["FileWriter"] = FileWriter::save();

    return j;
}

void NAV::UbloxDataLogger::restore(json const& j)
{
    LOG_TRACE("{}: called", nameId());

    if (j.contains("FileWriter"))
    {
        FileWriter::restore(j.at("FileWriter"));
    }
}

bool NAV::UbloxDataLogger::initialize()
{
    deinitialize();

    LOG_TRACE("{}: called", nameId());

    if (!Node::initialize()
        || !FileWriter::initialize())
    {
        return false;
    }

    return isInitialized = true;
}

void NAV::UbloxDataLogger::deinitialize()
{
    LOG_TRACE("{}: called", nameId());

    FileWriter::deinitialize();
    Node::deinitialize();
}

void NAV::UbloxDataLogger::writeObservation(const std::shared_ptr<NodeData>& nodeData, ax::NodeEditor::LinkId /*linkId*/)
{
    auto obs = std::static_pointer_cast<UbloxObs>(nodeData);

    if (obs->raw.getRawDataLength() > 0)
    {
        filestream.write(reinterpret_cast<const char*>(obs->raw.getRawData().data()), static_cast<std::streamsize>(obs->raw.getRawDataLength()));
    }
    else
    {
        LOG_ERROR("{}: Tried to write binary, but observation had no binary data.", nameId());
    }
}