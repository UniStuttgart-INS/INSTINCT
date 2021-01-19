#include "EmlidDataLogger.hpp"

#include "NodeData/GNSS/EmlidObs.hpp"

#include "util/Logger.hpp"

#include "gui/widgets/FileDialog.hpp"

#include <iomanip> // std::setprecision

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"

NAV::EmlidDataLogger::EmlidDataLogger()
{
    name = typeStatic();

    LOG_TRACE("{}: called", name);

    fileType = FileType::BINARY;

    color = ImColor(255, 128, 128);
    hasConfig = true;

    nm::CreateInputPin(this, "writeObservation", Pin::Type::Flow, NAV::EmlidObs::type(), &EmlidDataLogger::writeObservation);
}

NAV::EmlidDataLogger::~EmlidDataLogger()
{
    LOG_TRACE("{}: called", nameId());
}

std::string NAV::EmlidDataLogger::typeStatic()
{
    return "EmlidDataLogger";
}

std::string NAV::EmlidDataLogger::type() const
{
    return typeStatic();
}

std::string NAV::EmlidDataLogger::category()
{
    return "Data Logger";
}

void NAV::EmlidDataLogger::guiConfig()
{
    if (gui::widgets::FileDialogSave(path, "Save File", ".ubx", { ".ubx" }, size_t(id), nameId()))
    {
        flow::ApplyChanges();
        deinitializeNode();
    }
}

[[nodiscard]] json NAV::EmlidDataLogger::save() const
{
    LOG_TRACE("{}: called", nameId());

    json j;

    j["FileWriter"] = FileWriter::save();

    return j;
}

void NAV::EmlidDataLogger::restore(json const& j)
{
    LOG_TRACE("{}: called", nameId());

    if (j.contains("FileWriter"))
    {
        FileWriter::restore(j.at("FileWriter"));
    }
}

bool NAV::EmlidDataLogger::initialize()
{
    LOG_TRACE("{}: called", nameId());

    return FileWriter::initialize();
}

void NAV::EmlidDataLogger::deinitialize()
{
    LOG_TRACE("{}: called", nameId());

    FileWriter::deinitialize();
}

void NAV::EmlidDataLogger::writeObservation(const std::shared_ptr<NodeData>& nodeData, ax::NodeEditor::LinkId /*linkId*/)
{
    auto obs = std::static_pointer_cast<EmlidObs>(nodeData);

    if (obs->raw.getRawDataLength() > 0)
    {
        filestream.write(reinterpret_cast<const char*>(obs->raw.getRawData().data()), static_cast<std::streamsize>(obs->raw.getRawDataLength()));
    }
    else
    {
        LOG_ERROR("{}: Tried to write binary, but observation had no binary data.", nameId());
    }
}