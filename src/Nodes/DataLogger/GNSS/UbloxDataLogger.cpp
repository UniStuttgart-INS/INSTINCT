#include "UbloxDataLogger.hpp"

#include "NodeData/GNSS/UbloxObs.hpp"

#include "util/Logger.hpp"

#include <iomanip> // std::setprecision

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"

NAV::UbloxDataLogger::UbloxDataLogger()
{
    name = typeStatic();

    LOG_TRACE("{}: called", name);

    _fileType = FileType::BINARY;

    _hasConfig = true;
    _guiConfigDefaultWindowSize = { 380, 70 };

    nm::CreateInputPin(this, "writeObservation", Pin::Type::Flow, { NAV::UbloxObs::type() }, &UbloxDataLogger::writeObservation);
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
    if (FileWriter::guiConfig(".ubx", { ".ubx" }, size_t(id), nameId()))
    {
        flow::ApplyChanges();
        deinitializeNode();
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

void NAV::UbloxDataLogger::flush()
{
    _filestream.flush();
}

bool NAV::UbloxDataLogger::initialize()
{
    LOG_TRACE("{}: called", nameId());

    return FileWriter::initialize();
}

void NAV::UbloxDataLogger::deinitialize()
{
    LOG_TRACE("{}: called", nameId());

    FileWriter::deinitialize();
}

void NAV::UbloxDataLogger::writeObservation(const std::shared_ptr<const NodeData>& nodeData, ax::NodeEditor::LinkId /*linkId*/)
{
    auto obs = std::static_pointer_cast<const UbloxObs>(nodeData);

    if (obs->raw.getRawDataLength() > 0)
    {
        _filestream.write(reinterpret_cast<const char*>(obs->raw.getRawData().data()), static_cast<std::streamsize>(obs->raw.getRawDataLength()));
    }
    else
    {
        LOG_ERROR("{}: Tried to write binary, but observation had no binary data.", nameId());
    }
}