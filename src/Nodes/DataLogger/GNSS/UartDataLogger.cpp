#include "UartDataLogger.hpp"

#include "NodeData/General/UartPacket.hpp"

#include "util/Logger.hpp"

#include <iomanip> // std::setprecision

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"

NAV::UartDataLogger::UartDataLogger()
{
    name = typeStatic();

    LOG_TRACE("{}: called", name);

    _fileType = FileType::BINARY;

    _hasConfig = true;
    _guiConfigDefaultWindowSize = { 380, 70 };

    nm::CreateInputPin(this, "writeObservation", Pin::Type::Flow, { NAV::UartPacket::type() }, &UartDataLogger::writeObservation);
}

NAV::UartDataLogger::~UartDataLogger()
{
    LOG_TRACE("{}: called", nameId());
}

std::string NAV::UartDataLogger::typeStatic()
{
    return "UartDataLogger";
}

std::string NAV::UartDataLogger::type() const
{
    return typeStatic();
}

std::string NAV::UartDataLogger::category()
{
    return "Data Logger";
}

void NAV::UartDataLogger::guiConfig()
{
    if (FileWriter::guiConfig(".ubx", { ".ubx" }, size_t(id), nameId()))
    {
        flow::ApplyChanges();
        nm::DeinitializeNode(*this);
    }
}

[[nodiscard]] json NAV::UartDataLogger::save() const
{
    LOG_TRACE("{}: called", nameId());

    json j;

    j["FileWriter"] = FileWriter::save();

    return j;
}

void NAV::UartDataLogger::restore(json const& j)
{
    LOG_TRACE("{}: called", nameId());

    if (j.contains("FileWriter"))
    {
        FileWriter::restore(j.at("FileWriter"));
    }
}

void NAV::UartDataLogger::flush()
{
    _filestream.flush();
}

bool NAV::UartDataLogger::initialize()
{
    LOG_TRACE("{}: called", nameId());

    return FileWriter::initialize();
}

void NAV::UartDataLogger::deinitialize()
{
    LOG_TRACE("{}: called", nameId());

    FileWriter::deinitialize();
}

void NAV::UartDataLogger::writeObservation(const std::shared_ptr<const NodeData>& nodeData, ax::NodeEditor::LinkId /*linkId*/)
{
    auto obs = std::static_pointer_cast<const UartPacket>(nodeData);

    if (obs->raw.getRawDataLength() > 0)
    {
        _filestream.write(reinterpret_cast<const char*>(obs->raw.getRawData().data()), static_cast<std::streamsize>(obs->raw.getRawDataLength()));
    }
    else
    {
        LOG_ERROR("{}: Tried to write binary, but observation had no binary data.", nameId());
    }
}