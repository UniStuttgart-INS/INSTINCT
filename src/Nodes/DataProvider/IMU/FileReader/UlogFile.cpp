#include "UlogFile.hpp"

#include "util/Logger.hpp"

#include <exception>

#include "internal/gui/widgets/FileDialog.hpp"

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"

#include "UlogFileFormat.hpp"

// #include "NodeData/IMU/"
// #include "Nodes/DataProvider/IMU/Sensors/"

NAV::UlogFile::UlogFile()
{
    name = typeStatic();

    LOG_TRACE("{}: called", name);

    hasConfig = true;
    // guiConfigDefaultWindowSize = {}; //TODO

    nm::CreateOutputPin(this, "Output", Pin::Type::Flow, { NAV::UlogFile::type() }, &UlogFile::pollData);
}

NAV::UlogFile::~UlogFile()
{
    LOG_TRACE("{}: called", nameId());
}

std::string NAV::UlogFile::typeStatic()
{
    return "UlogFile";
}

std::string NAV::UlogFile::type() const
{
    return typeStatic();
}

std::string NAV::UlogFile::category()
{
    return "Data Provider";
}

void NAV::UlogFile::guiConfig()
{
    if (gui::widgets::FileDialogLoad(path, "Select File", ".ulg", { ".ulg" }, size_t(id), nameId()))
    {
        flow::ApplyChanges();
        initializeNode();
    }
}

[[nodiscard]] json NAV::UlogFile::save() const
{
    LOG_TRACE("{}: called", nameId());

    json j;

    j["FileReader"] = FileReader::save();
    j["Imu"] = Imu::save();

    return j;
}

void NAV::UlogFile::restore(json const& j)
{
    LOG_TRACE("{}: called", nameId());

    if (j.contains("FileReader"))
    {
        FileReader::restore(j.at("FileReader"));
    }
    if (j.contains("Imu"))
    {
        Imu::restore(j.at("Imu"));
    }
}

bool NAV::UlogFile::initialize()
{
    LOG_TRACE("{}: called", nameId());

    messageCount = 0;

    return FileReader::initialize();
}

void NAV::UlogFile::deinitialize()
{
    LOG_TRACE("{}: called", nameId());

    FileReader::deinitialize();
}

bool NAV::UlogFile::resetNode()
{
    FileReader::resetReader();

    return true;
}

std::shared_ptr<const NAV::NodeData> NAV::UlogFile::pollData([[maybe_unused]] bool peek) //NOLINT(readability-convert-member-functions-to-static)
{
    // Get current position
    // auto pos = filestream.tellg();
    // uint8_t i = 0;
    // std::unique_ptr<UlogFile> packet = nullptr;
    // while (filestream.readsome(reinterpret_cast<char*>(&i), 1))
    // {
    //     // packet =
    // }

    return nullptr; //TODO
}

NAV::FileReader::FileType NAV::UlogFile::determineFileType()
{
    LOG_TRACE("called for {}", nameId());

    auto filestream = std::ifstream(path);

    constexpr uint16_t BUFFER_SIZE = 10; //TODO: validate size

    std::array<char, BUFFER_SIZE> buffer{};
    if (filestream.good())
    {
        filestream.read(buffer.data(), BUFFER_SIZE);
        filestream.close();
        LOG_DEBUG("{} has the file type: binary", nameId());
        return FileType::BINARY;
    }
    LOG_ERROR("{} could not determine file type", nameId(), path);
    return FileType::NONE;
}
