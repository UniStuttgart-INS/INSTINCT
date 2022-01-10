#include "UlogFile.hpp"

#include "util/Logger.hpp"

#include <exception>

#include "internal/gui/widgets/FileDialog.hpp"

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"

#include "UlogFileFormat.hpp"

// ----------------------------------------------------------- Basic Node Functions --------------------------------------------------------------

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

// ------------------------------------------------------------ File Reading ---------------------------------------------------------------

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
        LOG_DEBUG("{} has the file type: CSV", nameId());
        return FileType::BINARY;
    }
    filestream.close();

    LOG_ERROR("{} could not open file", nameId(), path);
    return FileType::NONE;
}

void NAV::UlogFile::readHeader()
{
    if (fileType == FileType::BINARY)
    {
#pragma pack(push, 1) // Syntax for gcc for #pragma pack
        struct UlogHeaderStruct
        {                                    // Offset | Size
            std::array<char, 7> fileMagic{}; //   0    | 7 Byte
            char version{ 0 };               //   7    | 1 Byte
            std::array<char, 8> timeStamp{}; //   8    | 8 Byte
        };
#pragma pack(pop)

        union UlogHeader
        {
            std::array<char, 16> data{};
            UlogHeaderStruct header;
        };

        UlogHeader ulogHeader{};

        // Read "ULog" for check
        filestream.read(ulogHeader.header.fileMagic.data(), sizeof(ulogHeader.header.fileMagic));

        if (!((ulogHeader.header.fileMagic[0] == 'U') && (ulogHeader.header.fileMagic[1] == 'L') && (ulogHeader.header.fileMagic[2] == 'o') && (ulogHeader.header.fileMagic[3] == 'g')))
        {
            LOG_WARN("FileType is binary, but not ULog");
        }

        // Read ULog version (currently only 1, see https://docs.px4.io/master/en/dev_log/ulog_file_format.html)
        filestream.read(&ulogHeader.header.version, sizeof(ulogHeader.header.version));
        LOG_DATA("version: {}", static_cast<int>(ulogHeader.header.version)); // No use so far, hence just a LOG_DATA

        // Read ULog timeStamp
        filestream.read(ulogHeader.header.timeStamp.data(), sizeof(ulogHeader.header.timeStamp));
        auto timeStampMicroSec = static_cast<uint64_t>(std::abs(ulogHeader.header.timeStamp.at(0)))
                                 | static_cast<uint64_t>(std::abs(ulogHeader.header.timeStamp.at(1))) << 8UL
                                 | static_cast<uint64_t>(std::abs(ulogHeader.header.timeStamp.at(2))) << 16UL
                                 | static_cast<uint64_t>(std::abs(ulogHeader.header.timeStamp.at(3))) << 24UL
                                 | static_cast<uint64_t>(std::abs(ulogHeader.header.timeStamp.at(4))) << 32UL
                                 | static_cast<uint64_t>(std::abs(ulogHeader.header.timeStamp.at(5))) << 40UL
                                 | static_cast<uint64_t>(std::abs(ulogHeader.header.timeStamp.at(6))) << 48UL
                                 | static_cast<uint64_t>(std::abs(ulogHeader.header.timeStamp.at(7))) << 56UL;

        LOG_DEBUG("timeStampMicroSec: {}", timeStampMicroSec);

        readDefinitions();
    }
}

void NAV::UlogFile::readDefinitions()
{
    // Read message header
    struct UlogMsgHeader_s
    {
        std::array<char, 2> msg_size{};
        char msg_type{ 0 };
    };

    union UlogMsgHeader
    {
        std::array<char, 3> data{};
        UlogMsgHeader_s msgHeader;
    };

    UlogMsgHeader ulogMsgHeader{};

    filestream.read(ulogMsgHeader.msgHeader.msg_size.data(), sizeof(ulogMsgHeader.msgHeader.msg_size));
    auto msgSize = static_cast<uint16_t>(std::abs(ulogMsgHeader.msgHeader.msg_size.at(0)))
                   | static_cast<uint16_t>(std::abs(ulogMsgHeader.msgHeader.msg_size.at(1))) << 8;

    LOG_DEBUG("msgSize: {}", msgSize);

    filestream.read(&ulogMsgHeader.msgHeader.msg_type, sizeof(ulogMsgHeader.msgHeader.msg_type));
    auto msgType = ulogMsgHeader.msgHeader.msg_type;

    LOG_DEBUG("msgType: {}", msgType);

    // Read definition message
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
