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
        union
        {
            std::array<char, 16> data{};
            Ulog::ulog_Header_s header;
        } ulogHeader{};

        filestream.read(ulogHeader.data.data(), ulogHeader.data.size());

        // Check "ULog" at beginning of file
        if (!((ulogHeader.header.fileMagic[0] == 'U') && (ulogHeader.header.fileMagic[1] == 'L') && (ulogHeader.header.fileMagic[2] == 'o') && (ulogHeader.header.fileMagic[3] == 'g')))
        {
            LOG_WARN("FileType is binary, but not ULog");
        }

        // Read ULog version (currently only 1, see https://docs.px4.io/master/en/dev_log/ulog_file_format.html)
        LOG_DATA("version: {}", static_cast<int>(ulogHeader.header.version)); // No use so far, hence just a LOG_DATA

        LOG_DATA("time stamp in microseconds: {}", ulogHeader.header.timeStamp); // TODO: Woher weiß der Reader, dass das little Endian ist?

        readDefinitions();
    }
}

void NAV::UlogFile::readDefinitions()
{
    // Read message header
    union
    {
        std::array<char, 3> data{};
        Ulog::message_header_s msgHeader;
    } ulogMsgHeader{};

    filestream.read(ulogMsgHeader.data.data(), ulogMsgHeader.data.size());

    LOG_DATA("msgSize: {}", ulogMsgHeader.msgHeader.msg_size);
    LOG_DATA("msgType: {}", ulogMsgHeader.msgHeader.msg_type);

    // Read definition message
    // Flag bitset message
    if (ulogMsgHeader.msgHeader.msg_type == 'B')
    {
        if (ulogMsgHeader.msgHeader.msg_size > 40)
        {
            LOG_WARN("Exceeding bytes in 'flag bitset message' are ignored. Check for ULog file format update.");
        }

        union
        {
            std::array<char, 19> data{};
            Ulog::ulog_message_flag_bits_s ulogMsgFlagBits_s;
        } ulogMsgFlagBits{};

        filestream.read(ulogMsgFlagBits.data.data(), ulogMsgFlagBits.data.size());

        // TODO: Remove the following logs once when this data is really not necessary
        LOG_DEBUG("compat_flags: {}, {}, {}, {}, {}, {}, {}, {}", static_cast<int>(ulogMsgFlagBits.ulogMsgFlagBits_s.compat_flags[0]), static_cast<int>(ulogMsgFlagBits.ulogMsgFlagBits_s.compat_flags[1]), static_cast<int>(ulogMsgFlagBits.ulogMsgFlagBits_s.compat_flags[2]), static_cast<int>(ulogMsgFlagBits.ulogMsgFlagBits_s.compat_flags[3]), static_cast<int>(ulogMsgFlagBits.ulogMsgFlagBits_s.compat_flags[4]), static_cast<int>(ulogMsgFlagBits.ulogMsgFlagBits_s.compat_flags[5]), static_cast<int>(ulogMsgFlagBits.ulogMsgFlagBits_s.compat_flags[6]), static_cast<int>(ulogMsgFlagBits.ulogMsgFlagBits_s.compat_flags[7]));

        LOG_DEBUG("incompat_flags: {}, {}, {}, {}, {}, {}, {}, {}", static_cast<int>(ulogMsgFlagBits.ulogMsgFlagBits_s.incompat_flags[0]), static_cast<int>(ulogMsgFlagBits.ulogMsgFlagBits_s.incompat_flags[1]), static_cast<int>(ulogMsgFlagBits.ulogMsgFlagBits_s.incompat_flags[2]), static_cast<int>(ulogMsgFlagBits.ulogMsgFlagBits_s.incompat_flags[3]), static_cast<int>(ulogMsgFlagBits.ulogMsgFlagBits_s.incompat_flags[4]), static_cast<int>(ulogMsgFlagBits.ulogMsgFlagBits_s.incompat_flags[5]), static_cast<int>(ulogMsgFlagBits.ulogMsgFlagBits_s.incompat_flags[6]), static_cast<int>(ulogMsgFlagBits.ulogMsgFlagBits_s.incompat_flags[7]));

        LOG_DEBUG("appended_offsets: {}, {}, {}", static_cast<int>(ulogMsgFlagBits.ulogMsgFlagBits_s.appended_offsets[0]), static_cast<int>(ulogMsgFlagBits.ulogMsgFlagBits_s.appended_offsets[1]), static_cast<int>(ulogMsgFlagBits.ulogMsgFlagBits_s.appended_offsets[2]));

        // Move read cursor to make 40 bytes of this msg "full"
        LOG_DEBUG("Current read cursor position: {}", filestream.tellg());
        filestream.seekg(21, std::ios_base::cur);
        LOG_DEBUG("Current read cursor position (should be 59): {}", filestream.tellg()); // pos = 19 after header, then + 40
    }
    else
    {
        LOG_WARN("Flag bits not set.");
    }
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
