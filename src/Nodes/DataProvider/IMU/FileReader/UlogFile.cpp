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

    // while (true)
    for (int i = 0; i < 4; i++) // FIXME: Quick fix, remove later and enable while-loop, stop once there is no more msg type identifier
    {
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
                std::array<char, 40> data{};
                Ulog::ulog_message_flag_bits_s ulogMsgFlagBits_s;
            } ulogMsgFlagBits{};

            filestream.read(ulogMsgFlagBits.data.data(), ulogMsgFlagBits.data.size() * sizeof(char)); // 'sizeof' is optional here, but it is the solution in general, since data types can be larger than one byte
        }

        // Format definition for a single (composite) type that can be logged or used in another definition as a nested type
        else if (ulogMsgHeader.msgHeader.msg_type == 'F')
        {
            std::vector<char> format(ulogMsgHeader.msgHeader.msg_size, 0);
            format.push_back('\0');

            filestream.read(format.data(), ulogMsgHeader.msgHeader.msg_size);
            LOG_DEBUG("format[0]: {}", format[0]);
        }

        // Information message
        else if (ulogMsgHeader.msgHeader.msg_type == 'I')
        {
            // Read msg size (2B) and type (1B)
            Ulog::message_info_s messageInfo;
            messageInfo.header = ulogMsgHeader.msgHeader;
            uint8_t key_len{};
            filestream.read(reinterpret_cast<char*>(&messageInfo.key_len), sizeof(key_len));

            // Read 'key' identifier ('keylength' byte) and its associated 'value'
            messageInfo.key.resize(messageInfo.key_len);
            filestream.read(messageInfo.key.data(), messageInfo.key_len);
            messageInfo.value.resize(messageInfo.header.msg_size - 1 - messageInfo.key_len); // 'msg_size' contains key and value, but not header
            filestream.read(messageInfo.value.data(), messageInfo.header.msg_size - 1 - messageInfo.key_len);
            LOG_DEBUG("Information message - key: {}", messageInfo.key);
            LOG_DEBUG("Information message - value: {}", messageInfo.value);
        }

        // Information message multi
        else if (ulogMsgHeader.msgHeader.msg_type == 'M')
        {
            // Read msg size (2B) and type (1B)
            Ulog::ulog_message_info_multiple_header_s messageInfoMulti;
            messageInfoMulti.header = ulogMsgHeader.msgHeader;
            uint8_t key_len{};
            filestream.read(reinterpret_cast<char*>(&messageInfoMulti.key_len), sizeof(key_len));

            // Read 'key' identifier ('keylength' byte) and its associated 'value'
            messageInfoMulti.key.resize(messageInfoMulti.key_len);
            filestream.read(messageInfoMulti.key.data(), messageInfoMulti.key_len);
            messageInfoMulti.value.resize(messageInfoMulti.header.msg_size - 2 - messageInfoMulti.key_len); // contains 'is_continued' flag in contrast to information message
            filestream.read(messageInfoMulti.value.data(), messageInfoMulti.header.msg_size - 2 - messageInfoMulti.key_len);
            LOG_DEBUG("Information message multi - key: {}", messageInfoMulti.key);
            LOG_DEBUG("Information message multi - value: {}", messageInfoMulti.value);

            // Check, whether there is another msg with the same key
            if (filestream.read(reinterpret_cast<char*>(messageInfoMulti.is_continued), sizeof(messageInfoMulti.is_continued)))
            {
                LOG_WARN("'Information message multi' has not been read entirly");
            }
        }

        // Parameter message (same format as 'message_info_s')
        else if (ulogMsgHeader.msgHeader.msg_type == 'P')
        {
            // Read msg size (2B) and type (1B)
            Ulog::message_info_s messageParam;
            messageParam.header = ulogMsgHeader.msgHeader;
            uint8_t key_len{};
            filestream.read(reinterpret_cast<char*>(&messageParam.key_len), sizeof(key_len));

            // Read 'key' identifier ('keylength' byte) and its associated 'value'
            messageParam.key.resize(messageParam.key_len);
            filestream.read(messageParam.key.data(), messageParam.key_len);
            messageParam.value.resize(messageParam.header.msg_size - 1 - messageParam.key_len); // 'msg_size' contains key and value, but not header
            filestream.read(messageParam.value.data(), messageParam.header.msg_size - 1 - messageParam.key_len);
            LOG_DEBUG("Parameter message - key: {}", messageParam.key);
            LOG_DEBUG("Parameter message - value: {}", messageParam.value);
        }

        // Parameter default message
        else if (ulogMsgHeader.msgHeader.msg_type == 'Q')
        {
            Ulog::ulog_message_parameter_default_header_s messageParamDefault;
            messageParamDefault.header = ulogMsgHeader.msgHeader;
            uint8_t default_types{};
            filestream.read(reinterpret_cast<char*>(&messageParamDefault.default_types), sizeof(default_types));
            uint8_t key_len{};
            filestream.read(reinterpret_cast<char*>(&messageParamDefault.key_len), sizeof(key_len));

            messageParamDefault.key.resize(messageParamDefault.key_len);
            filestream.read(messageParamDefault.key.data(), messageParamDefault.key_len);
            messageParamDefault.value.resize(messageParamDefault.header.msg_size - 2 - messageParamDefault.key_len);
            filestream.read(messageParamDefault.value.data(), messageParamDefault.header.msg_size - 2 - messageParamDefault.key_len);
            LOG_DEBUG("Parameter default message - key: {}", messageParamDefault.key);
            LOG_DEBUG("Parameter default message - value: {}", messageParamDefault.value);
        }
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
