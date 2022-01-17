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

        LOG_DATA("time stamp [µs]: {}", ulogHeader.header.timeStamp);

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

    while (!((ulogMsgHeader.msgHeader.msg_type == 'A') || (ulogMsgHeader.msgHeader.msg_type == 'L')))
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
            uint8_t is_continued{};
            filestream.read(reinterpret_cast<char*>(&messageInfoMulti.is_continued), sizeof(is_continued));
            uint8_t key_len{};
            filestream.read(reinterpret_cast<char*>(&messageInfoMulti.key_len), sizeof(key_len));

            // Read 'key' identifier ('keylength' byte) and its associated 'value'
            messageInfoMulti.key.resize(messageInfoMulti.key_len);
            filestream.read(messageInfoMulti.key.data(), messageInfoMulti.key_len);
            messageInfoMulti.value.resize(messageInfoMulti.header.msg_size - 2 - messageInfoMulti.key_len); // contains 'is_continued' flag in contrast to information message
            filestream.read(messageInfoMulti.value.data(), messageInfoMulti.header.msg_size - 2 - messageInfoMulti.key_len);
            LOG_DEBUG("Information message multi - key_len: {}", messageInfoMulti.key_len);
            LOG_DEBUG("Information message multi - key: {}", messageInfoMulti.key);
            LOG_DEBUG("Information message multi - value: {}", messageInfoMulti.value);
            //TODO: Use 'is_continued' to generate a list of values with the same key
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

            if (!(messageParam.key.find("int32_t")) && !(messageParam.key.find("float")))
            {
                LOG_WARN("Parameter message contains invalid data type. It is neither 'int32_t', nor 'float', instead: {}", messageParam.key);
            }

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

            //TODO: Restriction on '1<<0' and '1<<1'
        }
    }
    LOG_DEBUG("Read 'Definitions Section' completed");

    readData(); //FIXME: use pollData
}

void NAV::UlogFile::readData()
{
    LOG_DEBUG("Start reading data");
    bool startDataFlag = true;
    // Read message header
    union
    {
        std::array<char, 3> data{};
        Ulog::message_header_s msgHeader;
    } ulogMsgHeader{};

    // while (true)
    for (size_t i = 0; i < 100; i++) //FIXME: Quick fix, enable while loop once eof is reached
    {
        // Reset cursor to position before the while-loop in 'readDefinitions()' reached its break condition
        if (startDataFlag)
        {
            filestream.seekg(-3, std::ios_base::cur); // 'msg_size' + 'msg_type' = 3 Byte
            startDataFlag = false;
        }

        filestream.read(ulogMsgHeader.data.data(), ulogMsgHeader.data.size());

        LOG_DATA("msgSize: {}", ulogMsgHeader.msgHeader.msg_size);
        LOG_DATA("msgType: {}", ulogMsgHeader.msgHeader.msg_type);

        if (ulogMsgHeader.msgHeader.msg_type == 'A')
        {
            Ulog::message_add_logged_s messageAddLog;
            messageAddLog.header = ulogMsgHeader.msgHeader;
            uint8_t multi_id{};
            filestream.read(reinterpret_cast<char*>(&messageAddLog.multi_id), sizeof(multi_id));
            LOG_DEBUG("multi_id: {}", multi_id);
            uint16_t msg_id{};
            filestream.read(reinterpret_cast<char*>(&messageAddLog.msg_id), sizeof(msg_id));
            LOG_DEBUG("msg_id: {}", msg_id);

            messageAddLog.msg_name.resize(messageAddLog.header.msg_size);
            filestream.read(messageAddLog.msg_name.data(), messageAddLog.header.msg_size - 3);
            LOG_DEBUG("messageAddLog.msg_name: {}", messageAddLog.msg_name);
        }
        else if (ulogMsgHeader.msgHeader.msg_type == 'R')
        {
            Ulog::message_remove_logged_s messageRemoveLog;
            messageRemoveLog.header = ulogMsgHeader.msgHeader;
            uint16_t msg_id{};
            filestream.read(reinterpret_cast<char*>(&messageRemoveLog.msg_id), sizeof(msg_id));
            LOG_DEBUG("msg_id: {}", msg_id); //TODO: once callback is enabled, make LOG_DATA
        }
        else if (ulogMsgHeader.msgHeader.msg_type == 'D')
        {
            Ulog::message_data_s messageData;
            messageData.header = ulogMsgHeader.msgHeader;
            uint16_t msg_id{};
            filestream.read(reinterpret_cast<char*>(&messageData.msg_id), sizeof(msg_id));
            LOG_DEBUG("msg_id: {}", msg_id); //TODO: once callback is enabled, make LOG_DATA

            messageData.data.resize(messageData.header.msg_size - 2);
            filestream.read(messageData.data.data(), messageData.header.msg_size - 2);
            LOG_DEBUG("messageData.data: {}", messageData.data);
        }
        else if (ulogMsgHeader.msgHeader.msg_type == 'L')
        {
            Ulog::message_logging_s messageLog;
            messageLog.header = ulogMsgHeader.msgHeader;
            uint8_t logLevel{};
            filestream.read(reinterpret_cast<char*>(&messageLog.log_level), sizeof(logLevel));
            LOG_DEBUG("messageLog.log_level: {}", messageLog.log_level);
            uint64_t timestamp{};
            filestream.read(reinterpret_cast<char*>(&messageLog.timestamp), sizeof(timestamp));
            LOG_DEBUG("messageLog.timestamp [µs]: {}", messageLog.timestamp);
        }
        else if (ulogMsgHeader.msgHeader.msg_type == 'C')
        {
            Ulog::message_logging_tagged_s messageLogTagged;
            messageLogTagged.header = ulogMsgHeader.msgHeader;
            uint8_t logLevel{};
            filestream.read(reinterpret_cast<char*>(&messageLogTagged.log_level), sizeof(logLevel));
            LOG_DEBUG("messageLogTagged.log_level: {}", messageLogTagged.log_level);
            uint16_t tag{};
            filestream.read(reinterpret_cast<char*>(&messageLogTagged.tag), sizeof(tag));
            LOG_DEBUG("messageLogTagged.tag: {}", messageLogTagged.tag);
            uint64_t timestamp{};
            filestream.read(reinterpret_cast<char*>(&messageLogTagged.timestamp), sizeof(timestamp));
            LOG_DEBUG("messageLogTagged.timestamp [µs]: {}", messageLogTagged.timestamp);

            messageLogTagged.message.resize(messageLogTagged.header.msg_size - 9);
            filestream.read(messageLogTagged.message.data(), messageLogTagged.header.msg_size - 9);
            LOG_DEBUG("messageLogTagged.header.msg_size: {}", messageLogTagged.header.msg_size);
        }
        else if (ulogMsgHeader.msgHeader.msg_type == 'S')
        {
            Ulog::message_sync_s messageSync;
            messageSync.header = ulogMsgHeader.msgHeader;
            std::array<uint8_t, 8> sync_magic{};
            filestream.read(reinterpret_cast<char*>(messageSync.snyc_magic.data()), sizeof(sync_magic));
            LOG_DEBUG("messageSync.snyc_magic: {}", messageSync.snyc_magic);
        }
        else if (ulogMsgHeader.msgHeader.msg_type == 'O')
        {
            LOG_DEBUG("Read O");
        }
        else if (ulogMsgHeader.msgHeader.msg_type == 'I')
        {
            LOG_DEBUG("Read I");
        }
        else if (ulogMsgHeader.msgHeader.msg_type == 'M')
        {
            LOG_DEBUG("Read M");
        }
        else if (ulogMsgHeader.msgHeader.msg_type == 'P')
        {
            LOG_DEBUG("Read P");
        }
        else if (ulogMsgHeader.msgHeader.msg_type == 'Q')
        {
            LOG_DEBUG("Read Q");
        }

        else
        {
            LOG_WARN("Read nothing");
        }
    }
}

std::shared_ptr<const NAV::NodeData> NAV::UlogFile::pollData([[maybe_unused]] bool peek) //NOLINT(readability-convert-member-functions-to-static)
{
    LOG_DEBUG("Start reading Ulog data section");
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
