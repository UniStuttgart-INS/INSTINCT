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
            Ulog::message_format_s messageFormat;
            messageFormat.header = ulogMsgHeader.msgHeader;

            LOG_DATA("messageFormat.header.msg_size: {}", messageFormat.header.msg_size);

            messageFormat.format.resize(messageFormat.header.msg_size);
            filestream.read(messageFormat.format.data(), ulogMsgHeader.msgHeader.msg_size);
            LOG_DATA("messageFormat.format.data(): {}", messageFormat.format.data());

            std::string msgName = messageFormat.format.substr(0, messageFormat.format.find(':'));

            // Decoding data format fields
            std::stringstream lineStream(messageFormat.format.substr(messageFormat.format.find(':') + 1));
            std::string cell;

            std::vector<DataField> msgDataFields;

            while (std::getline(lineStream, cell, ';'))
            {
                DataField data_field{ cell.substr(0, cell.find(' ')), cell.substr(cell.find(' ') + 1) };
                msgDataFields.push_back(data_field);
            }

            if (msgName == "sensor_accel")
            {
                messageFormats.insert_or_assign(msgName, msgDataFields);
            }
            else if (msgName == "sensor_gyro")
            {
                messageFormats.insert_or_assign(msgName, msgDataFields);
            }
            else if (msgName == "sensor_mag")
            {
                messageFormats.insert_or_assign(msgName, msgDataFields);
            }
            else if (msgName == "vehicle_gps_position")
            {
                messageFormats.insert_or_assign(msgName, msgDataFields);
            }
            else if (msgName == "vehicle_attitude")
            {
                messageFormats.insert_or_assign(msgName, msgDataFields);
            }
            else if (msgName == "vehicle_air_data")
            {
                messageFormats.insert_or_assign(msgName, msgDataFields);
            }
            else if (msgName == "vehicle_control_mode")
            {
                messageFormats.insert_or_assign(msgName, msgDataFields);
            }
            else if (msgName == "vehicle_status")
            {
                messageFormats.insert_or_assign(msgName, msgDataFields);
            }
            else if (msgName == "cpuload")
            {
                messageFormats.insert_or_assign(msgName, msgDataFields);
            }
            else
            {
                LOG_ERROR("Data format '{}' could not be decoded", msgName);
            }
        }

        // Information message
        else if (ulogMsgHeader.msgHeader.msg_type == 'I')
        {
            readInformationMessage(ulogMsgHeader.msgHeader.msg_size, ulogMsgHeader.msgHeader.msg_type);
        }

        // Information message multi
        else if (ulogMsgHeader.msgHeader.msg_type == 'M')
        {
            readInformationMessageMulti(ulogMsgHeader.msgHeader.msg_size, ulogMsgHeader.msgHeader.msg_type);
        }

        // Parameter message
        else if (ulogMsgHeader.msgHeader.msg_type == 'P')
        {
            readParameterMessage(ulogMsgHeader.msgHeader.msg_size, ulogMsgHeader.msgHeader.msg_type);
        }

        // Parameter message default
        else if (ulogMsgHeader.msgHeader.msg_type == 'Q')
        {
            readParameterMessageDefault(ulogMsgHeader.msgHeader.msg_size, ulogMsgHeader.msgHeader.msg_type);
        }
    }
    LOG_DEBUG("Read 'Definitions Section' completed");

    readData(); //FIXME: use pollData
}

auto hashSubscriptionData = [](const NAV::UlogFile::SubscriptionData& sData) {
    return std::hash<uint8_t>()(sData.multi_id) ^ (std::hash<std::string>()(sData.message_name) << 1);
};
/// comparison function
auto cmpSubscriptionData = [](const NAV::UlogFile::SubscriptionData& lhs, const NAV::UlogFile::SubscriptionData& rhs) {
    return lhs.message_name == rhs.message_name && lhs.multi_id == rhs.multi_id;
};

// Key: [multi_id, msg_name], e.g. [0, "sensor_accel"]
std::unordered_map<NAV::UlogFile::SubscriptionData,                                                               // key
                   std::variant<NAV::UlogFile::SensorAccel, NAV::UlogFile::SensorGyro, NAV::UlogFile::SensorMag>, // value
                   decltype(hashSubscriptionData),
                   decltype(cmpSubscriptionData)>
    epochData{
        6, hashSubscriptionData, cmpSubscriptionData
    };

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
            filestream.read(reinterpret_cast<char*>(&messageAddLog.multi_id), sizeof(messageAddLog.multi_id));
            LOG_DATA("multi_id: {}", messageAddLog.multi_id);
            filestream.read(reinterpret_cast<char*>(&messageAddLog.msg_id), sizeof(messageAddLog.msg_id));
            LOG_DATA("msg_id: {}", messageAddLog.msg_id);

            messageAddLog.msg_name.resize(messageAddLog.header.msg_size - 3);
            filestream.read(messageAddLog.msg_name.data(), messageAddLog.header.msg_size - 3);
            LOG_DATA("messageAddLog.msg_name: {}", messageAddLog.msg_name);

            /// Combines (sensor-)message name with an ID that indicates a possible multiple of a sensor
            subscribedMessages.insert_or_assign(messageAddLog.msg_id, SubscriptionData{ messageAddLog.multi_id, messageAddLog.msg_name });
        }
        else if (ulogMsgHeader.msgHeader.msg_type == 'R')
        {
            Ulog::message_remove_logged_s messageRemoveLog;
            messageRemoveLog.header = ulogMsgHeader.msgHeader;
            filestream.read(reinterpret_cast<char*>(&messageRemoveLog.msg_id), sizeof(messageRemoveLog.msg_id));
            LOG_INFO("Removed message with 'msg_id': {}", messageRemoveLog.msg_id);

            subscribedMessages.erase(messageRemoveLog.msg_id);
        }
        else if (ulogMsgHeader.msgHeader.msg_type == 'D')
        {
            Ulog::message_data_s messageData;
            messageData.header = ulogMsgHeader.msgHeader;
            filestream.read(reinterpret_cast<char*>(&messageData.msg_id), sizeof(messageData.msg_id));
            // LOG_DEBUG("msg_id: {}", messageData.msg_id); //TODO: once callback is enabled, make LOG_DATA

            messageData.data.resize(messageData.header.msg_size - 2);
            filestream.read(messageData.data.data(), messageData.header.msg_size - 2);
            // LOG_DEBUG("messageData.data: {}", std::atof(messageData.data.data())); //NOLINT(cert-err34-c)

            if (subscribedMessages.at(messageData.msg_id).message_name == "sensor_accel")
            {
                SensorAccel sensorAccel{};

                const auto& messageFormat = messageFormats.at(subscribedMessages.at(messageData.msg_id).message_name);

                size_t currentDataPos = 0;
                for (const auto& dataField : messageFormat)
                {
                    [[maybe_unused]] char* currentData = messageData.data.data() + currentDataPos;
                    LOG_WARN("currentData: {}", currentData);
                    if (dataField.name == "timestamp")
                    {
                        if (dataField.type == "uint64_t")
                        {
                            sensorAccel.timestamp = 1; // currentData << 0 | currentData << 8; // TODO: Bitshift the data in here
                            currentDataPos += sizeof(uint64_t);
                        }
                        // TODO: else if (if necessary)
                        // TODO: else WARN
                    }
                    // TODO: else if
                    // TODO: else WARN
                }
                if (currentTimestamp < sensorAccel.timestamp)
                {
                    // If new time has come, erase just the old IMU data
                    epochData.erase(SubscriptionData{ subscribedMessages.at(messageData.msg_id).multi_id, "sensor_accel" });
                    epochData.erase(SubscriptionData{ subscribedMessages.at(messageData.msg_id).multi_id, "sensor_gyro" });
                    epochData.erase(SubscriptionData{ subscribedMessages.at(messageData.msg_id).multi_id, "sensor_mag" });

                    // Update timestamp
                    currentTimestamp = sensorAccel.timestamp;
                }
                else if (currentTimestamp > sensorAccel.timestamp)
                {
                    LOG_WARN("currentTimestamp > sensorAccel.timestamp. Not handled. Needs to be handled?");
                }

                if (currentTimestamp == sensorAccel.timestamp)
                {
                    // Save the data
                    SubscriptionData subscriptionData{ subscribedMessages.at(messageData.msg_id).multi_id,
                                                       subscribedMessages.at(messageData.msg_id).message_name };

                    epochData.insert_or_assign(subscriptionData, sensorAccel);
                }
            }

            // TODO: for loop for multiple multi_ids
            if (epochData.contains(SubscriptionData{ 0, "sensor_accel" })
                && epochData.contains(SubscriptionData{ 0, "sensor_gyro" })
                && epochData.contains(SubscriptionData{ 0, "sensor_mag" }))
            {
                LOG_INFO("Construct ImuObs and invoke callback");

                // TODO: invoke callback here

                // Erase just the IMU data
                epochData.erase(SubscriptionData{ 0, "sensor_accel" });
                epochData.erase(SubscriptionData{ 0, "sensor_gyro" });
                epochData.erase(SubscriptionData{ 0, "sensor_mag" });
            }
        }
        else if (ulogMsgHeader.msgHeader.msg_type == 'L')
        {
            Ulog::message_logging_s messageLog;
            messageLog.header = ulogMsgHeader.msgHeader;
            filestream.read(reinterpret_cast<char*>(&messageLog.log_level), sizeof(messageLog.log_level));

            if (messageLog.log_level == 48) // '0'
            {
                LOG_INFO("Log-level: EMERG - System is unusable");
            }
            else if (messageLog.log_level == 49) // '1'
            {
                LOG_INFO("Log-level: ALERT - Action must be taken immediately");
            }
            else if (messageLog.log_level == 50) // '2'
            {
                LOG_INFO("Log-level: CRIT - Critical conditions");
            }
            else if (messageLog.log_level == 51) // '3'
            {
                LOG_INFO("Log-level: ERR - Error conditions");
            }
            else if (messageLog.log_level == 52) // '4'
            {
                LOG_INFO("Log-level: WARNING - Warning conditions");
            }
            else if (messageLog.log_level == 53) // '5'
            {
                LOG_INFO("Log-level: NOTICE - Normal but significant condition");
            }
            else if (messageLog.log_level == 54) // '6'
            {
                LOG_INFO("Log-level: INFO - Informational");
            }
            else if (messageLog.log_level == 55) // '7'
            {
                LOG_INFO("Log-level: DEBUG - Debug-level messages");
            }
            else
            {
                LOG_WARN("Log-level is out of scope ({}) - possible data loss", messageLog.log_level);
            }

            filestream.read(reinterpret_cast<char*>(&messageLog.timestamp), sizeof(messageLog.timestamp));
            LOG_DATA("messageLog.timestamp [µs]: {}", messageLog.timestamp);

            messageLog.message.resize(messageLog.header.msg_size - 9);
            filestream.read(messageLog.message.data(), messageLog.header.msg_size - 9);
            LOG_DATA("messageLog.message: {}", messageLog.message);
        }
        else if (ulogMsgHeader.msgHeader.msg_type == 'C')
        {
            Ulog::message_logging_tagged_s messageLogTagged;
            messageLogTagged.header = ulogMsgHeader.msgHeader;
            filestream.read(reinterpret_cast<char*>(&messageLogTagged.log_level), sizeof(messageLogTagged.log_level));

            if (messageLogTagged.log_level == 48) // '0'
            {
                LOG_INFO("Log-level: EMERG - System is unusable");
            }
            else if (messageLogTagged.log_level == 49) // '1'
            {
                LOG_INFO("Log-level: ALERT - Action must be taken immediately");
            }
            else if (messageLogTagged.log_level == 50) // '2'
            {
                LOG_INFO("Log-level: CRIT - Critical conditions");
            }
            else if (messageLogTagged.log_level == 51) // '3'
            {
                LOG_INFO("Log-level: ERR - Error conditions");
            }
            else if (messageLogTagged.log_level == 52) // '4'
            {
                LOG_INFO("Log-level: WARNING - Warning conditions");
            }
            else if (messageLogTagged.log_level == 53) // '5'
            {
                LOG_INFO("Log-level: NOTICE - Normal but significant condition");
            }
            else if (messageLogTagged.log_level == 54) // '6'
            {
                LOG_INFO("Log-level: INFO - Informational");
            }
            else if (messageLogTagged.log_level == 55) // '7'
            {
                LOG_INFO("Log-level: DEBUG - Debug-level messages");
            }
            else
            {
                LOG_WARN("Log-level is out of scope ({}) - possible data loss", messageLogTagged.log_level);
            }

            filestream.read(reinterpret_cast<char*>(&messageLogTagged.tag), sizeof(messageLogTagged.tag));
            LOG_DEBUG("messageLogTagged.tag: {}", messageLogTagged.tag);
            filestream.read(reinterpret_cast<char*>(&messageLogTagged.timestamp), sizeof(messageLogTagged.timestamp));
            LOG_DATA("messageLogTagged.timestamp [µs]: {}", messageLogTagged.timestamp);

            messageLogTagged.message.resize(messageLogTagged.header.msg_size - 11);
            filestream.read(messageLogTagged.message.data(), messageLogTagged.header.msg_size - 11);
            LOG_DATA("messageLogTagged.header.msg_size: {}", messageLogTagged.header.msg_size);
        }
        else if (ulogMsgHeader.msgHeader.msg_type == 'S')
        {
            Ulog::message_sync_s messageSync;
            messageSync.header = ulogMsgHeader.msgHeader;
            std::array<uint8_t, 8> sync_magic{};
            filestream.read(reinterpret_cast<char*>(messageSync.snyc_magic.data()), sizeof(sync_magic));
            LOG_DEBUG("messageSync.snyc_magic[0]: {}", messageSync.snyc_magic[0]);
        }
        else if (ulogMsgHeader.msgHeader.msg_type == 'O')
        {
            Ulog::message_dropout_s messageDropout;
            messageDropout.header = ulogMsgHeader.msgHeader;
            filestream.read(reinterpret_cast<char*>(&messageDropout.duration), sizeof(messageDropout.duration));
            LOG_WARN("Dropout of duration: {} ms", messageDropout.duration);
        }
        else if (ulogMsgHeader.msgHeader.msg_type == 'I')
        {
            readInformationMessage(ulogMsgHeader.msgHeader.msg_size, ulogMsgHeader.msgHeader.msg_type);
        }
        else if (ulogMsgHeader.msgHeader.msg_type == 'M')
        {
            readInformationMessageMulti(ulogMsgHeader.msgHeader.msg_size, ulogMsgHeader.msgHeader.msg_type);
        }
        else if (ulogMsgHeader.msgHeader.msg_type == 'P')
        {
            readParameterMessage(ulogMsgHeader.msgHeader.msg_size, ulogMsgHeader.msgHeader.msg_type);
        }
        else if (ulogMsgHeader.msgHeader.msg_type == 'Q')
        {
            readParameterMessageDefault(ulogMsgHeader.msgHeader.msg_size, ulogMsgHeader.msgHeader.msg_type);
        }

        else
        {
            std::string nextChars;
            auto unidentifiedPos = static_cast<uint64_t>(filestream.tellg());
            nextChars.resize(100);
            filestream.read(nextChars.data(), 100);
            LOG_WARN("Message type not identified. Position: {}, The next 100 chars: {}", unidentifiedPos, nextChars);

            // Reset read cursor
            filestream.seekg(-100, std::ios_base::cur);
        }
    }
}

void NAV::UlogFile::readInformationMessage(uint16_t msgSize, char msgType)
{
    // Read msg size (2B) and type (1B)
    Ulog::message_info_s messageInfo;
    messageInfo.header.msg_size = msgSize;
    messageInfo.header.msg_type = msgType;
    filestream.read(reinterpret_cast<char*>(&messageInfo.key_len), sizeof(messageInfo.key_len));

    // Read 'key' identifier ('keylength' byte) and its associated 'value'
    messageInfo.key.resize(messageInfo.key_len);
    filestream.read(messageInfo.key.data(), messageInfo.key_len);
    messageInfo.value.resize(static_cast<size_t>(messageInfo.header.msg_size - 1 - messageInfo.key_len)); // 'msg_size' contains key and value, but not header
    filestream.read(messageInfo.value.data(), messageInfo.header.msg_size - 1 - messageInfo.key_len);
    LOG_DATA("Information message - key: {}", messageInfo.key);
    LOG_DATA("Information message - value: {}", messageInfo.value);
}

void NAV::UlogFile::readInformationMessageMulti(uint16_t msgSize, char msgType)
{
    // Read msg size (2B) and type (1B)
    Ulog::ulog_message_info_multiple_header_s messageInfoMulti;
    messageInfoMulti.header.msg_size = msgSize;
    messageInfoMulti.header.msg_type = msgType;
    filestream.read(reinterpret_cast<char*>(&messageInfoMulti.is_continued), sizeof(messageInfoMulti.is_continued));
    filestream.read(reinterpret_cast<char*>(&messageInfoMulti.key_len), sizeof(messageInfoMulti.key_len));

    // Read 'key' identifier ('keylength' byte) and its associated 'value'
    messageInfoMulti.key.resize(messageInfoMulti.key_len);
    filestream.read(messageInfoMulti.key.data(), messageInfoMulti.key_len);
    messageInfoMulti.value.resize(static_cast<size_t>(messageInfoMulti.header.msg_size - 2 - messageInfoMulti.key_len)); // contains 'is_continued' flag in contrast to information message
    filestream.read(messageInfoMulti.value.data(), messageInfoMulti.header.msg_size - 2 - messageInfoMulti.key_len);
    LOG_DATA("Information message multi - key_len: {}", messageInfoMulti.key_len);
    LOG_DATA("Information message multi - key: {}", messageInfoMulti.key);
    LOG_DATA("Information message multi - value: {}", messageInfoMulti.value);
    //TODO: Use 'is_continued' to generate a list of values with the same key
}

void NAV::UlogFile::readParameterMessage(uint16_t msgSize, char msgType)
{
    // Read msg size (2B) and type (1B)
    Ulog::message_info_s messageParam;
    messageParam.header.msg_size = msgSize;
    messageParam.header.msg_type = msgType;
    filestream.read(reinterpret_cast<char*>(&messageParam.key_len), sizeof(messageParam.key_len));

    // Read 'key' identifier ('keylength' byte) and its associated 'value'
    messageParam.key.resize(messageParam.key_len);
    filestream.read(messageParam.key.data(), messageParam.key_len);

    if (!(messageParam.key.find("int32_t")) && !(messageParam.key.find("float")))
    {
        LOG_WARN("Parameter message contains invalid data type. It is neither 'int32_t', nor 'float', instead: {}", messageParam.key);
    }

    if (messageParam.header.msg_size - 1 - messageParam.key_len < 0)
    {
        LOG_WARN("Parameter msg has key_len: {}", messageParam.key_len);
    }
    else
    {
        messageParam.value.resize(static_cast<size_t>(messageParam.header.msg_size - 1 - messageParam.key_len)); // 'msg_size' contains key and value, but not header
        filestream.read(messageParam.value.data(), messageParam.header.msg_size - 1 - messageParam.key_len);
        LOG_DATA("Parameter message - key: {}", messageParam.key);
        LOG_DATA("Parameter message - value: {}", messageParam.value);
    }
}

void NAV::UlogFile::readParameterMessageDefault(uint16_t msgSize, char msgType)
{
    Ulog::ulog_message_parameter_default_header_s messageParamDefault;
    messageParamDefault.header.msg_size = msgSize;
    messageParamDefault.header.msg_type = msgType;
    filestream.read(reinterpret_cast<char*>(&messageParamDefault.default_types), sizeof(messageParamDefault.default_types));
    filestream.read(reinterpret_cast<char*>(&messageParamDefault.key_len), sizeof(messageParamDefault.key_len));

    messageParamDefault.key.resize(messageParamDefault.key_len);
    filestream.read(messageParamDefault.key.data(), messageParamDefault.key_len);
    messageParamDefault.value.resize(static_cast<size_t>(messageParamDefault.header.msg_size - 2 - messageParamDefault.key_len));
    filestream.read(messageParamDefault.value.data(), messageParamDefault.header.msg_size - 2 - messageParamDefault.key_len);
    LOG_DEBUG("Parameter default message - key: {}", messageParamDefault.key);
    LOG_DEBUG("Parameter default message - value: {}", messageParamDefault.value);

    //TODO: Restriction on '1<<0' and '1<<1'
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
