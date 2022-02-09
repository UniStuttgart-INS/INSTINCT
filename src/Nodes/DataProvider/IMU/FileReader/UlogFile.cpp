#include "UlogFile.hpp"

#include "util/Logger.hpp"

#include <exception>

#include "internal/gui/widgets/FileDialog.hpp"

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"

#include "UlogFileFormat.hpp"
#include "NodeData/IMU/ImuObs.hpp"
#include "NodeData/State/PosVelAtt.hpp"

// ----------------------------------------------------------- Basic Node Functions --------------------------------------------------------------

NAV::UlogFile::UlogFile()
{
    name = typeStatic();

    LOG_TRACE("{}: called", name);

    hasConfig = true;
    // guiConfigDefaultWindowSize = {}; //TODO

    // All message types are polled from the first output pin, but then send out on the correct pin over invokeCallbacks
    nm::CreateOutputPin(this, "ImuObs", Pin::Type::Flow, { NAV::ImuObs::type() }, &UlogFile::pollData);
    nm::CreateOutputPin(this, "PosVelAtt", Pin::Type::Flow, { NAV::PosVelAtt::type() });
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

    Imu::guiConfig();
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
    sensorStartupUTCTime_usec = 0;

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

        filestream.read(ulogHeader.data.data(), ulogHeader.data.size()); //TODO: move up?

        // Check "ULog" at beginning of file
        if (!((ulogHeader.header.fileMagic[0] == 'U') && (ulogHeader.header.fileMagic[1] == 'L') && (ulogHeader.header.fileMagic[2] == 'o') && (ulogHeader.header.fileMagic[3] == 'g')))
        {
            LOG_WARN("{}: FileType is binary, but not ULog", nameId());
        }

        // Read ULog version (currently only 1, see https://docs.px4.io/master/en/dev_log/ulog_file_format.html)
        LOG_DATA("{}: version: {}", nameId(), static_cast<int>(ulogHeader.header.version)); // No use so far, hence just a LOG_DATA

        LOG_DATA("{}: time stamp [µs]: {}", nameId(), ulogHeader.header.timeStamp);
        // Read message header
        union
        {
            std::array<char, 3> data{};
            Ulog::message_header_s msgHeader;
        } ulogMsgHeader{};

        while (!((ulogMsgHeader.msgHeader.msg_type == 'A') || (ulogMsgHeader.msgHeader.msg_type == 'L')))
        {
            filestream.read(ulogMsgHeader.data.data(), ulogMsgHeader.data.size());

            LOG_DATA("{}: msgSize: {}", nameId(), ulogMsgHeader.msgHeader.msg_size);
            LOG_DATA("{}: msgType: {}", nameId(), ulogMsgHeader.msgHeader.msg_type);

            // Read definition message
            // Flag bitset message
            if (ulogMsgHeader.msgHeader.msg_type == 'B')
            {
                if (ulogMsgHeader.msgHeader.msg_size > 40)
                {
                    LOG_WARN("{}: Exceeding bytes in 'flag bitset message' are ignored. Check for ULog file format update.", nameId());
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

                LOG_DATA("{}: messageFormat.header.msg_size: {}", nameId(), messageFormat.header.msg_size);

                messageFormat.format.resize(messageFormat.header.msg_size);
                filestream.read(messageFormat.format.data(), ulogMsgHeader.msgHeader.msg_size);
                LOG_DATA("{}: messageFormat.format.data(): {}", nameId(), messageFormat.format.data());

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
                    LOG_ERROR("{}: Data format '{}' could not be decoded", nameId(), msgName);
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
        filestream.seekg(-3, std::ios_base::cur); // 'msg_size' + 'msg_type' = 3 Byte
        LOG_DEBUG("{}: Read 'Definitions Section' completed", nameId());
    }
}

std::shared_ptr<const NAV::NodeData> NAV::UlogFile::pollData([[maybe_unused]] bool peek)
{
    // std::shared_ptr<NAV::NodeData> obs = nullptr;

    LOG_DEBUG("{}: Start reading Ulog data section", nameId());
    // Get current position
    auto pollStartPos = filestream.tellg();

    // TODO: Read in one imuObs (accel, gyro, )
    // Read message header
    union
    {
        std::array<char, 3> data{};
        Ulog::message_header_s msgHeader;
    } ulogMsgHeader{};

    while (true)
    // for (size_t i = 0; i < 100; i++) //TODO: Quick fix, enable while loop once eof is reached
    {
        filestream.read(ulogMsgHeader.data.data(), ulogMsgHeader.data.size());

        LOG_DATA("{}: msgSize: {}", nameId(), ulogMsgHeader.msgHeader.msg_size);
        LOG_DATA("{}: msgType: {}", nameId(), ulogMsgHeader.msgHeader.msg_type);

        if (ulogMsgHeader.msgHeader.msg_type == 'A')
        {
            Ulog::message_add_logged_s messageAddLog;
            messageAddLog.header = ulogMsgHeader.msgHeader;
            filestream.read(reinterpret_cast<char*>(&messageAddLog.multi_id), sizeof(messageAddLog.multi_id));
            LOG_DATA("{}: multi_id: {}", nameId(), messageAddLog.multi_id);
            filestream.read(reinterpret_cast<char*>(&messageAddLog.msg_id), sizeof(messageAddLog.msg_id));
            LOG_DATA("{}: msg_id: {}", nameId() messageAddLog.msg_id);

            messageAddLog.msg_name.resize(messageAddLog.header.msg_size - 3);
            filestream.read(messageAddLog.msg_name.data(), messageAddLog.header.msg_size - 3);
            LOG_DATA("{}: messageAddLog.msg_name: {}", nameId(), messageAddLog.msg_name);

            /// Combines (sensor-)message name with an ID that indicates a possible multiple of a sensor
            subscribedMessages.insert_or_assign(messageAddLog.msg_id, SubscriptionData{ messageAddLog.multi_id, messageAddLog.msg_name });
        }
        else if (ulogMsgHeader.msgHeader.msg_type == 'R')
        {
            Ulog::message_remove_logged_s messageRemoveLog;
            messageRemoveLog.header = ulogMsgHeader.msgHeader;
            filestream.read(reinterpret_cast<char*>(&messageRemoveLog.msg_id), sizeof(messageRemoveLog.msg_id));
            LOG_INFO("{}: Removed message with 'msg_id': {}", nameId(), messageRemoveLog.msg_id);

            subscribedMessages.erase(messageRemoveLog.msg_id);
        }
        else if (ulogMsgHeader.msgHeader.msg_type == 'D')
        {
            Ulog::message_data_s messageData;
            messageData.header = ulogMsgHeader.msgHeader;
            filestream.read(reinterpret_cast<char*>(&messageData.msg_id), sizeof(messageData.msg_id));
            LOG_DEBUG("{}: msg_id: {}", nameId(), messageData.msg_id); //TODO: once callback is enabled, make LOG_DATA

            messageData.data.resize(messageData.header.msg_size - 2);
            filestream.read(messageData.data.data(), messageData.header.msg_size - 2);
            LOG_DEBUG("{}: messageData.header.msg_size: {}", nameId(), messageData.header.msg_size);

            const auto& messageFormat = messageFormats.at(subscribedMessages.at(messageData.msg_id).message_name);

            size_t currentExtractLocation = 0;
            if (subscribedMessages.at(messageData.msg_id).message_name == "sensor_accel")
            {
                SensorAccel sensorAccel{};
                for (const auto& dataField : messageFormat)
                {
                    char* currentData = messageData.data.data() + currentExtractLocation;
                    if (dataField.name == "timestamp")
                    {
                        std::memcpy(&sensorAccel.timestamp, currentData, sizeof(sensorAccel.timestamp));
                        LOG_DEBUG("{}: sensorAccel.timestamp: {}", nameId(), sensorAccel.timestamp);
                        currentExtractLocation += sizeof(sensorAccel.timestamp);
                    }
                    else if (dataField.name == "timestamp_sample")
                    {
                        std::memcpy(&sensorAccel.timestamp_sample, currentData, sizeof(sensorAccel.timestamp_sample));
                        LOG_DATA("{}: sensorAccel.timestamp_sample: {}", nameId(), sensorAccel.timestamp_sample);
                        currentExtractLocation += sizeof(sensorAccel.timestamp_sample);
                    }
                    else if (dataField.name == "device_id")
                    {
                        std::memcpy(&sensorAccel.device_id, currentData, sizeof(sensorAccel.device_id));
                        LOG_DATA("{}: sensorAccel.device_id: {}", nameId(), sensorAccel.device_id);
                        currentExtractLocation += sizeof(sensorAccel.device_id);
                    }
                    else if (dataField.name == "error_count")
                    {
                        std::memcpy(&sensorAccel.error_count, currentData, sizeof(sensorAccel.error_count));
                        LOG_DATA("{}: sensorAccel.error_count: {}", nameId(), sensorAccel.error_count);
                        currentExtractLocation += sizeof(sensorAccel.error_count);
                    }
                    else if (dataField.name == "x")
                    {
                        std::memcpy(&sensorAccel.x, currentData, sizeof(sensorAccel.x));
                        LOG_DATA("{}: sensorAccel.x: {}", nameId(), sensorAccel.x);
                        currentExtractLocation += sizeof(sensorAccel.x);
                        messageCount++;
                    }
                    else if (dataField.name == "y")
                    {
                        std::memcpy(&sensorAccel.y, currentData, sizeof(sensorAccel.y));
                        LOG_DATA("{}: sensorAccel.y: {}", nameId(), sensorAccel.y);
                        currentExtractLocation += sizeof(sensorAccel.y);
                        messageCount++;
                    }
                    else if (dataField.name == "z")
                    {
                        std::memcpy(&sensorAccel.z, currentData, sizeof(sensorAccel.z));
                        LOG_DATA("{}: sensorAccel.z: {}", nameId(), sensorAccel.z);
                        currentExtractLocation += sizeof(sensorAccel.z);
                        messageCount++;
                    }
                    else if (dataField.name == "temperature")
                    {
                        std::memcpy(&sensorAccel.temperature, currentData, sizeof(sensorAccel.temperature));
                        LOG_DATA("{}: sensorAccel.temperature: {}", nameId(), sensorAccel.temperature);
                        currentExtractLocation += sizeof(sensorAccel.temperature);
                    }
                    else if (dataField.name == "clip_counter")
                    {
                        std::memcpy(sensorAccel.clip_counter.data(), currentData, sensorAccel.clip_counter.size());
                        LOG_DATA("{}: sensorAccel.clip_counter: {}", nameId(), fmt::join(sensorAccel.clip_counter, ", "));
                        currentExtractLocation += sensorAccel.clip_counter.size();
                    }
                    else if (dataField.name.compare(0, 7, "_padding")) // e.g. '_padding0', '_padding1'
                    {
                        currentExtractLocation += SensorAccel::padding; // Extraction Location should be moved to account for multiple padding
                        LOG_DATA("{}: sensorAccel: padding", nameId());
                    }
                    else
                    {
                        //FIXME: move 'currentExtractLocation', if yes, how far?
                        LOG_WARN("{}: dataField.name = '{}' or dataField.type = '{}' is unknown", nameId(), dataField.name, dataField.type);
                    }
                }
                epochData.insert(std::make_pair(sensorAccel.timestamp,
                                                MeasurementData{ subscribedMessages.at(messageData.msg_id).multi_id,
                                                                 subscribedMessages.at(messageData.msg_id).message_name,
                                                                 sensorAccel }));
            }
            else if (subscribedMessages.at(messageData.msg_id).message_name == "sensor_gyro")
            {
                SensorGyro sensorGyro{};

                for (const auto& dataField : messageFormat)
                {
                    char* currentData = messageData.data.data() + currentExtractLocation;
                    if (dataField.name == "timestamp")
                    {
                        std::memcpy(&sensorGyro.timestamp, currentData, sizeof(sensorGyro.timestamp));
                        LOG_DEBUG("{}: sensorGyro.timestamp: {}", nameId(), sensorGyro.timestamp);
                        currentExtractLocation += sizeof(sensorGyro.timestamp);
                    }
                    else if (dataField.name == "timestamp_sample")
                    {
                        std::memcpy(&sensorGyro.timestamp_sample, currentData, sizeof(sensorGyro.timestamp_sample));
                        LOG_DATA("{}: sensorGyro.timestamp_sample: {}", nameId(), sensorGyro.timestamp_sample);
                        currentExtractLocation += sizeof(sensorGyro.timestamp_sample);
                    }
                    else if (dataField.name == "device_id")
                    {
                        std::memcpy(&sensorGyro.device_id, currentData, sizeof(sensorGyro.device_id));
                        LOG_DATA("{}: sensorGyro.device_id: {}", nameId(), sensorGyro.device_id);
                        currentExtractLocation += sizeof(sensorGyro.device_id);
                    }
                    else if (dataField.name == "x")
                    {
                        std::memcpy(&sensorGyro.x, currentData, sizeof(sensorGyro.x));
                        LOG_DATA("{}: sensorGyro.x: {}", nameId(), sensorGyro.x);
                        currentExtractLocation += sizeof(sensorGyro.x);
                    }
                    else if (dataField.name == "y")
                    {
                        std::memcpy(&sensorGyro.y, currentData, sizeof(sensorGyro.y));
                        LOG_DATA("{}: sensorGyro.y: {}", nameId(), sensorGyro.y);
                        currentExtractLocation += sizeof(sensorGyro.y);
                    }
                    else if (dataField.name == "z")
                    {
                        std::memcpy(&sensorGyro.z, currentData, sizeof(sensorGyro.z));
                        LOG_DATA("{}: sensorGyro.z: {}", nameId(), sensorGyro.z);
                        currentExtractLocation += sizeof(sensorGyro.z);
                    }
                    else if (dataField.name == "temperature")
                    {
                        std::memcpy(&sensorGyro.temperature, currentData, sizeof(sensorGyro.temperature));
                        LOG_DATA("{}: sensorGyro.temperature: {}", nameId(), sensorGyro.temperature);
                        currentExtractLocation += sizeof(sensorGyro.temperature);
                    }
                    else if (dataField.name == "error_count")
                    {
                        std::memcpy(&sensorGyro.error_count, currentData, sizeof(sensorGyro.error_count));
                        LOG_DATA("{}: sensorGyro.error_count: {}", nameId(), sensorGyro.error_count);
                        currentExtractLocation += sizeof(sensorGyro.error_count);
                    }
                    else
                    {
                        //FIXME: move 'currentExtractLocation', if yes, how far?
                        LOG_WARN("{}: dataField.name = '{}' or dataField.type = '{}' is unknown", nameId(), dataField.name, dataField.type);
                    }
                }
                epochData.insert(std::make_pair(sensorGyro.timestamp,
                                                MeasurementData{ subscribedMessages.at(messageData.msg_id).multi_id,
                                                                 subscribedMessages.at(messageData.msg_id).message_name,
                                                                 sensorGyro }));
            }
            else if (subscribedMessages.at(messageData.msg_id).message_name == "sensor_mag")
            {
                SensorMag sensorMag{};
                for (const auto& dataField : messageFormat)
                {
                    char* currentData = messageData.data.data() + currentExtractLocation;
                    if (dataField.name == "timestamp")
                    {
                        std::memcpy(&sensorMag.timestamp, currentData, sizeof(sensorMag.timestamp));
                        LOG_DEBUG("{}: sensorMag.timestamp: {}", nameId(), sensorMag.timestamp);
                        currentExtractLocation += sizeof(sensorMag.timestamp);
                    }
                    else if (dataField.name == "timestamp_sample")
                    {
                        std::memcpy(&sensorMag.timestamp_sample, currentData, sizeof(sensorMag.timestamp_sample));
                        LOG_DATA("{}: sensorMag.timestamp_sample: {}", nameId(), sensorMag.timestamp_sample);
                        currentExtractLocation += sizeof(sensorMag.timestamp_sample);
                    }
                    else if (dataField.name == "device_id")
                    {
                        std::memcpy(&sensorMag.device_id, currentData, sizeof(sensorMag.device_id));
                        LOG_DATA("{}: sensorMag.device_id: {}", nameId(), sensorMag.device_id);
                        currentExtractLocation += sizeof(sensorMag.device_id);
                    }
                    else if (dataField.name == "x")
                    {
                        std::memcpy(&sensorMag.x, currentData, sizeof(sensorMag.x));
                        LOG_DATA("{}: sensorMag.x: {}", nameId(), sensorMag.x);
                        currentExtractLocation += sizeof(sensorMag.x);
                    }
                    else if (dataField.name == "y")
                    {
                        std::memcpy(&sensorMag.y, currentData, sizeof(sensorMag.y));
                        LOG_DATA("{}: sensorMag.y: {}", nameId(), sensorMag.y);
                        currentExtractLocation += sizeof(sensorMag.y);
                    }
                    else if (dataField.name == "z")
                    {
                        std::memcpy(&sensorMag.z, currentData, sizeof(sensorMag.z));
                        LOG_DATA("{}: sensorMag.z: {}", nameId(), sensorMag.z);
                        currentExtractLocation += sizeof(sensorMag.z);
                    }
                    else if (dataField.name == "temperature")
                    {
                        std::memcpy(&sensorMag.temperature, currentData, sizeof(sensorMag.temperature));
                        LOG_DATA("{}: sensorMag.temperature: {}", nameId(), sensorMag.temperature);
                        currentExtractLocation += sizeof(sensorMag.temperature);
                    }
                    else if (dataField.name == "error_count")
                    {
                        std::memcpy(&sensorMag.error_count, currentData, sizeof(sensorMag.error_count));
                        LOG_DATA("{}: sensorMag.error_count: {}", nameId(), sensorMag.error_count);
                        currentExtractLocation += sizeof(sensorMag.error_count);
                    }
                    else if (dataField.name == "is_external")
                    {
                        std::memcpy(&sensorMag.is_external, currentData, sizeof(sensorMag.is_external));
                        LOG_DATA("{}: sensorMag.is_external: {}", nameId(), sensorMag.is_external);
                        currentExtractLocation += sizeof(sensorMag.is_external);
                    }
                    else if (dataField.name.compare(0, 7, "_padding")) // e.g. '_padding0', '_padding1'
                    {
                        currentExtractLocation += SensorMag::padding; // Extraction Location should be moved to account for multiple padding
                        LOG_DATA("{}: sensorAccel: padding", nameId());
                    }
                    else
                    {
                        //FIXME: move 'currentExtractLocation', if yes, how far?
                        LOG_WARN("{}: dataField.name = '{}' or dataField.type = '{}' is unknown", nameId(), dataField.name, dataField.type);
                    }
                }
                epochData.insert(std::make_pair(sensorMag.timestamp,
                                                MeasurementData{ subscribedMessages.at(messageData.msg_id).multi_id,
                                                                 subscribedMessages.at(messageData.msg_id).message_name,
                                                                 sensorMag }));
            }
            // TODO:
            // else if (subscribedMessages.at(messageData.msg_id).message_name == "sensor_gps")
            // {
            //     // FIXME: If GNSS message
            //     if (!sensorStartupUTCTime_usec)
            //     {
            //         sensorStartupUTCTime_usec = time_utc_usec - timestamp;
            //     }
            // }
            else
            {
                LOG_ERROR("{}: UKNOWN: subscribedMessages.at(messageData.msg_id).message_name = {}", nameId(), subscribedMessages.at(messageData.msg_id).message_name);
            }

            // TODO: for loop for multiple multi_ids (redundant sensors)

            // Callbacks
            bool hasEnoughPosVelAttDataToSend = false; // FIXME: Make function

            // Breakpoint to debug 'epochData' --> see how it's filled
            if (std::holds_alternative<SensorAccel>(epochData.end()->second.data) && messageCount == 3) // This is the hasEnoughData check
            {
                LOG_INFO("{}: Construct ImuObs and invoke callback", nameId());
                // obs = std::make_shared<NAV::ImuObs>(imuPos);
                auto obs = std::make_shared<ImuObs>(this->imuPos);

                // auto test1 = epochData.begin()->first; // key

                // auto test2 = std::get<SensorAccel>(epochData.end()->second.data).timestamp;

                // LOG_ERROR("{}: value: {}", nameId(), test2);

                obs->timeSinceStartup = 1000 * epochData.end()->first; // latest timestamp in [ns]
                // auto latestTimeStamp = obs->timeSinceStartup;
                // LOG_DEBUG("{}: obs->timeSinceStartup = {}", nameId(), latestTimeStamp);

                // auto accelX = std::get<SensorAccel>(epochData.end()->second.data).x;
                // auto accelY = std::get<SensorAccel>(epochData.end()->second.data).y;
                // auto accelZ = std::get<SensorAccel>(epochData.end()->second.data).z;
                // obs->accelUncompXYZ.emplace(accelX, accelY, accelZ);

                // LOG_ERROR("{}: accelX: {}", nameId(), accelX);

                // TODO: Befüllen

                // TODO: Erase just the first used measurements from epochdata

                if (!peek)
                {
                    invokeCallbacks(OutputPortIndex_ImuObs, obs);
                }
                else
                {
                    // Return to position before "Read line".
                    filestream.seekg(pollStartPos, std::ios_base::beg);
                }
                return obs;
            }
            if (hasEnoughPosVelAttDataToSend) // This is the hasEnoughData check
            {
                LOG_INFO("{}: Construct PosVelAtt and invoke callback", nameId());
                auto obs = std::make_shared<NAV::PosVelAtt>();

                // TODO: Befüllen

                // TODO: Erase just the first used measurements from epochdata

                if (!peek)
                {
                    invokeCallbacks(OutputPortIndex_PosVelAtt, obs);
                }
                else
                {
                    // Return to position before "Read line".
                    filestream.seekg(pollStartPos, std::ios_base::beg);
                }
                return obs;
            }
        }
        else if (ulogMsgHeader.msgHeader.msg_type == 'L')
        {
            Ulog::message_logging_s messageLog;
            messageLog.header = ulogMsgHeader.msgHeader;
            filestream.read(reinterpret_cast<char*>(&messageLog.log_level), sizeof(messageLog.log_level));

            if (messageLog.log_level == 48) // '0'
            {
                LOG_INFO("{}: Log-level: EMERG - System is unusable", nameId());
            }
            else if (messageLog.log_level == 49) // '1'
            {
                LOG_INFO("{}: Log-level: ALERT - Action must be taken immediately", nameId());
            }
            else if (messageLog.log_level == 50) // '2'
            {
                LOG_INFO("{}: Log-level: CRIT - Critical conditions", nameId());
            }
            else if (messageLog.log_level == 51) // '3'
            {
                LOG_INFO("{}: Log-level: ERR - Error conditions", nameId());
            }
            else if (messageLog.log_level == 52) // '4'
            {
                LOG_INFO("{}: Log-level: WARNING - Warning conditions", nameId());
            }
            else if (messageLog.log_level == 53) // '5'
            {
                LOG_INFO("{}: Log-level: NOTICE - Normal but significant condition", nameId());
            }
            else if (messageLog.log_level == 54) // '6'
            {
                LOG_INFO("{}: Log-level: INFO - Informational", nameId());
            }
            else if (messageLog.log_level == 55) // '7'
            {
                LOG_INFO("{}: Log-level: DEBUG - Debug-level messages", nameId());
            }
            else
            {
                LOG_WARN("{}: Log-level is out of scope ({}) - possible data loss", nameId(), messageLog.log_level);
            }

            filestream.read(reinterpret_cast<char*>(&messageLog.timestamp), sizeof(messageLog.timestamp));
            LOG_DATA("{}: messageLog.timestamp [µs]: {}", nameId(), messageLog.timestamp);

            messageLog.message.resize(messageLog.header.msg_size - 9);
            filestream.read(messageLog.message.data(), messageLog.header.msg_size - 9);
            LOG_DATA("{}: messageLog.message: {}", nameId(), messageLog.message);
        }
        else if (ulogMsgHeader.msgHeader.msg_type == 'C')
        {
            Ulog::message_logging_tagged_s messageLogTagged;
            messageLogTagged.header = ulogMsgHeader.msgHeader;
            filestream.read(reinterpret_cast<char*>(&messageLogTagged.log_level), sizeof(messageLogTagged.log_level));

            if (messageLogTagged.log_level == 48) // '0'
            {
                LOG_INFO("{}: Log-level: EMERG - System is unusable", nameId());
            }
            else if (messageLogTagged.log_level == 49) // '1'
            {
                LOG_INFO("{}: Log-level: ALERT - Action must be taken immediately", nameId());
            }
            else if (messageLogTagged.log_level == 50) // '2'
            {
                LOG_INFO("{}: Log-level: CRIT - Critical conditions", nameId());
            }
            else if (messageLogTagged.log_level == 51) // '3'
            {
                LOG_INFO("{}: Log-level: ERR - Error conditions", nameId());
            }
            else if (messageLogTagged.log_level == 52) // '4'
            {
                LOG_INFO("{}: Log-level: WARNING - Warning conditions", nameId());
            }
            else if (messageLogTagged.log_level == 53) // '5'
            {
                LOG_INFO("{}: Log-level: NOTICE - Normal but significant condition", nameId());
            }
            else if (messageLogTagged.log_level == 54) // '6'
            {
                LOG_INFO("{}: Log-level: INFO - Informational", nameId());
            }
            else if (messageLogTagged.log_level == 55) // '7'
            {
                LOG_INFO("{}: Log-level: DEBUG - Debug-level messages", nameId());
            }
            else
            {
                LOG_WARN("{}: Log-level is out of scope ({}) - possible data loss", nameId(), messageLogTagged.log_level);
            }

            filestream.read(reinterpret_cast<char*>(&messageLogTagged.tag), sizeof(messageLogTagged.tag));
            LOG_DEBUG("{}: messageLogTagged.tag: {}", nameId(), messageLogTagged.tag);
            filestream.read(reinterpret_cast<char*>(&messageLogTagged.timestamp), sizeof(messageLogTagged.timestamp));
            LOG_DATA("{}: messageLogTagged.timestamp [µs]: {}", nameId(), messageLogTagged.timestamp);

            messageLogTagged.message.resize(messageLogTagged.header.msg_size - 11);
            filestream.read(messageLogTagged.message.data(), messageLogTagged.header.msg_size - 11);
            LOG_DATA("{}: messageLogTagged.header.msg_size: {}", nameId(), messageLogTagged.header.msg_size);
        }
        else if (ulogMsgHeader.msgHeader.msg_type == 'S')
        {
            Ulog::message_sync_s messageSync;
            messageSync.header = ulogMsgHeader.msgHeader;
            std::array<uint8_t, 8> sync_magic{};
            filestream.read(reinterpret_cast<char*>(messageSync.snyc_magic.data()), sizeof(sync_magic));
            LOG_DEBUG("{}: messageSync.snyc_magic[0]: {}", nameId(), messageSync.snyc_magic[0]);
        }
        else if (ulogMsgHeader.msgHeader.msg_type == 'O')
        {
            Ulog::message_dropout_s messageDropout;
            messageDropout.header = ulogMsgHeader.msgHeader;
            filestream.read(reinterpret_cast<char*>(&messageDropout.duration), sizeof(messageDropout.duration));
            LOG_WARN("{}: Dropout of duration: {} ms", nameId(), messageDropout.duration);
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
            LOG_WARN("{}: Message type not identified. Position: {}, The next 100 chars: {}", nameId(), unidentifiedPos, nextChars);

            // Reset read cursor
            filestream.seekg(-100, std::ios_base::cur);
        }
    }
    return nullptr; // FIXME: Quick fix, since 'return obs' only makes sense in the 'data' msg type
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
    LOG_DATA("{}: Information message - key: {}", nameId(), messageInfo.key);
    LOG_DATA("{}: Information message - value: {}", nameId(), messageInfo.value);
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
    LOG_DATA("{}: Information message multi - key_len: {}", nameId(), messageInfoMulti.key_len);
    LOG_DATA("{}: Information message multi - key: {}", nameId(), messageInfoMulti.key);
    LOG_DATA("{}: Information message multi - value: {}", nameId(), messageInfoMulti.value);
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
        LOG_WARN("{}: Parameter message contains invalid data type. It is neither 'int32_t', nor 'float', instead: {}", nameId(), messageParam.key);
    }

    if (messageParam.header.msg_size - 1 - messageParam.key_len < 0)
    {
        LOG_WARN("{}: Parameter msg has key_len: {}", nameId(), messageParam.key_len);
    }
    else
    {
        messageParam.value.resize(static_cast<size_t>(messageParam.header.msg_size - 1 - messageParam.key_len)); // 'msg_size' contains key and value, but not header
        filestream.read(messageParam.value.data(), messageParam.header.msg_size - 1 - messageParam.key_len);
        LOG_DATA("{}: Parameter message - key: {}", nameId(), messageParam.key);
        LOG_DATA("{}: Parameter message - value: {}", nameId(), messageParam.value);
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
    LOG_DEBUG("{}: Parameter default message - key: {}", nameId(), messageParamDefault.key);
    LOG_DEBUG("{}: Parameter default message - value: {}", nameId(), messageParamDefault.value);

    //TODO: Restriction on '1<<0' and '1<<1'
}
