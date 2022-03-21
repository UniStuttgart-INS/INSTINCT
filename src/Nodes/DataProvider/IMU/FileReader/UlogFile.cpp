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

#include <ctime>

// ----------------------------------------------------------- Basic Node Functions --------------------------------------------------------------

NAV::UlogFile::UlogFile()
{
    name = typeStatic();

    LOG_TRACE("{}: called", name);

    _hasConfig = true;
    _guiConfigDefaultWindowSize = { 589, 257 };

    // All message types are polled from the first output pin, but then send out on the correct pin over invokeCallbacks
    nm::CreateOutputPin(this, "ImuObs #1", Pin::Type::Flow, { NAV::ImuObs::type() }, &UlogFile::pollData);
    nm::CreateOutputPin(this, "ImuObs #2", Pin::Type::Flow, { NAV::ImuObs::type() });
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
    if (FileReader::guiConfig(".ulg", { ".ulg" }, size_t(id), nameId()))
    {
        flow::ApplyChanges();
        deinitializeNode();
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

    sensorStartupUTCTime_usec = 0;

    lastGnssTime.timeSinceStartup = 0;

    return FileReader::initialize();
}

void NAV::UlogFile::deinitialize()
{
    LOG_TRACE("{}: called", nameId());

    FileReader::deinitialize();
}

bool NAV::UlogFile::resetNode()
{
    LOG_INFO("{}: called", nameId());

    FileReader::resetReader();

    _epochData.clear();

    return true;
}

// ------------------------------------------------------------ File Reading ---------------------------------------------------------------

NAV::FileReader::FileType NAV::UlogFile::determineFileType()
{
    LOG_TRACE("{}: called", nameId());

    std::filesystem::path filepath = getFilepath();

    auto filestream = std::ifstream(filepath);

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

    LOG_ERROR("{} could not open file", nameId(), filepath);
    return FileType::NONE;
}

void NAV::UlogFile::readHeader()
{
    LOG_TRACE("{}: called", nameId());

    if (_fileType == FileType::BINARY)
    {
        union
        {
            std::array<char, 16> data{};
            Ulog::ulog_Header_s header;
        } ulogHeader{};

        _filestream.read(ulogHeader.data.data(), ulogHeader.data.size());

        // Check "ULog" at beginning of file
        if (!((ulogHeader.header.fileMagic[0] == 'U') && (ulogHeader.header.fileMagic[1] == 'L') && (ulogHeader.header.fileMagic[2] == 'o') && (ulogHeader.header.fileMagic[3] == 'g')))
        {
            LOG_WARN("{}: FileType is binary, but not ULog", nameId());
        }

        LOG_DEBUG("{}: version: {}", nameId(), static_cast<int>(ulogHeader.header.version));

        LOG_DEBUG("{}: time stamp [µs]: {}", nameId(), ulogHeader.header.timeStamp);

        // Read message header
        union
        {
            std::array<char, 3> data{};
            Ulog::message_header_s msgHeader;
        } ulogMsgHeader{};

        while (!((ulogMsgHeader.msgHeader.msg_type == 'A') || (ulogMsgHeader.msgHeader.msg_type == 'L')))
        {
            _filestream.read(ulogMsgHeader.data.data(), ulogMsgHeader.data.size());

            LOG_DATA("{}: msgSize: {},  msgType: {}", nameId(), ulogMsgHeader.msgHeader.msg_size, ulogMsgHeader.msgHeader.msg_type);

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

                _filestream.read(ulogMsgFlagBits.data.data(), ulogMsgFlagBits.data.size() * sizeof(char)); // 'sizeof' is optional here, but it is the solution in general, since data types can be larger than one byte
            }

            // Format definition for a single (composite) type that can be logged or used in another definition as a nested type
            else if (ulogMsgHeader.msgHeader.msg_type == 'F')
            {
                Ulog::message_format_s messageFormat;
                messageFormat.header = ulogMsgHeader.msgHeader;

                LOG_DATA("{}: messageFormat.header.msg_size: {}", nameId(), messageFormat.header.msg_size);

                messageFormat.format.resize(messageFormat.header.msg_size);
                _filestream.read(messageFormat.format.data(), ulogMsgHeader.msgHeader.msg_size);
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
                    _messageFormats.insert_or_assign(msgName, msgDataFields);
                }
                else if (msgName == "sensor_gyro")
                {
                    _messageFormats.insert_or_assign(msgName, msgDataFields);
                }
                else if (msgName == "sensor_mag")
                {
                    _messageFormats.insert_or_assign(msgName, msgDataFields);
                }
                else if (msgName == "vehicle_gps_position")
                {
                    _messageFormats.insert_or_assign(msgName, msgDataFields);
                }
                else if (msgName == "vehicle_attitude")
                {
                    _messageFormats.insert_or_assign(msgName, msgDataFields);
                }
                else if (msgName == "vehicle_air_data")
                {
                    _messageFormats.insert_or_assign(msgName, msgDataFields);
                }
                else if (msgName == "vehicle_control_mode")
                {
                    _messageFormats.insert_or_assign(msgName, msgDataFields);
                }
                else if (msgName == "vehicle_status")
                {
                    _messageFormats.insert_or_assign(msgName, msgDataFields);
                }
                else if (msgName == "cpuload")
                {
                    _messageFormats.insert_or_assign(msgName, msgDataFields);
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
        _filestream.seekg(-3, std::ios_base::cur); // 'msg_size' + 'msg_type' = 3 Byte
        LOG_DEBUG("{}: Read 'Definitions Section' completed", nameId());
    }
}

std::shared_ptr<const NAV::NodeData> NAV::UlogFile::pollData(bool peek)
{
    // Get current position
    auto pollStartPos = _filestream.tellg();
    std::multimap<uint64_t, MeasurementData> peekEpochData;
    if (peek)
    {
        peekEpochData = _epochData;
    }

    // Read message header
    union
    {
        std::array<char, 3> data{};
        Ulog::message_header_s msgHeader;
    } ulogMsgHeader{};

    while (true)
    {
        _filestream.read(ulogMsgHeader.data.data(), ulogMsgHeader.data.size());

        LOG_DATA("{}: msgSize: {}", nameId(), ulogMsgHeader.msgHeader.msg_size);
        LOG_DATA("{}: msgType: {}", nameId(), ulogMsgHeader.msgHeader.msg_type);

        if (ulogMsgHeader.msgHeader.msg_type == 'A')
        {
            Ulog::message_add_logged_s messageAddLog;
            messageAddLog.header = ulogMsgHeader.msgHeader;
            _filestream.read(reinterpret_cast<char*>(&messageAddLog.multi_id), sizeof(messageAddLog.multi_id));
            LOG_DATA("{}: multi_id: {}", nameId(), messageAddLog.multi_id);
            _filestream.read(reinterpret_cast<char*>(&messageAddLog.msg_id), sizeof(messageAddLog.msg_id));
            LOG_DATA("{}: msg_id: {}", nameId(), messageAddLog.msg_id);

            messageAddLog.msg_name.resize(messageAddLog.header.msg_size - 3);
            _filestream.read(messageAddLog.msg_name.data(), messageAddLog.header.msg_size - 3);
            LOG_DATA("{}: messageAddLog.msg_name: {}", nameId(), messageAddLog.msg_name);

            /// Combines (sensor-)message name with an ID that indicates a possible multiple of a sensor
            _subscribedMessages.insert_or_assign(messageAddLog.msg_id, SubscriptionData{ messageAddLog.multi_id, messageAddLog.msg_name });
        }
        else if (ulogMsgHeader.msgHeader.msg_type == 'R')
        {
            Ulog::message_remove_logged_s messageRemoveLog;
            messageRemoveLog.header = ulogMsgHeader.msgHeader;
            _filestream.read(reinterpret_cast<char*>(&messageRemoveLog.msg_id), sizeof(messageRemoveLog.msg_id));
            LOG_DATA("{}: Removed message with 'msg_id': {}", nameId(), messageRemoveLog.msg_id);

            _subscribedMessages.erase(messageRemoveLog.msg_id);
        }
        else if (ulogMsgHeader.msgHeader.msg_type == 'D')
        {
            Ulog::message_data_s messageData;
            messageData.header = ulogMsgHeader.msgHeader;
            _filestream.read(reinterpret_cast<char*>(&messageData.msg_id), sizeof(messageData.msg_id));
            LOG_DATA("{}: msg_id: {}", nameId(), messageData.msg_id);

            messageData.data.resize(messageData.header.msg_size - 2);
            _filestream.read(messageData.data.data(), messageData.header.msg_size - 2);
            LOG_DATA("{}: messageData.header.msg_size: {}", nameId(), messageData.header.msg_size);

            const auto& messageFormat = _messageFormats.at(_subscribedMessages.at(messageData.msg_id).message_name);

            size_t currentExtractLocation = 0;
            if (_subscribedMessages.at(messageData.msg_id).message_name == "sensor_accel")
            {
                SensorAccel sensorAccel{};
                for (const auto& dataField : messageFormat)
                {
                    char* currentData = messageData.data.data() + currentExtractLocation;
                    if (dataField.name == "timestamp")
                    {
                        std::memcpy(&sensorAccel.timestamp, currentData, sizeof(sensorAccel.timestamp));
                        LOG_DATA("{}: sensorAccel.timestamp: {}", nameId(), sensorAccel.timestamp);
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
                    }
                    else if (dataField.name == "y")
                    {
                        std::memcpy(&sensorAccel.y, currentData, sizeof(sensorAccel.y));
                        LOG_DATA("{}: sensorAccel.y: {}", nameId(), sensorAccel.y);
                        currentExtractLocation += sizeof(sensorAccel.y);
                    }
                    else if (dataField.name == "z")
                    {
                        std::memcpy(&sensorAccel.z, currentData, sizeof(sensorAccel.z));
                        LOG_DATA("{}: sensorAccel.z: {}", nameId(), sensorAccel.z);
                        currentExtractLocation += sizeof(sensorAccel.z);
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
                _epochData.insert(std::make_pair(sensorAccel.timestamp,
                                                 MeasurementData{ _subscribedMessages.at(messageData.msg_id).multi_id,
                                                                  _subscribedMessages.at(messageData.msg_id).message_name,
                                                                  sensorAccel }));
            }
            else if (_subscribedMessages.at(messageData.msg_id).message_name == "sensor_gyro")
            {
                SensorGyro sensorGyro{};

                for (const auto& dataField : messageFormat)
                {
                    char* currentData = messageData.data.data() + currentExtractLocation;
                    if (dataField.name == "timestamp")
                    {
                        std::memcpy(&sensorGyro.timestamp, currentData, sizeof(sensorGyro.timestamp));
                        LOG_DATA("{}: sensorGyro.timestamp: {}", nameId(), sensorGyro.timestamp);
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
                _epochData.insert(std::make_pair(sensorGyro.timestamp,
                                                 MeasurementData{ _subscribedMessages.at(messageData.msg_id).multi_id,
                                                                  _subscribedMessages.at(messageData.msg_id).message_name,
                                                                  sensorGyro }));
            }
            else if (_subscribedMessages.at(messageData.msg_id).message_name == "sensor_mag")
            {
                SensorMag sensorMag{};
                for (const auto& dataField : messageFormat)
                {
                    char* currentData = messageData.data.data() + currentExtractLocation;
                    if (dataField.name == "timestamp")
                    {
                        std::memcpy(&sensorMag.timestamp, currentData, sizeof(sensorMag.timestamp));
                        LOG_DATA("{}: sensorMag.timestamp: {}", nameId(), sensorMag.timestamp);
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
                        LOG_DATA("{}: sensorMag: padding", nameId());
                    }
                    else
                    {
                        //FIXME: move 'currentExtractLocation', if yes, how far?
                        LOG_WARN("{}: dataField.name = '{}' or dataField.type = '{}' is unknown", nameId(), dataField.name, dataField.type);
                    }
                }
                _epochData.insert(std::make_pair(sensorMag.timestamp,
                                                 MeasurementData{ _subscribedMessages.at(messageData.msg_id).multi_id,
                                                                  _subscribedMessages.at(messageData.msg_id).message_name,
                                                                  sensorMag }));
            }
            else if (_subscribedMessages.at(messageData.msg_id).message_name == "vehicle_gps_position")
            {
                VehicleGpsPosition vehicleGpsPosition{};
                for (const auto& dataField : messageFormat)
                {
                    char* currentData = messageData.data.data() + currentExtractLocation;
                    if (dataField.name == "timestamp")
                    {
                        std::memcpy(&vehicleGpsPosition.timestamp, currentData, sizeof(vehicleGpsPosition.timestamp));
                        LOG_DATA("{}: vehicleGpsPosition.timestamp: {}", nameId(), vehicleGpsPosition.timestamp);
                        currentExtractLocation += sizeof(vehicleGpsPosition.timestamp);
                    }
                    else if (dataField.name == "time_utc_usec")
                    {
                        std::memcpy(&vehicleGpsPosition.time_utc_usec, currentData, sizeof(vehicleGpsPosition.time_utc_usec));
                        LOG_DATA("{}: vehicleGpsPosition.time_utc_usec: {}", nameId(), vehicleGpsPosition.time_utc_usec);
                        currentExtractLocation += sizeof(vehicleGpsPosition.time_utc_usec);
                    }
                    else if (dataField.name == "lat")
                    {
                        std::memcpy(&vehicleGpsPosition.lat, currentData, sizeof(vehicleGpsPosition.lat));
                        LOG_DATA("{}: vehicleGpsPosition.lat: {}", nameId(), vehicleGpsPosition.lat);
                        currentExtractLocation += sizeof(vehicleGpsPosition.lat);
                    }
                    else if (dataField.name == "lon")
                    {
                        std::memcpy(&vehicleGpsPosition.lon, currentData, sizeof(vehicleGpsPosition.lon));
                        LOG_DATA("{}: vehicleGpsPosition.lon: {}", nameId(), vehicleGpsPosition.lon);
                        currentExtractLocation += sizeof(vehicleGpsPosition.lon);
                    }
                    else if (dataField.name == "alt")
                    {
                        std::memcpy(&vehicleGpsPosition.alt, currentData, sizeof(vehicleGpsPosition.alt));
                        LOG_DATA("{}: vehicleGpsPosition.alt: {}", nameId(), vehicleGpsPosition.alt);
                        currentExtractLocation += sizeof(vehicleGpsPosition.alt);
                    }
                    else if (dataField.name == "alt_ellipsoid")
                    {
                        std::memcpy(&vehicleGpsPosition.alt_ellipsoid, currentData, sizeof(vehicleGpsPosition.alt_ellipsoid));
                        LOG_DATA("{}: vehicleGpsPosition.alt_ellipsoid: {}", nameId(), vehicleGpsPosition.alt_ellipsoid);
                        currentExtractLocation += sizeof(vehicleGpsPosition.alt_ellipsoid);
                    }
                    else if (dataField.name == "s_variance_m_s")
                    {
                        std::memcpy(&vehicleGpsPosition.s_variance_m_s, currentData, sizeof(vehicleGpsPosition.s_variance_m_s));
                        LOG_DATA("{}: vehicleGpsPosition.s_variance_m_s: {}", nameId(), vehicleGpsPosition.s_variance_m_s);
                        currentExtractLocation += sizeof(vehicleGpsPosition.s_variance_m_s);
                    }
                    else if (dataField.name == "c_variance_rad")
                    {
                        std::memcpy(&vehicleGpsPosition.c_variance_rad, currentData, sizeof(vehicleGpsPosition.c_variance_rad));
                        LOG_DATA("{}: vehicleGpsPosition.c_variance_rad: {}", nameId(), vehicleGpsPosition.c_variance_rad);
                        currentExtractLocation += sizeof(vehicleGpsPosition.c_variance_rad);
                    }
                    else if (dataField.name == "eph")
                    {
                        std::memcpy(&vehicleGpsPosition.eph, currentData, sizeof(vehicleGpsPosition.eph));
                        LOG_DATA("{}: vehicleGpsPosition.eph: {}", nameId(), vehicleGpsPosition.eph);
                        currentExtractLocation += sizeof(vehicleGpsPosition.eph);
                    }
                    else if (dataField.name == "epv")
                    {
                        std::memcpy(&vehicleGpsPosition.epv, currentData, sizeof(vehicleGpsPosition.epv));
                        LOG_DATA("{}: vehicleGpsPosition.epv: {}", nameId(), vehicleGpsPosition.epv);
                        currentExtractLocation += sizeof(vehicleGpsPosition.epv);
                    }
                    else if (dataField.name == "hdop")
                    {
                        std::memcpy(&vehicleGpsPosition.hdop, currentData, sizeof(vehicleGpsPosition.hdop));
                        LOG_DATA("{}: vehicleGpsPosition.hdop: {}", nameId(), vehicleGpsPosition.hdop);
                        currentExtractLocation += sizeof(vehicleGpsPosition.hdop);
                    }
                    else if (dataField.name == "vdop")
                    {
                        std::memcpy(&vehicleGpsPosition.vdop, currentData, sizeof(vehicleGpsPosition.lat));
                        LOG_DATA("{}: vehicleGpsPosition.vdop: {}", nameId(), vehicleGpsPosition.vdop);
                        currentExtractLocation += sizeof(vehicleGpsPosition.vdop);
                    }
                    else if (dataField.name == "noise_per_ms")
                    {
                        std::memcpy(&vehicleGpsPosition.noise_per_ms, currentData, sizeof(vehicleGpsPosition.noise_per_ms));
                        LOG_DATA("{}: vehicleGpsPosition.noise_per_ms: {}", nameId(), vehicleGpsPosition.noise_per_ms);
                        currentExtractLocation += sizeof(vehicleGpsPosition.noise_per_ms);
                    }
                    else if (dataField.name == "jamming_indicator")
                    {
                        std::memcpy(&vehicleGpsPosition.jamming_indicator, currentData, sizeof(vehicleGpsPosition.jamming_indicator));
                        LOG_DATA("{}: vehicleGpsPosition.jamming_indicator: {}", nameId(), vehicleGpsPosition.jamming_indicator);
                        currentExtractLocation += sizeof(vehicleGpsPosition.jamming_indicator);
                    }
                    else if (dataField.name == "vel_m_s")
                    {
                        std::memcpy(&vehicleGpsPosition.vel_m_s, currentData, sizeof(vehicleGpsPosition.vel_m_s));
                        LOG_DATA("{}: vehicleGpsPosition.vel_m_s: {}", nameId(), vehicleGpsPosition.vel_m_s);
                        currentExtractLocation += sizeof(vehicleGpsPosition.vel_m_s);
                    }
                    else if (dataField.name == "vel_n_m_s")
                    {
                        std::memcpy(&vehicleGpsPosition.vel_n_m_s, currentData, sizeof(vehicleGpsPosition.vel_n_m_s));
                        LOG_DATA("{}: vehicleGpsPosition.vel_n_m_s: {}", nameId(), vehicleGpsPosition.vel_n_m_s);
                        currentExtractLocation += sizeof(vehicleGpsPosition.vel_n_m_s);
                    }
                    else if (dataField.name == "vel_e_m_s")
                    {
                        std::memcpy(&vehicleGpsPosition.vel_e_m_s, currentData, sizeof(vehicleGpsPosition.vel_e_m_s));
                        LOG_DATA("{}: vehicleGpsPosition.vel_e_m_s: {}", nameId(), vehicleGpsPosition.vel_e_m_s);
                        currentExtractLocation += sizeof(vehicleGpsPosition.vel_e_m_s);
                    }
                    else if (dataField.name == "vel_d_m_s")
                    {
                        std::memcpy(&vehicleGpsPosition.vel_d_m_s, currentData, sizeof(vehicleGpsPosition.vel_d_m_s));
                        LOG_DATA("{}: vehicleGpsPosition.vel_d_m_s: {}", nameId(), vehicleGpsPosition.vel_d_m_s);
                        currentExtractLocation += sizeof(vehicleGpsPosition.vel_d_m_s);
                    }
                    else if (dataField.name == "cog_rad")
                    {
                        std::memcpy(&vehicleGpsPosition.cog_rad, currentData, sizeof(vehicleGpsPosition.cog_rad));
                        LOG_DATA("{}: vehicleGpsPosition.cog_rad: {}", nameId(), vehicleGpsPosition.cog_rad);
                        currentExtractLocation += sizeof(vehicleGpsPosition.cog_rad);
                    }
                    else if (dataField.name == "timestamp_time_relative")
                    {
                        std::memcpy(&vehicleGpsPosition.timestamp_time_relative, currentData, sizeof(vehicleGpsPosition.timestamp_time_relative));
                        LOG_DATA("{}: vehicleGpsPosition.timestamp_time_relative: {}", nameId(), vehicleGpsPosition.timestamp_time_relative);
                        currentExtractLocation += sizeof(vehicleGpsPosition.timestamp_time_relative);
                    }
                    else if (dataField.name == "heading")
                    {
                        std::memcpy(&vehicleGpsPosition.heading, currentData, sizeof(vehicleGpsPosition.heading));
                        LOG_DATA("{}: vehicleGpsPosition.heading: {}", nameId(), vehicleGpsPosition.heading);
                        currentExtractLocation += sizeof(vehicleGpsPosition.heading);
                    }
                    else if (dataField.name == "heading_offset")
                    {
                        std::memcpy(&vehicleGpsPosition.heading_offset, currentData, sizeof(vehicleGpsPosition.heading_offset));
                        LOG_DATA("{}: vehicleGpsPosition.heading_offset: {}", nameId(), vehicleGpsPosition.heading_offset);
                        currentExtractLocation += sizeof(vehicleGpsPosition.heading_offset);
                    }
                    else if (dataField.name == "fix_type")
                    {
                        std::memcpy(&vehicleGpsPosition.fix_type, currentData, sizeof(vehicleGpsPosition.fix_type));
                        LOG_DATA("{}: vehicleGpsPosition.fix_type: {}", nameId(), vehicleGpsPosition.fix_type);
                        currentExtractLocation += sizeof(vehicleGpsPosition.fix_type);
                    }
                    else if (dataField.name == "vel_ned_valid")
                    {
                        std::memcpy(&vehicleGpsPosition.vel_ned_valid, currentData, sizeof(vehicleGpsPosition.vel_ned_valid));
                        LOG_DATA("{}: vehicleGpsPosition.vel_ned_valid: {}", nameId(), vehicleGpsPosition.vel_ned_valid);
                        currentExtractLocation += sizeof(vehicleGpsPosition.vel_ned_valid);
                    }
                    else if (dataField.name == "satellites_used")
                    {
                        std::memcpy(&vehicleGpsPosition.satellites_used, currentData, sizeof(vehicleGpsPosition.satellites_used));
                        LOG_DATA("{}: vehicleGpsPosition.satellites_used: {}", nameId(), vehicleGpsPosition.satellites_used);
                        currentExtractLocation += sizeof(vehicleGpsPosition.satellites_used);
                    }
                    else if (dataField.name.compare(0, 7, "_padding")) // e.g. '_padding0', '_padding1'
                    {
                        currentExtractLocation += VehicleGpsPosition::padding; // Extraction Location should be moved to account for multiple padding
                        LOG_DATA("{}: vehicleGpsPosition: padding", nameId());
                    }
                    else
                    {
                        //FIXME: move 'currentExtractLocation', if yes, how far?
                        LOG_WARN("{}: dataField.name = '{}' or dataField.type = '{}' is unknown", nameId(), dataField.name, dataField.type);
                    }
                }

                lastGnssTime.gnssTime = InsTime(0, 0, 0, 0, 0, vehicleGpsPosition.time_utc_usec * 1e-6L);
                lastGnssTime.timeSinceStartup = vehicleGpsPosition.timestamp;

                while (true) // Delete all old VehicleGpsPosition entries
                {
                    auto iter = std::find_if(_epochData.begin(), _epochData.end(), [](const std::pair<uint64_t, MeasurementData>& v) {
                        return std::holds_alternative<UlogFile::VehicleGpsPosition>(v.second.data);
                    });
                    if (iter == _epochData.end())
                    {
                        break;
                    }
                    _epochData.erase(iter);
                }

                _epochData.insert(std::make_pair(vehicleGpsPosition.timestamp,
                                                 MeasurementData{ _subscribedMessages.at(messageData.msg_id).multi_id,
                                                                  _subscribedMessages.at(messageData.msg_id).message_name,
                                                                  vehicleGpsPosition }));
            }
            else if (_subscribedMessages.at(messageData.msg_id).message_name == "vehicle_attitude")
            {
                VehicleAttitude vehicleAttitude{};
                for (const auto& dataField : messageFormat)
                {
                    char* currentData = messageData.data.data() + currentExtractLocation;
                    if (dataField.name == "timestamp")
                    {
                        std::memcpy(&vehicleAttitude.timestamp, currentData, sizeof(vehicleAttitude.timestamp));
                        LOG_DATA("{}: vehicleAttitude.timestamp: {}", nameId(), vehicleAttitude.timestamp);
                        currentExtractLocation += sizeof(vehicleAttitude.timestamp);
                    }
                    else if (dataField.name == "q")
                    {
                        std::memcpy(vehicleAttitude.q.data(), currentData, vehicleAttitude.q.size());
                        LOG_DATA("{}: vehicleAttitude.q: {}", nameId(), fmt::join(vehicleAttitude.q, ", "));
                        currentExtractLocation += vehicleAttitude.q.size();
                    }
                    else if (dataField.name == "delta_q_reset")
                    {
                        std::memcpy(vehicleAttitude.delta_q_reset.data(), currentData, vehicleAttitude.delta_q_reset.size());
                        LOG_DATA("{}: vehicleAttitude.delta_q_reset: {}", nameId(), fmt::join(vehicleAttitude.delta_q_reset, ", "));
                        currentExtractLocation += vehicleAttitude.delta_q_reset.size();
                    }
                    if (dataField.name == "quat_reset_counter")
                    {
                        std::memcpy(&vehicleAttitude.quat_reset_counter, currentData, sizeof(vehicleAttitude.quat_reset_counter));
                        LOG_DATA("{}: vehicleAttitude.quat_reset_counter: {}", nameId(), vehicleAttitude.quat_reset_counter);
                        currentExtractLocation += sizeof(vehicleAttitude.quat_reset_counter);
                    }
                    else if (dataField.name.compare(0, 7, "_padding")) // e.g. '_padding0', '_padding1'
                    {
                        currentExtractLocation += VehicleAttitude::padding; // Extraction Location should be moved to account for multiple padding
                        LOG_DATA("{}: VehicleAttitude: padding", nameId());
                    }
                    else
                    {
                        //FIXME: move 'currentExtractLocation', if yes, how far?
                        LOG_WARN("{}: dataField.name = '{}' or dataField.type = '{}' is unknown", nameId(), dataField.name, dataField.type);
                    }
                }

                while (true) // Delete all old VehicleAttitude entries
                {
                    auto iter = std::find_if(_epochData.begin(), _epochData.end(), [](const std::pair<uint64_t, MeasurementData>& v) {
                        return std::holds_alternative<UlogFile::VehicleAttitude>(v.second.data);
                    });
                    if (iter == _epochData.end())
                    {
                        break;
                    }
                    _epochData.erase(iter);
                }

                _epochData.insert(std::make_pair(vehicleAttitude.timestamp,
                                                 MeasurementData{ _subscribedMessages.at(messageData.msg_id).multi_id,
                                                                  _subscribedMessages.at(messageData.msg_id).message_name,
                                                                  vehicleAttitude }));
            }
            // else if (_subscribedMessages.at(messageData.msg_id).message_name == "vehicle_control_mode")
            // {
            //     VehicleControlMode vehicleControlMode{};
            //     for (const auto& dataField : messageFormat)
            //     {
            //         char* currentData = messageData.data.data() + currentExtractLocation;
            //         if (dataField.name == "timestamp")
            //         {
            //             std::memcpy(&vehicleControlMode.timestamp, currentData, sizeof(vehicleControlMode.timestamp));
            //             LOG_DATA("{}: vehicleControlMode.timestamp: {}", nameId(), vehicleControlMode.timestamp);
            //             currentExtractLocation += sizeof(vehicleControlMode.timestamp);
            //         }
            //         else if (dataField.name == "flag_armed")
            //         {
            //             std::memcpy(&vehicleControlMode.flag_armed, currentData, sizeof(vehicleControlMode.flag_armed));
            //             LOG_DATA("{}: vehicleControlMode.flag_armed: {}", nameId(), vehicleControlMode.flag_armed);
            //             currentExtractLocation += sizeof(vehicleControlMode.flag_armed);
            //         }
            //         else if (dataField.name == "flag_external_manual_override_ok")
            //         {
            //             std::memcpy(&vehicleControlMode.flag_external_manual_override_ok, currentData, sizeof(vehicleControlMode.flag_external_manual_override_ok));
            //             LOG_DATA("{}: vehicleControlMode.flag_external_manual_override_ok: {}", nameId(), vehicleControlMode.flag_external_manual_override_ok);
            //             currentExtractLocation += sizeof(vehicleControlMode.flag_external_manual_override_ok);
            //         }
            //         else if (dataField.name == "flag_control_manual_enabled")
            //         {
            //             std::memcpy(&vehicleControlMode.flag_control_manual_enabled, currentData, sizeof(vehicleControlMode.flag_control_manual_enabled));
            //             LOG_DATA("{}: vehicleControlMode.flag_control_manual_enabled: {}", nameId(), vehicleControlMode.flag_control_manual_enabled);
            //             currentExtractLocation += sizeof(vehicleControlMode.flag_control_manual_enabled);
            //         }
            //         else if (dataField.name == "flag_control_auto_enabled")
            //         {
            //             std::memcpy(&vehicleControlMode.flag_control_auto_enabled, currentData, sizeof(vehicleControlMode.flag_control_auto_enabled));
            //             LOG_DATA("{}: vehicleControlMode.flag_control_auto_enabled: {}", nameId(), vehicleControlMode.flag_control_auto_enabled);
            //             currentExtractLocation += sizeof(vehicleControlMode.flag_control_auto_enabled);
            //         }
            //         else if (dataField.name == "flag_control_offboard_enabled")
            //         {
            //             std::memcpy(&vehicleControlMode.flag_control_offboard_enabled, currentData, sizeof(vehicleControlMode.flag_control_offboard_enabled));
            //             LOG_DATA("{}: vehicleControlMode.flag_control_offboard_enabled: {}", nameId(), vehicleControlMode.flag_control_offboard_enabled);
            //             currentExtractLocation += sizeof(vehicleControlMode.flag_control_offboard_enabled);
            //         }
            //         else if (dataField.name == "flag_control_rates_enabled")
            //         {
            //             std::memcpy(&vehicleControlMode.flag_control_rates_enabled, currentData, sizeof(vehicleControlMode.flag_control_rates_enabled));
            //             LOG_DATA("{}: vehicleControlMode.flag_control_rates_enabled: {}", nameId(), vehicleControlMode.flag_control_rates_enabled);
            //             currentExtractLocation += sizeof(vehicleControlMode.flag_control_rates_enabled);
            //         }
            //         else if (dataField.name == "flag_control_attitude_enabled")
            //         {
            //             std::memcpy(&vehicleControlMode.flag_control_attitude_enabled, currentData, sizeof(vehicleControlMode.flag_control_attitude_enabled));
            //             LOG_DATA("{}: vehicleControlMode.flag_control_attitude_enabled: {}", nameId(), vehicleControlMode.flag_control_attitude_enabled);
            //             currentExtractLocation += sizeof(vehicleControlMode.flag_control_attitude_enabled);
            //         }
            //         else if (dataField.name == "flag_control_yawrate_override_enabled")
            //         {
            //             std::memcpy(&vehicleControlMode.flag_control_yawrate_override_enabled, currentData, sizeof(vehicleControlMode.flag_control_yawrate_override_enabled));
            //             LOG_DATA("{}: vehicleControlMode.flag_control_yawrate_override_enabled: {}", nameId(), vehicleControlMode.flag_control_yawrate_override_enabled);
            //             currentExtractLocation += sizeof(vehicleControlMode.flag_control_yawrate_override_enabled);
            //         }
            //         else if (dataField.name == "flag_control_rattitude_enabled")
            //         {
            //             std::memcpy(&vehicleControlMode.flag_control_rattitude_enabled, currentData, sizeof(vehicleControlMode.flag_control_rattitude_enabled));
            //             LOG_DATA("{}: vehicleControlMode.flag_control_rattitude_enabled: {}", nameId(), vehicleControlMode.flag_control_rattitude_enabled);
            //             currentExtractLocation += sizeof(vehicleControlMode.flag_control_rattitude_enabled);
            //         }
            //         else if (dataField.name == "flag_control_force_enabled")
            //         {
            //             std::memcpy(&vehicleControlMode.flag_control_force_enabled, currentData, sizeof(vehicleControlMode.flag_control_force_enabled));
            //             LOG_DATA("{}: vehicleControlMode.flag_control_force_enabled: {}", nameId(), vehicleControlMode.flag_control_force_enabled);
            //             currentExtractLocation += sizeof(vehicleControlMode.flag_control_force_enabled);
            //         }
            //         else if (dataField.name == "flag_control_acceleration_enabled")
            //         {
            //             std::memcpy(&vehicleControlMode.flag_control_acceleration_enabled, currentData, sizeof(vehicleControlMode.flag_control_acceleration_enabled));
            //             LOG_DATA("{}: vehicleControlMode.flag_control_acceleration_enabled: {}", nameId(), vehicleControlMode.flag_control_acceleration_enabled);
            //             currentExtractLocation += sizeof(vehicleControlMode.flag_control_acceleration_enabled);
            //         }
            //         else if (dataField.name == "flag_control_velocity_enabled")
            //         {
            //             std::memcpy(&vehicleControlMode.flag_control_velocity_enabled, currentData, sizeof(vehicleControlMode.flag_control_velocity_enabled));
            //             LOG_DATA("{}: vehicleControlMode.flag_control_velocity_enabled: {}", nameId(), vehicleControlMode.flag_control_velocity_enabled);
            //             currentExtractLocation += sizeof(vehicleControlMode.flag_control_velocity_enabled);
            //         }
            //         else if (dataField.name == "flag_control_position_enabled")
            //         {
            //             std::memcpy(&vehicleControlMode.flag_control_position_enabled, currentData, sizeof(vehicleControlMode.flag_control_position_enabled));
            //             LOG_DATA("{}: vehicleControlMode.flag_control_position_enabled: {}", nameId(), vehicleControlMode.flag_control_position_enabled);
            //             currentExtractLocation += sizeof(vehicleControlMode.flag_control_position_enabled);
            //         }
            //         else if (dataField.name == "flag_control_altitude_enabled")
            //         {
            //             std::memcpy(&vehicleControlMode.flag_control_altitude_enabled, currentData, sizeof(vehicleControlMode.flag_control_altitude_enabled));
            //             LOG_DATA("{}: vehicleControlMode.flag_control_altitude_enabled: {}", nameId(), vehicleControlMode.flag_control_altitude_enabled);
            //             currentExtractLocation += sizeof(vehicleControlMode.flag_control_altitude_enabled);
            //         }
            //         else if (dataField.name == "flag_control_climb_rate_enabled")
            //         {
            //             std::memcpy(&vehicleControlMode.flag_control_climb_rate_enabled, currentData, sizeof(vehicleControlMode.flag_control_climb_rate_enabled));
            //             LOG_DATA("{}: vehicleControlMode.flag_control_climb_rate_enabled: {}", nameId(), vehicleControlMode.flag_control_climb_rate_enabled);
            //             currentExtractLocation += sizeof(vehicleControlMode.flag_control_climb_rate_enabled);
            //         }
            //         else if (dataField.name == "flag_control_termination_enabled")
            //         {
            //             std::memcpy(&vehicleControlMode.flag_control_termination_enabled, currentData, sizeof(vehicleControlMode.flag_control_termination_enabled));
            //             LOG_DATA("{}: vehicleControlMode.flag_control_termination_enabled: {}", nameId(), vehicleControlMode.flag_control_termination_enabled);
            //             currentExtractLocation += sizeof(vehicleControlMode.flag_control_termination_enabled);
            //         }
            //         else if (dataField.name == "flag_control_fixed_hdg_enabled")
            //         {
            //             std::memcpy(&vehicleControlMode.flag_control_fixed_hdg_enabled, currentData, sizeof(vehicleControlMode.flag_control_fixed_hdg_enabled));
            //             LOG_DATA("{}: vehicleControlMode.flag_control_fixed_hdg_enabled: {}", nameId(), vehicleControlMode.flag_control_fixed_hdg_enabled);
            //             currentExtractLocation += sizeof(vehicleControlMode.flag_control_fixed_hdg_enabled);
            //         }
            //         else if (dataField.name.compare(0, 7, "_padding")) // e.g. '_padding0', '_padding1'
            //         {
            //             currentExtractLocation += VehicleControlMode::padding; // Extraction Location should be moved to account for multiple padding
            //             LOG_DATA("{}: VehicleControlMode: padding", nameId());
            //         }
            //         else
            //         {
            //             //FIXME: move 'currentExtractLocation', if yes, how far?
            //             LOG_WARN("{}: dataField.name = '{}' or dataField.type = '{}' is unknown", nameId(), dataField.name, dataField.type);
            //         }
            //     }
            //     //TODO: insert to _epochData necessary for 'vehicleControlMode'?
            // }
            // else if (_subscribedMessages.at(messageData.msg_id).message_name == "vehicle_air_data")
            // {
            //     VehicleAirData vehicleAirData{};
            //     for (const auto& dataField : messageFormat)
            //     {
            //         char* currentData = messageData.data.data() + currentExtractLocation;
            //         if (dataField.name == "timestamp")
            //         {
            //             std::memcpy(&vehicleAirData.timestamp, currentData, sizeof(vehicleAirData.timestamp));
            //             LOG_DATA("{}: vehicleAirData.timestamp: {}", nameId(), vehicleAirData.timestamp);
            //             currentExtractLocation += sizeof(vehicleAirData.timestamp);
            //         }
            //         else if (dataField.name == "timestamp_sample")
            //         {
            //             std::memcpy(&vehicleAirData.timestamp_sample, currentData, sizeof(vehicleAirData.timestamp_sample));
            //             LOG_DATA("{}: vehicleAirData.timestamp_sample: {}", nameId(), vehicleAirData.timestamp_sample);
            //             currentExtractLocation += sizeof(vehicleAirData.timestamp_sample);
            //         }
            //         else if (dataField.name == "baro_device_id")
            //         {
            //             std::memcpy(&vehicleAirData.baro_device_id, currentData, sizeof(vehicleAirData.baro_device_id));
            //             LOG_DATA("{}: vehicleAirData.baro_device_id: {}", nameId(), vehicleAirData.baro_device_id);
            //             currentExtractLocation += sizeof(vehicleAirData.baro_device_id);
            //         }
            //         else if (dataField.name == "baro_alt_meter")
            //         {
            //             std::memcpy(&vehicleAirData.baro_alt_meter, currentData, sizeof(vehicleAirData.baro_alt_meter));
            //             LOG_DATA("{}: vehicleAirData.baro_alt_meter: {}", nameId(), vehicleAirData.baro_alt_meter);
            //             currentExtractLocation += sizeof(vehicleAirData.baro_alt_meter);
            //         }
            //         else if (dataField.name == "baro_temp_celcius")
            //         {
            //             std::memcpy(&vehicleAirData.baro_temp_celcius, currentData, sizeof(vehicleAirData.baro_temp_celcius));
            //             LOG_DATA("{}: vehicleAirData.baro_temp_celcius: {}", nameId(), vehicleAirData.baro_temp_celcius);
            //             currentExtractLocation += sizeof(vehicleAirData.baro_temp_celcius);
            //         }
            //         else if (dataField.name == "baro_pressure_pa")
            //         {
            //             std::memcpy(&vehicleAirData.baro_pressure_pa, currentData, sizeof(vehicleAirData.baro_pressure_pa));
            //             LOG_DATA("{}: vehicleAirData.baro_pressure_pa: {}", nameId(), vehicleAirData.baro_pressure_pa);
            //             currentExtractLocation += sizeof(vehicleAirData.baro_pressure_pa);
            //         }
            //         else if (dataField.name == "rho")
            //         {
            //             std::memcpy(&vehicleAirData.rho, currentData, sizeof(vehicleAirData.rho));
            //             LOG_DATA("{}: vehicleAirData.rho: {}", nameId(), vehicleAirData.rho);
            //             currentExtractLocation += sizeof(vehicleAirData.rho);
            //         }
            //         else if (dataField.name.compare(0, 7, "_padding")) // e.g. '_padding0', '_padding1'
            //         {
            //             currentExtractLocation += VehicleAirData::padding; // Extraction Location should be moved to account for multiple padding
            //             LOG_DATA("{}: VehicleControlMode: padding", nameId());
            //         }
            //         else
            //         {
            //             //FIXME: move 'currentExtractLocation', if yes, how far?
            //             LOG_WARN("{}: dataField.name = '{}' or dataField.type = '{}' is unknown", nameId(), dataField.name, dataField.type);
            //         }
            //     }
            //     //TODO: insert to _epochData necessary for 'vehicleAirData'?
            // }
            else
            {
                LOG_DATA("{}: UKNOWN: _subscribedMessages.at(messageData.msg_id).message_name = {}", nameId(), _subscribedMessages.at(messageData.msg_id).message_name);
            }

            // #########################################################################################################################################
            //                                                                Callbacks
            // #########################################################################################################################################
            // This is the hasEnoughData check for an ImuObs
            if (auto multi_id = enoughImuDataAvailable();
                multi_id >= 0)
            {
                LOG_DATA("{}: Construct ImuObs and invoke callback", nameId());

                auto obs = std::make_shared<ImuObs>(this->_imuPos);

                uint64_t timeSinceStartupNew{};

                // Construct ImuObs
                for (auto it = _epochData.begin(); it != _epochData.end();)
                {
                    // Add accel data to ImuObs
                    if (std::holds_alternative<SensorAccel>(it->second.data) && (it->second.multi_id == static_cast<uint8_t>(multi_id))
                        && !obs->accelUncompXYZ.has_value())
                    {
                        timeSinceStartupNew = it->first;
                        float accelX = std::get<SensorAccel>(it->second.data).x;
                        float accelY = std::get<SensorAccel>(it->second.data).y;
                        float accelZ = std::get<SensorAccel>(it->second.data).z;
                        obs->accelUncompXYZ.emplace(accelX, accelY, accelZ);
                        auto delIt = it;
                        ++it;
                        _epochData.erase(delIt);
                        LOG_DATA("{}: accelX = {}, accelY = {}, accelZ = {}", nameId(), accelX, accelY, accelZ);
                    }
                    // Add gyro data to ImuObs
                    else if (std::holds_alternative<SensorGyro>(it->second.data) && (it->second.multi_id == static_cast<uint8_t>(multi_id))
                             && !obs->gyroUncompXYZ.has_value())
                    {
                        timeSinceStartupNew = it->first;
                        float gyroX = std::get<SensorGyro>(it->second.data).x;
                        float gyroY = std::get<SensorGyro>(it->second.data).y;
                        float gyroZ = std::get<SensorGyro>(it->second.data).z;
                        obs->gyroUncompXYZ.emplace(gyroX, gyroY, gyroZ);
                        auto delIt = it;
                        ++it;
                        _epochData.erase(delIt);
                        LOG_DATA("{}: gyroX = {}, gyroY = {}, gyroZ = {}", nameId(), gyroX, gyroY, gyroZ);
                    }
                    // Add mag data to ImuObs
                    else if (std::holds_alternative<SensorMag>(it->second.data) && (it->second.multi_id == static_cast<uint8_t>(multi_id))
                             && !obs->magUncompXYZ.has_value()) // TODO: Find out what is multi_id = 1. Px4 Mini is supposed to have only one magnetometer
                    {
                        timeSinceStartupNew = it->first;
                        float magX = std::get<SensorMag>(it->second.data).x;
                        float magY = std::get<SensorMag>(it->second.data).y;
                        float magZ = std::get<SensorMag>(it->second.data).z;
                        obs->magUncompXYZ.emplace(magX, magY, magZ);
                        auto delIt = it;
                        ++it;
                        _epochData.erase(delIt);
                        LOG_DATA("{}: magX = {}, magY = {}, magZ = {}", nameId(), magX, magY, magZ);
                    }
                    else
                    {
                        ++it;
                    }
                }

                obs->insTime = lastGnssTime.gnssTime + std::chrono::microseconds(static_cast<int64_t>(timeSinceStartupNew) - static_cast<int64_t>(lastGnssTime.timeSinceStartup));

                obs->timeSinceStartup = 1000 * timeSinceStartupNew; // latest timestamp in [ns]
                LOG_DATA("{}: *obs->timeSinceStartup = {} s", nameId(), static_cast<double>(*obs->timeSinceStartup) * 1e-9);

                if (!peek)
                {
                    if (multi_id == 0)
                    {
                        invokeCallbacks(OUTPUT_PORT_INDEX_IMUOBS_1, obs);
                    }
                    else if (multi_id == 1)
                    {
                        invokeCallbacks(OUTPUT_PORT_INDEX_IMUOBS_2, obs);
                    }
                    else
                    {
                        LOG_ERROR("{}: multi_id = {} is invalid", nameId(), multi_id);
                    }
                }
                else
                {
                    // Return to position before "Read line".
                    _filestream.seekg(pollStartPos, std::ios_base::beg);
                    _epochData = peekEpochData;
                }
                return obs;
            }
            if (auto [gpsIter, attIter] = findPosVelAttData();
                gpsIter != _epochData.end() && attIter != _epochData.end())
            {
                LOG_DATA("{}: Construct PosVelAtt and invoke callback", nameId());

                auto obs = std::make_shared<NAV::PosVelAtt>();

                const auto& vehicleGpsPosition = std::get<VehicleGpsPosition>(gpsIter->second.data);
                const auto& vehicleAttitude = std::get<VehicleAttitude>(attIter->second.data);

                obs->insTime.emplace(0, 0, 0, 0, 0, vehicleGpsPosition.time_utc_usec * 1e-6L);
                obs->setState_n(Eigen::Vector3d{ trafo::deg2rad(1e-7 * static_cast<double>(vehicleGpsPosition.lat)), trafo::deg2rad(1e-7 * static_cast<double>(vehicleGpsPosition.lon)), 1e-3 * (static_cast<double>(vehicleGpsPosition.alt_ellipsoid)) },
                                Eigen::Vector3d{ vehicleGpsPosition.vel_n_m_s, vehicleGpsPosition.vel_e_m_s, vehicleGpsPosition.vel_d_m_s },
                                Eigen::Quaterniond{ vehicleAttitude.q.at(0), vehicleAttitude.q.at(1), vehicleAttitude.q.at(2), vehicleAttitude.q.at(3) });
                // TODO: Check order of w,x,y,z
                // TODO: Check if this is quaternion_nb

                // Above it is ensured that only one gps and att element is present.
                // Here we still need to delete the used elements, otherwise the next iteration would find the elements again.
                _epochData.erase(gpsIter);
                _epochData.erase(attIter);

                if (!peek)
                {
                    invokeCallbacks(OUTPUT_PORT_INDEX_POSVELATT, obs);
                }
                else
                {
                    // Return to position before "Read line".
                    _filestream.seekg(pollStartPos, std::ios_base::beg);
                    _epochData = peekEpochData;
                }
                return obs;
            }
        }
        else if (ulogMsgHeader.msgHeader.msg_type == 'L')
        {
            Ulog::message_logging_s messageLog;
            messageLog.header = ulogMsgHeader.msgHeader;
            _filestream.read(reinterpret_cast<char*>(&messageLog.log_level), sizeof(messageLog.log_level));

            if (messageLog.log_level == 48) // '0'
            {
                LOG_DATA("{}: Log-level: EMERG - System is unusable", nameId());
            }
            else if (messageLog.log_level == 49) // '1'
            {
                LOG_DATA("{}: Log-level: ALERT - Action must be taken immediately", nameId());
            }
            else if (messageLog.log_level == 50) // '2'
            {
                LOG_DATA("{}: Log-level: CRIT - Critical conditions", nameId());
            }
            else if (messageLog.log_level == 51) // '3'
            {
                LOG_DATA("{}: Log-level: ERR - Error conditions", nameId());
            }
            else if (messageLog.log_level == 52) // '4'
            {
                LOG_DATA("{}: Log-level: WARNING - Warning conditions", nameId());
            }
            else if (messageLog.log_level == 53) // '5'
            {
                LOG_DATA("{}: Log-level: NOTICE - Normal but significant condition", nameId());
            }
            else if (messageLog.log_level == 54) // '6'
            {
                LOG_DATA("{}: Log-level: INFO - Informational", nameId());
            }
            else if (messageLog.log_level == 55) // '7'
            {
                LOG_DATA("{}: Log-level: DEBUG - Debug-level messages", nameId());
            }
            else
            {
                LOG_WARN("{}: Log-level is out of scope ({}) - possible data loss", nameId(), messageLog.log_level);
            }

            _filestream.read(reinterpret_cast<char*>(&messageLog.timestamp), sizeof(messageLog.timestamp));
            LOG_DATA("{}: messageLog.timestamp [µs]: {}", nameId(), messageLog.timestamp);

            messageLog.message.resize(messageLog.header.msg_size - 9);
            _filestream.read(messageLog.message.data(), messageLog.header.msg_size - 9);
            LOG_DATA("{}: messageLog.message: {}", nameId(), messageLog.message);
        }
        else if (ulogMsgHeader.msgHeader.msg_type == 'C')
        {
            Ulog::message_logging_tagged_s messageLogTagged;
            messageLogTagged.header = ulogMsgHeader.msgHeader;
            _filestream.read(reinterpret_cast<char*>(&messageLogTagged.log_level), sizeof(messageLogTagged.log_level));

            if (messageLogTagged.log_level == 48) // '0'
            {
                LOG_DATA("{}: Log-level: EMERG - System is unusable", nameId());
            }
            else if (messageLogTagged.log_level == 49) // '1'
            {
                LOG_DATA("{}: Log-level: ALERT - Action must be taken immediately", nameId());
            }
            else if (messageLogTagged.log_level == 50) // '2'
            {
                LOG_DATA("{}: Log-level: CRIT - Critical conditions", nameId());
            }
            else if (messageLogTagged.log_level == 51) // '3'
            {
                LOG_DATA("{}: Log-level: ERR - Error conditions", nameId());
            }
            else if (messageLogTagged.log_level == 52) // '4'
            {
                LOG_DATA("{}: Log-level: WARNING - Warning conditions", nameId());
            }
            else if (messageLogTagged.log_level == 53) // '5'
            {
                LOG_DATA("{}: Log-level: NOTICE - Normal but significant condition", nameId());
            }
            else if (messageLogTagged.log_level == 54) // '6'
            {
                LOG_DATA("{}: Log-level: INFO - Informational", nameId());
            }
            else if (messageLogTagged.log_level == 55) // '7'
            {
                LOG_DATA("{}: Log-level: DEBUG - Debug-level messages", nameId());
            }
            else
            {
                LOG_WARN("{}: Log-level is out of scope ({}) - possible data loss", nameId(), messageLogTagged.log_level);
            }

            _filestream.read(reinterpret_cast<char*>(&messageLogTagged.tag), sizeof(messageLogTagged.tag));
            LOG_DATA("{}: messageLogTagged.tag: {}", nameId(), messageLogTagged.tag);
            _filestream.read(reinterpret_cast<char*>(&messageLogTagged.timestamp), sizeof(messageLogTagged.timestamp));
            LOG_DATA("{}: messageLogTagged.timestamp [µs]: {}", nameId(), messageLogTagged.timestamp);

            messageLogTagged.message.resize(messageLogTagged.header.msg_size - 11);
            _filestream.read(messageLogTagged.message.data(), messageLogTagged.header.msg_size - 11);
            LOG_DATA("{}: messageLogTagged.header.msg_size: {}", nameId(), messageLogTagged.header.msg_size);
        }
        else if (ulogMsgHeader.msgHeader.msg_type == 'S')
        {
            Ulog::message_sync_s messageSync;
            messageSync.header = ulogMsgHeader.msgHeader;
            std::array<uint8_t, 8> sync_magic{};
            _filestream.read(reinterpret_cast<char*>(messageSync.snyc_magic.data()), sizeof(sync_magic));
            LOG_DATA("{}: messageSync.snyc_magic[0]: {}", nameId(), messageSync.snyc_magic[0]);
        }
        else if (ulogMsgHeader.msgHeader.msg_type == 'O')
        {
            Ulog::message_dropout_s messageDropout;
            messageDropout.header = ulogMsgHeader.msgHeader;
            _filestream.read(reinterpret_cast<char*>(&messageDropout.duration), sizeof(messageDropout.duration));
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
            [[maybe_unused]] auto unidentifiedPos = static_cast<uint64_t>(_filestream.tellg());
            nextChars.resize(100);
            _filestream.read(nextChars.data(), 100);
            LOG_WARN("{}: Message type not identified. Position: {}, The next 100 chars: {}", nameId(), unidentifiedPos, nextChars);

            // Reset read cursor
            _filestream.seekg(-100, std::ios_base::cur);
        }

        if (!_filestream.good() || _filestream.eof())
        {
            break;
        }
    }
    return nullptr;
}

void NAV::UlogFile::readInformationMessage(uint16_t msgSize, char msgType)
{
    // Read msg size (2B) and type (1B)
    Ulog::message_info_s messageInfo;
    messageInfo.header.msg_size = msgSize;
    messageInfo.header.msg_type = msgType;
    _filestream.read(reinterpret_cast<char*>(&messageInfo.key_len), sizeof(messageInfo.key_len));

    // Read 'key' identifier ('keylength' byte) and its associated 'value'
    messageInfo.key.resize(messageInfo.key_len);
    _filestream.read(messageInfo.key.data(), messageInfo.key_len);
    // if (!_filestream.good() || _filestream.eof())
    // {
    //     return false;
    // }

    messageInfo.value.resize(static_cast<size_t>(messageInfo.header.msg_size - 1 - messageInfo.key_len)); // 'msg_size' contains key and value, but not header
    _filestream.read(messageInfo.value.data(), messageInfo.header.msg_size - 1 - messageInfo.key_len);
    LOG_DATA("{}: Information message - key: {}", nameId(), messageInfo.key);
    LOG_DATA("{}: Information message - value: {}", nameId(), messageInfo.value);
}

void NAV::UlogFile::readInformationMessageMulti(uint16_t msgSize, char msgType)
{
    // Read msg size (2B) and type (1B)
    Ulog::ulog_message_info_multiple_header_s messageInfoMulti;
    messageInfoMulti.header.msg_size = msgSize;
    messageInfoMulti.header.msg_type = msgType;
    _filestream.read(reinterpret_cast<char*>(&messageInfoMulti.is_continued), sizeof(messageInfoMulti.is_continued));
    _filestream.read(reinterpret_cast<char*>(&messageInfoMulti.key_len), sizeof(messageInfoMulti.key_len));

    // Read 'key' identifier ('keylength' byte) and its associated 'value'
    messageInfoMulti.key.resize(messageInfoMulti.key_len);
    _filestream.read(messageInfoMulti.key.data(), messageInfoMulti.key_len);
    messageInfoMulti.value.resize(static_cast<size_t>(messageInfoMulti.header.msg_size - 2 - messageInfoMulti.key_len)); // contains 'is_continued' flag in contrast to information message
    _filestream.read(messageInfoMulti.value.data(), messageInfoMulti.header.msg_size - 2 - messageInfoMulti.key_len);
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
    _filestream.read(reinterpret_cast<char*>(&messageParam.key_len), sizeof(messageParam.key_len));

    // Read 'key' identifier ('keylength' byte) and its associated 'value'
    messageParam.key.resize(messageParam.key_len);
    _filestream.read(messageParam.key.data(), messageParam.key_len);

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
        _filestream.read(messageParam.value.data(), messageParam.header.msg_size - 1 - messageParam.key_len);
        LOG_DATA("{}: Parameter message - key: {}", nameId(), messageParam.key);
        LOG_DATA("{}: Parameter message - value: {}", nameId(), messageParam.value);
    }
}

void NAV::UlogFile::readParameterMessageDefault(uint16_t msgSize, char msgType)
{
    Ulog::ulog_message_parameter_default_header_s messageParamDefault;
    messageParamDefault.header.msg_size = msgSize;
    messageParamDefault.header.msg_type = msgType;
    _filestream.read(reinterpret_cast<char*>(&messageParamDefault.default_types), sizeof(messageParamDefault.default_types));
    _filestream.read(reinterpret_cast<char*>(&messageParamDefault.key_len), sizeof(messageParamDefault.key_len));

    messageParamDefault.key.resize(messageParamDefault.key_len);
    _filestream.read(messageParamDefault.key.data(), messageParamDefault.key_len);
    messageParamDefault.value.resize(static_cast<size_t>(messageParamDefault.header.msg_size - 2 - messageParamDefault.key_len));
    _filestream.read(messageParamDefault.value.data(), messageParamDefault.header.msg_size - 2 - messageParamDefault.key_len);
    LOG_DEBUG("{}: Parameter default message - key: {}", nameId(), messageParamDefault.key);
    LOG_DEBUG("{}: Parameter default message - value: {}", nameId(), messageParamDefault.value);

    //TODO: Restriction on '1<<0' and '1<<1'
}

int8_t NAV::UlogFile::enoughImuDataAvailable()
{
    std::array<bool, 2> accelHasData{};
    std::array<bool, 2> gyroHasData{};

    for ([[maybe_unused]] const auto& [timestamp, measurement] : _epochData)
    {
        if (std::holds_alternative<SensorAccel>(measurement.data))
        {
            accelHasData.at(measurement.multi_id) = true;
        }
        else if (std::holds_alternative<SensorGyro>(measurement.data))
        {
            gyroHasData.at(measurement.multi_id) = true;
        }
    }

    if (lastGnssTime.timeSinceStartup)
    {
        for (size_t i = 0; i < 2; ++i)
        {
            if (accelHasData.at(i) && gyroHasData.at(i))
            {
                return static_cast<int8_t>(i);
            }
        }
    }

    return -1;
}

std::array<std::multimap<uint64_t, NAV::UlogFile::MeasurementData>::iterator, 2> NAV::UlogFile::findPosVelAttData()
{
    auto gpsIter = _epochData.end();
    auto attIter = _epochData.end();

    for (auto iter = _epochData.begin(); iter != _epochData.end(); ++iter)
    {
        if (std::holds_alternative<VehicleGpsPosition>(iter->second.data))
        {
            gpsIter = iter;
        }
        else if (std::holds_alternative<VehicleAttitude>(iter->second.data))
        {
            attIter = iter;
        }
    }

    return { gpsIter, attIter };
}