#include "EmlidFile.hpp"

#include "util/Logger.hpp"
#include <ios>
#include <cmath>
#include <array>

#include "util/UartSensors/Emlid/EmlidUtilities.hpp"

NAV::EmlidFile::EmlidFile(const std::string& name, const std::map<std::string, std::string>& options)
    : FileReader(name, options), Gnss(name, options), sensor(name)
{
    LOG_TRACE("called for {}", name);

    fileType = determineFileType();

    readHeader();

    dataStart = filestream.tellg();

    if (fileType == FileType::ASCII)
    {
        LOG_DEBUG("{}-ASCII-File successfully initialized", name);
    }
    else
    {
        LOG_DEBUG("{}-Binary-File successfully initialized", name);
    }
}

void NAV::EmlidFile::resetNode()
{
    // Return to position
    filestream.clear();
    filestream.seekg(dataStart, std::ios_base::beg);
}

std::shared_ptr<NAV::EmlidObs> NAV::EmlidFile::pollData(bool peek)
{
    if (fileType == FileType::ASCII)
    {
        // TODO: Implement EmlidFile Ascii reading
        LOG_CRITICAL("Ascii EmlidFile pollData is not implemented yet.");
        return nullptr;
    }

    // Get current position
    auto pos = filestream.tellg();
    uint8_t i = 0;
    std::unique_ptr<uart::protocol::Packet> packet = nullptr;
    while (filestream.readsome(reinterpret_cast<char*>(&i), 1))
    {
        packet = sensor.findPacket(i);

        if (packet != nullptr)
        {
            break;
        }
    }

    if (!packet)
    {
        return nullptr;
    }

    auto obs = std::make_shared<EmlidObs>(*packet);

    // Check if package is empty
    if (obs->raw.getRawDataLength() == 0)
    {
        return nullptr;
    }

    sensors::emlid::decryptEmlidObs(obs, currentInsTime, peek);

    if (peek)
    {
        // Return to position before "Read line".
        filestream.seekg(pos, std::ios_base::beg);
    }
    else
    {
        // Calls all the callbacks
        invokeCallbacks(obs);
    }

    return obs;
}

NAV::FileReader::FileType NAV::EmlidFile::determineFileType()
{
    LOG_TRACE("called for {}", name);

    auto filestream = std::ifstream(path);

    constexpr uint16_t BUFFER_SIZE = 10;

    std::array<char, BUFFER_SIZE> buffer{};
    if (filestream.good())
    {
        filestream.read(buffer.data(), BUFFER_SIZE);

        if ((static_cast<uint8_t>(buffer.at(0)) == sensors::emlid::EmlidUartSensor::BinarySyncChar1
             && static_cast<uint8_t>(buffer.at(1)) == sensors::emlid::EmlidUartSensor::BinarySyncChar2)
            || buffer.at(0) == sensors::emlid::EmlidUartSensor::AsciiStartChar)
        {
            filestream.close();
            LOG_DEBUG("{} has the file type: Binary", name);
            return FileType::BINARY;
        }
        filestream.close();

        LOG_CRITICAL("{} could not determine file type", name);
    }

    LOG_CRITICAL("{} could not open file {}", name, path);
    return FileType::NONE;
}

void NAV::EmlidFile::readHeader()
{
    if (fileType == FileType::ASCII)
    {
        // Read header line
        std::string line;
        std::getline(filestream, line);
        // Remove any starting non text characters
        line.erase(line.begin(), std::find_if(line.begin(), line.end(), [](int ch) { return std::isalnum(ch); }));
        // Convert line into stream
        std::stringstream lineStream(line);
        std::string cell;
        // Split line at comma
        while (std::getline(lineStream, cell, ','))
        {
            // Remove any trailing non text characters
            cell.erase(std::find_if(cell.begin(), cell.end(), [](int ch) { return std::iscntrl(ch); }), cell.end());
            columns.push_back(cell);
        }
    }
}