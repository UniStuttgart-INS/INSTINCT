#include "EmlidFile.hpp"

#include "util/Logger.hpp"
#include <ios>
#include <cmath>
#include <array>

#include "util/Emlid/EmlidDecryptor.hpp"

NAV::EmlidFile::EmlidFile(const std::string& name, const std::map<std::string, std::string>& options)
    : FileReader(options), Gnss(name, options)
{
    LOG_TRACE("called for {}", name);

    fileType = determineFileType();

    if (fileType == FileType::BINARY)
    {
        filestream = std::ifstream(path, std::ios_base::in | std::ios_base::binary);
    }
    else
    {
        filestream = std::ifstream(path);
    }

    if (filestream.good())
    {
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
    else
    {
        LOG_CRITICAL("{} could not open file {}", name, path);
    }
}

NAV::EmlidFile::~EmlidFile()
{
    LOG_TRACE("called for {}", name);

    // removeAllCallbacks();
    if (filestream.is_open())
    {
        filestream.close();
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
    auto obs = std::make_shared<EmlidObs>();

    if (fileType == FileType::ASCII)
    {
        // TODO: Implement EmlidFile Ascii reading
        LOG_CRITICAL("Ascii EmlidFile pollData is not implemented yet.");
        return obs;
    }

    constexpr uint8_t AsciiStartChar = '$';
    constexpr uint8_t BinaryStartChar1 = 0x45;
    constexpr uint8_t BinaryStartChar2 = 0x52;
    // constexpr uint8_t AsciiEndChar1 = '\r'; // 0x0D
    constexpr uint8_t AsciiEndChar2 = '\n'; // 0x0A

    // Get current position
    auto pos = filestream.tellg();
    uint8_t i = 0;
    while (filestream >> i)
    {
        if (i == BinaryStartChar1)
        {
            // 0 = Sync Char 1
            // 1 = Sync Char 2
            // 2 = Class
            // 3 = ID
            // 4+5 = Payload Length
            // Payload
            // CK_A
            // CK_B

            constexpr size_t HEAD_BUFFER_SIZE = 5;
            // Buffer for reading binary data
            std::array<uint8_t, HEAD_BUFFER_SIZE> headBuffer{};

            filestream.read(reinterpret_cast<char*>(headBuffer.data()), HEAD_BUFFER_SIZE);

            if (headBuffer.at(0) != BinaryStartChar2)
            {
                continue;
            }

            uint16_t payloadLength = Emlid::EmlidPacket::U2(headBuffer.data() + 3);

            std::vector<uint8_t> buffer{ BinaryStartChar1 };
            buffer.insert(buffer.end(), headBuffer.data(), headBuffer.data() + HEAD_BUFFER_SIZE);

            buffer.resize(HEAD_BUFFER_SIZE + 1 + payloadLength + 2);
            filestream.read(reinterpret_cast<char*>(buffer.data()) + HEAD_BUFFER_SIZE + 1, payloadLength + 2);

            obs->raw.setData(buffer.data(), buffer.size());

            break;
        }

        if (i == AsciiStartChar)
        {
            std::string line;
            std::getline(filestream, line);
            line.insert(0, 1, AsciiStartChar);
            line.push_back(AsciiEndChar2);

            obs->raw.setData(reinterpret_cast<unsigned char*>(line.data()), line.size());

            break;
        }
    }
    if (obs->raw.getRawDataLength() == 0)
    {
        return nullptr;
    }

    Emlid::decryptEmlidObs(obs, currentInsTime, peek);

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

    constexpr uint8_t BinaryErbStartChar1 = 0x45;
    constexpr uint8_t BinaryErbStartChar2 = 0x52;
    constexpr char BinaryNmeaStartChar = '$';

    filestream = std::ifstream(path);

    constexpr uint16_t BUFFER_SIZE = 10;

    std::array<char, BUFFER_SIZE> buffer{};
    if (filestream.good())
    {
        filestream.read(buffer.data(), BUFFER_SIZE);

        if ((static_cast<uint8_t>(buffer.at(0)) == BinaryErbStartChar1 && static_cast<uint8_t>(buffer.at(1)) == BinaryErbStartChar2)
            || buffer.at(0) == BinaryNmeaStartChar)
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