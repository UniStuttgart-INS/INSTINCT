#include "UbloxFile.hpp"

#include "util/Logger.hpp"
#include <ios>
#include <cmath>
#include <array>

#include "util/Ublox/UbloxDecryptor.hpp"

NAV::UbloxFile::UbloxFile(const std::string& name, std::deque<std::string>& options)
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

NAV::UbloxFile::~UbloxFile()
{
    LOG_TRACE("called for {}", name);

    // removeAllCallbacks();
    if (filestream.is_open())
    {
        filestream.close();
    }
}

std::shared_ptr<NAV::UbloxObs> NAV::UbloxFile::pollData(bool peek)
{
    auto obs = std::make_shared<UbloxObs>();

    if (fileType == FileType::ASCII)
    {
        // TODO: Implement UbloxFile Ascii reading
        LOG_CRITICAL("Ascii UbloxFile pollData is not implemented yet.");
        return obs;
    }

    constexpr uint8_t AsciiStartChar = '$';
    constexpr uint8_t BinaryStartChar1 = 0xB5;
    constexpr uint8_t BinaryStartChar2 = 0x62;
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

            uint16_t payloadLength = ublox::UbloxPacket::U2(headBuffer.data() + 3);

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

    ublox::decryptUbloxObs(obs, currentInsTime, peek);

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

NAV::FileReader::FileType NAV::UbloxFile::determineFileType()
{
    LOG_TRACE("called for {}", name);

    constexpr uint8_t BinaryUbxStartChar1 = 0xB5;
    constexpr uint8_t BinaryUbxStartChar2 = 0x62;
    constexpr char BinaryNmeaStartChar = '$';

    filestream = std::ifstream(path);

    constexpr uint16_t BUFFER_SIZE = 10;

    std::array<char, BUFFER_SIZE> buffer{};
    if (filestream.good())
    {
        filestream.read(buffer.data(), BUFFER_SIZE);

        if ((static_cast<uint8_t>(buffer.at(0)) == BinaryUbxStartChar1 && static_cast<uint8_t>(buffer.at(1)) == BinaryUbxStartChar2)
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