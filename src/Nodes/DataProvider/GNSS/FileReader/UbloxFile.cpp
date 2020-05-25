#include "UbloxFile.hpp"

#include "util/Logger.hpp"
#include <ios>
#include <cmath>

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
        if (fileType != FileType::BINARY)
        {
            // Read header line
            std::string line;
            std::getline(filestream, line);
            // Remove any starting non text characters
            line.erase(line.begin(), std::find_if(line.begin(), line.end(),
                                                  std::ptr_fun<int, int>(std::isalnum)));
            // Convert line into stream
            std::stringstream lineStream(line);
            std::string cell;
            // Split line at comma
            while (std::getline(lineStream, cell, ','))
            {
                // Remove any trailing non text characters
                cell.erase(std::find_if(cell.begin(), cell.end(),
                                        std::ptr_fun<int, int>(std::iscntrl)),
                           cell.end());
                columns.push_back(cell);
            }

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
    columns.clear();
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
    }
    // Ascii

    // TODO: Implement!!!!!!!!!!!!!!!!!!

    // Calls all the callbacks
    if (!peek)
    {
        invokeCallbacks(obs);
    }

    return obs;
}

NAV::FileReader::FileType NAV::UbloxFile::determineFileType()
{
    LOG_TRACE("called for {}", name);

    return FileType::BINARY;
    // constexpr uint8_t BinaryStartChar = 0xFA;

    // filestream = std::ifstream(path);

    // constexpr uint16_t BUFFER_SIZE = 256;

    // std::array<char, BUFFER_SIZE> buffer{};
    // if (filestream.good())
    // {
    //     filestream.read(buffer.data(), BUFFER_SIZE);

    //     if (std::strstr(buffer.data(), "TimeStartup"))
    //     {
    //         filestream.close();
    //         LOG_DEBUG("{} has the file type: ASCII", name);
    //         return FileType::ASCII;
    //     }
    //     if (memmem(buffer.data(), BUFFER_SIZE, "Control Center", sizeof("Control Center")) != nullptr
    //         || static_cast<unsigned char>(buffer.at(0)) == BinaryStartChar)
    //     {
    //         filestream.close();
    //         LOG_DEBUG("{} has the file type: Binary", name);
    //         return FileType::BINARY;
    //     }
    //     filestream.close();
    //     LOG_CRITICAL("{} could not determine file type", name);
    // }

    // LOG_CRITICAL("{} could not open file {}", name, path);
    // return FileType::NONE;
}