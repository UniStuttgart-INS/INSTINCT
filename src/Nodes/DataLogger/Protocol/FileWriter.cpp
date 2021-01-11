#include "FileWriter.hpp"

#include "util/Logger.hpp"

[[nodiscard]] json NAV::FileWriter::save() const
{
    LOG_TRACE("called");

    json j;

    j["path"] = path;

    return j;
}

void NAV::FileWriter::restore(json const& j)
{
    LOG_TRACE("called");

    if (j.contains("path"))
    {
        j.at("path").get_to(path);
    }
}

bool NAV::FileWriter::initialize()
{
    deinitialize();

    LOG_TRACE("called");

    if (fileType == FileType::NONE)
    {
        LOG_ERROR("FileWriter needs the fileType set in the child class.");
        return false;
    }

    if (fileType == FileType::ASCII)
    {
        filestream.open(path, std::ios_base::trunc);
    }
    else if (fileType == FileType::BINARY)
    {
        // Does not enable binary read/write, but disables OS dependant treatment of \n, \r
        filestream.open(path, std::ios_base::trunc | std::ios_base::binary);
    }

    if (!filestream.good())
    {
        LOG_ERROR("Could not open file {}", path);
        return false;
    }

    return true;
}

void NAV::FileWriter::deinitialize()
{
    LOG_TRACE("called");

    try
    {
        if (filestream.is_open())
        {
            filestream.flush();
            filestream.close();
        }
    }
    catch (...)
    {
    }

    filestream.clear();
}
