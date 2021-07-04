#include "FileWriter.hpp"

#include "util/Logger.hpp"

#include "internal/FlowManager.hpp"

[[nodiscard]] json NAV::FileWriter::save() const
{
    LOG_TRACE("called");

    json j;

    j["path"] = path;
    j["fileType"] = fileType;

    return j;
}

void NAV::FileWriter::restore(json const& j)
{
    LOG_TRACE("called");

    if (j.contains("path"))
    {
        j.at("path").get_to(path);
    }
    if (j.contains("fileType"))
    {
        j.at("fileType").get_to(fileType);
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

    std::string filepath = path;
    if (!path.starts_with('/') && !path.starts_with('~'))
    {
        filepath = flow::GetProgramRootPath() + '/' + path;
    }

    if (fileType == FileType::ASCII || fileType == FileType::BINARY)
    {
        // Does not enable binary read/write, but disables OS dependant treatment of \n, \r
        filestream.open(filepath, std::ios_base::trunc | std::ios_base::binary);
    }

    if (!filestream.good())
    {
        LOG_ERROR("Could not open file {}", filepath);
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

std::string NAV::FileWriter::str(NAV::FileWriter::FileType type)
{
    switch (type)
    {
    case FileType::NONE:
        return "None";
    case FileType::ASCII:
        return "Csv";
    case FileType::BINARY:
        return "Binary";
    }
}