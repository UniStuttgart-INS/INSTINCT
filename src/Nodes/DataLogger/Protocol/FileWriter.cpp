#include "FileWriter.hpp"

#include "util/Logger.hpp"

#include "internal/FlowManager.hpp"

[[nodiscard]] json NAV::FileWriter::save() const
{
    LOG_TRACE("called");

    json j;

    j["path"] = _path;
    j["fileType"] = _fileType;

    return j;
}

void NAV::FileWriter::restore(json const& j)
{
    LOG_TRACE("called");

    if (j.contains("path"))
    {
        j.at("path").get_to(_path);
    }
    if (j.contains("fileType"))
    {
        j.at("fileType").get_to(_fileType);
    }
}

bool NAV::FileWriter::initialize()
{
    deinitialize();

    LOG_TRACE("called");

    if (_fileType == FileType::NONE)
    {
        LOG_ERROR("FileWriter needs the _fileType set in the child class.");
        return false;
    }

    std::string filepath = _path;
    if (!_path.starts_with('/') && !_path.starts_with('~'))
    {
        filepath = flow::GetProgramRootPath() + '/' + _path;
    }

    if (_fileType == FileType::CSV || _fileType == FileType::BINARY)
    {
        // Does not enable binary read/write, but disables OS dependant treatment of \n, \r
        _filestream.open(filepath, std::ios_base::trunc | std::ios_base::binary);
    }

    if (!_filestream.good())
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
        if (_filestream.is_open())
        {
            _filestream.flush();
            _filestream.close();
        }
    }
    catch (...)
    {
    }

    _filestream.clear();
}

const char* NAV::FileWriter::to_string(NAV::FileWriter::FileType type)
{
    switch (type)
    {
    case FileType::NONE:
        return "None";
    case FileType::CSV:
        return "CSV";
    case FileType::BINARY:
        return "Binary";
    default:
        return "Unkown";
    }
}