#include "FileReader.hpp"

#include "util/Logger.hpp"

NAV::FileReader::FileReader(std::deque<std::string>& options)
{
    LOG_TRACE("called");

    if (!options.empty())
    {
        path = options.at(0);
        options.pop_front();
    }
}