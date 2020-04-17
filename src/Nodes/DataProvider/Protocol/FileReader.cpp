#include "FileReader.hpp"

NAV::FileReader::FileReader(std::deque<std::string>& options)
{
    if (options.size() >= 1)
    {
        path = options.at(0);
        options.pop_front();
    }
}

NAV::FileReader::~FileReader() {}