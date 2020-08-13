#include "GnssFileReader.hpp"

#include "util/Logger.hpp"

NAV::GnssFileReader::GnssFileReader(const std::string& name, [[maybe_unused]] const std::map<std::string, std::string>& options)
    : FileReader(name, options), Gnss(name, options) {}

void NAV::GnssFileReader::resetNode()
{
    FileReader::resetReader();
}

void NAV::GnssFileReader::initialize()
{
    FileReader::initialize();
}