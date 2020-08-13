#include "ImuFileReader.hpp"

#include "util/Logger.hpp"

NAV::ImuFileReader::ImuFileReader(const std::string& name, [[maybe_unused]] const std::map<std::string, std::string>& options)
    : FileReader(name, options), Imu(name, options) {}

void NAV::ImuFileReader::resetNode()
{
    FileReader::resetReader();
}

void NAV::ImuFileReader::initialize()
{
    FileReader::initialize();
}