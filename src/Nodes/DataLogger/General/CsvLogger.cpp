// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "CsvLogger.hpp"

#include "NodeData/NodeData.hpp"

#include "util/Logger.hpp"

#include <iomanip> // std::setprecision

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"
#include "NodeRegistry.hpp"

NAV::CsvLogger::CsvLogger()
    : Node(typeStatic())
{
    LOG_TRACE("{}: called", name);

    _fileType = FileType::ASCII;

    _hasConfig = true;
    _guiConfigDefaultWindowSize = { 380, 70 };

    nm::CreateInputPin(this, "writeObservation", Pin::Type::Flow,
                       { NodeData::type() },
                       &CsvLogger::writeObservation);
}

NAV::CsvLogger::~CsvLogger()
{
    LOG_TRACE("{}: called", nameId());
}

std::string NAV::CsvLogger::typeStatic()
{
    return "CsvLogger";
}

std::string NAV::CsvLogger::type() const
{
    return typeStatic();
}

std::string NAV::CsvLogger::category()
{
    return "Data Logger";
}

void NAV::CsvLogger::guiConfig()
{
    if (FileWriter::guiConfig(".csv", { ".csv" }, size_t(id), nameId()))
    {
        flow::ApplyChanges();
        doDeinitialize();
    }
}

[[nodiscard]] json NAV::CsvLogger::save() const
{
    LOG_TRACE("{}: called", nameId());

    json j;

    j["FileWriter"] = FileWriter::save();

    return j;
}

void NAV::CsvLogger::restore(json const& j)
{
    LOG_TRACE("{}: called", nameId());

    if (j.contains("FileWriter"))
    {
        FileWriter::restore(j.at("FileWriter"));
    }
}

void NAV::CsvLogger::flush()
{
    _filestream.flush();
}

bool NAV::CsvLogger::initialize()
{
    LOG_TRACE("{}: called", nameId());

    if (!FileWriter::initialize())
    {
        return false;
    }

    CommonLog::initialize();

    _headerWritten = false;
    _dynamicHeader.clear();

    return true;
}

void NAV::CsvLogger::deinitialize()
{
    LOG_TRACE("{}: called", nameId());

    FileWriter::deinitialize();
}

void NAV::CsvLogger::writeHeader(const std::shared_ptr<const NodeData>& obs)
{
    _filestream << "Time [s],GpsCycle,GpsWeek,GpsToW [s]";

    for (const auto& desc : obs->staticDataDescriptors())
    {
        _filestream << "," << desc;
    }
    for (const auto& desc : _dynamicHeader)
    {
        _filestream << "," << desc;
    }
    _filestream << std::endl; // NOLINT(performance-avoid-endl)

    _headerWritten = true;
}

void NAV::CsvLogger::rewriteData(size_t oldSize, size_t newSize, const std::shared_ptr<const NodeData>& obs)
{
    FileWriter::deinitialize();
    auto tmpFilePath = getFilepath().concat("_temp");
    std::filesystem::rename(getFilepath(), tmpFilePath);
    FileWriter::initialize();
    writeHeader(obs);

    std::ifstream tmpFilestream(tmpFilePath, std::ios_base::in | std::ios_base::binary);
    if (tmpFilestream.good())
    {
        std::string delimiterEnd(newSize - oldSize, ',');
        std::string line;
        std::getline(tmpFilestream, line); // Old header
        while (std::getline(tmpFilestream, line) && !tmpFilestream.eof())
        {
            _filestream << line << delimiterEnd << '\n';
        }
    }
    if (tmpFilestream.is_open()) { tmpFilestream.close(); }
    tmpFilestream.clear();
    std::filesystem::remove(tmpFilePath);
}

void NAV::CsvLogger::writeObservation(NAV::InputPin::NodeDataQueue& queue, size_t /* pinIdx */)
{
    auto obs = queue.extract_front();

    size_t dynHeaderLength = _dynamicHeader.size();
    for (const auto& desc : obs->dynamicDataDescriptors())
    {
        if (std::ranges::none_of(_dynamicHeader, [&](const auto& header) { return header == desc; }))
        {
            _dynamicHeader.push_back(desc);
        }
    }

    if (!_headerWritten) { writeHeader(obs); }
    else if (dynHeaderLength != _dynamicHeader.size())
    {
        rewriteData(dynHeaderLength, _dynamicHeader.size(), obs);
    }

    constexpr int gpsCyclePrecision = 3;
    constexpr int gpsTimePrecision = 12;
    constexpr int valuePrecision = 15;

    if (!obs->insTime.empty())
    {
        _filestream << std::setprecision(valuePrecision) << std::round(calcTimeIntoRun(obs->insTime) * 1e9) / 1e9;
    }
    _filestream << ",";
    if (!obs->insTime.empty())
    {
        _filestream << std::fixed << std::setprecision(gpsCyclePrecision) << obs->insTime.toGPSweekTow().gpsCycle;
    }
    _filestream << ",";
    if (!obs->insTime.empty())
    {
        _filestream << std::defaultfloat << std::setprecision(gpsTimePrecision) << obs->insTime.toGPSweekTow().gpsWeek;
    }
    _filestream << ",";
    if (!obs->insTime.empty())
    {
        _filestream << std::defaultfloat << std::setprecision(gpsTimePrecision) << obs->insTime.toGPSweekTow().tow;
    }
    _filestream << std::setprecision(valuePrecision);

    for (size_t i = 0; i < obs->staticDescriptorCount(); ++i)
    {
        _filestream << ',';
        if (auto val = obs->getValueAt(i)) { _filestream << *val; }
    }

    for (const auto& desc : _dynamicHeader)
    {
        _filestream << ',';
        if (auto val = obs->getDynamicDataAt(desc)) { _filestream << *val; }
    }

    _filestream << '\n';
}