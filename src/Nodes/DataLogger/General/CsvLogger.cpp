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
#include <ranges>
#include <regex>

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"
#include "NodeRegistry.hpp"
#include "internal/gui/NodeEditorApplication.hpp"

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

    if (CommonLog::ShowOriginInput(nameId().c_str()))
    {
        flow::ApplyChanges();
    }

    if (ImGui::Button(fmt::format("Clear header##{}", size_t(id)).c_str()))
    {
        _headerLogging.clear();
    }
    ImGui::SameLine();
    if (ImGui::Button(fmt::format("Select all##{}", size_t(id)).c_str()))
    {
        for (auto& header : _headerLogging) { header.second = true; }
        flow::ApplyChanges();
    }
    ImGui::SameLine();
    if (ImGui::Button(fmt::format("Deselect all##{}", size_t(id)).c_str()))
    {
        for (auto& header : _headerLogging) { header.second = false; }
        flow::ApplyChanges();
    }
    ImGui::SameLine();
    if (ImGui::Checkbox(fmt::format("Default for new##{}", size_t(id)).c_str(), &_headerLoggingDefault))
    {
        flow::ApplyChanges();
    }
    ImGui::SameLine();
    if (ImGui::Checkbox(fmt::format("Sort headers in GUI##{}", size_t(id)).c_str(), &_headerLoggingSortGui))
    {
        flow::ApplyChanges();
    }

    if (_headerLoggingRegex.empty()) { ImGui::BeginDisabled(); }
    std::optional<bool> regexSelect;
    if (ImGui::Button(fmt::format("Select regex##{}", size_t(id)).c_str()))
    {
        regexSelect = true;
    }
    ImGui::SameLine();
    if (ImGui::Button(fmt::format("Deselect regex##{}", size_t(id)).c_str()))
    {
        regexSelect = false;
    }
    if (regexSelect.has_value())
    {
        bool anyChanged = false;
        for (auto& [desc, checked] : _headerLogging)
        {
            std::regex self_regex(_headerLoggingRegex,
                                  std::regex_constants::ECMAScript | std::regex_constants::icase);
            if (std::regex_search(desc, self_regex) && checked != *regexSelect)
            {
                anyChanged = true;
                checked = *regexSelect;
            }
        }
        if (anyChanged)
        {
            flow::ApplyChanges();
        }
    }
    if (_headerLoggingRegex.empty()) { ImGui::EndDisabled(); }
    ImGui::SameLine();
    ImGui::SetNextItemWidth(300.0F * gui::NodeEditorApplication::windowFontRatio());
    if (ImGui::InputText(fmt::format("##Select Regex {}", size_t(id)).c_str(), &_headerLoggingRegex))
    {
        flow::ApplyChanges();
    }

    if (!_headerLogging.empty())
    {
        auto* headerLogging = &_headerLogging;
        decltype(_headerLogging) sortedHeaderLogging;
        if (_headerLoggingSortGui)
        {
            sortedHeaderLogging = _headerLogging;
            std::ranges::sort(sortedHeaderLogging);
            headerLogging = &sortedHeaderLogging;
        }
        int nCols = std::min((static_cast<int>(headerLogging->size()) - 1) / 5 + 1, 3);
        if (ImGui::BeginChild(fmt::format("Headers Scrolling {}", size_t(id)).c_str(), ImGui::GetContentRegionAvail(), false))
        {
            if (ImGui::BeginTable(fmt::format("Logging headers##{}", size_t(id)).c_str(), nCols, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_NoHostExtendX, ImVec2(0.0F, 0.0F)))
            {
                for (auto& [desc, checked] : *headerLogging)
                {
                    ImGui::TableNextColumn();
                    if (ImGui::Checkbox(fmt::format("{}##{}", desc, size_t(id)).c_str(), &checked))
                    {
                        if (_headerLoggingSortGui)
                        {
                            if (auto iter = std::ranges::find_if(_headerLogging, [&](const std::pair<std::string, bool>& header) {
                                    return desc == header.first; // NOLINT(clang-analyzer-core.CallAndMessage)
                                });
                                iter != _headerLogging.end())
                            {
                                iter->second = checked;
                            }
                        }
                        flow::ApplyChanges();
                    }
                }

                ImGui::EndTable();
            }
        }
        ImGui::EndChild();
    }
    else
    {
        ImGui::TextUnformatted("Please run the flow to collect information about the available data.");
    }
}

[[nodiscard]] json NAV::CsvLogger::save() const
{
    LOG_TRACE("{}: called", nameId());

    return {
        { "FileWriter", FileWriter::save() },
        { "header", _headerLogging },
        { "headerLoggingRegex", _headerLoggingRegex },
        { "headerLoggingDefault", _headerLoggingDefault },
        { "headerLoggingSortGui", _headerLoggingSortGui },
    };
}

void NAV::CsvLogger::restore(json const& j)
{
    LOG_TRACE("{}: called", nameId());

    if (j.contains("FileWriter")) { FileWriter::restore(j.at("FileWriter")); }
    if (j.contains("header")) { j.at("header").get_to(_headerLogging); }
    if (j.contains("headerLoggingRegex")) { j.at("headerLoggingRegex").get_to(_headerLoggingRegex); }
    if (j.contains("headerLoggingDefault")) { j.at("headerLoggingDefault").get_to(_headerLoggingDefault); }
}

void NAV::CsvLogger::flush()
{
    _filestream.flush();
}

void NAV::CsvLogger::onDeleteLink([[maybe_unused]] OutputPin& startPin, [[maybe_unused]] InputPin& endPin)
{
    LOG_TRACE("{}: called for {} ==> {}", nameId(), size_t(startPin.id), size_t(endPin.id));

    _headerLogging.clear();
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
        if (auto iter = std::ranges::find_if(_headerLogging, [&](const std::pair<std::string, bool>& header) {
                return desc == header.first;
            });
            iter != _headerLogging.end() && !iter->second)
        {
            continue;
        }

        _filestream << "," << desc;
    }
    for (const auto& desc : _dynamicHeader)
    {
        if (auto iter = std::ranges::find_if(_headerLogging, [&](const std::pair<std::string, bool>& header) {
                return desc == header.first;
            });
            iter != _headerLogging.end() && !iter->second)
        {
            continue;
        }

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

    auto oldHeaderLength = static_cast<size_t>(std::ranges::count_if(_headerLogging, [](const auto& header) { return header.second; }));
    if (!_headerWritten)
    {
        for (const auto& desc : obs->staticDataDescriptors())
        {
            if (auto iter = std::ranges::find_if(_headerLogging, [&](const std::pair<std::string, bool>& header) {
                    return desc == header.first;
                });
                iter == _headerLogging.end())
            {
                _headerLogging.emplace_back(desc, _headerLoggingDefault);
                flow::ApplyChanges();
            }
        }
    }
    for (const auto& desc : obs->dynamicDataDescriptors())
    {
        if (std::ranges::none_of(_dynamicHeader, [&](const auto& header) { return header == desc; }))
        {
            _dynamicHeader.push_back(desc);
            if (auto iter = std::ranges::find_if(_headerLogging, [&](const std::pair<std::string, bool>& header) {
                    return desc == header.first;
                });
                iter == _headerLogging.end())
            {
                _headerLogging.emplace_back(desc, _headerLoggingDefault);
                flow::ApplyChanges();
            }
        }
    }

    if (!_headerWritten) { writeHeader(obs); }
    else if (auto newHeaderLength = static_cast<size_t>(std::ranges::count_if(_headerLogging, [](const auto& header) { return header.second; }));
             oldHeaderLength != newHeaderLength)
    {
        rewriteData(oldHeaderLength, newHeaderLength, obs);
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

    const auto staticDataDescriptors = obs->staticDataDescriptors();
    for (size_t i = 0; i < obs->staticDescriptorCount(); ++i)
    {
        const auto& desc = staticDataDescriptors.at(i);
        if (auto iter = std::ranges::find_if(_headerLogging, [&](const std::pair<std::string, bool>& header) {
                return desc == header.first;
            });
            iter != _headerLogging.end() && !iter->second)
        {
            continue;
        }
        _filestream << ',';
        if (auto val = obs->getValueAt(i)) { _filestream << *val; }
    }

    for (const auto& desc : _dynamicHeader)
    {
        if (auto iter = std::ranges::find_if(_headerLogging, [&](const std::pair<std::string, bool>& header) {
                return desc == header.first;
            });
            iter != _headerLogging.end() && !iter->second)
        {
            continue;
        }
        _filestream << ',';
        if (auto val = obs->getDynamicDataAt(desc)) { _filestream << *val; }
    }

    _filestream << '\n';
}