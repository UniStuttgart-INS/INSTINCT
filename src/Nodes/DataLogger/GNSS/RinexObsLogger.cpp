// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "RinexObsLogger.hpp"

#include <chrono>
#include <string>
#include "Navigation/GNSS/Core/SatelliteSystem.hpp"
#include <fmt/chrono.h>
#include <fmt/format.h>
using namespace fmt::literals; // NOLINT(google-build-using-namespace)

#include "internal/gui/widgets/imgui_ex.hpp"
#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"

#include "internal/gui/widgets/EnumComboWithTooltip.hpp"
#include "internal/gui/widgets/HelpMarker.hpp"

#include "NodeData/GNSS/GnssObs.hpp"

#include "Nodes/DataProvider/GNSS/FileReader/RinexObsFile.hpp"

#include "util/Logger.hpp"

NAV::RinexObsLogger::RinexObsLogger()
    : Node(typeStatic())
{
    LOG_TRACE("{}: called", name);

    _fileType = FileType::ASCII;

    _hasConfig = true;
    _guiConfigDefaultWindowSize = { 643, 728 };

    _dynamicInputPins.addPin(this);
}

NAV::RinexObsLogger::~RinexObsLogger()
{
    LOG_TRACE("{}: called", nameId());
}

std::string NAV::RinexObsLogger::typeStatic()
{
    return "RinexObsLogger";
}

std::string NAV::RinexObsLogger::type() const
{
    return typeStatic();
}

std::string NAV::RinexObsLogger::category()
{
    return "Data Logger";
}

void NAV::RinexObsLogger::guiConfig()
{
    if (_dynamicInputPins.ShowGuiWidgets(size_t(id), inputPins, this, {}))
    {
        flow::ApplyChanges();
    }

    constexpr float COL1_WIDTH = 470.0F;

    const auto now = std::chrono::system_clock::now();
    ImGui::SetNextItemWidth(COL1_WIDTH);
    if (FileWriter::guiConfig(fmt::format(".obs,.rnx,.{:%y}O", now).c_str(), { ".obs", ".rnx", fmt::format(".{:%y}O", now) }, size_t(id), nameId()))
    {
        flow::ApplyChanges();
        doDeinitialize();
    }

    if (ImGui::CollapsingHeader(fmt::format("General##{}", size_t(id)).c_str(), ImGuiTreeNodeFlags_DefaultOpen))
    {
        if (ImGui::BeginTable(fmt::format("Table1##{}", size_t(id)).c_str(), 2, ImGuiTableFlags_SizingFixedFit))
        {
            ImGui::TableNextColumn();
            ImGui::BeginDisabled();
            ImGui::SetNextItemWidth(COL1_WIDTH);
            if (ImGui::BeginCombo(fmt::format("##Version {}", size_t(id)).c_str(), fmt::format("{}", _header.version).c_str()))
            {
                for (const auto& version : _supportedVersions)
                {
                    const bool is_selected = _header.version == version;
                    if (ImGui::Selectable(fmt::format("{}##Version {}", version, size_t(id)).c_str(), is_selected))
                    {
                        _header.version = version;
                        flow::ApplyChanges();
                    }
                    if (is_selected) { ImGui::SetItemDefaultFocus(); }
                }
                ImGui::EndCombo();
            }
            ImGui::EndDisabled();
            ImGui::TableNextColumn();
            ImGui::TextUnformatted("Version");

            ImGui::TableNextColumn();
            ImGui::SetNextItemWidth(COL1_WIDTH);
            if (ImGui::InputTextL(fmt::format("##Run By {}", size_t(id)).c_str(), &_header.runBy, 20))
            {
                flow::ApplyChanges();
            }
            ImGui::TableNextColumn();
            ImGui::TextUnformatted("Run By");

            ImGui::TableNextColumn();
            ImGui::SetNextItemWidth(COL1_WIDTH);
            if (ImGui::InputTextL(fmt::format("##Observer {}", size_t(id)).c_str(), &_header.observer, 20))
            {
                flow::ApplyChanges();
            }
            ImGui::TableNextColumn();
            ImGui::TextUnformatted("Observer");

            ImGui::TableNextColumn();
            ImGui::SetNextItemWidth(COL1_WIDTH);
            if (ImGui::InputTextL(fmt::format("##Agency {}", size_t(id)).c_str(), &_header.agency, 40))
            {
                flow::ApplyChanges();
            }
            ImGui::TableNextColumn();
            ImGui::TextUnformatted("Agency");

            ImGui::EndTable();
        }
    }

    if (ImGui::CollapsingHeader(fmt::format("Comments##{}", size_t(id)).c_str(), ImGuiTreeNodeFlags_DefaultOpen))
    {
        std::vector<size_t> commentsToRemove;
        if (ImGui::BeginTable(fmt::format("Table2##{}", size_t(id)).c_str(), 2, ImGuiTableFlags_SizingFixedFit))
        {
            for (size_t i = 0; i < _header.comments.size(); i++)
            {
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                auto& comment = _header.comments.at(i);
                float startPos = ImGui::GetCursorPosX();
                if (ImGui::Button(fmt::format("X##Remove comment {} {}", i, size_t(id)).c_str()))
                {
                    commentsToRemove.push_back(i);
                }
                ImGui::SameLine();
                ImGui::SetNextItemWidth(COL1_WIDTH - (ImGui::GetCursorPosX() - startPos));
                if (ImGui::InputTextL(fmt::format("##Comment {} {}", i, size_t(id)).c_str(), &comment, 60))
                {
                    flow::ApplyChanges();
                }
            }
            ImGui::EndTable();
        }
        for (const size_t& idx : commentsToRemove)
        {
            _header.comments.erase(std::next(_header.comments.begin(), static_cast<int>(idx)));
            flow::ApplyChanges();
        }
        if (ImGui::Button(fmt::format("Add##Comment {}", size_t(id)).c_str()))
        {
            _header.comments.emplace_back();
            flow::ApplyChanges();
        }
    }

    if (ImGui::CollapsingHeader(fmt::format("Marker##{}", size_t(id)).c_str(), ImGuiTreeNodeFlags_DefaultOpen))
    {
        if (ImGui::BeginTable(fmt::format("Table3##{}", size_t(id)).c_str(), 2, ImGuiTableFlags_SizingFixedFit))
        {
            ImGui::TableNextColumn();
            ImGui::SetNextItemWidth(COL1_WIDTH);
            if (gui::widgets::EnumComboWithToolTip(fmt::format("##Marker Type {}", size_t(id)).c_str(), _header.markerType))
            {
                flow::ApplyChanges();
            }
            if (_header.markerType != vendor::RINEX::ObsHeader::MarkerTypes::USER_DEFINED) { ImGui::BeginDisabled(); }
            ImGui::TableNextColumn();
            ImGui::TextUnformatted("Type");

            ImGui::TableNextColumn();
            ImGui::SetNextItemWidth(COL1_WIDTH);
            if (ImGui::InputTextL(fmt::format("##Marker User Type {}", size_t(id)).c_str(), &_header.markerTypeUser, 20))
            {
                flow::ApplyChanges();
            }
            if (_header.markerType != vendor::RINEX::ObsHeader::MarkerTypes::USER_DEFINED) { ImGui::EndDisabled(); }
            ImGui::TableNextColumn();
            ImGui::TextUnformatted("User Type");
            ImGui::SameLine();
            ImGui::Dummy(ImVec2(ImGui::GetContentRegionAvail().x, 0.0F));

            ImGui::TableNextColumn();
            ImGui::SetNextItemWidth(COL1_WIDTH);
            if (ImGui::InputTextL(fmt::format("##Marker Name {}", size_t(id)).c_str(), &_header.markerName, 60))
            {
                flow::ApplyChanges();
            }
            ImGui::TableNextColumn();
            ImGui::TextUnformatted("Name");

            ImGui::TableNextColumn();
            ImGui::SetNextItemWidth(COL1_WIDTH);
            if (ImGui::InputTextL(fmt::format("##Marker Number {}", size_t(id)).c_str(), &_header.markerNumber, 20))
            {
                flow::ApplyChanges();
            }
            ImGui::TableNextColumn();
            ImGui::TextUnformatted("Number");

            ImGui::EndTable();
        }
    }

    if (ImGui::CollapsingHeader(fmt::format("Receiver##{}", size_t(id)).c_str(), ImGuiTreeNodeFlags_DefaultOpen))
    {
        if (ImGui::BeginTable(fmt::format("Table4##{}", size_t(id)).c_str(), 2, ImGuiTableFlags_SizingFixedFit))
        {
            ImGui::TableNextColumn();
            ImGui::SetNextItemWidth(COL1_WIDTH);
            if (ImGui::InputTextL(fmt::format("##Receiver Number {}", size_t(id)).c_str(), &_header.receiverNumber, 20))
            {
                flow::ApplyChanges();
            }
            ImGui::TableNextColumn();
            ImGui::TextUnformatted("Number");

            ImGui::TableNextColumn();
            ImGui::SetNextItemWidth(COL1_WIDTH);
            if (ImGui::InputTextL(fmt::format("##Receiver Type {}", size_t(id)).c_str(), &_header.receiverType, 20))
            {
                flow::ApplyChanges();
            }
            ImGui::TableNextColumn();
            ImGui::TextUnformatted("Type");

            ImGui::TableNextColumn();
            ImGui::SetNextItemWidth(COL1_WIDTH);
            if (ImGui::InputTextL(fmt::format("##Receiver Version {}", size_t(id)).c_str(), &_header.receiverVersion, 20))
            {
                flow::ApplyChanges();
            }
            ImGui::TableNextColumn();
            ImGui::TextUnformatted("Version");

            ImGui::EndTable();
        }
    }

    if (ImGui::CollapsingHeader(fmt::format("Antenna##{}", size_t(id)).c_str(), ImGuiTreeNodeFlags_DefaultOpen))
    {
        if (ImGui::BeginTable(fmt::format("Table5##{}", size_t(id)).c_str(), 2, ImGuiTableFlags_SizingFixedFit))
        {
            ImGui::TableNextColumn();
            ImGui::SetNextItemWidth(COL1_WIDTH);
            if (ImGui::InputTextL(fmt::format("##Antenna Number {}", size_t(id)).c_str(), &_header.antennaNumber, 20))
            {
                flow::ApplyChanges();
            }
            ImGui::TableNextColumn();
            ImGui::TextUnformatted("Number");

            ImGui::TableNextColumn();
            ImGui::SetNextItemWidth(COL1_WIDTH);
            if (ImGui::InputTextL(fmt::format("##Antenna Type {}", size_t(id)).c_str(), &_header.antennaType, 20))
            {
                flow::ApplyChanges();
            }
            ImGui::TableNextColumn();
            ImGui::TextUnformatted("Type");

            ImGui::TableNextColumn();
            float startPos = ImGui::GetCursorPosX();
            if (ImGui::Checkbox(fmt::format("##ApproxEnabled {}", size_t(id)).c_str(), &_header.approxPositionEnabled))
            {
                flow::ApplyChanges();
            }
            ImGui::SameLine();
            ImGui::SetNextItemWidth(COL1_WIDTH - (ImGui::GetCursorPosX() - startPos));
            if (!_header.approxPositionEnabled) { ImGui::BeginDisabled(); }
            if (ImGui::InputDouble3L(fmt::format("##Approx position XYZ {}", size_t(id)).c_str(), _header.approxPositionXYZ.data(), -99999999.9999, 99999999.9999, "%.4f m"))
            {
                flow::ApplyChanges();
            }
            if (!_header.approxPositionEnabled) { ImGui::EndDisabled(); }
            ImGui::TableNextColumn();
            ImGui::TextUnformatted("Approx position XYZ");
            ImGui::SameLine();
            gui::widgets::HelpMarker("Geocentric approximate marker position\n"
                                     "System: ITRS recommended\n"
                                     "Optional for moving platforms");

            ImGui::TableNextColumn();
            ImGui::SetNextItemWidth(COL1_WIDTH);
            if (ImGui::InputDouble3L(fmt::format("##Antenna delta HEN {}", size_t(id)).c_str(), _header.antennaDeltaHeightEastNorth.data(), -99999999.9999, 99999999.9999, "%.4f m"))
            {
                flow::ApplyChanges();
            }
            ImGui::TableNextColumn();
            ImGui::TextUnformatted("Delta HEN");

            ImGui::EndTable();
        }
    }
}

[[nodiscard]] json NAV::RinexObsLogger::save() const
{
    LOG_TRACE("{}: called", nameId());

    return {
        { "dynamicInputPins", _dynamicInputPins },
        { "FileWriter", FileWriter::save() },
        { "HeaderInfo", _header },
    };
}

void NAV::RinexObsLogger::restore(json const& j)
{
    LOG_TRACE("{}: called", nameId());

    if (j.contains("dynamicInputPins")) { NAV::gui::widgets::from_json(j.at("dynamicInputPins"), _dynamicInputPins, this); }
    if (j.contains("FileWriter")) { FileWriter::restore(j.at("FileWriter")); }
    if (j.contains("HeaderInfo")) { j.at("HeaderInfo").get_to(_header); }
}

void NAV::RinexObsLogger::flush()
{
    LOG_TRACE("{}: called", nameId());
    _filestream.close();

    LOG_DATA("{}: ", nameId());

    auto fs = std::fstream(getFilepath(), std::ios::in | std::ios::out | std::ios::binary);
    if (!fs.good()) { LOG_CRITICAL("{}: Could not open file: {}", nameId(), getFilepath()); }

    while (!fs.eof())
    {
        std::string line;
        auto pos = fs.tellg();
        std::getline(fs, line);
        LOG_DATA("{}: line: {}", nameId(), line);
        if (line.find("RINEX VERSION / TYPE", 60) != std::string::npos)
        {
            fs.seekg(pos);
            LOG_DATA("{}: new : {}", nameId(), _header.headerLineRinexVersionType());
            fs << _header.headerLineRinexVersionType();
            fs.flush();
        }
        else if (line.find("TIME OF LAST OBS", 60) != std::string::npos)
        {
            fs.seekg(pos);
            LOG_DATA("{}: new : {}", nameId(), _header.headerLineTimeOfLastObs());
            fs << _header.headerLineTimeOfLastObs();
            fs.flush();
        }
        else if (line.find("INTERVAL", 60) != std::string::npos)
        {
            fs.seekg(pos);
            LOG_DATA("{}: new : {}", nameId(), _header.headerLineInterval());
            fs << _header.headerLineInterval();
            fs.flush();
        }
        else if (line.find("# OF SATELLITES", 60) != std::string::npos)
        {
            fs.seekg(pos);
            LOG_DATA("{}: new : {}", nameId(), _header.headerLineNumSatellites());
            fs << _header.headerLineNumSatellites();
            fs.flush();
        }
        else if (line[0] == '>') { break; }
    }
}

bool NAV::RinexObsLogger::initialize()
{
    LOG_TRACE("{}: called", nameId());

    if (!FileWriter::initialize())
    {
        return false;
    }

    _header.reset();

    _filestream << _header.generateHeader();
    _filestream.flush();

    return true;
}

void NAV::RinexObsLogger::deinitialize()
{
    LOG_TRACE("{}: called", nameId());

    FileWriter::deinitialize();
}

void NAV::RinexObsLogger::pinAddCallback(Node* node)
{
    nm::CreateInputPin(node, GnssObs::type().c_str(), Pin::Type::Flow, { GnssObs::type() }, &RinexObsLogger::writeObservation);
}

void NAV::RinexObsLogger::pinDeleteCallback(Node* node, size_t pinIdx)
{
    nm::DeleteInputPin(node->inputPins.at(pinIdx));
}

void NAV::RinexObsLogger::updateFileHeader(TimeSystem oldTimeSys)
{
    LOG_TRACE("{}: called", nameId());

    _filestream.close();
    _filestream.clear();

    std::string pathTmp = getFilepath().string() + ".tmp";
    {
        auto fsTmp = std::ofstream(pathTmp, std::ios_base::trunc | std::ios_base::binary);
        if (!fsTmp.good()) { LOG_CRITICAL("{}: Could not create temporary file: {}", nameId(), pathTmp); }

        auto fsOld = std::ifstream(getFilepath(), std::ios_base::in);
        if (!fsOld.good()) { LOG_CRITICAL("{}: Could not open old file: {}", nameId(), getFilepath()); }

        fsTmp << _header.generateHeader();
        LOG_DATA("{}: New Header\n{}", nameId(), _header.generateHeader());

        bool dataRecords = false;
        while (!fsOld.eof())
        {
            std::string line;
            std::getline(fsOld, line);
            LOG_DATA("{}: Line: {}", nameId(), line);
            if (line[0] == '>')
            {
                dataRecords = true;
                if (oldTimeSys == _header.timeSys)
                {
                    LOG_DATA("{}: Read whole file", nameId());
                    fsTmp << line << '\n';
                    fsTmp << fsOld.rdbuf(); // Read whole data record at once
                    break;
                }

                LOG_DATA("{}: Read epoch [{}][{}][{}][{}][{}][{}] [{}]", nameId(), line.substr(2, 4), line.substr(7, 2), line.substr(10, 2),
                         line.substr(13, 2), line.substr(16, 2), line.substr(18, 11), line.substr(29 + 3, 3));
                auto epochTime = InsTime{ static_cast<uint16_t>(std::stoi(line.substr(2, 4))),  // year  [1X,I4]
                                          static_cast<uint16_t>(std::stoi(line.substr(7, 2))),  // month [1X,I2.2]
                                          static_cast<uint16_t>(std::stoi(line.substr(10, 2))), // day   [1X,I2.2]
                                          static_cast<uint16_t>(std::stoi(line.substr(13, 2))), // hour  [1X,I2.2]
                                          static_cast<uint16_t>(std::stoi(line.substr(16, 2))), // min   [1X,I2.2]
                                          std::stold(line.substr(18, 11)),                      // sec   [F11.7,2X,I1,]
                                          oldTimeSys };
                fsTmp << _header.epochRecordLine(epochTime, std::stoul(line.substr(29 + 3, 3))); // Num satellites I3,6X,F15.12
            }
            else if (dataRecords)
            {
                fsTmp << line;
                if (!line.empty()) { fsTmp << '\n'; }
            }
        }
    }
    std::filesystem::rename(pathTmp, getFilepath());

    _filestream.open(getFilepath(), std::ios_base::app | std::ios_base::binary);
    if (!_filestream.good()) { LOG_CRITICAL("{}: Could not open file: {}", nameId(), getFilepath()); }
}

void NAV::RinexObsLogger::writeObservation(NAV::InputPin::NodeDataQueue& queue, size_t /* pinIdx */)
{
    auto obs = std::static_pointer_cast<const GnssObs>(queue.extract_front());
    LOG_DATA("{}: [{}]", nameId(), obs->insTime.toYMDHMS(GPST));

    _header.interval = std::min(_header.interval, std::round(static_cast<double>((obs->insTime - _header.timeLastObs).count()) * 1e3) / 1e3);
    if (_header.timeFirstObs.empty()) { _header.timeFirstObs = obs->insTime; }
    _header.timeLastObs = obs->insTime;

    std::set<SatId> satellites;
    bool satelliteSystemDescriptionChanged = false;
    for (const auto& sig : obs->data)
    {
        auto satId = sig.satSigId.toSatId();
        satellites.insert(satId);
        _header.satellites.insert(satId);
        _header.satSys |= satId.satSys;

        if (sig.pseudorange) { satelliteSystemDescriptionChanged |= _header.addObsType(sig.satSigId.code, vendor::RINEX::ObsType::C); }
        if (sig.carrierPhase) { satelliteSystemDescriptionChanged |= _header.addObsType(sig.satSigId.code, vendor::RINEX::ObsType::L); }
        if (sig.doppler) { satelliteSystemDescriptionChanged |= _header.addObsType(sig.satSigId.code, vendor::RINEX::ObsType::D); }
        if (sig.CN0) { satelliteSystemDescriptionChanged |= _header.addObsType(sig.satSigId.code, vendor::RINEX::ObsType::S); }
    }
    if (satelliteSystemDescriptionChanged)
    {
        TimeSystem oldTimeSys = _header.timeSys;
        _header.timeSys = vendor::RINEX::timeSystem(_header.satSys);
        updateFileHeader(oldTimeSys);
    }

    _filestream << _header.epochRecordLine(obs->insTime, satellites.size());

    for (const auto& satId : satellites)
    {
        _filestream << fmt::format("{0}{1:02d}", satId.satSys.toChar(),
                                   satId.satSys == SBAS && satId.satNum > 100 ? satId.satNum - 100 : satId.satNum);
        const auto& obsDescriptions = _header.systemObsTypes.at(satId.satSys);
        for (size_t i = 0; i < obsDescriptions.size(); i++)
        {
            const auto& obsDesc = obsDescriptions.at(i);

            auto signal = std::ranges::find_if(obs->data, [&obsDesc, &satId](const auto& sig) {
                return sig.satSigId == SatSigId{ obsDesc.code, satId.satNum };
            });

            bool obsWritten = false;
            if (signal != obs->data.end())
            {
                switch (obsDesc.type)
                {
                case vendor::RINEX::ObsType::C:
                    if (signal->pseudorange && signal->pseudorange->value < 100'000'000.0)
                    {
                        _filestream << fmt::format("{obs:14.3f} {SSI}",
                                                   "obs"_a = signal->pseudorange->value,
                                                   "SSI"_a = signal->pseudorange->SSI == 0 ? " " : std::to_string(signal->pseudorange->SSI));
                        obsWritten = true;
                    }
                    break;
                case vendor::RINEX::ObsType::L:
                    if (signal->carrierPhase && signal->carrierPhase->value < 1'000'000'000.0)
                    {
                        _filestream << fmt::format("{obs:14.3f}{LLI:1}{SSI:1}",
                                                   "obs"_a = signal->carrierPhase->value,
                                                   "LLI"_a = signal->carrierPhase->LLI == 0 ? " " : std::to_string(signal->carrierPhase->LLI),
                                                   "SSI"_a = signal->carrierPhase->SSI == 0 ? " " : std::to_string(signal->carrierPhase->SSI));
                        obsWritten = true;
                    }
                    break;
                case vendor::RINEX::ObsType::D:
                    if (signal->doppler)
                    {
                        _filestream << fmt::format("{:14.3f}  ", signal->doppler.value());
                        obsWritten = true;
                    }
                    break;
                case vendor::RINEX::ObsType::S:
                    if (signal->CN0)
                    {
                        _filestream << fmt::format("{:14.3f}  ", signal->CN0.value());
                        obsWritten = true;
                    }
                    break;
                case vendor::RINEX::ObsType::I:
                case vendor::RINEX::ObsType::X:
                case vendor::RINEX::ObsType::Error:
                    break;
                }
            }
            if (!obsWritten)
            {
                _filestream << fmt::format("{X:16}", "X"_a = "");
            }
            if (i == obsDescriptions.size() - 1)
            {
                _filestream << "\n";
            }
        }
    }
}