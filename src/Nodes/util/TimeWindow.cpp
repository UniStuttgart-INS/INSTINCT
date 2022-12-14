// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "TimeWindow.hpp"

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"

#include "internal/gui/widgets/imgui_ex.hpp"
#include "internal/gui/NodeEditorApplication.hpp"

// ---------------------------------------------------------- Member functions -------------------------------------------------------------

NAV::TimeWindow::TimeWindow() : Node(typeStatic())
{
    LOG_TRACE("{}: called", name);
    _hasConfig = true;
    _guiConfigDefaultWindowSize = { 362, 263 };

    nm::CreateInputPin(this, "Input", Pin::Type::Flow, { NodeData::type() }, &TimeWindow::receiveObs);

    nm::CreateOutputPin(this, "Output", Pin::Type::Flow, { NodeData::type() });
}

NAV::TimeWindow::~TimeWindow()
{
    LOG_TRACE("{}: called", nameId());
}

std::string NAV::TimeWindow::typeStatic()
{
    return "TimeWindow";
}

std::string NAV::TimeWindow::type() const
{
    return typeStatic();
}

std::string NAV::TimeWindow::category()
{
    return "Utility";
}

void NAV::TimeWindow::guiConfig()
{
    float configWidth = 100.0F;

    ImGui::SetNextItemWidth(configWidth + ImGui::GetStyle().ItemSpacing.x);
    if (ImGui::Combo(fmt::format("Time format##{}", size_t(id)).c_str(), reinterpret_cast<int*>(&_timeFormat), "MJD\0JD\0GPST\0YMDHMS\0\0"))
    {
        LOG_DEBUG("{}: Time format changed to {}", nameId(), fmt::underlying(_timeFormat));
        flow::ApplyChanges();
        doReinitialize();
    }

    ImGui::Separator();

    if (ImGui::BeginTable(fmt::format("Time Window##{}", size_t(id)).c_str(), 3, ImGuiTableFlags_Borders | ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_NoHostExtendX, ImVec2(0.0F, 0.0F)))
    {
        ImGui::TableSetupColumn("Unit");
        ImGui::TableSetupColumn("Beginning");
        ImGui::TableSetupColumn("End");

        ImGui::TableHeadersRow();

        ImGui::TableNextRow();
        if (_timeFormat == TimeFormats::MJD || _timeFormat == TimeFormats::JD)
        {
            // Days
            ImGui::TableSetColumnIndex(0);
            ImGui::TextUnformatted("Days");

            ImGui::TableSetColumnIndex(1);
            ImGui::SetNextItemWidth(100.0F * gui::NodeEditorApplication::windowFontRatio());
            if (ImGui::InputInt(fmt::format("##Days_S{}", size_t(id)).c_str(), &_daysStart, 0, 0, ImGuiInputTextFlags_EnterReturnsTrue))
            {
                LOG_DEBUG("{}: beginning - days = {}", nameId(), _daysStart);
                flow::ApplyChanges();
                doReinitialize();
            }
            ImGui::TableSetColumnIndex(2);
            ImGui::SetNextItemWidth(100.0F * gui::NodeEditorApplication::windowFontRatio());
            if (ImGui::InputInt(fmt::format("##Days_E{}", size_t(id)).c_str(), &_daysEnd, 0, 0, ImGuiInputTextFlags_EnterReturnsTrue))
            {
                LOG_DEBUG("{}: end - days = {}", nameId(), _daysEnd);
                flow::ApplyChanges();
                doReinitialize();
            }

            // Decimal Fraction of a day
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::TextUnformatted("Decimal Fraction");

            ImGui::TableSetColumnIndex(1);
            ImGui::SetNextItemWidth(100.0F * gui::NodeEditorApplication::windowFontRatio());
            if (ImGui::InputDoubleL(fmt::format("##DecFrac_S{}", size_t(id)).c_str(), &_decFracStart, 1e-3, 1e4, 0.0, 0.0, "%f"))
            {
                LOG_DEBUG("{}: beginning - decimal fraction = {}", nameId(), _decFracStart);
                flow::ApplyChanges();
                doReinitialize();
            }
            ImGui::TableSetColumnIndex(2);
            ImGui::SetNextItemWidth(100.0F * gui::NodeEditorApplication::windowFontRatio());
            if (ImGui::InputDoubleL(fmt::format("##DecFrac_E{}", size_t(id)).c_str(), &_decFracEnd, 1e-3, 1e4, 0.0, 0.0, "%f"))
            {
                LOG_DEBUG("{}: end - decimal fraction = {}", nameId(), _decFracEnd);
                flow::ApplyChanges();
                doReinitialize();
            }
        }
        else if (_timeFormat == TimeFormats::GPST)
        {
            // GPS Cycle
            ImGui::TableSetColumnIndex(0);
            ImGui::TextUnformatted("GPS Cycle");

            ImGui::TableSetColumnIndex(1);
            ImGui::SetNextItemWidth(100.0F * gui::NodeEditorApplication::windowFontRatio());
            if (ImGui::InputInt(fmt::format("##GPScycle_S{}", size_t(id)).c_str(), &_gpsCycleStart, 0, 0, ImGuiInputTextFlags_EnterReturnsTrue))
            {
                LOG_DEBUG("{}: beginning - GPS Cycle = {}", nameId(), _gpsCycleStart);
                flow::ApplyChanges();
                doReinitialize();
            }
            ImGui::TableSetColumnIndex(2);
            ImGui::SetNextItemWidth(100.0F * gui::NodeEditorApplication::windowFontRatio());
            if (ImGui::InputInt(fmt::format("##GPScycle_E{}", size_t(id)).c_str(), &_gpsCycleEnd, 0, 0, ImGuiInputTextFlags_EnterReturnsTrue))
            {
                LOG_DEBUG("{}: end - GPS Cycle = {}", nameId(), _gpsCycleEnd);
                flow::ApplyChanges();
                doReinitialize();
            }

            // GPS Week
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::TextUnformatted("GPS Week");

            ImGui::TableSetColumnIndex(1);
            ImGui::SetNextItemWidth(100.0F * gui::NodeEditorApplication::windowFontRatio());
            if (ImGui::InputInt(fmt::format("##GPSweek_S{}", size_t(id)).c_str(), &_gpsWeekStart, 0, 0, ImGuiInputTextFlags_EnterReturnsTrue))
            {
                LOG_DEBUG("{}: beginning - GPS Week = {}", nameId(), _gpsWeekStart);
                flow::ApplyChanges();
                doReinitialize();
            }
            ImGui::TableSetColumnIndex(2);
            ImGui::SetNextItemWidth(100.0F * gui::NodeEditorApplication::windowFontRatio());
            if (ImGui::InputInt(fmt::format("##GPSweek_E{}", size_t(id)).c_str(), &_gpsWeekEnd, 0, 0, ImGuiInputTextFlags_EnterReturnsTrue))
            {
                LOG_DEBUG("{}: end - GPS Week = {}", nameId(), _gpsWeekEnd);
                flow::ApplyChanges();
                doReinitialize();
            }

            // GPS ToW
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::TextUnformatted("GPS ToW");

            ImGui::TableSetColumnIndex(1);
            ImGui::SetNextItemWidth(100.0F * gui::NodeEditorApplication::windowFontRatio());
            if (ImGui::InputDouble(fmt::format("##GPStow_S{}", size_t(id)).c_str(), &_gpsTowStart, 0, 0, "%.6f", ImGuiInputTextFlags_EnterReturnsTrue))
            {
                LOG_DEBUG("{}: beginning - GPS ToW = {}", nameId(), _gpsTowStart);
                flow::ApplyChanges();
                doReinitialize();
            }
            ImGui::TableSetColumnIndex(2);
            ImGui::SetNextItemWidth(100.0F * gui::NodeEditorApplication::windowFontRatio());
            if (ImGui::InputDouble(fmt::format("##GPStow_E{}", size_t(id)).c_str(), &_gpsTowEnd, 0, 0, "%.6f", ImGuiInputTextFlags_EnterReturnsTrue))
            {
                LOG_DEBUG("{}: end - GPS ToW = {}", nameId(), _gpsTowEnd);
                flow::ApplyChanges();
                doReinitialize();
            }
        }
        else if (_timeFormat == TimeFormats::YMDHMS)
        {
            // Year
            ImGui::TableSetColumnIndex(0);
            ImGui::TextUnformatted("Year");

            ImGui::TableSetColumnIndex(1);
            ImGui::SetNextItemWidth(100.0F * gui::NodeEditorApplication::windowFontRatio());
            if (ImGui::InputInt(fmt::format("##year_S{}", size_t(id)).c_str(), &_yearStart, 0, 0, ImGuiInputTextFlags_EnterReturnsTrue))
            {
                LOG_DEBUG("{}: beginning - Year = {}", nameId(), _yearStart);
                flow::ApplyChanges();
                doReinitialize();
            }
            ImGui::TableSetColumnIndex(2);
            ImGui::SetNextItemWidth(100.0F * gui::NodeEditorApplication::windowFontRatio());
            if (ImGui::InputInt(fmt::format("##year_E{}", size_t(id)).c_str(), &_yearEnd, 0, 0, ImGuiInputTextFlags_EnterReturnsTrue))
            {
                LOG_DEBUG("{}: end - Year = {}", nameId(), _yearEnd);
                flow::ApplyChanges();
                doReinitialize();
            }

            // Month
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::TextUnformatted("Month");

            ImGui::TableSetColumnIndex(1);
            ImGui::SetNextItemWidth(100.0F * gui::NodeEditorApplication::windowFontRatio());
            if (ImGui::InputInt(fmt::format("##month_S{}", size_t(id)).c_str(), &_monthStart, 0, 0, ImGuiInputTextFlags_EnterReturnsTrue))
            {
                LOG_DEBUG("{}: beginning - Month = {}", nameId(), _monthStart);
                flow::ApplyChanges();
                doReinitialize();
            }
            ImGui::TableSetColumnIndex(2);
            ImGui::SetNextItemWidth(100.0F * gui::NodeEditorApplication::windowFontRatio());
            if (ImGui::InputInt(fmt::format("##month_E{}", size_t(id)).c_str(), &_monthEnd, 0, 0, ImGuiInputTextFlags_EnterReturnsTrue))
            {
                LOG_DEBUG("{}: end - Month = {}", nameId(), _monthEnd);
                flow::ApplyChanges();
                doReinitialize();
            }

            // Day
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::TextUnformatted("Day");

            ImGui::TableSetColumnIndex(1);
            ImGui::SetNextItemWidth(100.0F * gui::NodeEditorApplication::windowFontRatio());
            if (ImGui::InputInt(fmt::format("##day_S{}", size_t(id)).c_str(), &_dayStart, 0, 0, ImGuiInputTextFlags_EnterReturnsTrue))
            {
                LOG_DEBUG("{}: beginning - Day = {}", nameId(), _dayStart);
                flow::ApplyChanges();
                doReinitialize();
            }
            ImGui::TableSetColumnIndex(2);
            ImGui::SetNextItemWidth(100.0F * gui::NodeEditorApplication::windowFontRatio());
            if (ImGui::InputInt(fmt::format("##day_E{}", size_t(id)).c_str(), &_dayEnd, 0, 0, ImGuiInputTextFlags_EnterReturnsTrue))
            {
                LOG_DEBUG("{}: end - Day = {}", nameId(), _dayEnd);
                flow::ApplyChanges();
                doReinitialize();
            }

            // Hour
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::TextUnformatted("Hour");

            ImGui::TableSetColumnIndex(1);
            ImGui::SetNextItemWidth(100.0F * gui::NodeEditorApplication::windowFontRatio());
            if (ImGui::InputInt(fmt::format("##hour_S{}", size_t(id)).c_str(), &_hourStart, 0, 0, ImGuiInputTextFlags_EnterReturnsTrue))
            {
                LOG_DEBUG("{}: beginning - Hour = {}", nameId(), _hourStart);
                flow::ApplyChanges();
                doReinitialize();
            }
            ImGui::TableSetColumnIndex(2);
            ImGui::SetNextItemWidth(100.0F * gui::NodeEditorApplication::windowFontRatio());
            if (ImGui::InputInt(fmt::format("##hour_E{}", size_t(id)).c_str(), &_hourEnd, 0, 0, ImGuiInputTextFlags_EnterReturnsTrue))
            {
                LOG_DEBUG("{}: end - Hour = {}", nameId(), _hourEnd);
                flow::ApplyChanges();
                doReinitialize();
            }

            // Minute
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::TextUnformatted("Minute");

            ImGui::TableSetColumnIndex(1);
            ImGui::SetNextItemWidth(100.0F * gui::NodeEditorApplication::windowFontRatio());
            if (ImGui::InputInt(fmt::format("##min_S{}", size_t(id)).c_str(), &_minStart, 0, 0, ImGuiInputTextFlags_EnterReturnsTrue))
            {
                LOG_DEBUG("{}: beginning - Minute = {}", nameId(), _minStart);
                flow::ApplyChanges();
                doReinitialize();
            }
            ImGui::TableSetColumnIndex(2);
            ImGui::SetNextItemWidth(100.0F * gui::NodeEditorApplication::windowFontRatio());
            if (ImGui::InputInt(fmt::format("##min_E{}", size_t(id)).c_str(), &_minEnd, 0, 0, ImGuiInputTextFlags_EnterReturnsTrue))
            {
                LOG_DEBUG("{}: end - Minute = {}", nameId(), _minEnd);
                flow::ApplyChanges();
                doReinitialize();
            }

            // Second
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::TextUnformatted("Second");

            ImGui::TableSetColumnIndex(1);
            ImGui::SetNextItemWidth(100.0F * gui::NodeEditorApplication::windowFontRatio());
            if (ImGui::InputDouble(fmt::format("##sec_S{}", size_t(id)).c_str(), &_secStart, 0, 0, "%.6f", ImGuiInputTextFlags_EnterReturnsTrue))
            {
                LOG_DEBUG("{}: beginning - Second = {}", nameId(), _secStart);
                flow::ApplyChanges();
                doReinitialize();
            }
            ImGui::TableSetColumnIndex(2);
            ImGui::SetNextItemWidth(100.0F * gui::NodeEditorApplication::windowFontRatio());
            if (ImGui::InputDouble(fmt::format("##sec_E{}", size_t(id)).c_str(), &_secEnd, 0, 0, "%.6f", ImGuiInputTextFlags_EnterReturnsTrue))
            {
                LOG_DEBUG("{}: end - Second = {}", nameId(), _secEnd);
                flow::ApplyChanges();
                doReinitialize();
            }
        }

        ImGui::EndTable();
    }
}

json NAV::TimeWindow::save() const
{
    LOG_TRACE("{}: called", nameId());

    json j;

    j["timeFormat"] = _timeFormat;
    j["daysStart"] = _daysStart;
    j["decFracStart"] = _decFracStart;
    j["gpsCycleStart"] = _gpsCycleStart;
    j["gpsWeekStart"] = _gpsWeekStart;
    j["gpsTowStart"] = _gpsTowStart;
    j["yearStart"] = _yearStart;
    j["monthStart"] = _monthStart;
    j["dayStart"] = _dayStart;
    j["hourStart"] = _hourStart;
    j["minStart"] = _minStart;
    j["secStart"] = _secStart;
    j["daysEnd"] = _daysEnd;
    j["decFracEnd"] = _decFracEnd;
    j["gpsCycleEnd"] = _gpsCycleEnd;
    j["gpsWeekEnd"] = _gpsWeekEnd;
    j["gpsTowEnd"] = _gpsTowEnd;
    j["yearEnd"] = _yearEnd;
    j["monthEnd"] = _monthEnd;
    j["dayEnd"] = _dayEnd;
    j["hourEnd"] = _hourEnd;
    j["minEnd"] = _minEnd;
    j["secEnd"] = _secEnd;

    return j;
}

void NAV::TimeWindow::restore(json const& j)
{
    LOG_TRACE("{}: called", nameId());

    if (j.contains("timeFormat"))
    {
        j.at("timeFormat").get_to(_timeFormat);
    }
    if (j.contains("daysStart"))
    {
        j.at("daysStart").get_to(_daysStart);
    }
    if (j.contains("decFracStart"))
    {
        j.at("decFracStart").get_to(_decFracStart);
    }
    if (j.contains("gpsCycleStart"))
    {
        j.at("gpsCycleStart").get_to(_gpsCycleStart);
    }
    if (j.contains("gpsWeekStart"))
    {
        j.at("gpsWeekStart").get_to(_gpsWeekStart);
    }
    if (j.contains("gpsTowStart"))
    {
        j.at("gpsTowStart").get_to(_gpsTowStart);
    }
    if (j.contains("yearStart"))
    {
        j.at("yearStart").get_to(_yearStart);
    }
    if (j.contains("monthStart"))
    {
        j.at("monthStart").get_to(_monthStart);
    }
    if (j.contains("dayStart"))
    {
        j.at("dayStart").get_to(_dayStart);
    }
    if (j.contains("hourStart"))
    {
        j.at("hourStart").get_to(_hourStart);
    }
    if (j.contains("minStart"))
    {
        j.at("minStart").get_to(_minStart);
    }
    if (j.contains("secStart"))
    {
        j.at("secStart").get_to(_secStart);
    }
    if (j.contains("daysEnd"))
    {
        j.at("daysEnd").get_to(_daysEnd);
    }
    if (j.contains("decFracEnd"))
    {
        j.at("decFracEnd").get_to(_decFracEnd);
    }
    if (j.contains("gpsCycleEnd"))
    {
        j.at("gpsCycleEnd").get_to(_gpsCycleEnd);
    }
    if (j.contains("gpsWeekEnd"))
    {
        j.at("gpsWeekEnd").get_to(_gpsWeekEnd);
    }
    if (j.contains("gpsTowEnd"))
    {
        j.at("gpsTowEnd").get_to(_gpsTowEnd);
    }
    if (j.contains("yearEnd"))
    {
        j.at("yearEnd").get_to(_yearEnd);
    }
    if (j.contains("monthEnd"))
    {
        j.at("monthEnd").get_to(_monthEnd);
    }
    if (j.contains("dayEnd"))
    {
        j.at("dayEnd").get_to(_dayEnd);
    }
    if (j.contains("hourEnd"))
    {
        j.at("hourEnd").get_to(_hourEnd);
    }
    if (j.contains("minEnd"))
    {
        j.at("minEnd").get_to(_minEnd);
    }
    if (j.contains("secEnd"))
    {
        j.at("secEnd").get_to(_secEnd);
    }
}

bool NAV::TimeWindow::initialize()
{
    LOG_TRACE("{}: called", nameId());

    InsTime startTime;
    InsTime endTime;

    switch (_timeFormat)
    {
    case TimeFormats::MJD:
        startTime = InsTime(InsTime_MJD{ _daysStart, _decFracStart });
        endTime = InsTime(InsTime_MJD{ _daysEnd, _decFracEnd });
        break;
    case TimeFormats::JD:
        startTime = InsTime(InsTime_JD{ _daysStart, _decFracStart });
        endTime = InsTime(InsTime_JD{ _daysEnd, _decFracEnd });
        break;
    case TimeFormats::GPST:
        startTime = InsTime(InsTime_GPSweekTow{ _gpsCycleStart, _gpsWeekStart, _gpsTowStart });
        endTime = InsTime(InsTime_GPSweekTow{ _gpsCycleEnd, _gpsWeekEnd, _gpsTowEnd });
        break;
    case TimeFormats::YMDHMS:
        startTime = InsTime(InsTime_YMDHMS{ _yearStart, _monthStart, _dayStart, _hourStart, _minStart, _secStart });
        endTime = InsTime(InsTime_YMDHMS{ _yearEnd, _monthEnd, _dayEnd, _hourEnd, _minEnd, _secEnd });
        break;
    }

    if (startTime < endTime) // Everything is okay
    {
        _startTime = startTime;
        _endTime = endTime;
        return true;
    }

    LOG_ERROR("{}: startTime >= endTime --> TimeWindow blocks all packages. Please reconfigure the node.", nameId());
    return false;
}

void NAV::TimeWindow::afterCreateLink(OutputPin& startPin, InputPin& endPin)
{
    LOG_TRACE("{}: called for {} ==> {}", nameId(), size_t(startPin.id), size_t(endPin.id));

    if (endPin.parentNode->id != id)
    {
        return; // Link on Output Port
    }

    // Store previous output pin identifier
    auto previousOutputPinDataIdentifier = outputPins.at(OUTPUT_PORT_INDEX_FLOW).dataIdentifier;
    // Overwrite output pin identifier with input pin identifier
    outputPins.at(OUTPUT_PORT_INDEX_FLOW).dataIdentifier = startPin.dataIdentifier;

    if (previousOutputPinDataIdentifier != outputPins.at(OUTPUT_PORT_INDEX_FLOW).dataIdentifier) // If the identifier changed
    {
        // Check if connected links on output port are still valid
        for (auto& link : outputPins.at(OUTPUT_PORT_INDEX_FLOW).links)
        {
            if (auto* endPin = link.getConnectedPin())
            {
                if (!outputPins.at(OUTPUT_PORT_INDEX_FLOW).canCreateLink(*endPin))
                {
                    // If the link is not valid anymore, delete it
                    outputPins.at(OUTPUT_PORT_INDEX_FLOW).deleteLink(*endPin);
                }
            }
        }

        // Refresh all links connected to the output pin if the type changed
        if (outputPins.at(OUTPUT_PORT_INDEX_FLOW).dataIdentifier != previousOutputPinDataIdentifier)
        {
            for (auto& link : outputPins.at(OUTPUT_PORT_INDEX_FLOW).links)
            {
                if (auto* connectedPin = link.getConnectedPin())
                {
                    outputPins.at(OUTPUT_PORT_INDEX_FLOW).recreateLink(*connectedPin);
                }
            }
        }
    }
}

void NAV::TimeWindow::afterDeleteLink(OutputPin& startPin, InputPin& endPin)
{
    LOG_TRACE("{}: called for {} ==> {}", nameId(), size_t(startPin.id), size_t(endPin.id));

    if ((endPin.parentNode->id != id                                  // Link on Output port is removed
         && !inputPins.at(INPUT_PORT_INDEX_FLOW).isPinLinked())       //     and the Input port is not linked
        || (startPin.parentNode->id != id                             // Link on Input port is removed
            && !outputPins.at(OUTPUT_PORT_INDEX_FLOW).isPinLinked())) //     and the Output port is not linked
    {
        outputPins.at(OUTPUT_PORT_INDEX_FLOW).dataIdentifier = { NodeData::type() };
    }
}

void NAV::TimeWindow::receiveObs(NAV::InputPin::NodeDataQueue& queue, size_t /* pinIdx */)
{
    // Check whether timestamp is within the time window
    auto obs = queue.extract_front();
    if (obs->insTime >= _startTime && obs->insTime <= _endTime)
    {
        invokeCallbacks(OUTPUT_PORT_INDEX_FLOW, obs);
    }
    else if (obs->insTime < _startTime)
    {
        LOG_DATA("{}: insTime < startTime --> message skipped", nameId());
    }
    else if (obs->insTime > _endTime)
    {
        LOG_DATA("{}: insTime > endTime --> message skipped", nameId());
    }
}